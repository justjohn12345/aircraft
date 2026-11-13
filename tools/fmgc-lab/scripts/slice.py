#!/usr/bin/env python3
# Copyright (c) 2026 FlyByWire Simulations
#
# SPDX-License-Identifier: GPL-3.0
"""
Backward slicer for the generated step() functions: prints only the source lines that can affect a given variable
(data and control dependencies), with parameter values annotated. Built on the clang AST from scripts/ast.sh.

The generator reuses temporaries (rtb_Compare_ji is assigned dozens of times), so dependencies are resolved per use with
reaching definitions: walking back from the use, a definition that dominates it (same or enclosing branch) ends the
search, definitions in other branches of an enclosing if/switch are skipped, and definitions in branches that closed
before the use are included as "may reach". Struct, array, pointer and block-state (localDW) writes are partial and
never end the search. Variables read before they are written in step() come from the previous frame: by default the
slice follows them into their end-of-step definitions (--no-wrap to stop there).

Usage:
  slice.py <ast.json> <source.cpp> <key> [--at LINE] [--stop REGEX] [--no-wrap] [--params params.txt] [--keys]
  key examples: FmgcComputer_Y.out.ap_fd_logic.longitudinal_modes.alt_acq_active
                FmgcComputer_DWork.Memory_PreviousInput_ne    rtb_Compare_ji --at 2547
"""
import argparse
import json
import re
import sys


def load_function(path):
    txt = open(path).read()
    dec = json.JSONDecoder()
    i = 0
    while i < len(txt):
        while i < len(txt) and txt[i].isspace():
            i += 1
        if i >= len(txt):
            break
        obj, i = dec.raw_decode(txt, i)
        body = [c for c in obj.get('inner', []) if c['kind'] == 'CompoundStmt']
        if body:
            return obj, body[0]
    raise SystemExit('no function body in ' + path)


def off(loc):
    if 'offset' in loc:
        return loc['offset']
    for k in ('expansionLoc', 'spellingLoc'):
        if k in loc and 'offset' in loc[k]:
            return loc[k]['offset']
    return None


def strip(e):
    while e['kind'] in ('ImplicitCastExpr', 'ParenExpr', 'CStyleCastExpr', 'CXXStaticCastExpr', 'CXXFunctionalCastExpr',
                        'MaterializeTemporaryExpr', 'ExprWithCleanups', 'CXXBindTemporaryExpr', 'ConstantExpr'):
        if not e.get('inner'):
            break
        e = e['inner'][0]
    return e


def lvalue(e):
    """Returns (key, partial) for an lvalue expression, or (None, False)."""
    e = strip(e)
    k = e['kind']
    if k == 'DeclRefExpr':
        d = e.get('referencedDecl', {})
        if d.get('kind') in ('VarDecl', 'ParmVarDecl'):
            return d['name'], False
        return None, False
    if k == 'MemberExpr':
        base = strip(e['inner'][0])
        if base['kind'] == 'CXXThisExpr':
            return e['name'], False
        bk, bp = lvalue(base)
        if bk is None:
            return None, False
        return bk + '.' + e['name'], bp
    if k == 'ArraySubscriptExpr':
        bk, _ = lvalue(e['inner'][0])
        return bk, True
    if k == 'UnaryOperator' and e.get('opcode') == '*':
        bk, _ = lvalue(e['inner'][0])
        return bk, True
    return None, False


def split_params(fn_type):
    """'void (const base_arinc_429 *, real_T, uint32_T *)' -> list of param type strings."""
    m = re.search(r'\((.*)\)\s*$', fn_type)
    if not m:
        return []
    s, depth, cur, out = m.group(1), 0, '', []
    for ch in s:
        if ch in '(<[':
            depth += 1
        elif ch in ')>]':
            depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


class Node:
    __slots__ = ('id', 'begin', 'end', 'ctrl', 'defs', 'uses', 'kind', 'loop', 'nbranches')

    def __init__(self, nid, begin, end, ctrl, kind, loop):
        self.id, self.begin, self.end, self.ctrl, self.kind, self.loop = nid, begin, end, ctrl, kind, loop
        self.defs = []  # (key, strong)
        self.uses = set()
        self.nbranches = 0  # if/switch: number of branches that together cover every case


class Builder:
    def __init__(self):
        self.nodes = []

    def new(self, stmt, ctrl, kind, loop, begin=None, end=None):
        r = stmt.get('range', {})
        n = Node(len(self.nodes), begin if begin is not None else off(r.get('begin', {})),
                 end if end is not None else off(r.get('end', {})), ctrl, kind, loop)
        self.nodes.append(n)
        return n

    # ---- expressions
    def expr(self, e, n):
        if e is None:
            return
        k = e['kind']
        if k in ('BinaryOperator', 'CompoundAssignOperator') and e.get('opcode', '').endswith('=') and \
                e['opcode'] not in ('==', '!=', '<=', '>='):
            lhs, rhs = e['inner']
            key, partial = lvalue(lhs)
            if key:
                n.defs.append((key, not partial and k == 'BinaryOperator' and not n.loop))
                if partial or k == 'CompoundAssignOperator':
                    n.uses.add(key)
            self.index_uses(lhs, n)
            self.expr(rhs, n)
            return
        if k == 'CXXOperatorCallExpr' and len(e.get('inner', [])) == 3:
            callee = strip(e['inner'][0])
            if callee.get('referencedDecl', {}).get('name') == 'operator=':
                key, partial = lvalue(e['inner'][1])
                if key:
                    n.defs.append((key, not partial and not n.loop))
                    if partial:
                        n.uses.add(key)
                self.expr(e['inner'][2], n)
                return
        if k == 'UnaryOperator' and e.get('opcode') in ('++', '--'):
            key, _ = lvalue(e['inner'][0])
            if key:
                n.defs.append((key, False))
                n.uses.add(key)
            return
        if k in ('CallExpr', 'CXXMemberCallExpr'):
            inner = e.get('inner', [])
            callee = inner[0]
            ptypes = split_params(callee.get('type', {}).get('qualType', ''))
            if k == 'CXXMemberCallExpr':
                # obj.method(args): the object (e.g. LawMDLOBJ1) holds state, read and written
                mobj = strip(callee)
                if mobj['kind'] == 'MemberExpr':
                    key, _ = lvalue(mobj['inner'][0])
                    if key:
                        n.defs.append((key, False))
                        n.uses.add(key)
                ptypes = []
            else:
                self.expr(callee, n)
            for i, a in enumerate(inner[1:]):
                pt = ptypes[i] if i < len(ptypes) else ''
                sa = strip(a)
                if sa['kind'] == 'UnaryOperator' and sa.get('opcode') == '&' and not pt.startswith('const'):
                    key, partial = lvalue(sa['inner'][0])
                    if key:
                        state = 'rtDW' in pt or 'DW' in key.split('.')[-1] or key.split('.')[-1].startswith('sf_') or \
                            '_DWork.' in key and not key.startswith('rtb')
                        is_output_ptr = pt.endswith('*') and not state
                        n.defs.append((key, is_output_ptr and not partial and not n.loop))
                        if not is_output_ptr or partial:
                            n.uses.add(key)
                        self.index_uses(sa['inner'][0], n)
                        continue
                if pt.endswith('*') and not pt.startswith('const') and k == 'CallExpr':
                    # array passed as pointer, written by the callee
                    key, _ = lvalue(sa)
                    if key:
                        n.defs.append((key, False))
                        n.uses.add(key)
                        continue
                self.expr(a, n)
            return
        if k in ('DeclRefExpr', 'MemberExpr', 'ArraySubscriptExpr'):
            key, _ = lvalue(e)
            if key:
                n.uses.add(key)
            if k == 'ArraySubscriptExpr':
                self.expr(e['inner'][1], n)
                if key is None:
                    self.expr(e['inner'][0], n)
            elif key is None:
                for c in e.get('inner', []):
                    self.expr(c, n)
            return
        for c in e.get('inner', []):
            self.expr(c, n)

    def index_uses(self, lhs, n):
        e = strip(lhs)
        while e['kind'] in ('ArraySubscriptExpr', 'MemberExpr'):
            if e['kind'] == 'ArraySubscriptExpr':
                self.expr(e['inner'][1], n)
            e = strip(e['inner'][0])

    # ---- statements
    def stmt(self, s, ctrl, loop=False):
        k = s['kind']
        inner = s.get('inner', [])
        if k == 'CompoundStmt':
            for c in inner:
                self.stmt(c, ctrl, loop)
        elif k == 'IfStmt':
            # inner: [cond, then, else?]; (C++17 init statements are not used by the generator)
            cond = inner[0]
            n = self.new(s, ctrl, 'if', loop, end=off(cond.get('range', {}).get('end', {})))
            self.expr(cond, n)
            self.stmt(inner[1], ctrl + ((n.id, 0),), loop)
            n.nbranches = 2 if len(inner) > 2 else 0
            if len(inner) > 2:
                self.stmt(inner[2], ctrl + ((n.id, 1),), loop)
        elif k in ('ForStmt', 'WhileStmt', 'DoStmt'):
            n = self.new(s, ctrl, 'loop', True, end=None)
            parts = [c for c in inner if c]
            body = parts[-1]
            for c in parts[:-1]:
                if c['kind'] == 'DeclStmt':
                    self.decl(c, n)
                else:
                    self.expr(c, n)
            n.end = off(body.get('range', {}).get('begin', {}))
            # loop header defs/uses: treat as partial (loop-carried)
            n.defs = [(key, False) for key, _ in n.defs]
            self.stmt(body, ctrl + ((n.id, 0),), True)
        elif k == 'SwitchStmt':
            cond = [c for c in inner if c['kind'] != 'CompoundStmt'][0]
            body = [c for c in inner if c['kind'] == 'CompoundStmt'][0]
            n = self.new(s, ctrl, 'switch', loop, end=off(cond.get('range', {}).get('end', {})))
            self.expr(cond, n)
            case = -1
            has_default = False
            for c in body.get('inner', []):
                while c['kind'] in ('CaseStmt', 'DefaultStmt'):
                    case += 1
                    has_default |= c['kind'] == 'DefaultStmt'
                    if c['kind'] == 'CaseStmt':
                        self.expr(c['inner'][0], n)
                    c = c['inner'][-1]
                self.stmt(c, ctrl + ((n.id, case),), loop)
            # fallthrough between labels is merged into one branch index per label group; only trust full coverage
            # when there is a default
            n.nbranches = case + 1 if has_default else 0
        elif k == 'DeclStmt':
            n = self.new(s, ctrl, 'decl', loop)
            self.decl(s, n)
            if not n.defs and not n.uses:
                self.nodes.pop()
        elif k in ('BreakStmt', 'NullStmt', 'ContinueStmt'):
            pass
        elif k == 'ReturnStmt':
            n = self.new(s, ctrl, 'return', loop)
            for c in inner:
                self.expr(c, n)
        else:
            n = self.new(s, ctrl, 'expr', loop)
            self.expr(s, n)

    def decl(self, s, n):
        for d in s.get('inner', []):
            if d['kind'] == 'VarDecl':
                if d.get('inner'):
                    n.defs.append((d['name'], not n.loop))
                    for c in d['inner']:
                        self.expr(c, n)


def relation(use, dkey):
    """How a definition of dkey relates to a use of `use`: 'full' (covers it), 'part' (writes part of it) or None."""
    if dkey == use or use.startswith(dkey + '.'):
        return 'full'
    if dkey.startswith(use + '.'):
        return 'part'
    return None


def dominates(def_ctrl, use_ctrl):
    return len(def_ctrl) <= len(use_ctrl) and use_ctrl[:len(def_ctrl)] == def_ctrl


def exclusive(a, b):
    """Two control paths in different branches of the same if/switch."""
    for x, y in zip(a, b):
        if x != y:
            return x[0] == y[0]
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ast')
    ap.add_argument('source')
    ap.add_argument('keys', nargs='+')
    ap.add_argument('--at', type=int, help='slice from this source line (default: end of step)')
    ap.add_argument('--stop', action='append', default=[], help='regex of keys not to follow (shown as leaves)')
    ap.add_argument('--no-wrap', action='store_true', help='do not follow values from the previous frame')
    ap.add_argument('--params', action='append', default=[], help='params dump to annotate values from')
    ap.add_argument('--keys', dest='list_keys', action='store_true', help='also list the leaf inputs/params/state')
    args = ap.parse_args()

    src = open(args.source).read()
    line_starts = [0]
    for i, ch in enumerate(src):
        if ch == '\n':
            line_starts.append(i + 1)

    def line_of(o):
        import bisect
        return bisect.bisect_right(line_starts, o)

    _, body = load_function(args.ast)
    b = Builder()
    b.stmt(body, ())
    nodes = b.nodes
    defs_by_key = {}
    for n in nodes:
        for key, strong in n.defs:
            defs_by_key.setdefault(key, []).append(n.id)

    def defs_related(key):
        out = []
        for dkey, ids in defs_by_key.items():
            r = relation(key, dkey)
            if r:
                out.extend((i, dkey, r) for i in ids)
        out.sort(reverse=True)
        return out

    stops = [re.compile(s) for s in args.stop]
    leaves = set()
    wrapped = set()
    included = set()

    def reaching(key, at_index, use_ctrl):
        """Definitions of key that can reach a use located just before node index at_index with control path use_ctrl."""
        found = []
        covered = {}  # if/switch node id -> branches whose every path defines the key

        def cover(ctrl):
            # a full definition at control path ctrl: does it, possibly with sibling branches, dominate the use?
            while True:
                if dominates(ctrl, use_ctrl):
                    return True
                cid, branch = ctrl[-1]
                c = nodes[cid]
                covered.setdefault(cid, set()).add(branch)
                if c.nbranches and len(covered[cid]) == c.nbranches:
                    ctrl = c.ctrl
                    continue
                return False

        for i, dkey, r in defs_related(key):
            if i >= at_index:
                continue
            n = nodes[i]
            if exclusive(n.ctrl, use_ctrl):
                continue
            found.append(i)
            strong = any(k == dkey and s for k, s in n.defs) and r == 'full'
            if strong and cover(n.ctrl):
                return found, True
        return found, False

    work = []
    end = len(nodes)
    if args.at:
        end = next((n.id for n in nodes if n.begin is not None and line_of(n.begin) > args.at), len(nodes))
    for key in args.keys:
        work.append((key, end, ()))
    seen = set()
    while work:
        key, at, ctrl = work.pop()
        if (key, at, ctrl) in seen:
            continue
        seen.add((key, at, ctrl))
        if key not in args.keys and any(s.search(key) for s in stops):
            leaves.add(key)
            continue
        if key.startswith(('FmgcComputer_P.', 'FmgcOuterLoops_rtP.', 'FmgcComputer_U.')) or key.startswith('rtu_'):
            leaves.add(key)
            continue
        found, killed = reaching(key, at, ctrl)
        for i in found:
            if i not in included:
                included.add(i)
                n = nodes[i]
                for u in n.uses:
                    work.append((u, i, n.ctrl))
                for cid, _ in n.ctrl:
                    if cid not in included:
                        included.add(cid)
                        c = nodes[cid]
                        for u in c.uses:
                            work.append((u, cid, c.ctrl))
        if not killed:
            if args.no_wrap or key in wrapped or not defs_related(key):
                leaves.add(key + ('  (previous frame)' if defs_related(key) else ''))
            else:
                wrapped.add(key)
                leaves.add(key + '  (previous frame, followed)')
                work.append((key, len(nodes), ()))

    params = {}
    for p in args.params:
        for line in open(p):
            if ' = ' in line:
                name, val = line.rstrip('\n').split(' = ', 1)
                params[name.split('[')[0]] = val
    lines = set()
    for i in included:
        n = nodes[i]
        if n.begin is None:
            continue
        a, z = line_of(n.begin), line_of(n.end if n.end is not None else n.begin)
        lines.update(range(a, z + 1))
    src_lines = src.split('\n')
    prev = None
    for ln in sorted(lines):
        if prev is not None and ln != prev + 1:
            print('      ...')
        text = src_lines[ln - 1]
        notes = []
        for m in re.finditer(r'(?:FmgcComputer_P|FmgcOuterLoops_rtP)\.(\w+)', text):
            if m.group(1) in params and not m.group(1).startswith('Logic_table'):
                notes.append(f'{m.group(1)}={params[m.group(1)]}')
        print(f'{ln:6d}  {text}' + (('    // ' + ', '.join(notes)) if notes else ''))
        prev = ln
    print(f'\n// {len(included)} statements, {len(lines)} lines')
    if args.list_keys:
        print('// leaves:')
        for k in sorted(leaves):
            print('//   ' + k + (f' = {params[k.split(".")[-1]]}' if k.split('.')[-1] in params else ''))


if __name__ == '__main__':
    sys.exit(main())
