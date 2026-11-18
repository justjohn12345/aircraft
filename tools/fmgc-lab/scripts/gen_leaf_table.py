#!/usr/bin/env python3
# Copyright (c) 2026 FlyByWire Simulations
#
# SPDX-License-Identifier: GPL-3.0
"""
Generates FmgcOutputLeaves.gen.h for the FMGC shadow checker (fbw_a320/src/fmgc/FmgcShadow.cpp): a constexpr table of
every scalar leaf of fmgc_outputs (full name, byte offset, size, kind), in declaration order, so the sim can compare the
readable and generated outputs field by field with no heap and no per-frame string work. The struct layout is also
computed here (natural alignment, which both x86-64 and wasm32 use for these types) and checked by a static_assert, so
a changed FmgcComputer_types.h fails the build until the table is regenerated.

The output is committed (see tools/fmgc-lab/docs/shadow.md, "Build integration"). Regenerate after the Simulink model
is regenerated:

  python3 tools/fmgc-lab/scripts/gen_leaf_table.py fbw-a32nx/src/wasm/fbw_a320/src/model/FmgcComputer_types.h \
      fbw-a32nx/src/wasm/fbw_a320/src/fmgc/FmgcOutputLeaves.gen.h

The lab build (CMake target fmgc-shadow-leaves-check) runs this with --check and fails if the committed copy is stale.

The FCU and FADEC shadow checkers (fbw_a320/src/shadow/) use the same generator with another root struct and namespace;
their tables use the shared leaf type of shadow/ShadowChecker.h:

  python3 tools/fmgc-lab/scripts/gen_leaf_table.py fbw-a32nx/src/wasm/fbw_a320/src/model/FcuComputer_types.h \
      fbw-a32nx/src/wasm/fbw_a320/src/shadow/FcuOutputLeaves.gen.h --root fcu_outputs --namespace fcu_shadow
  python3 tools/fmgc-lab/scripts/gen_leaf_table.py fbw-a32nx/src/wasm/fbw_a320/src/model/FadecComputer_types.h \
      fbw-a32nx/src/wasm/fbw_a320/src/shadow/FadecOutputLeaves.gen.h --root athr_out --namespace fadec_shadow

Usage: gen_leaf_table.py <Model_types.h> <out.h> [--root STRUCT --namespace NS] [--check]
  default root fmgc_outputs, namespace fmgc_shadow (FmgcOutputLeaves.gen.h, with its own Leaf type);
  --check: fail if <out.h> differs, write nothing
"""
import os
import re
import sys

sys.dont_write_bytecode = True  # no __pycache__ in the source tree
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_fields import parse  # noqa: E402

# type -> (size, alignment, kind)
SCALARS = {
    'boolean_T': (1, 1, 'Bool'),
    'real_T': (8, 8, 'Float'),
    'real32_T': (4, 4, 'Float'),
    'uint32_T': (4, 4, 'Int'),
    'int32_T': (4, 4, 'Int'),
    'int8_T': (1, 1, 'Int'),
    'uint8_T': (1, 1, 'Int'),
}

ROOT = 'fmgc_outputs'
NAMESPACE = 'fmgc_shadow'


def enum_types(text):
    return {m.group(1): m.group(2) for m in re.finditer(r'enum class (\w+)\s*:\s*(\w+)\s*{', text)}


def layout(structs, enums):
    """Returns {struct: (size, align)} with natural alignment."""
    memo = {}

    def of(type_):
        if type_ in SCALARS:
            return SCALARS[type_][:2]
        if type_ in enums:
            return SCALARS[enums[type_]][:2]
        if type_ in memo:
            return memo[type_]
        offset, align = 0, 1
        for member_type, _, count in structs[type_]:
            if count is not None:
                raise SystemExit(f'arrays not supported: {type_}')
            size, a = of(member_type)
            offset = (offset + a - 1) // a * a + size
            align = max(align, a)
        memo[type_] = ((offset + align - 1) // align * align, align)
        return memo[type_]

    return of


def leaves(structs, enums, type_, prefix):
    for member_type, name, _ in structs[type_]:
        if member_type in structs:
            yield from leaves(structs, enums, member_type, f'{prefix}{name}.')
        elif member_type in SCALARS:
            yield f'{prefix}{name}', member_type, SCALARS[member_type][2]
        elif member_type in enums:
            yield f'{prefix}{name}', member_type, 'Enum'
        else:
            raise SystemExit(f'unknown type {member_type} in {type_}.{name}')


def option(name, default):
    args = sys.argv[3:]
    return args[args.index(name) + 1] if name in args else default


def main():
    root = option('--root', ROOT)
    namespace = option('--namespace', NAMESPACE)
    shared_leaf = root != ROOT  # the FCU/FADEC tables use shadow::Leaf (ShadowChecker.h); FMGC keeps its own type
    types_name = os.path.basename(sys.argv[1])
    out_name = os.path.basename(sys.argv[2])
    out_rel = ('fbw-a32nx/src/wasm/fbw_a320/src/' + os.path.basename(os.path.dirname(os.path.abspath(sys.argv[2]))) + '/' +
               out_name)
    extra = f' --root {root} --namespace {namespace}' if shared_leaf else ''
    text = open(sys.argv[1]).read()
    structs, _ = parse(text)
    enums = enum_types(text)
    size, _ = layout(structs, enums)(root)
    rows = list(leaves(structs, enums, root, ''))
    if shared_leaf:
        leaf_type = [
            f'#include "{types_name}"',
            '#include "ShadowChecker.h"',
            '',
            f'namespace {namespace} {{',
            '',
            'using shadow::Leaf;',
            'using shadow::LeafKind;',
        ]
    else:
        leaf_type = [
            f'#include "{types_name}"',
            '',
            f'namespace {namespace} {{',
            '',
            'enum class LeafKind : uint8_t { Bool, Float, Int, Enum };',
            '',
            'struct Leaf {',
            '  const char* name;',
            '  uint32_t offset;',
            '  uint8_t size;',
            '  LeafKind kind;',
            '};',
        ]
    out = [
        '// Copyright (c) 2026 FlyByWire Simulations',
        '//',
        '// SPDX-License-Identifier: GPL-3.0',
        '',
        f'// Generated by tools/fmgc-lab/scripts/gen_leaf_table.py from model/{types_name}. Do not edit.',
        '// Regenerate after the Simulink model changes (from the repository root):',
        f'//   python3 tools/fmgc-lab/scripts/gen_leaf_table.py fbw-a32nx/src/wasm/fbw_a320/src/model/{types_name} \\',
        f'//       {out_rel}{extra}',
        f'// Every scalar leaf of {root} in declaration order: name, byte offset, size, kind.',
        '',
        '#pragma once',
        '',
        '#include <cstddef>',
        '#include <cstdint>',
        '',
    ] + leaf_type + [
        '',
        f'static_assert(sizeof({root}) == {size}, "{types_name} changed: regenerate {out_name}");',
        '',
        f'inline constexpr size_t kOutputLeafCount = {len(rows)};',
        '',
        '// clang-format off',
        'inline constexpr Leaf kOutputLeaves[kOutputLeafCount] = {',
    ]
    for name, type_, kind in rows:
        out.append(f'    {{"{name}", offsetof({root}, {name}), sizeof({type_}), LeafKind::{kind}}},')
    out += [
        '};',
        '// clang-format on',
        '',
        f'}}  // namespace {namespace}',
        '',
    ]
    code = '\n'.join(out)
    if '--check' in sys.argv[3:]:
        current = open(sys.argv[2]).read() if os.path.exists(sys.argv[2]) else ''
        if current != code:
            raise SystemExit(f'{sys.argv[2]} is stale: regenerate it with\n  python3 tools/fmgc-lab/scripts/gen_leaf_table.py '
                             f'{sys.argv[1]} {sys.argv[2]}{extra}')
        return
    with open(sys.argv[2], 'w') as f:
        f.write(code)


if __name__ == '__main__':
    main()
