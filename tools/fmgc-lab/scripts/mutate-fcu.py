#!/usr/bin/env python3
# Copyright (c) 2026 FlyByWire Simulations
#
# SPDX-License-Identifier: GPL-3.0
"""
Mutation test of the FCU differential check: does fcu-check notice when the readable FCU (readable/fcu/) is changed?

  list              enumerate the mutants (one source change each: && <-> ||, ! removed, == <-> !=, < <-> <=,
                    > <-> >=, true <-> false, += -> -=, a numeric literal changed, an ARINC bit number + 1) into
                    build/mutants/list.json
  build             (inside the dev container: scripts/mutate-fcu.sh) for each mutant: apply it, rebuild
                    fcu-check-stock, keep the binary as build/mutants/mNNNN (or note a build error), restore the source;
                    finally rebuild the unmutated binary
  (MUTANTS=139,169,...: build/test only those mutants, e.g. the survivors after adding a scenario)
  test [JOBS]       run every mutant binary on the quick plan (3 recordings x 9 configs), then the survivors on every
                    recording; a mutant is caught when a run exits 1. Writes build/mutants/results.txt
Usage: scripts/mutate-fcu.sh (list + build in the container + test on the host, JOBS=4)
"""
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(HERE, 'readable', 'fcu')
OUT = os.path.join(HERE, 'build', 'mutants')
CORPUS = os.environ.get('CORPUS', os.path.expanduser('~/fdr-corpus/raw'))

OPS = [
    (r'&&', '||', 'and->or'),
    (r'\|\|', '&&', 'or->and'),
    (r'!(?=[A-Za-z_(])', '', 'drop not'),
    (r'==', '!=', 'eq->ne'),
    (r'!=', '==', 'ne->eq'),
    (r'(?<![<\-])<(?![<=])(?=\s)', '<=', 'lt->le'),
    (r'<=', '<', 'le->lt'),
    (r'(?<![>\-])>(?![>=])(?=\s)', '>=', 'gt->ge'),
    (r'>=', '>', 'ge->gt'),
    (r'\btrue\b', 'false', 'true->false'),
    (r'\bfalse\b', 'true', 'false->true'),
    (r'\+=', '-=', 'add->sub'),
]
NUM = re.compile(r'(?<![\w.])(\d+\.\d+)(F?)\b')
BIT = re.compile(r'(bit\((?:[\w.]+), )(\d+)(\))')


def mutants():
    out = []
    for name in sorted(os.listdir(SRC)):
        if not (name.endswith('.cpp') or name.endswith('.h')):
            continue
        path = os.path.join(SRC, name)
        for i, line in enumerate(open(path).read().split('\n')):
            code = line.split('//')[0]
            s = code.strip()
            if not s or s.startswith('#') or s.startswith('*') or s.startswith('/*'):
                continue
            if name.endswith('.h') and 'constexpr' not in code:
                continue  # headers: only the named constants
            for pat, rep, desc in OPS:
                for m in re.finditer(pat, code):
                    new = line[:m.start()] + rep + line[m.end():]
                    out.append({'file': name, 'line': i, 'old': line, 'new': new, 'desc': desc})
            for m in NUM.finditer(code):
                v = float(m.group(1))
                nv = v * 1.5 if v != 0 else 1.0
                new = line[:m.start()] + repr(nv) + m.group(2) + line[m.end():]
                out.append({'file': name, 'line': i, 'old': line, 'new': new, 'desc': f'{m.group(1)}->{nv}'})
            for m in BIT.finditer(code):
                nb = int(m.group(2)) + 1
                new = line[:m.start(2)] + str(nb) + line[m.end(2):]
                out.append({'file': name, 'line': i, 'old': line, 'new': new, 'desc': f'bit {m.group(2)}->{nb}'})
    return out


def cmd_list():
    os.makedirs(OUT, exist_ok=True)
    ms = mutants()
    json.dump(ms, open(os.path.join(OUT, 'list.json'), 'w'), indent=0)
    print(f'{len(ms)} mutants')


def selected(n):
    """Mutant indices from MUTANTS (comma-separated, e.g. 139,169), else all."""
    ids = os.environ.get('MUTANTS', '')
    return [int(x) for x in ids.split(',') if x] if ids else list(range(n))


def cmd_build():
    ms = json.load(open(os.path.join(OUT, 'list.json')))
    log = open(os.path.join(OUT, 'build.log'), 'w')
    try:
        for k in selected(len(ms)):
            m = ms[k]
            path = os.path.join(SRC, m['file'])
            original = open(path).read()
            lines = original.split('\n')
            lines[m['line']] = m['new']
            try:
                open(path, 'w').write('\n'.join(lines))
                r = subprocess.run(['cmake', '--build', os.path.join(HERE, 'build', 'cmake'), '-j', '4', '--target',
                                    'fcu-check-stock'], capture_output=True, text=True)
                binary = os.path.join(OUT, f'm{k:04d}')
                if r.returncode == 0:
                    subprocess.run(['cp', os.path.join(HERE, 'build', 'cmake', 'fcu-check-stock'), binary], check=True)
                    log.write(f'{k} built\n')
                else:
                    log.write(f'{k} build error\n')
            finally:
                open(path, 'w').write(original)
            log.flush()
    finally:
        subprocess.run(['cmake', '--build', os.path.join(HERE, 'build', 'cmake'), '-j', '4', '--target',
                        'fcu-check-stock'], capture_output=True)
        subprocess.run(['cp', os.path.join(HERE, 'build', 'cmake', 'fcu-check-stock'), os.path.join(HERE, 'build')])


CONFIGS = [
    ['--synthetic'], ['--synthetic', '--reconstruct'], ['--synthetic', '--script', '1'],
    ['--synthetic', '--panel', '1'], ['--synthetic', '--bus', '1'], ['--synthetic', '--priority', '1'],
    ['--synthetic', '--power', '1', '--panel', '2'], ['--synthetic', '--sim', '1', '--script', '2'],
    ['--synthetic', '--fmgc2', 'swapped', '--reconstruct', '--script', '3', '--panel', '3', '--priority', '3',
     '--bus', '3', '--sim', '3', '--power', '3'],
]


def caught(binary, files):
    for f in files:
        for c in CONFIGS:
            r = subprocess.run([binary] + c + [f], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if r.returncode == 1:
                return f'{os.path.basename(f)} {" ".join(c)}'
    return None


def cmd_test(jobs):
    ms = json.load(open(os.path.join(OUT, 'list.json')))
    files = sorted(os.path.join(CORPUS, f) for f in os.listdir(CORPUS) if f.endswith('.fdr'))
    quick = [files[i] for i in (len(files) - 4, len(files) // 2, 3) if 0 <= i < len(files)]

    def one(k):
        b = os.path.join(OUT, f'm{k:04d}')
        if not os.path.exists(b):
            return k, 'build error', None
        how = caught(b, quick) or caught(b, [f for f in files if f not in quick])
        return k, 'caught' if how else 'SURVIVED', how

    with ThreadPoolExecutor(jobs) as ex:
        results = sorted(ex.map(one, selected(len(ms))))
    counts = {}
    with open(os.path.join(OUT, 'results.txt'), 'w') as out:
        for k, status, how in results:
            counts[status] = counts.get(status, 0) + 1
            m = ms[k]
            out.write(f'm{k:04d} {status:11s} {m["file"]}:{m["line"] + 1} {m["desc"]}  [{how or ""}]\n')
            if status == 'SURVIVED':
                out.write(f'      - {m["old"].strip()}\n      + {m["new"].strip()}\n')
    print(' '.join(f'{v} {k}' for k, v in sorted(counts.items())), f'(of {len(ms)}); details in {OUT}/results.txt')


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'list':
        cmd_list()
    elif cmd == 'build':
        cmd_build()
    elif cmd == 'test':
        cmd_test(int(sys.argv[2]) if len(sys.argv) > 2 else 4)
    else:
        print(__doc__)
        sys.exit(2)
