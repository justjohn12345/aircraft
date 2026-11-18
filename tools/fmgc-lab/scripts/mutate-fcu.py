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
UNIT=fadec: the same for the readable FADEC (readable/fadec/, fadec-check-stock, build/mutants-fadec; also mutates the
bit numbers of the ECU bus packing): scripts/mutate-fadec.sh
UNIT=fac: the readable FAC (readable/fac/, fac-check-stock, build/mutants-fac): also the integer constants (ARINC bit
numbers, + 1); not the table rows (lines with 4 or more numbers) or the buffet table header: scripts/mutate-fac.sh
BUILD_JOBS: parallel compiles per mutant build (default 4).
"""
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
UNIT = os.environ.get('UNIT', 'fcu')
SRC = os.path.join(HERE, 'readable', UNIT)
OUT = os.path.join(HERE, 'build', 'mutants' if UNIT == 'fcu' else f'mutants-{UNIT}')
TARGET = f'{UNIT}-check-stock'
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
PACKED_BIT = re.compile(r'(\.bit\()(\d+)(,)')  # FADEC ECU bus packing
INT_CONST = re.compile(r'(?<![\w.])(\d+)(?![\w.])')  # FAC: integer constants of `constexpr int` lines
BUILD_JOBS = os.environ.get('BUILD_JOBS', '4')


def mutants():
    out = []
    for name in sorted(os.listdir(SRC)):
        if not (name.endswith('.cpp') or name.endswith('.h')) or name == 'vls_buffet_table.h':
            continue
        path = os.path.join(SRC, name)
        for i, line in enumerate(open(path).read().split('\n')):
            code = line.split('//')[0]
            s = code.strip()
            if not s or s.startswith('#') or s.startswith('*') or s.startswith('/*'):
                continue
            if name.endswith('.h') and 'constexpr' not in code and name not in ('fadec_blocks.h', 'fac_blocks.h'):
                continue  # headers: only the named constants (and the FADEC's / FAC's block library)
            for pat, rep, desc in OPS:
                for m in re.finditer(pat, code):
                    new = line[:m.start()] + rep + line[m.end():]
                    out.append({'file': name, 'line': i, 'old': line, 'new': new, 'desc': desc})
            nums = list(NUM.finditer(code))
            if UNIT == 'fac' and len(nums) >= 4:
                nums = []  # table rows: covered by the corpus replay, too many mutants
            for m in nums:
                v = float(m.group(1))
                nv = v * 1.5 if v != 0 else 1.0
                new = line[:m.start()] + repr(nv) + m.group(2) + line[m.end():]
                out.append({'file': name, 'line': i, 'old': line, 'new': new, 'desc': f'{m.group(1)}->{nv}'})
            bits = list(BIT.finditer(code)) + (list(PACKED_BIT.finditer(code)) if UNIT == 'fadec' else [])
            for m in bits:
                nb = int(m.group(2)) + 1
                new = line[:m.start(2)] + str(nb) + line[m.end(2):]
                out.append({'file': name, 'line': i, 'old': line, 'new': new, 'desc': f'bit {m.group(2)}->{nb}'})
            if UNIT == 'fac' and 'constexpr int' in code:
                for m in INT_CONST.finditer(code):
                    nb = int(m.group(1)) + 1
                    new = line[:m.start(1)] + str(nb) + line[m.end(1):]
                    out.append({'file': name, 'line': i, 'old': line, 'new': new, 'desc': f'{m.group(1)}->{nb}'})
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
                r = subprocess.run(['cmake', '--build', os.path.join(HERE, 'build', 'cmake'), '-j', BUILD_JOBS, '--target',
                                    TARGET], capture_output=True, text=True)
                binary = os.path.join(OUT, f'm{k:04d}')
                if r.returncode == 0:
                    subprocess.run(['cp', os.path.join(HERE, 'build', 'cmake', TARGET), binary], check=True)
                    log.write(f'{k} built\n')
                else:
                    log.write(f'{k} build error\n')
            finally:
                open(path, 'w').write(original)
            log.flush()
    finally:
        subprocess.run(['cmake', '--build', os.path.join(HERE, 'build', 'cmake'), '-j', BUILD_JOBS, '--target', TARGET],
                       capture_output=True)
        subprocess.run(['cp', os.path.join(HERE, 'build', 'cmake', TARGET), os.path.join(HERE, 'build')])


CONFIGS = [
    ['--synthetic'], ['--synthetic', '--reconstruct'], ['--synthetic', '--script', '1'],
    ['--synthetic', '--panel', '1'], ['--synthetic', '--bus', '1'], ['--synthetic', '--priority', '1'],
    ['--synthetic', '--power', '1', '--panel', '2'], ['--synthetic', '--sim', '1', '--script', '2'],
    ['--synthetic', '--fmgc2', 'swapped', '--reconstruct', '--script', '3', '--panel', '3', '--priority', '3',
     '--bus', '3', '--sim', '3', '--power', '3'],
]
if UNIT == 'fadec':
    CONFIGS = [
        ['--synthetic'], ['--synthetic', '--script', '1'], ['--synthetic', '--lever', '1'],
        ['--synthetic', '--athr', '1'], ['--synthetic', '--sim', '1'],
        ['--synthetic', '--script', '2', '--lever', '2', '--athr', '2', '--sim', '2'],
    ]

if UNIT == 'fac':
    CONFIGS = [
        [], ['--script', '1'], ['--failures', '1'], ['--discretes', '1'], ['--power', '1'], ['--sim', '1'],
        ['--script', '2', '--failures', '2', '--discretes', '2', '--power', '2', '--sim', '2'],
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
    if UNIT == 'fac':
        # FAC runs take seconds each: three short recordings first (the scripted tour brings the flight phases), then
        # four more (two long flights) for the survivors, not the whole corpus
        def pick(names):
            return [f for f in files if os.path.basename(f)[:19] in names] or files[:len(names)]
        quick = pick({'2026-11-14-13-57-13', '2026-11-16-01-07-03', '2026-11-11-00-25-42'})
        files = quick + pick({'2026-11-08-01-16-09', '2026-11-14-14-07-54', '2026-11-16-15-22-40',
                              '2026-11-09-23-30-14'})

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
