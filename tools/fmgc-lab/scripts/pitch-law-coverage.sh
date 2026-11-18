#!/bin/bash
# Source-based coverage (llvm-cov) of the generated pitch laws (PitchNormalLaw.cpp, PitchAlternateLaw.cpp,
# PitchDirectLaw.cpp) and the readable ones, driven by the synthetic differential check. Needs
# `PITCH_ONLY=1 scripts/build-fuzz.sh` (build/efcs-pitch-law-cov-check). The checker runs on the host; llvm-profdata /
# llvm-cov run in the dev container. Writes build/coverage/pitch/: report.txt (per file), <File>.cov.txt (llvm-cov show
# with branch counts) and uncovered.txt (every branch with a side never taken: file:line, true / false counts).
# Usage: [SEEDS=8] [STEPS=1000000] [JOBS=4] scripts/pitch-law-coverage.sh
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
bin=$here/build/efcs-pitch-law-cov-check
[ -x "$bin" ] || { echo "error: $bin missing (PITCH_ONLY=1 scripts/build-fuzz.sh)" >&2; exit 2; }
seeds=${SEEDS:-8}
steps=${STEPS:-1000000}
jobs=${JOBS:-4}
covdir=$here/build/coverage/pitch
rm -rf "$covdir" && mkdir -p "$covdir/raw" "$covdir/runs"
export bin covdir steps
# shellcheck disable=SC2016
seq 1 "$seeds" | xargs -P "$jobs" -I{} bash -c 'LLVM_PROFILE_FILE="$covdir/raw/cov-{}.profraw" "$bin" --seed {} \
  --steps "$steps" > "$covdir/runs/seed-{}.txt" 2>&1 || echo "seed {}: MISMATCH"'
grep -h '^OK\|^MISMATCH' "$covdir"/runs/*.txt | sort | uniq -c
rel=${covdir#"$repo"/}
(cd "$repo" && ./scripts/dev-env/run.sh bash -c "set -e; cd $rel && \
  llvm-profdata-19 merge -sparse raw/*.profraw -o merged.profdata && \
  llvm-cov-19 report /external/tools/fmgc-lab/build/efcs-pitch-law-cov-check -instr-profile=merged.profdata \
    -show-branch-summary > report.txt && \
  for f in PitchNormalLaw PitchAlternateLaw PitchDirectLaw; do \
    llvm-cov-19 show /external/tools/fmgc-lab/build/efcs-pitch-law-cov-check -instr-profile=merged.profdata \
      -show-branches=count /external/tools/fmgc-lab/build/models/stock/\$f.cpp > \$f.cov.txt; done && \
  llvm-cov-19 export /external/tools/fmgc-lab/build/efcs-pitch-law-cov-check -instr-profile=merged.profdata \
    -format=text > export.json && chown -R $(id -u):$(id -g) .")
python3 - "$covdir/export.json" > "$covdir/uncovered.txt" <<'EOF'
import json, sys
data = json.load(open(sys.argv[1]))
for f in data['data'][0]['files']:
    name = f['filename'].rsplit('/', 1)[-1]
    if name not in ('PitchNormalLaw.cpp', 'PitchAlternateLaw.cpp', 'PitchDirectLaw.cpp'):
        continue
    seen = set()
    for b in f['branches']:
        line, col, _, _, t, fl = b[:6]
        if (t == 0 or fl == 0) and (line, col) not in seen:
            seen.add((line, col))
            print(f'{name}:{line}:{col} true {t} false {fl}')
EOF
grep -E 'Filename|Pitch.*Law.cpp|pitch_.*law|TOTAL' "$covdir/report.txt"
echo "branch sides never taken: $(wc -l < "$covdir/uncovered.txt") (build/coverage/pitch/uncovered.txt)"
