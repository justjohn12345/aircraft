#!/bin/bash
# Runs the ELAC or SEC differential check (readable vs generated, every computer, free-running lockstep, every output
# field and hidden memory bit-exact; build/<unit>-check-<variant>, src/check_<unit>.cpp) over every recording, with
# these configs (inputs rebuilt from the record, src/efcs_drive.h):
#   plain; and for each seed 1..SEEDS: every injector alone (--script --failures --discretes --power --sim), two mixes
#   and everything at once; one --warm run (the readable initialize() is a complete reset).
# Then <unit>-replay on every recording (the generated computers against the recorded outputs), summarised.
# UNIT: elac (default) or sec. JOBS: parallel runs (default 2). The generated stock model is the oracle.
# COVERAGE=1: run build/<unit>-cov-check instead (scripts/build-fuzz.sh with EFCS_ONLY=1) and write the source-based
# coverage of the generated ElacComputer / SecComputer and law .cpp files and of the readable sources (llvm-cov, in
# the dev container) to build/coverage/<unit>/.
# Usage: [UNIT=elac] [JOBS=2] [SEEDS=2] [COVERAGE=1] scripts/check-efcs-corpus.sh [corpus dir, default ~/fdr-corpus/raw]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
unit=${UNIT:-elac}
corpus=${1:-${CORPUS:-$HOME/fdr-corpus/raw}}
seeds=${SEEDS:-2}
jobs=${JOBS:-2}
if [ -n "${COVERAGE:-}" ]; then
  bin=$here/build/$unit-cov-check
else
  bin=$here/build/$unit-check-stock
fi
replay=$here/build/$unit-replay-stock
if [ ! -x "$bin" ]; then
  echo "error: $bin missing (scripts/build.sh, or scripts/build-fuzz.sh with EFCS_ONLY=1 for COVERAGE=1)" >&2
  exit 2
fi
if [ -n "$(find "$here/readable/$unit" "$here/readable/efcs" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=${OUT:-$here/build/check-$unit}
rm -rf "$out" && mkdir -p "$out"
covdir=$here/build/coverage/$unit
if [ -n "${COVERAGE:-}" ]; then
  rm -rf "$covdir" && mkdir -p "$covdir/raw"
fi
run() {
  local name=$1
  shift
  while [ "$(jobs -r | wc -l)" -ge "$jobs" ]; do wait -n; done
  LLVM_PROFILE_FILE="$covdir/raw/cov-%4m.profraw" "$bin" "$@" > "$out/$name.txt" 2>&1 &
}
files=("$corpus"/*.fdr)
for path in "${files[@]}"; do
  f=$(basename "$path" .fdr)
  run "$f.plain" "$path"
  for ((s = 1; s <= seeds; s++)); do
    for inj in script failures discretes power sim; do
      run "$f.$inj.$s" --$inj $((s * 100 + 1)) "$path"
    done
    run "$f.mix1.$s" --script $((s * 100 + 2)) --failures $((s * 100 + 2)) "$path"
    run "$f.mix2.$s" --discretes $((s * 100 + 3)) --power $((s * 100 + 3)) --sim $((s * 100 + 3)) "$path"
    run "$f.all.$s" --script $((s * 100 + 4)) --failures $((s * 100 + 4)) --discretes $((s * 100 + 4)) \
      --power $((s * 100 + 4)) --sim $((s * 100 + 4)) "$path"
  done
done
run "warm" --script 5 --warm "${files[0]}" "${files[${#files[@]} - 1]}"
wait

awk '/^field /{on=1; next} /^cov /{on=0} on && NF >= 3 && $2 ~ /^[0-9]+$/ {n[$1]+=$2; r[$1]++}
     END {for (k in n) printf "%-64s %10d mismatching steps in %d runs\n", k, n[k], r[k]}' "$out"/*.txt | sort
grep -h '^hidden\.' "$out"/*.txt | awk '{n[$1]+=$2; r[$1]++} END {for (k in n) printf "%-64s %10d steps in %d runs\n", k, n[k], r[k]}' | sort
grep -h '^FIRST MISMATCH' -A 4 "$out"/*.txt | head -40
grep -h '^cov ' "$out"/*.txt | awk '{c[$2]+=$4; z[$2]+=$3} END {for (k in c) if (c[k] == 0) printf "never changed: %s (nonzero on %d steps)\n", k, z[k]}' | sort

total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'fields compared' "$out"/*.txt | wc -l)
fail=$(grep -L ' 0 with mismatches, 0 mismatching steps; [0-9]* hidden memories, 0 diverged; 0 init' "$out"/*.txt | wc -l)
steps=$(grep -h '^steps ' "$out"/*.txt | awk '{s+=$2} END {printf "%d steps", s}')
restarts=$(grep -h '^steps ' "$out"/*.txt | awk '{for (i = 1; i <= NF; i++) if ($i == "restarts") {s += $(i + 1); t += $(i + 3)}} END {print s + t}')
timing=$(grep -h '^time ' "$out"/*.txt |
  awk '{g+=$3; r+=$6; k++} END {printf "mean step time: generated %.3f us, readable %.3f us", g/k, r/k}')
echo "$total runs, $steps, $restarts restarts; $timing"
echo "$fail with mismatches or no report, $bad without a report (reports in $out)"
status=0
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ] || status=1

# replay of every recording: the generated computers against the recorded outputs
if [ -x "$replay" ] && [ -z "${NO_REPLAY:-}" ]; then
  mkdir -p "$out/replay"
  for path in "${files[@]}"; do
    while [ "$(jobs -r | wc -l)" -ge "$jobs" ]; do wait -n; done
    "$replay" --list 200 "$path" > "$out/replay/$(basename "$path" .fdr).txt" 2>&1 &
  done
  wait
  awk '
    /^known inputs:/ {kc += $6 + $8; ke += $10}
    /^approximate inputs:/ {ac += $6 + $8; ae += $10}
    /^(elac|sec)[0-9]\./ {
      name = $1; sub(/^(elac|sec)[0-9]\./, "", name); m[name] += $2; if ($5 + 0 > d[name]) d[name] = $5 + 0
    }
    END {
      printf "replay: %d recordings; known inputs %d computer frames, %d bit-exact (%.2f%%); approximate inputs %d, %d bit-exact\n",
        ARGC - 1, kc, ke, kc ? 100.0 * ke / kc : 0, ac, ae
      for (k in m) printf "replay known-input mismatches: %-48s %9d frames, max |diff| %g\n", k, m[k], d[k]
    }' "$out"/replay/*.txt
fi

if [ -n "${COVERAGE:-}" ]; then
  rel=${covdir#"$repo"/}
  model=$([ "$unit" = sec ] && echo SecComputer || echo ElacComputer)
  (cd "$repo" && ./scripts/dev-env/run.sh bash -c "set -e; cd $rel && \
    llvm-profdata-19 merge -sparse raw/*.profraw -o merged.profdata && \
    llvm-cov-19 report /external/tools/fmgc-lab/build/$unit-cov-check -instr-profile=merged.profdata \
      -show-branch-summary > report.txt && \
    rm -rf html && llvm-cov-19 show /external/tools/fmgc-lab/build/$unit-cov-check -instr-profile=merged.profdata \
      -format=html -show-branches=count -output-dir=html >/dev/null && \
    for m in $model PitchNormalLaw PitchAlternateLaw PitchDirectLaw LateralNormalLaw LateralDirectLaw; do \
      llvm-cov-19 show /external/tools/fmgc-lab/build/$unit-cov-check -instr-profile=merged.profdata -show-branches=count \
        /external/tools/fmgc-lab/build/models/stock/\$m.cpp > \$m.cov.txt 2>/dev/null || true; done && \
    chown -R $(id -u):$(id -g) .")
  grep -E 'Filename|Computer.cpp|Law.cpp|readable/|TOTAL' "$covdir/report.txt"
fi
exit $status
