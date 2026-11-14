#!/bin/bash
# Runs the readable vertical logic check over every recording: resync, free-running, and N fuzzed runs per file
# (plus N with synthetic approaches, src/scenario_approach.h, for G/S, LAND and FLARE, and N with synthetic lateral
# episodes, src/scenario_lateral.h, alone and on top of the approaches). JOBS: parallel runs (default nproc).
# The generated stock model is the oracle, so every recording works whatever build flew it.
# Usage: scripts/check-vertical-corpus.sh [fuzz runs per file, default 3] [binary, default build/fmgc-check-vertical-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
runs=${1:-3}
bin=${2:-$here/build/fmgc-check-vertical-stock}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
if [ -n "$(find "$here/readable" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=$here/build/check-vertical
rm -rf "$out" && mkdir -p "$out"
# run NAME ARGS...: one check in the background, at most JOBS (default nproc) at a time
run() {
  local name=$1
  shift
  while [ "$(jobs -r | wc -l)" -ge "${JOBS:-$(nproc)}" ]; do wait -n; done
  "$bin" "$@" > "$out/$name.txt" 2>&1 &
}
for path in "$corpus"/*.fdr; do
  f=$(basename "$path" .fdr)
  run "$f.resync" "$path"
  run "$f.free" --free "$path"
  for ((seed = 1; seed <= runs; seed++)); do
    run "$f.fuzz$seed" --fuzz $seed "$path"
    run "$f.free-fuzz$seed" --free --fuzz $seed "$path"
    run "$f.approach$seed" --approach $seed --fuzz $seed "$path"
    run "$f.lateral$seed" --lateral $seed --fuzz $seed "$path"
    run "$f.lateral-approach$seed" --free --approach $seed --lateral $((seed + 100)) "$path"
  done
done
wait
# coverage: signals never true in any run
grep -h -E '^[A-Za-z]+ +yes ' "$out"/*.txt | awk '{t[$1]+=$8; r[$1]+=$9} END {for (k in t) if (r[k] == 0) printf "never true: %s\n", k}' | sort
grep -h '^law ' "$out"/*.txt | awk '{n[$2]+=$3} END {for (k in n) if (n[k] == 0) printf "law never active: %s\n", k}' | sort
# signals with mismatches, over all runs
grep -h -E '^[A-Za-z]+ +yes +[1-9]' "$out"/*.txt | awk '{n[$1]+=$3; r[$1]++} END {for (k in n) printf "%-16s %8d mismatching steps in %d runs\n", k, n[k], r[k]}' | sort
total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'with mismatches' "$out"/*.txt | wc -l)
fail=$(grep -l -E '^[A-Za-z]+ +yes +[1-9]' "$out"/*.txt | wc -l)
echo "$total runs, $fail with mismatches (reports in $out)"
[ "$fail" -eq 0 ]
