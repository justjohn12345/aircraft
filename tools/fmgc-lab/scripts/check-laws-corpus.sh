#!/bin/bash
# Runs the readable vertical control-law check over every recording: generated commands into the output stage,
# free-running, and N fuzzed runs per file (half of them with synthetic approaches). JOBS limits parallel runs
# (default: number of cores).
# Usage: JOBS=4 scripts/check-laws-corpus.sh [fuzz runs per file, default 2] [binary, default build/fmgc-check-laws]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
runs=${1:-2}
bin=${2:-$here/build/fmgc-check-laws}
jobs=${JOBS:-$(nproc)}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
out=$here/build/check-laws
rm -rf "$out" && mkdir -p "$out"
run() { "$bin" "$@" > "$out/$name.txt" 2>&1; }
for path in "$corpus"/*.fdr; do
  f=$(basename "$path" .fdr)
  name=$f.gen; run "$path" &
  name=$f.free; run --free "$path" &
  for ((seed = 1; seed <= runs; seed++)); do
    name=$f.fuzz$seed; run --free --fuzz $seed "$path" &
    name=$f.approach$seed; run --free --approach $seed --fuzz $seed "$path" &
    while [ "$(jobs -r | wc -l)" -ge "$jobs" ]; do wait -n; done
  done
  while [ "$(jobs -r | wc -l)" -ge "$jobs" ]; do wait -n; done
done
wait
grep -h -E '^[a-z_.]+ +[0-9]+ +[1-9]' "$out"/*.txt | awk '{n[$1]+=$3; r[$1]++; if ($NF > m[$1]) m[$1] = $NF} END {for (k in n) printf "%-30s %9d mismatching steps in %d runs, max |diff| %s\n", k, n[k], r[k], m[k]}' | sort
total=$(ls "$out"/*.txt | wc -l)
fail=$(grep -l -E '^[a-z_.]+ +[0-9]+ +[1-9]' "$out"/*.txt | wc -l)
crash=$(grep -L 'channels checked' "$out"/*.txt | wc -l)
echo "$total runs, $fail with mismatches, $crash without a report (reports in $out)"
[ "$fail" -eq 0 ] && [ "$crash" -eq 0 ]
