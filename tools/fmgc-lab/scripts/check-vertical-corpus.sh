#!/bin/bash
# Runs the readable vertical logic check over every recording: resync, free-running, and N fuzzed runs per file.
# The generated stock model is the oracle, so every recording works whatever build flew it.
# Usage: scripts/check-vertical-corpus.sh [fuzz runs per file, default 3] [binary, default build/fmgc-check-vertical-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
runs=${1:-3}
bin=${2:-$here/build/fmgc-check-vertical-stock}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
out=$here/build/check-vertical
rm -rf "$out" && mkdir -p "$out"
jobs=()
for path in "$corpus"/*.fdr; do
  f=$(basename "$path" .fdr)
  "$bin" "$path" > "$out/$f.resync.txt" 2>&1 &
  "$bin" --free "$path" > "$out/$f.free.txt" 2>&1 &
  for ((seed = 1; seed <= runs; seed++)); do
    "$bin" --fuzz $seed "$path" > "$out/$f.fuzz$seed.txt" 2>&1 &
    "$bin" --free --fuzz $seed "$path" > "$out/$f.free-fuzz$seed.txt" 2>&1 &
  done
  while [ "$(jobs -r | wc -l)" -ge "$(nproc)" ]; do wait -n; done
done
wait
# signals with mismatches, over all runs
grep -h -E '^[A-Za-z]+ +yes +[1-9]' "$out"/*.txt | awk '{n[$1]+=$3; r[$1]++} END {for (k in n) printf "%-16s %8d mismatching steps in %d runs\n", k, n[k], r[k]}' | sort
total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'with mismatches' "$out"/*.txt | wc -l)
fail=$(grep -l -E '^[A-Za-z]+ +yes +[1-9]' "$out"/*.txt | wc -l)
echo "$total runs, $fail with mismatches (reports in $out)"
[ "$fail" -eq 0 ]
