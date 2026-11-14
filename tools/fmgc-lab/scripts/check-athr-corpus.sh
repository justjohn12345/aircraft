#!/bin/bash
# Runs the readable autothrust check over every recording: resync, free-running, and N fuzzed runs per file
# (plus N with synthetic approaches, src/scenario_approach.h, for LAND, FLARE and RETARD).
# The generated stock model is the oracle, so every recording works whatever build flew it.
# Usage: scripts/check-athr-corpus.sh [fuzz runs per file, default 3] [binary, default build/fmgc-check-athr-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
runs=${1:-3}
bin=${2:-$here/build/fmgc-check-athr-stock}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
if [ -n "$(find "$here/readable" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=$here/build/check-athr
rm -rf "$out" && mkdir -p "$out"
jobs=()
for path in "$corpus"/*.fdr; do
  f=$(basename "$path" .fdr)
  "$bin" "$path" > "$out/$f.resync.txt" 2>&1 &
  "$bin" --free "$path" > "$out/$f.free.txt" 2>&1 &
  for ((seed = 1; seed <= runs; seed++)); do
    "$bin" --fuzz $seed "$path" > "$out/$f.fuzz$seed.txt" 2>&1 &
    "$bin" --free --fuzz $seed "$path" > "$out/$f.free-fuzz$seed.txt" 2>&1 &
    "$bin" --approach $seed --fuzz $seed "$path" > "$out/$f.approach$seed.txt" 2>&1 &
  done
  while [ "$(jobs -r | wc -l)" -ge "${JOBS:-$(nproc)}" ]; do wait -n; done
done
wait
# coverage: flags never non-zero in any run
grep -h -E '^[a-z0-9_]+ +(yes|no) ' "$out"/*.txt | awk '{t[$1]+=$9; r[$1]+=$10} END {for (k in t) if (t[k] == 0) printf "never non-zero: %s\n", k}' | sort
# FMA values seen (generated), over all runs
grep -h '^fma_mode steps:' "$out"/*.txt | tr ' ' '\n' | grep ':' | grep -v steps | awk -F: '{n[$1]+=$2} END {for (k in n) printf "fma_mode %s: %d steps\n", k, n[k]}' | sort -t' ' -k2 -n
grep -h '^fma_message steps:' "$out"/*.txt | tr ' ' '\n' | grep ':' | grep -v steps | awk -F: '{n[$1]+=$2} END {for (k in n) printf "fma_message %s: %d steps\n", k, n[k]}' | sort -t' ' -k2 -n
# fields with mismatches, over all runs
grep -h -E '^[a-z0-9_]+ +yes +[1-9]' "$out"/*.txt | awk '{n[$1]+=$3; r[$1]++} END {for (k in n) printf "%-24s %8d mismatching steps in %d runs\n", k, n[k], r[k]}' | sort
total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'with mismatches' "$out"/*.txt | wc -l)
fail=$(grep -l -E '^[a-z0-9_]+ +yes +[1-9]' "$out"/*.txt | wc -l)
echo "$total runs, $fail with mismatches, $bad without a report (reports in $out)"
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ]
