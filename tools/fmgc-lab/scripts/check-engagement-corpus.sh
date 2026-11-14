#!/bin/bash
# Runs the readable AP / FD / A/THR engagement check over every recording: resync, free-running, and per file N runs
# each of: fuzzed (resync and free), synthetic engagement episodes (src/scenario_engagement.h) alone, free-running with
# the fuzzer, with the approaches (src/scenario_approach.h: LAND, ROLL OUT, go-around), and with the lateral and
# autothrust scenarios (scenario_lateral.h, scenario_athr.h). JOBS: parallel runs (default nproc).
# The generated stock model is the oracle, so every recording works whatever build flew it.
# Usage: scripts/check-engagement-corpus.sh [fuzz runs per file, default 3] [binary, default build/fmgc-check-engagement-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
runs=${1:-3}
bin=${2:-$here/build/fmgc-check-engagement-stock}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
if [ -n "$(find "$here/readable" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=$here/build/check-engagement
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
    run "$f.engagement$seed" --engagement $seed "$path"
    run "$f.free-engagement$seed" --free --engagement $seed --fuzz $((seed + 100)) "$path"
    run "$f.engagement-approach$seed" --engagement $((seed + 200)) --approach $seed "$path"
    run "$f.free-engagement-approach$seed" --free --engagement $((seed + 300)) --approach $((seed + 300)) --fuzz $((seed + 300)) "$path"
    run "$f.engagement-lateral$seed" --engagement $((seed + 400)) --lateral $seed --approach $((seed + 400)) "$path"
    run "$f.engagement-athr$seed" --free --engagement $((seed + 500)) --athr $seed "$path"
  done
done
wait
# coverage: outputs never true, intermediate conditions never reached (generated outputs; conditions from the readable
# code, which equals the generated code in resync mode)
grep -h -E '^[a-z0-9_]+ +yes ' "$out"/*.txt | awk '{t[$1]+=$7; r[$1]+=$8} END {for (k in t) if (t[k] == 0) printf "never true: %s\n", k; for (k in t) if (t[k] > 0 && r[k] < 3) printf "rarely rises: %s (%d rises)\n", k, r[k]}' | sort
grep -h '^conditions:' "$out"/*.txt | tr ' ' '\n' | grep ':' | grep -v conditions | awk -F: '{n[$1]+=$2} END {for (k in n) printf "condition %-26s %10d steps\n", k, n[k]}' | sort
# fields with mismatches, over all runs
grep -h -E '^[a-z0-9_]+ +yes +[1-9]' "$out"/*.txt | awk '{n[$1]+=$3; r[$1]++} END {for (k in n) printf "%-28s %8d mismatching steps in %d runs\n", k, n[k], r[k]}' | sort
total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'with mismatches' "$out"/*.txt | wc -l)
fail=$(grep -l -E '^[a-z0-9_]+ +yes +[1-9]' "$out"/*.txt | wc -l)
echo "$total runs, $fail with mismatches, $bad without a report (reports in $out)"
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ]
