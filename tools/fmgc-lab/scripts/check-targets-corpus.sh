#!/bin/bash
# Runs the readable targets block check (HDG/TRK preset, selected/managed speed, Mach, speed targets, ALT soft) over
# every recording: resync, free-running, and per seed: fuzzed (resync and free), synthetic approaches with the fuzzer
# (src/scenario_approach.h), and synthetic targets episodes (src/scenario_targets.h) alone, free-running with the fuzzer,
# and free-running with the approaches. JOBS: parallel runs (default nproc).
# The generated stock model is the oracle, so every recording works whatever build flew it.
# Usage: scripts/check-targets-corpus.sh [fuzz runs per file, default 3] [binary, default build/fmgc-check-targets-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
runs=${1:-3}
bin=${2:-$here/build/fmgc-check-targets-stock}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
if [ -n "$(find "$here/readable" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=${OUT:-$here/build/check-targets}
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
    run "$f.targets$seed" --targets $seed "$path"
    run "$f.free-targets$seed" --free --targets $((seed + 100)) --fuzz $((seed + 100)) "$path"
    run "$f.targets-approach$seed" --free --targets $((seed + 200)) --approach $seed "$path"
  done
done
wait
# coverage: fields never non-zero / never changing, conditions never seen, over all runs
grep -h -E '^[a-z_]+ +yes ' "$out"/*.txt | awk '{n[$1]+=$9; c[$1]+=$10} END {for (k in n) {if (n[k] == 0) printf "never non-zero: %s\n", k; if (c[k] == 0) printf "never changed: %s\n", k}}' | sort
grep -h '^conditions:' "$out"/*.txt | tr ' ' '\n' | grep ':' | grep -v conditions |
  awk -F: '{n[$1]+=$2} END {for (k in n) printf "condition %-34s %10d steps\n", k, n[k]}' | sort > "$out/conditions.summary"
awk '$3 == 0 {print "condition never seen: " $2}' "$out/conditions.summary"
# fields with mismatches, over all runs
grep -h -E '^[a-z_]+ +yes +[1-9]' "$out"/*.txt | awk '{n[$1]+=$3; r[$1]++} END {for (k in n) printf "%-16s %8d mismatching steps in %d runs\n", k, n[k], r[k]}' | sort
total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'with mismatches' "$out"/*.txt | wc -l)
fail=$(grep -l -E '^[a-z_]+ +yes +[1-9]' "$out"/*.txt | wc -l)
echo "$total runs, $fail with mismatches, $bad without a report (reports and conditions.summary in $out)"
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ]
