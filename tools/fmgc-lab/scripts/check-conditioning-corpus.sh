#!/bin/bash
# Runs the readable input conditioning check over every recording: resync, free-running, and per file N runs each of:
# fuzzed (resync and free), synthetic failure episodes (src/scenario_conditioning.h) alone (resync and free), free with
# the fuzzer, with the approaches (src/scenario_approach.h: LAND armed / RWY LOC tune inhibit), with the engagement
# scenarios (scenario_engagement.h), and all together with the lateral scenarios. JOBS: parallel runs (default nproc).
# The generated stock model is the oracle, so every recording works whatever build flew it.
# Usage: scripts/check-conditioning-corpus.sh [runs per file, default 3] [binary, default build/fmgc-check-conditioning-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
runs=${1:-3}
bin=${2:-$here/build/fmgc-check-conditioning-stock}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
if [ -n "$(find "$here/readable" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=$here/build/check-conditioning
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
    run "$f.failures$seed" --failures $seed "$path"
    run "$f.free-failures$seed" --free --failures $((seed + 100)) "$path"
    run "$f.free-failures-fuzz$seed" --free --failures $((seed + 200)) --fuzz $((seed + 200)) "$path"
    run "$f.failures-approach$seed" --failures $((seed + 300)) --approach $seed "$path"
    run "$f.free-failures-approach$seed" --free --failures $((seed + 400)) --approach $((seed + 400)) "$path"
    run "$f.failures-engagement$seed" --failures $((seed + 500)) --engagement $seed "$path"
    run "$f.free-all$seed" --free --failures $((seed + 600)) --engagement $((seed + 600)) --approach $((seed + 600)) \
      --lateral $((seed + 600)) --fuzz $((seed + 600)) "$path"
  done
done
wait
# coverage: fields that never changed or (booleans) were never true in the generated outputs, conditions (source
# selection branches, from the readable code, which equals the generated code in resync mode) never reached
grep -h -E '^[a-zA-Z0-9_.]+ +yes ' "$out"/*.txt | awk '{c[$1]+=$8; t[$1]+=$9; b[$1]=($9>=0)} END {for (k in c) {if (c[k] == 0) printf "never changed: %s\n", k; if (b[k] && t[k] == 0) printf "never true: %s\n", k}}' | sort
grep -h '^conditions:' "$out"/*.txt | tr ' ' '\n' | grep ':' | grep -v conditions | awk -F: '{n[$1]+=$2} END {for (k in n) printf "condition %-28s %10d steps\n", k, n[k]}' | sort
# fields with mismatches, over all runs
grep -h -E '^[a-zA-Z0-9_.]+ +yes +[1-9]' "$out"/*.txt | awk '{n[$1]+=$3; r[$1]++} END {for (k in n) printf "%-58s %8d mismatching steps in %d runs\n", k, n[k], r[k]}' | sort
total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'with mismatches' "$out"/*.txt | wc -l)
fail=$(grep -l -E '^[a-zA-Z0-9_.]+ +yes +[1-9]' "$out"/*.txt | wc -l)
echo "$total runs, $fail with mismatches, $bad without a report (reports in $out)"
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ]
