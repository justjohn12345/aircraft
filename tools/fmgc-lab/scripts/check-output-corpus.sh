#!/bin/bash
# Runs the readable output packing check over every recording: plain replay, and per file N runs each of: fuzzed
# inputs, synthetic approaches (src/scenario_approach.h: LAND, FLARE, ROLL OUT, go-around), lateral, autothrust,
# engagement and targets scenarios (src/scenario_*.h), alone and combined, so that every packed bit gets toggled.
# The packing has no state, so there is no resync/free distinction. JOBS: parallel runs (default nproc).
# The generated stock model is the oracle, so every recording works whatever build flew it.
# Usage: scripts/check-output-corpus.sh [runs per file, default 3] [binary, default build/fmgc-check-output-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
runs=${1:-3}
bin=${2:-$here/build/fmgc-check-output-stock}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
if [ -n "$(find "$here/readable" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=${OUT:-$here/build/check-output}
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
  run "$f.plain" "$path"
  for ((seed = 1; seed <= runs; seed++)); do
    run "$f.fuzz$seed" --fuzz $seed "$path"
    run "$f.approach$seed" --approach $seed --fuzz $((seed + 100)) "$path"
    run "$f.lateral$seed" --lateral $seed --approach $((seed + 200)) "$path"
    run "$f.athr$seed" --athr $seed "$path"
    run "$f.engagement$seed" --engagement $seed --approach $((seed + 300)) "$path"
    run "$f.targets$seed" --targets $seed --fuzz $((seed + 400)) "$path"
    run "$f.all$seed" --engagement $((seed + 500)) --athr $((seed + 500)) --lateral $((seed + 500)) \
      --approach $((seed + 500)) --targets $((seed + 500)) --fuzz $((seed + 500)) "$path"
  done
done
wait
# coverage over all runs: discrete-word bits never set (spares are expected here) or rarely rising, bits set outside
# the data field, SSM values seen per word
grep -h '^bit ' "$out"/*.txt | grep -v OUTSIDE |
  awk '{k=$2" "$3; n[k]+=$4; r[k]+=$5} END {for (k in n) {if (n[k] == 0) printf "bit never set: %s\n", k; else if (r[k] < 3) printf "bit rarely rises: %s (%d rises)\n", k, r[k]}}' |
  sort -k4,4 -k5,5n
grep -h 'OUTSIDE DATA FIELD' "$out"/*.txt | sort -u
grep -h '^word ' "$out"/*.txt |
  awk '{fw[$2]+=$4; ncd[$2]+=$5; ft[$2]+=$6; no[$2]+=$7; nz[$2]+=$9} END {for (k in no) printf "ssm %-32s FW %9d NCD %9d FT %9d NO %9d  data!=0 %9d\n", k, fw[k], ncd[k], ft[k], no[k], nz[k]}' |
  sort
# fields with mismatches, over all runs (report lines between the header and the coverage section)
awk '/^field /{on=1; next} /^word /{on=0} on && NF >= 4 {n[$1]+=$2; r[$1]++} END {for (k in n) printf "%-60s %8d mismatching steps in %d runs\n", k, n[k], r[k]}' "$out"/*.txt | sort
total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'fields compared' "$out"/*.txt | wc -l)
fail=$(grep -L ' 0 with mismatches' "$out"/*.txt | wc -l)
echo "$total runs, $fail with mismatches or no report, $bad without a report (reports in $out)"
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ]
