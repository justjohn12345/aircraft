#!/bin/bash
# Runs the whole-computer check (readable FMGC vs generated, free-running lockstep, every output field bit-exact) over
# every recording: plain replay, then per file and seed every scenario alone, every pair of scenarios, every
# combination of all but one, and all together. Scenarios: --fuzz, --approach, --athr, --lateral, --engagement,
# --targets, --failures (src/input_fuzzer.h, src/scenario_*.h) and --power (computer_running off / on, flicker,
# opposite FMGC lost: src/scenario_power.h). JOBS: parallel runs (default nproc).
# The generated stock model is the oracle, so every recording works whatever build flew it.
# Usage: scripts/check-fmgc-corpus.sh [seeds per file, default 2] [binary, default build/fmgc-check-fmgc-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
runs=${1:-2}
bin=${2:-$here/build/fmgc-check-fmgc-stock}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
if [ -n "$(find "$here/readable" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=${OUT:-$here/build/check-fmgc}
rm -rf "$out" && mkdir -p "$out"
# run NAME ARGS...: one check in the background, at most JOBS (default nproc) at a time
run() {
  local name=$1
  shift
  while [ "$(jobs -r | wc -l)" -ge "${JOBS:-$(nproc)}" ]; do wait -n; done
  "$bin" "$@" > "$out/$name.txt" 2>&1 &
}
flags=(fuzz approach athr lateral engagement targets failures power)
n=${#flags[@]}
all=$(((1 << n) - 1))
# scenario combinations as bit masks over flags: singles, pairs, all but one, all
masks=()
for ((i = 0; i < n; i++)); do masks+=($((1 << i))); done
for ((i = 0; i < n; i++)); do for ((j = i + 1; j < n; j++)); do masks+=($(((1 << i) | (1 << j)))); done; done
for ((i = 0; i < n; i++)); do masks+=($((all & ~(1 << i)))); done
masks+=($all)
for path in "$corpus"/*.fdr; do
  f=$(basename "$path" .fdr)
  run "$f.plain" "$path"
  for ((seed = 1; seed <= runs; seed++)); do
    for m in "${masks[@]}"; do
      args=()
      name=""
      for ((i = 0; i < n; i++)); do
        if (((m >> i) & 1)); then
          # a different seed per flag and combination, so combined runs are not replays of the single ones
          args+=(--${flags[$i]} $((seed * 1000 + m * 10 + i)))
          name="$name${name:+-}${flags[$i]}"
        fi
      done
      run "$f.$name.$seed" "${args[@]}" "$path"
    done
  done
done
wait
# coverage over all runs (generated values on running frames): computed-bus fields that never changed, and the values
# seen of every enum field
grep -h '^cov ' "$out"/*.txt | grep -v -E '^cov (bus_outputs|discrete_outputs)' |
  awk '{c[$2]+=$4} END {for (k in c) if (c[k] == 0) printf "never changed: %s\n", k}' | sort
grep -h '^enum ' "$out"/*.txt | awk '{n[$2" "$3]+=$4} END {for (k in n) printf "enum %-60s %12d steps\n", k, n[k]}' |
  sort -k2,2 -k3,3n
# fields and hidden memories with mismatches, over all runs
awk '/^field /{on=1; next} /^cov /{on=0} on && NF >= 3 && $2 ~ /^[0-9]+$/ {n[$1]+=$2; r[$1]++}
     END {for (k in n) printf "%-64s %10d mismatching steps in %d runs\n", k, n[k], r[k]}' "$out"/*.txt | sort
grep -h '^FIRST MISMATCH' -A 3 "$out"/*.txt | head -40
total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'fields compared' "$out"/*.txt | wc -l)
fail=$(grep -L ' 0 with mismatches, 0 mismatching steps; [0-9]* hidden memories, 0 diverged; 0 init' "$out"/*.txt | wc -l)
steps=$(grep -h '^steps ' "$out"/*.txt | awk '{s+=$2; r+=substr($3,2)} END {printf "%d steps (%d running)", s, r}')
restarts=$(grep -h '^steps ' "$out"/*.txt | sed 's/.*running, \([0-9]*\) (re)starts.*/\1/' | awk '{s+=$1} END {print s}')
timing=$(grep -h '^time ' "$out"/*.txt |
  awk '{g+=$3; r+=$6; k++} END {printf "mean step time over runs: generated %.3f us, readable %.3f us", g/k, r/k}')
echo "$total runs, $steps, $restarts (re)starts; $timing"
echo "$fail with mismatches or no report, $bad without a report (reports in $out)"
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ]
