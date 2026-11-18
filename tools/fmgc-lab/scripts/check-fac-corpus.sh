#!/bin/bash
# Runs the FAC differential check (readable FAC vs generated FacComputer, both FACs, free-running lockstep, every output
# field and hidden memory bit-exact; build/fac-check-<variant>, src/check_fac.cpp) over every recording, with these
# configs (inputs rebuilt from the record, src/fac_drive.h):
#   plain; and for each seed 1..SEEDS: every injector alone (--script --failures --discretes --power --sim), two mixes
#   and everything at once; one --warm run (the readable initialize() is a complete reset).
# Then fac-replay on every recording (the generated FACs against the recorded FAC outputs), summarised: on frames with
# known inputs every field must be bit-exact except the two washout filter outputs that differ by rounding in the sim
# (speed trend <= 1e-6 kt, yaw damper order <= 1e-12 deg; docs/fac.md).
# JOBS: parallel runs (default 4; never more than the machine budget). The generated stock model is the oracle.
# COVERAGE=1: run build/fac-cov-check instead (scripts/build-fuzz.sh with FAC_ONLY=1) and write the source-based
# coverage of FacComputer.cpp and readable/fac/*.cpp (llvm-cov, in the dev container) to build/coverage/fac/.
# Usage: [JOBS=4] [SEEDS=2] [COVERAGE=1] scripts/check-fac-corpus.sh [corpus dir, default ~/fdr-corpus/raw]
#        [binary, default build/fac-check-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
corpus=${1:-${CORPUS:-$HOME/fdr-corpus/raw}}
seeds=${SEEDS:-2}
jobs=${JOBS:-4}
if [ -n "${COVERAGE:-}" ]; then
  bin=${2:-$here/build/fac-cov-check}
else
  bin=${2:-$here/build/fac-check-stock}
fi
replay=$here/build/fac-replay-stock
if [ ! -x "$bin" ]; then
  echo "error: $bin missing (scripts/build.sh, or scripts/build-fuzz.sh with FAC_ONLY=1 for COVERAGE=1)" >&2
  exit 2
fi
if [ -n "$(find "$here/readable/fac" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/fac/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=${OUT:-$here/build/check-fac}
rm -rf "$out" && mkdir -p "$out"
covdir=$here/build/coverage/fac
if [ -n "${COVERAGE:-}" ]; then
  rm -rf "$covdir" && mkdir -p "$covdir/raw"
fi
# run NAME ARGS...: one check in the background, at most JOBS at a time
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

# fields and hidden memories with mismatches, over all runs
awk '/^field /{on=1; next} /^cov /{on=0} on && NF >= 3 && $2 ~ /^[0-9]+$/ {n[$1]+=$2; r[$1]++}
     END {for (k in n) printf "%-64s %10d mismatching steps in %d runs\n", k, n[k], r[k]}' "$out"/*.txt | sort
grep -h '^hidden\.' "$out"/*.txt | awk '{n[$1]+=$2; r[$1]++} END {for (k in n) printf "%-64s %10d steps in %d runs\n", k, n[k], r[k]}' | sort
grep -h '^FIRST MISMATCH' -A 4 "$out"/*.txt | head -40
# output coverage over all runs (generated values): fields that never changed
grep -h '^cov ' "$out"/*.txt | awk '{c[$2]+=$4; z[$2]+=$3} END {for (k in c) if (c[k] == 0) printf "never changed: %s (nonzero on %d steps)\n", k, z[k]}' | sort
# event totals (FAC 1 and 2)
grep -h '^events' "$out"/*.txt | awk '{for (i = 3; i <= NF; i++) if ($i ~ /^[0-9]+,?$/) {gsub(",", "", $i); s[i] += $i}}
  END {printf "events (all runs, both FACs): restarts %d, alpha floor %d, low energy %d, pitch-pitch %d, trim moving %d steps, trim resets %d\n", s[4], s[7], s[10], s[12], s[15], s[19]}'

total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'fields compared' "$out"/*.txt | wc -l)
fail=$(grep -L ' 0 with mismatches, 0 mismatching steps; [0-9]* hidden memories, 0 diverged; 0 init' "$out"/*.txt | wc -l)
steps=$(grep -h '^steps ' "$out"/*.txt | awk '{s+=$2} END {printf "%d steps x 2 FACs", s}')
restarts=$(grep -h '^events' "$out"/*.txt | awk '{gsub(",", "", $4); s+=$4} END {print s}')
timing=$(grep -h '^time ' "$out"/*.txt |
  awk '{g+=$3; r+=$6; k++} END {printf "mean step time: generated %.3f us, readable %.3f us", g/k, r/k}')
echo "$total runs, $steps, $restarts FAC restarts; $timing"
echo "$fail with mismatches or no report, $bad without a report (reports in $out)"
status=0
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ] || status=1

# replay of every recording: the generated FACs against the recorded outputs (the harness reproduces the sim)
mkdir -p "$out/replay"
for path in "${files[@]}"; do
  while [ "$(jobs -r | wc -l)" -ge "$jobs" ]; do wait -n; done
  "$replay" "$path" > "$out/replay/$(basename "$path" .fdr).txt" 2>&1 &
done
wait
awk '
  /^frames/ {frames += $2; fac += $14 + $16}
  /^approximate inputs/ {approx += $11}
  /^fac[12]\./ {
    name = $1; sub(/^fac[12]\./, "", name)
    m[name] += $2; if ($5 + 0 > d[name]) d[name] = $5 + 0
    ok = (name == "bus_outputs.speed_trend_kn.Data" && $5 + 0 <= 1e-6) ||
         (name == "analog_outputs.yaw_damper_order_deg" && $5 + 0 <= 1e-12)
    if (!ok) real[FILENAME ": " $0] = 1
  }
  END {
    printf "replay: %d recordings, %d frames, %d FAC frames compared with known inputs, %d with approximate inputs\n",
      ARGC - 1, frames, fac, approx
    for (k in m) printf "replay rounding: %-40s %9d FAC frames, max |diff| %g\n", k, m[k], d[k]
    n = 0; for (k in real) {print "REPLAY MISMATCH " k; n++}
    printf "replay: %d mismatches beyond rounding on frames with known inputs\n", n
    exit n > 0
  }' "$out"/replay/*.txt || status=1

if [ -n "${COVERAGE:-}" ]; then
  rel=${covdir#"$repo"/}
  (cd "$repo" && ./scripts/dev-env/run.sh bash -c "set -e; cd $rel && \
    llvm-profdata-19 merge -sparse raw/*.profraw -o merged.profdata && \
    llvm-cov-19 report /external/tools/fmgc-lab/build/fac-cov-check -instr-profile=merged.profdata \
      -show-branch-summary > report.txt && \
    rm -rf html && llvm-cov-19 show /external/tools/fmgc-lab/build/fac-cov-check -instr-profile=merged.profdata \
      -format=html -show-branches=count -output-dir=html >/dev/null && \
    llvm-cov-19 show /external/tools/fmgc-lab/build/fac-cov-check -instr-profile=merged.profdata -show-branches=count \
      /external/tools/fmgc-lab/build/models/stock/FacComputer.cpp > FacComputer.cov.txt && \
    chown -R $(id -u):$(id -g) .")
  grep -E 'Filename|FacComputer.cpp|readable/fac|TOTAL' "$covdir/report.txt"
fi
exit $status
