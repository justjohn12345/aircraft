#!/bin/bash
# Runs the FCU differential check (readable FCU vs generated FcuComputer, free-running lockstep, every output field and
# hidden memory bit-exact; build/fcu-check-<variant>, src/check_fcu.cpp) over every recording, with these configs:
#   3209001 files: the recorded FCU inputs, alone and with --panel / --bus / --power / all injectors
#   every file (synthetic drive from the FMGC record): plain; --fmgc2 copy|invalid|ncd|swapped; --reconstruct; and for
#   each seed 1..SEEDS: every injector alone (--script --panel --priority --bus --sim --power, with --reconstruct on odd
#   seeds), three mixes, and everything at once with a rotating FMGC 2 variant.
# Then, for 3209001 files, fcu-replay (the generated FCU against the recorded outputs).
# JOBS: parallel runs (default 4; never more than the machine budget). The generated stock model is the oracle.
# COVERAGE=1: run build/fcu-cov-check instead (scripts/build-fuzz.sh with FCU_ONLY=1) and write the source-based
# coverage of FcuComputer.cpp and readable/fcu/*.cpp (llvm-cov, in the dev container) to build/coverage/fcu/.
# Usage: [JOBS=4] [SEEDS=2] [COVERAGE=1] scripts/check-fcu-corpus.sh [corpus dir, default ~/fdr-corpus/raw]
#        [binary, default build/fcu-check-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
corpus=${1:-${CORPUS:-$HOME/fdr-corpus/raw}}
seeds=${SEEDS:-2}
jobs=${JOBS:-4}
if [ -n "${COVERAGE:-}" ]; then
  bin=${2:-$here/build/fcu-cov-check}
else
  bin=${2:-$here/build/fcu-check-stock}
fi
replay=$here/build/fcu-replay-stock
if [ ! -x "$bin" ]; then
  echo "error: $bin missing (scripts/build.sh, or scripts/build-fuzz.sh with FCU_ONLY=1 for COVERAGE=1)" >&2
  exit 2
fi
if [ -n "$(find "$here/readable/fcu" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/fcu/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=${OUT:-$here/build/check-fcu}
rm -rf "$out" && mkdir -p "$out"
covdir=$here/build/coverage/fcu
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
is_fcu() {  # interface version 3209001-3209004 (with the FCU)?
  python3 -c "import gzip,struct,sys; sys.exit(0 if struct.unpack('<Q', gzip.open(sys.argv[1]).read(8))[0] in (3209001, 3209002, 3209003, 3209004) else 1)" "$1"
}
f2=(copy invalid ncd swapped opp)
fcu_files=()
for path in "$corpus"/*.fdr; do
  f=$(basename "$path" .fdr)
  if is_fcu "$path"; then
    fcu_files+=("$path")
    run "$f.recorded" "$path"
    run "$f.recorded-panel" --panel 11 "$path"
    run "$f.recorded-bus" --bus 11 "$path"
    run "$f.recorded-power" --power 11 "$path"
    run "$f.recorded-all" --script 12 --panel 12 --priority 12 --bus 12 --sim 12 --power 12 "$path"
  fi
  run "$f.plain" --synthetic "$path"
  for m in copy invalid ncd swapped; do run "$f.fmgc2-$m" --synthetic --fmgc2 "$m" "$path"; done
  run "$f.reconstruct" --synthetic --reconstruct "$path"
  for ((s = 1; s <= seeds; s++)); do
    rec=()
    if ((s % 2 == 1)); then rec=(--reconstruct); fi
    for inj in script panel priority bus sim power; do
      run "$f.$inj.$s" --synthetic "${rec[@]}" --$inj $((s * 100 + 1)) "$path"
    done
    run "$f.mix1.$s" --synthetic --reconstruct --script $((s * 100 + 2)) --bus $((s * 100 + 2)) "$path"
    run "$f.mix2.$s" --synthetic --panel $((s * 100 + 3)) --priority $((s * 100 + 3)) --power $((s * 100 + 3)) "$path"
    run "$f.mix3.$s" --synthetic --fmgc2 "${f2[$((s % 5))]}" --bus $((s * 100 + 4)) --sim $((s * 100 + 4)) \
      --panel $((s * 100 + 4)) "$path"
    run "$f.all.$s" --synthetic --reconstruct --fmgc2 "${f2[$(((s + 2) % 5))]}" --script $((s * 100 + 5)) \
      --panel $((s * 100 + 5)) --priority $((s * 100 + 5)) --bus $((s * 100 + 5)) --sim $((s * 100 + 5)) \
      --power $((s * 100 + 5)) "$path"
  done
done
wait

# fields and hidden memories with mismatches, over all runs
awk '/^field /{on=1; next} /^cov /{on=0} on && NF >= 3 && $2 ~ /^[0-9]+$/ {n[$1]+=$2; r[$1]++}
     END {for (k in n) printf "%-60s %10d mismatching steps in %d runs\n", k, n[k], r[k]}' "$out"/*.txt | sort
grep -h '^hidden\.' "$out"/*.txt | awk '{n[$1]+=$2; r[$1]++} END {for (k in n) printf "%-60s %10d steps in %d runs\n", k, n[k], r[k]}' | sort
grep -h '^FIRST MISMATCH' -A 4 "$out"/*.txt | head -40
# output coverage over all runs (generated values on running steps): fields that never changed
grep -h '^cov ' "$out"/*.txt | awk '{c[$2]+=$4; z[$2]+=$3} END {for (k in c) if (c[k] == 0) printf "never changed: %s (nonzero on %d steps)\n", k, z[k]}' | sort

total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'fields compared' "$out"/*.txt | wc -l)
fail=$(grep -L ' 0 with mismatches, 0 mismatching steps; [0-9]* hidden memories, 0 diverged; 0 init' "$out"/*.txt | wc -l)
steps=$(grep -h '^steps ' "$out"/*.txt | awk '{s+=$2; r+=substr($3,2)} END {printf "%d steps (%d running)", s, r}')
restarts=$(grep -h '^steps ' "$out"/*.txt | sed 's/.*running, \([0-9]*\) (re)starts.*/\1/' | awk '{s+=$1} END {print s}')
timing=$(grep -h '^time ' "$out"/*.txt |
  awk '{g+=$3; r+=$6; k++} END {printf "mean step time: generated %.3f us, readable %.3f us", g/k, r/k}')
echo "$total runs, $steps, $restarts (re)starts; $timing"
echo "$fail with mismatches or no report, $bad without a report (reports in $out)"
status=0
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ] || status=1

# replay of the 3209001 recordings: the generated FCU against the recorded outputs (the harness reproduces the sim)
if [ ${#fcu_files[@]} -gt 0 ]; then
  for path in "${fcu_files[@]}"; do
    "$replay" "$path" > "$out/replay-$(basename "$path" .fdr).txt" 2>&1
    echo "replay $(basename "$path"): rc $?: $(tail -1 "$out/replay-$(basename "$path" .fdr).txt")"
  done
else
  echo "no 3209001 recording in $corpus: fcu-replay not run"
fi

if [ -n "${COVERAGE:-}" ]; then
  rel=${covdir#"$repo"/}
  (cd "$repo" && ./scripts/dev-env/run.sh bash -c "set -e; cd $rel && \
    llvm-profdata-19 merge -sparse raw/*.profraw -o merged.profdata && \
    llvm-cov-19 report /external/tools/fmgc-lab/build/fcu-cov-check -instr-profile=merged.profdata \
      -show-branch-summary > report.txt && \
    rm -rf html && llvm-cov-19 show /external/tools/fmgc-lab/build/fcu-cov-check -instr-profile=merged.profdata \
      -format=html -show-branches=count -output-dir=html >/dev/null && \
    llvm-cov-19 show /external/tools/fmgc-lab/build/fcu-cov-check -instr-profile=merged.profdata -show-branches=count \
      /external/tools/fmgc-lab/build/models/stock/FcuComputer.cpp > FcuComputer.cov.txt && \
    chown -R $(id -u):$(id -g) .")
  grep -E 'Filename|FcuComputer.cpp|readable/fcu|TOTAL' "$covdir/report.txt"
fi
exit $status
