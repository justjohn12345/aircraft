#!/bin/bash
# Runs the FADEC differential check (readable FADEC vs generated FadecComputer, both engines, free-running lockstep,
# every output field and hidden memory bit-exact; build/fadec-check-<variant>, src/check_fadec.cpp) over every
# recording, with these configs:
#   3209004 files: the recorded FADEC inputs, alone and with --athr / --sim / all injectors
#   every file (synthetic drive): plain; and for each seed 1..SEEDS: every injector alone (--script --lever --athr
#   --sim), two mixes, and everything at once; one --warm run (the readable initialize() is a complete reset).
# Then, for 3209004 files, fadec-replay (the generated FADECs against the recorded outputs).
# JOBS: parallel runs (default 4; never more than the machine budget). The generated stock model is the oracle.
# COVERAGE=1: run build/fadec-cov-check instead (scripts/build-fuzz.sh with FADEC_ONLY=1) and write the source-based
# coverage of FadecComputer.cpp and readable/fadec/*.cpp (llvm-cov, in the dev container) to build/coverage/fadec/.
# Usage: [JOBS=4] [SEEDS=2] [COVERAGE=1] scripts/check-fadec-corpus.sh [corpus dir, default ~/fdr-corpus/raw]
#        [binary, default build/fadec-check-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
corpus=${1:-${CORPUS:-$HOME/fdr-corpus/raw}}
seeds=${SEEDS:-2}
jobs=${JOBS:-4}
if [ -n "${COVERAGE:-}" ]; then
  bin=${2:-$here/build/fadec-cov-check}
else
  bin=${2:-$here/build/fadec-check-stock}
fi
replay=$here/build/fadec-replay-stock
if [ ! -x "$bin" ]; then
  echo "error: $bin missing (scripts/build.sh, or scripts/build-fuzz.sh with FADEC_ONLY=1 for COVERAGE=1)" >&2
  exit 2
fi
if [ -n "$(find "$here/readable/fadec" "$here/src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources in readable/fadec/ or src/ (rebuild failed or not run)" >&2
  exit 2
fi
out=${OUT:-$here/build/check-fadec}
rm -rf "$out" && mkdir -p "$out"
covdir=$here/build/coverage/fadec
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
is_fadec() {  # interface version 3209004 or 3209005 (with the FADEC model record)?
  python3 -c "import gzip,struct,sys; sys.exit(0 if struct.unpack('<Q', gzip.open(sys.argv[1]).read(8))[0] in (3209004, 3209005) else 1)" "$1"
}
fadec_files=()
files=("$corpus"/*.fdr)
for path in "${files[@]}"; do
  f=$(basename "$path" .fdr)
  if is_fadec "$path"; then
    fadec_files+=("$path")
    run "$f.recorded" "$path"
    run "$f.recorded-athr" --athr 11 "$path"
    run "$f.recorded-sim" --sim 11 "$path"
    run "$f.recorded-all" --script 12 --lever 12 --athr 12 --sim 12 "$path"
  fi
  run "$f.plain" --synthetic "$path"
  for ((s = 1; s <= seeds; s++)); do
    for inj in script lever athr sim; do
      run "$f.$inj.$s" --synthetic --$inj $((s * 100 + 1)) "$path"
    done
    run "$f.mix1.$s" --synthetic --script $((s * 100 + 2)) --athr $((s * 100 + 2)) "$path"
    run "$f.mix2.$s" --synthetic --lever $((s * 100 + 3)) --athr $((s * 100 + 3)) --sim $((s * 100 + 3)) "$path"
    run "$f.all.$s" --synthetic --script $((s * 100 + 4)) --lever $((s * 100 + 4)) --athr $((s * 100 + 4)) \
      --sim $((s * 100 + 4)) "$path"
  done
done
run "warm" --synthetic --script 5 --warm "${files[0]}" "${files[${#files[@]} - 1]}"
wait

# fields and hidden memories with mismatches, over all runs
awk '/^field /{on=1; next} /^cov /{on=0} on && NF >= 3 && $2 ~ /^[0-9]+$/ {n[$1]+=$2; r[$1]++}
     END {for (k in n) printf "%-60s %10d mismatching steps in %d runs\n", k, n[k], r[k]}' "$out"/*.txt | sort
grep -h '^hidden\.' "$out"/*.txt | awk '{n[$1]+=$2; r[$1]++} END {for (k in n) printf "%-60s %10d steps in %d runs\n", k, n[k], r[k]}' | sort
grep -h '^FIRST MISMATCH' -A 4 "$out"/*.txt | head -40
# output coverage over all runs (generated values): fields that never changed
grep -h '^cov ' "$out"/*.txt | awk '{c[$2]+=$4; z[$2]+=$3} END {for (k in c) if (c[k] == 0) printf "never changed: %s (nonzero on %d steps)\n", k, z[k]}' | sort
# recorded thrust lock episodes (FADEC 1, from the plain runs)
grep -h '^recorded fadec1' "$out"/*.plain.txt | awk '{e+=$5; d+=substr($7,2); f+=$(NF-1)} END {printf "recorded FADEC 1 thrust lock: %d episodes (%d within 0.5 s of an instinctive disconnect), %d frames\n", e, d, f}'

total=$(ls "$out"/*.txt | wc -l)
bad=$(grep -L 'fields compared' "$out"/*.txt | wc -l)
fail=$(grep -L ' 0 with mismatches, 0 mismatching steps; [0-9]* hidden memories, 0 diverged; 0 init' "$out"/*.txt | wc -l)
steps=$(grep -h '^steps ' "$out"/*.txt | awk '{s+=$2} END {printf "%d steps x 2 FADECs", s}')
timing=$(grep -h '^time ' "$out"/*.txt |
  awk '{g+=$3; r+=$6; k++} END {printf "mean step time: generated %.3f us, readable %.3f us", g/k, r/k}')
echo "$total runs, $steps; $timing"
echo "$fail with mismatches or no report, $bad without a report (reports in $out)"
status=0
[ "$fail" -eq 0 ] && [ "$bad" -eq 0 ] || status=1

# replay of the 3209004 recordings: the generated FADECs against the recorded outputs (the harness reproduces the sim)
if [ ${#fadec_files[@]} -gt 0 ]; then
  for path in "${fadec_files[@]}"; do
    "$replay" "$path" > "$out/replay-$(basename "$path" .fdr).txt" 2>&1
    rc=$?
    echo "replay $(basename "$path"): rc $rc: $(tail -1 "$out/replay-$(basename "$path" .fdr).txt")"
    [ "$rc" -eq 0 ] || status=1
  done
else
  echo "no 3209004 recording in $corpus: fadec-replay not run"
fi

if [ -n "${COVERAGE:-}" ]; then
  rel=${covdir#"$repo"/}
  (cd "$repo" && ./scripts/dev-env/run.sh bash -c "set -e; cd $rel && \
    llvm-profdata-19 merge -sparse raw/*.profraw -o merged.profdata && \
    llvm-cov-19 report /external/tools/fmgc-lab/build/fadec-cov-check -instr-profile=merged.profdata \
      -show-branch-summary > report.txt && \
    rm -rf html && llvm-cov-19 show /external/tools/fmgc-lab/build/fadec-cov-check -instr-profile=merged.profdata \
      -format=html -show-branches=count -output-dir=html >/dev/null && \
    llvm-cov-19 show /external/tools/fmgc-lab/build/fadec-cov-check -instr-profile=merged.profdata -show-branches=count \
      /external/tools/fmgc-lab/build/models/stock/FadecComputer.cpp > FadecComputer.cov.txt && \
    chown -R $(id -u):$(id -g) .")
  grep -E 'Filename|FadecComputer.cpp|readable/fadec|TOTAL' "$covdir/report.txt"
fi
exit $status
