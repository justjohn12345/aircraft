#!/bin/bash
# Source-based coverage (llvm-cov) of the generated model (FmgcComputer.cpp, FmgcOuterLoops.cpp) driven by the replay
# corpus with every scenario injector, or by a fuzz corpus. Needs scripts/build-fuzz.sh (fmgc-cov-drive,
# fmgc-cov-fuzz-run) and, for `fuzz`, the bank (scripts/fuzz.sh builds it). Binaries run on the host; llvm-profdata /
# llvm-cov run in the dev container.
#
# usage:
#   [JOBS=4] [SEEDS=2] scripts/coverage.sh replay [CORPUS_DIR]   corpus x scenario configs  -> build/coverage/replay
#   [JOBS=4] scripts/coverage.sh fuzz NAME CORPUS...             fuzz corpus dirs/files     -> build/coverage/NAME
#   scripts/coverage.sh merge NAME A B                           union of two results       -> build/coverage/NAME
# Each result dir holds merged.profdata, report.txt (llvm-cov report: lines/regions/branches per file), html/
# (llvm-cov show), export.json and uncovered.txt (scripts/coverage_report.py: branch sides never taken, by subsystem;
# for fuzz/merge also the sides gained over build/coverage/replay). `replay` also writes props.txt (the property
# table over every run, scripts/props_summary.py).
#
# Replay configs per recording: plain; and for each seed 1..SEEDS: --fuzz; --approach; --athr --approach;
# --lateral --approach; --engagement --approach; --targets --approach; --failures --approach; --tcas --situation;
# everything at once.
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
jobs=${JOBS:-4}
mode=${1:-replay}
cov="$here/build/coverage"

report() {  # name, object binary
  local name=$1 obj=$2
  (cd "$repo" && ./scripts/dev-env/run.sh bash -c "set -e; cd tools/fmgc-lab/build/coverage/$name && \
    llvm-profdata-19 merge -sparse raw/*.profraw -o merged.profdata && \
    llvm-cov-19 report /external/tools/fmgc-lab/build/$obj -instr-profile=merged.profdata -show-branch-summary > report.txt && \
    rm -rf html && llvm-cov-19 show /external/tools/fmgc-lab/build/$obj -instr-profile=merged.profdata -format=html \
      -show-branches=count -output-dir=html >/dev/null && \
    llvm-cov-19 export /external/tools/fmgc-lab/build/$obj -instr-profile=merged.profdata -format=text > export.json && \
    chown -R $(id -u):$(id -g) .")
  local baseline=()
  if [ "$name" != replay ] && [ -f "$cov/replay/export.json" ]; then
    baseline=(--baseline "$cov/replay/export.json")
  fi
  python3 "$here/scripts/coverage_report.py" "$cov/$name/export.json" "${baseline[@]}" \
    --source-dir "$here/build/models/stock" > "$cov/$name/uncovered.txt"
  cat "$cov/$name/report.txt"
  sed -n '/branch conditions per subsystem/,/uncovered branch sides/p' "$cov/$name/uncovered.txt" | head -120
}

case "$mode" in
  replay)
    corpus=${2:-$HOME/fdr-corpus/raw}
    seeds=${SEEDS:-2}
    rm -rf "$cov/replay"
    mkdir -p "$cov/replay/raw" "$cov/replay/runs"
    list="$cov/replay/runs.txt"
    : > "$list"
    for f in "$corpus"/*.fdr; do
      echo "$f|plain|" >> "$list"
      for s in $(seq 1 "$seeds"); do
        echo "$f|fuzz$s|--fuzz $s" >> "$list"
        echo "$f|approach$s|--approach $s" >> "$list"
        echo "$f|athr$s|--athr $s --approach $s" >> "$list"
        echo "$f|lateral$s|--lateral $s --approach $s" >> "$list"
        echo "$f|engagement$s|--engagement $s --approach $s" >> "$list"
        echo "$f|targets$s|--targets $s --approach $s" >> "$list"
        echo "$f|failures$s|--failures $s --approach $s" >> "$list"
        echo "$f|tcas$s|--tcas $s --situation $s" >> "$list"
        echo "$f|all$s|--fuzz $s --approach $s --athr $s --lateral $s --engagement $s --targets $s --failures $s --tcas $s --situation $s" >> "$list"
      done
    done
    echo "$(wc -l < "$list") runs, $jobs at a time"
    export cov here
    t0=$(date +%s)
    # shellcheck disable=SC2016
    xargs -P "$jobs" -d '\n' -n 1 bash -c 'IFS="|" read -r f tag args <<< "$0"; b=$(basename "$f" .fdr); \
      LLVM_PROFILE_FILE="$cov/replay/raw/cov-%4m.profraw" "$here/build/fmgc-cov-drive" $args "$f" \
        > "$cov/replay/runs/$b-$tag.txt" 2>&1 || echo "FAILED $b $tag"' < "$list"
    echo "runs done in $(( $(date +%s) - t0 )) s"
    python3 "$here/scripts/props_summary.py" "$cov"/replay/runs/*.txt > "$cov/replay/props.txt"
    report replay fmgc-cov-drive
    ;;
  fuzz)
    name=$2
    shift 2
    rm -rf "${cov:?}/$name"
    mkdir -p "$cov/$name/raw"
    # one runner process per input chunk, JOBS at a time
    files=$(for p in "$@"; do find "$p" -type f ! -name '*.txt'; done | sort)
    echo "$(echo "$files" | wc -l) inputs"
    export cov here name
    echo "$files" | xargs -P "$jobs" -n 200 bash -c 'LLVM_PROFILE_FILE="$cov/$name/raw/cov-%4m.profraw" \
      "$here/build/fmgc-cov-fuzz-run" run --summary "$@" > /dev/null' _
    report "$name" fmgc-cov-fuzz-run
    ;;
  merge)
    name=$2 a=$3 b=$4
    rm -rf "${cov:?}/$name"
    mkdir -p "$cov/$name/raw"
    cp "$cov/$a/merged.profdata" "$cov/$name/raw/a.profraw"
    cp "$cov/$b/merged.profdata" "$cov/$name/raw/b.profraw"
    report "$name" fmgc-cov-drive
    ;;
  *)
    echo "usage: scripts/coverage.sh replay [CORPUS] | fuzz NAME CORPUS... | merge NAME A B" >&2
    exit 2
    ;;
esac
