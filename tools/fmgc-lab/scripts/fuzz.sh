#!/bin/bash
# Time-boxed libFuzzer campaign on the generated FMGC (build/fmgc-fuzz, scripts/build-fuzz.sh).
#
# usage: [WORKERS=4] [TIME=3600] [CORPUS_DIR=~/fdr-corpus/raw] scripts/fuzz.sh [--rebuild-bank] [--name NAME] \
#          [--coverage] [libFuzzer flags...]
#
# Layout under build/fuzz/:
#   bank.bin              snapshot bank (fmgc-fuzz-bank; built when missing or with --rebuild-bank: every 120 s of
#                         each recording, 30 s windows, decimation 2)
#   seeds/                seed corpus (fmgc-fuzz-run seeds: one plain input per snapshot + scenario seeds per recording)
#   corpus/               the evolving corpus (kept across campaigns)
#   findings/             reproducers written by the harness (<check>-<hash>.bin + .txt), property and differential
#   artifacts/            libFuzzer crash artifacts (differential findings abort by default: FMGC_FUZZ_ABORT=diff)
#   logs/NAME.log         the campaign log; NAME defaults to the start time
# With --coverage, the corpus is replayed through the coverage build afterwards (scripts/coverage.sh fuzz NAME) and
# merged with build/coverage/replay (build/coverage/NAME+replay).
#
# Workers: libFuzzer fork mode (-fork=WORKERS, at most 4: the machine is shared), crashes/timeouts/OOMs ignored so the
# campaign continues after a differential finding.
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
workers=${WORKERS:-4}
if [ "$workers" -gt 4 ]; then
  echo "WORKERS capped at 4" >&2
  workers=4
fi
time=${TIME:-3600}
corpus_src=${CORPUS_DIR:-$HOME/fdr-corpus/raw}
fz="$here/build/fuzz"
name=$(date +%Y%m%d-%H%M%S)
rebuild=0
coverage=0
extra=()
while [ $# -gt 0 ]; do
  case "$1" in
    --rebuild-bank) rebuild=1 ;;
    --name) name=$2; shift ;;
    --coverage) coverage=1 ;;
    *) extra+=("$1") ;;
  esac
  shift
done
mkdir -p "$fz/corpus" "$fz/findings" "$fz/artifacts" "$fz/logs"
if [ $rebuild = 1 ] || [ ! -f "$fz/bank.bin" ]; then
  "$here/build/fmgc-fuzz-bank" --interval 120 --window 30 --decimation 2 --out "$fz/bank.bin" "$corpus_src"/*.fdr
  rm -rf "$fz/seeds"
fi
if [ ! -d "$fz/seeds" ]; then
  FMGC_FUZZ_BANK="$fz/bank.bin" "$here/build/fmgc-fuzz-run" seeds "$fz/seeds" 3
fi
log="$fz/logs/$name.log"
echo "campaign $name: $workers workers, $time s, log $log"
cd "$fz"
FMGC_FUZZ_BANK="$fz/bank.bin" FMGC_FUZZ_FINDINGS="$fz/findings" "$here/build/fmgc-fuzz" \
  -fork="$workers" -ignore_crashes=1 -ignore_timeouts=1 -ignore_ooms=1 \
  -max_total_time="$time" -max_len=520 -timeout=20 -rss_limit_mb=4096 \
  -artifact_prefix="$fz/artifacts/" -print_final_stats=1 "${extra[@]}" \
  "$fz/corpus" "$fz/seeds" > "$log" 2>&1 || true
echo "corpus: $(ls "$fz/corpus" | wc -l) inputs; findings: $(ls "$fz/findings" | grep -c '\.bin$' || true) reproducers"
grep -E "^#[0-9]+: cov" "$log" | tail -1 || true
grep -E "^#[0-9]+: cov" "$log" | awk '{for(i=1;i<=NF;i++) if($i=="exec/s:") {s+=$(i+1); n++}} END {if(n) printf "mean exec/s over log lines: %.0f\n", s/n}'
ls "$fz/findings" | sed -n 's/-[0-9a-f]\{16\}\.bin$//p' | sort | uniq -c
if [ $coverage = 1 ]; then
  "$here/scripts/coverage.sh" fuzz "$name" "$fz/corpus"
  if [ -d "$here/build/coverage/replay" ]; then
    "$here/scripts/coverage.sh" merge "$name+replay" replay "$name"
  fi
fi
