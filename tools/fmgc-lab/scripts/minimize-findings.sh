#!/bin/bash
# Minimises one reproducer per check from a findings directory (build/fuzz/findings by default) into build/fuzz/min:
# <check>.bin (the input), <check>.txt (minimiser log with the case description and its bytes) and <check>.trace (the
# mode timeline of the minimised input). Inputs that no longer trigger their check with the current build are skipped.
# usage: scripts/minimize-findings.sh [FINDINGS_DIR] [OUT_DIR]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
src=${1:-$here/build/fuzz/findings}
out=${2:-$here/build/fuzz/min}
export FMGC_FUZZ_BANK=${FMGC_FUZZ_BANK:-$here/build/fuzz/bank.bin}
mkdir -p "$out"
for check in $(ls "$src" | sed -n 's/-[0-9a-f]\{16\}\.bin$//p' | sort -u); do
  done_one=0
  for f in "$src/$check"-*.bin; do
    if timeout 300 "$here/build/fmgc-fuzz-run" minimize "$check" "$f" "$out/$check.bin" > "$out/$check.txt" 2>&1; then
      "$here/build/fmgc-fuzz-run" run --trace "$out/$check.bin" > "$out/$check.trace" 2>&1
      echo "$check: $(grep '^after' "$out/$check.txt")"
      done_one=1
      break
    fi
  done
  [ $done_one = 1 ] || echo "$check: no input reproduces it with this build"
done
