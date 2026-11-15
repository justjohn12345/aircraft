#!/bin/bash
# Runs the sim's FMGC shadow checker (fbw_a320/src/fmgc/FmgcShadow.cpp, via build/fmgc-shadow-test-stock) over every
# recording, FMGC 1 and FMGC 2 (FMGC 1's recorded inputs with is_unit_1 false), plain and with POWER_SEEDS (default 1)
# seeds of power / health episodes (--power: restarts), and its self-test (leaf table, bit
# flips, input perturbation) on one recording. Logs in the sim's format go to build/check-shadow/.
# The generated stock model is the oracle, so every recording works whatever build flew it.
# Usage: [JOBS=n] [CORPUS=dir] scripts/check-shadow-corpus.sh [binary, default build/fmgc-shadow-test-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
bin=${1:-$here/build/fmgc-shadow-test-stock}
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
shadow_src="$here/../../fbw-a32nx/src/wasm/fbw_a320/src/fmgc"
if [ -n "$(find "$here/readable" "$here/src" "$shadow_src" -newer "$bin" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "error: $bin is older than the sources (rebuild failed or not run)" >&2
  exit 2
fi
out=${OUT:-$here/build/check-shadow}
rm -rf "$out" && mkdir -p "$out/self-test"
# run NAME ARGS...: one run in the background, at most JOBS (default 8) at a time
run() {
  local name=$1
  shift
  while [ "$(jobs -r | wc -l)" -ge "${JOBS:-8}" ]; do wait -n; done
  "$bin" "$@" > "$out/$name.txt" 2>&1 &
}
files=("$corpus"/*.fdr)
for path in "${files[@]}"; do
  f=$(basename "$path" .fdr)
  for unit in 1 2; do
    run "$f.unit$unit" --unit "$unit" --time --log "$out/$f.unit$unit.log" "$path"
    for ((seed = 1; seed <= ${POWER_SEEDS:-1}; seed++)); do
      run "$f.unit$unit.power$seed" --unit "$unit" --power "$seed" --log "$out/$f.unit$unit.power$seed.log" "$path"
    done
  done
done
run self-test --self-test --log-dir "$out/self-test" "${files[0]}"
wait

cat "$out"/*.unit?.txt | grep '^RESULT' | awk '
  { for (i = 2; i <= NF; i++) { split($i, kv, "="); v[kv[1]] = kv[2] }
    runs++; frames += v["frames"]; mism += v["mismatch_frames"]; bad += v["mismatch_frames"] > 0
    gen += v["us_generated"] * v["frames"]; sh += v["us_shadow"] * v["frames"]; bare += v["us_readable_bare"] * v["frames"] }
  END { printf "plain runs %d, frames %d, mismatching frames %d, runs with mismatches %d\n", runs, frames, mism, bad
        printf "cost per frame (mean over frames): generated %.3f us, shadow %.3f us (readable step %.3f us + comparison %.3f us)\n",
               gen / frames, sh / frames, bare / frames, (sh - bare) / frames }'
cat "$out"/*.power*.txt | grep '^RESULT' | awk '
  { for (i = 2; i <= NF; i++) { split($i, kv, "="); v[kv[1]] = kv[2] }
    runs++; frames += v["frames"]; mism += v["mismatch_frames"]; bad += v["mismatch_frames"] > 0 }
  END { printf "power runs %d, frames %d, mismatching frames %d, runs with mismatches %d\n", runs, frames, mism, bad }'
grep -h 'error=' "$out"/*.unit*.txt
grep -E '^(PASS|FAIL|SELF-TEST)' "$out/self-test.txt"
status=0
grep -q 'mismatch_frames=[1-9]\|error=' "$out"/*.unit*.txt && status=1
grep -q '^SELF-TEST PASSED' "$out/self-test.txt" || status=1
echo "results: $out"
exit $status
