#!/bin/bash
# Runs the sim's shadow checkers over every recording:
#  - FMGC (fbw_a320/src/fmgc/FmgcShadow.cpp, via build/fmgc-shadow-test-stock): FMGC 1 and FMGC 2 (FMGC 1's recorded
#    inputs with is_unit_1 false), plain and with POWER_SEEDS (default 1) seeds of power / health episodes (--power:
#    restarts), its self-test (leaf table, bit flips, input perturbation) on one recording, and the readable-primary test
#    (phase 6b, --primary-test: the FMGC wrapper's outputs identical with either computer primary) per recording and unit,
#    plain and with power seed 1;
#  - FCU and FADEC (fbw_a320/src/shadow/, via build/fcu-shadow-test-stock and build/fadec-shadow-test-stock): every
#    recording (recorded model inputs on 3209001+ / 3209004 files, the synthetic drive on the others), the FCU also with
#    POWER_SEEDS seeds of power episodes, and both self-tests;
#  - FAC (fbw_a320/src/shadow/FacShadow.cpp, via build/fac-shadow-test-stock): FAC 1 and 2 on every recording (recorded
#    model inputs on 3209005 files, else the inputs rebuilt from the record), also with POWER_SEEDS seeds of power
#    episodes, and its self-test (with the NaN rule).
# Logs in the sim's format go to build/check-shadow/. The generated stock models are the oracle, so every recording works
# whatever build flew it.
# Usage: [JOBS=n] [CORPUS=dir] [ONLY=fmgc|fcu|fadec|fac|primary] scripts/check-shadow-corpus.sh [FMGC binary, default
#        build/fmgc-shadow-test-stock]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
bin=${1:-$here/build/fmgc-shadow-test-stock}
fcu_bin=$(dirname "$bin")/fcu-shadow-test-stock
fadec_bin=$(dirname "$bin")/fadec-shadow-test-stock
fac_bin=$(dirname "$bin")/fac-shadow-test-stock
corpus=${CORPUS:-$HOME/fdr-corpus/raw}
only=${ONLY:-}
wasm_src="$here/../../fbw-a32nx/src/wasm/fbw_a320/src"
for b in "$bin" "$fcu_bin" "$fadec_bin" "$fac_bin"; do
  if [ -n "$(find "$here/readable" "$here/src" "$wasm_src/fmgc" "$wasm_src/shadow" -newer "$b" \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
    echo "error: $b is older than the sources (rebuild failed or not run)" >&2
    exit 2
  fi
done
out=${OUT:-$here/build/check-shadow}
rm -rf "$out" && mkdir -p "$out/self-test" "$out/fcu-self-test" "$out/fadec-self-test" "$out/fac-self-test"
# run NAME BINARY ARGS...: one run in the background, at most JOBS (default 8) at a time
run() {
  local name=$1
  shift
  while [ "$(jobs -r | wc -l)" -ge "${JOBS:-8}" ]; do wait -n; done
  "$@" > "$out/$name.txt" 2>&1 &
}
want() { [ -z "$only" ] || [ "$only" = "$1" ]; }
files=("$corpus"/*.fdr)
for path in "${files[@]}"; do
  f=$(basename "$path" .fdr)
  for unit in 1 2; do
    if want fmgc; then
      run "$f.unit$unit" "$bin" --unit "$unit" --time --log "$out/$f.unit$unit.log" "$path"
      for ((seed = 1; seed <= ${POWER_SEEDS:-1}; seed++)); do
        run "$f.unit$unit.power$seed" "$bin" --unit "$unit" --power "$seed" --log "$out/$f.unit$unit.power$seed.log" "$path"
      done
    fi
    if want primary; then
      run "$f.primary$unit" "$bin" --primary-test --unit "$unit" --log "$out/$f.primary$unit.log" "$path"
      run "$f.primary$unit.power1" "$bin" --primary-test --unit "$unit" --power 1 --log "$out/$f.primary$unit.power1.log" "$path"
    fi
  done
  if want fcu; then
    run "$f.fcu" "$fcu_bin" --time --log "$out/$f.fcu.log" "$path"
    for ((seed = 1; seed <= ${POWER_SEEDS:-1}; seed++)); do
      run "$f.fcu.power$seed" "$fcu_bin" --power "$seed" --log "$out/$f.fcu.power$seed.log" "$path"
    done
  fi
  if want fadec; then
    run "$f.fadec" "$fadec_bin" --time --log "$out/$f.fadec.log" "$path"
  fi
  if want fac; then
    run "$f.fac" "$fac_bin" --time --log "$out/$f.fac.log" "$path"
    for ((seed = 1; seed <= ${POWER_SEEDS:-1}; seed++)); do
      run "$f.fac.power$seed" "$fac_bin" --power "$seed" --log "$out/$f.fac.power$seed.log" "$path"
    done
  fi
done
# the self-tests on a recording with recorded FCU and FADEC (and FAC) model inputs if there is one (3209004 or 3209005),
# else the first
selftest_file=${files[0]}
for path in "${files[@]}"; do
  version=$(od -An -t u8 -N 8 <(gzip -dc "$path" 2>/dev/null) | tr -d ' ')
  if [ "$version" = 3209004 ] || [ "$version" = 3209005 ]; then
    selftest_file=$path
    [ "$version" = 3209005 ] && break
  fi
done
want fmgc && run self-test "$bin" --self-test --log-dir "$out/self-test" "$selftest_file"
want fcu && run fcu-self-test "$fcu_bin" --self-test --log-dir "$out/fcu-self-test" "$selftest_file"
want fadec && run fadec-self-test "$fadec_bin" --self-test --log-dir "$out/fadec-self-test" "$selftest_file"
want fac && run fac-self-test "$fac_bin" --self-test --log-dir "$out/fac-self-test" "$selftest_file"
wait

summary() {  # summary LABEL GLOB: totals of the RESULT lines (and the mean cost over frames when --time was given)
  cat $2 2>/dev/null | grep '^RESULT' | awk -v label="$1" '
    { delete v; for (i = 2; i <= NF; i++) { split($i, kv, "="); v[kv[1]] = kv[2] }
      runs++; frames += v["frames"]; mism += v["mismatch_frames"]; bad += v["mismatch_frames"] > 0
      if ("us_generated" in v) { timed += v["frames"]; gen += v["us_generated"] * v["frames"]; sh += v["us_shadow"] * v["frames"];
                                 bare += v["us_readable_bare"] * v["frames"] } }
    END { if (runs == 0) exit
          printf "%s: runs %d, frames %d, mismatching frames %d, runs with mismatches %d\n", label, runs, frames, mism, bad
          if (timed > 0) printf "%s: cost per unit and frame (mean over frames): generated %.3f us, shadow %.3f us (readable step %.3f us + comparison %.3f us)\n",
                                label, gen / timed, sh / timed, bare / timed, (sh - bare) / timed }'
}
summary "FMGC plain" "$out/*.unit?.txt"
summary "FMGC power" "$out/*.unit?.power*.txt"
summary "FCU plain" "$out/*.fcu.txt"
summary "FCU power" "$out/*.fcu.power*.txt"
summary "FADEC" "$out/*.fadec.txt"
summary "FAC plain" "$out/*.fac.txt"
summary "FAC power" "$out/*.fac.power*.txt"
if want primary; then
  echo "FMGC readable primary: $(grep -l '^PRIMARY-TEST PASSED' "$out"/*.primary*.txt | wc -l) of $(ls "$out"/*.primary*.txt | wc -l) runs passed"
  grep -h '^INFO' "$out"/*.primary*.txt | sed -E 's/.*: ([0-9]+) frames, ([0-9]+) healthy, ([0-9]+) health changes/\1 \2 \3/' |
    awk '{ frames += $1; healthy += $2; changes += $3 }
    END { printf "FMGC readable primary: %d frames (%d healthy, %d health changes), wrapper outputs compared every frame\n", frames, healthy, changes }'
fi
grep -h 'error=' "$out"/*.txt
for t in self-test fcu-self-test fadec-self-test fac-self-test; do
  [ -f "$out/$t.txt" ] && grep -E '^(PASS|FAIL|SELF-TEST)' "$out/$t.txt" | sed "s/^/$t: /"
done
status=0
grep -q 'mismatch_frames=[1-9]\|error=' "$out"/*.txt && status=1
for t in self-test fcu-self-test fadec-self-test fac-self-test; do
  [ -f "$out/$t.txt" ] && { grep -q '^SELF-TEST PASSED' "$out/$t.txt" || status=1; }
done
if want primary; then
  for f in "$out"/*.primary*.txt; do grep -q '^PRIMARY-TEST PASSED' "$f" || { echo "FAILED: $f"; status=1; }; done
fi
echo "results: $out"
exit $status
