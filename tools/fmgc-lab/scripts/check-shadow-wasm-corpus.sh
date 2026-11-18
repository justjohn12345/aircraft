#!/bin/bash
# The shadow check of scripts/check-shadow-corpus.sh, but in wasm: build/wasm/fmgc-shadow-test.wasm
# (scripts/build-shadow-wasm.sh: MSFS wasi-sysroot libc/libm, fbw.wasm's compiler flags) run with node's WASI in the dev
# container, the corpus mounted read-only. Every recording, FMGC 1 and 2, with --time and with --power 1; the self-test;
# the FCU (--time, --power 1), FADEC (--time) and FAC (--time, --power 1) shadow tests (build/wasm/fcu-, fadec-,
# fac-shadow-test.wasm) and their self-tests;
# the stack depth of the generated model alone and with the shadow. Logs go to build/check-shadow-wasm/.
# Usage: [JOBS=n] [CORPUS=dir] scripts/check-shadow-wasm-corpus.sh
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
corpus=$(cd "${CORPUS:-$HOME/fdr-corpus/raw}" && pwd)
image=$(grep -m1 '^IMAGE=' "$repo/scripts/dev-env/run.sh" | cut -d'"' -f2)
wasm=tools/fmgc-lab/build/wasm/fmgc-shadow-test.wasm
[ -f "$repo/$wasm" ] || { echo "error: $wasm missing (scripts/build-shadow-wasm.sh)" >&2; exit 2; }
fcu_wasm=tools/fmgc-lab/build/wasm/fcu-shadow-test.wasm
fadec_wasm=tools/fmgc-lab/build/wasm/fadec-shadow-test.wasm
fac_wasm=tools/fmgc-lab/build/wasm/fac-shadow-test.wasm
out=tools/fmgc-lab/build/check-shadow-wasm
rm -rf "${repo:?}/$out"
# the FCU / FADEC / FAC self-tests on a recording with recorded FCU and FADEC model inputs if there is one (3209004+)
selftest_file=
for path in "$corpus"/*.fdr; do
  [ -z "$selftest_file" ] && selftest_file=/corpus/$(basename "$path")
  version=$(od -An -t u8 -N 8 <(gzip -dc "$path" 2>/dev/null) | tr -d ' ')
  if [ "$version" = 3209004 ] || [ "$version" = 3209005 ]; then
    selftest_file=/corpus/$(basename "$path")
    break
  fi
done

docker run --rm -v "$repo":/external -v "$corpus":/corpus:ro -w /external "$image" bash -c "
  set -u
  mkdir -p $out/self-test
  mkdir -p $out/fcu-self-test $out/fadec-self-test $out/fac-self-test
  # runm NAME MODULE ARGS...
  runm() {
    local name=\$1 module=\$2
    shift 2
    while [ \"\$(jobs -r | wc -l)\" -ge ${JOBS:-8} ]; do wait -n; done
    node tools/fmgc-lab/scripts/wasi-run.mjs /external/\$module \"\$@\" > $out/\$name.txt 2>&1 &
  }
  run() {
    local name=\$1
    shift
    runm \$name $wasm \"\$@\"
  }
  files=(/corpus/*.fdr)
  for path in \"\${files[@]}\"; do
    f=\$(basename \"\$path\" .fdr)
    for unit in 1 2; do
      run \$f.unit\$unit --unit \$unit --time --log /external/$out/\$f.unit\$unit.log \"\$path\"
      run \$f.unit\$unit.power1 --unit \$unit --power 1 --log /external/$out/\$f.unit\$unit.power1.log \"\$path\"
    done
    if [ -f $fcu_wasm ]; then
      runm \$f.fcu $fcu_wasm --time --log /external/$out/\$f.fcu.log \"\$path\"
      runm \$f.fcu.power1 $fcu_wasm --power 1 --log /external/$out/\$f.fcu.power1.log \"\$path\"
      runm \$f.fadec $fadec_wasm --time --log /external/$out/\$f.fadec.log \"\$path\"
    fi
    if [ -f $fac_wasm ]; then
      runm \$f.fac $fac_wasm --time --log /external/$out/\$f.fac.log \"\$path\"
      runm \$f.fac.power1 $fac_wasm --power 1 --log /external/$out/\$f.fac.power1.log \"\$path\"
    fi
  done
  run self-test --self-test --log-dir /external/$out/self-test \"\${files[0]}\"
  if [ -f $fcu_wasm ]; then
    runm fcu-self-test $fcu_wasm --self-test --log-dir /external/$out/fcu-self-test \"$selftest_file\"
    runm fadec-self-test $fadec_wasm --self-test --log-dir /external/$out/fadec-self-test \"$selftest_file\"
  fi
  if [ -f $fac_wasm ]; then
    runm fac-self-test $fac_wasm --self-test --log-dir /external/$out/fac-self-test \"$selftest_file\"
  fi
  run stack-generated --no-shadow --stack \"\${files[@]}\"
  run stack-shadow --stack \"\${files[@]}\"
  wait
  chmod -R a+rwX $out
"

cd "$repo/$out" || exit 2
cat ./*.unit?.power1.txt | grep '^RESULT' | awk '
  { for (i = 2; i <= NF; i++) { split($i, kv, "="); v[kv[1]] = kv[2] }
    runs++; frames += v["frames"]; mism += v["mismatch_frames"]; bad += v["mismatch_frames"] > 0 }
  END { printf "wasm power runs %d, frames %d, mismatching frames %d, runs with mismatches %d\n", runs, frames, mism, bad }'
cat ./*.unit?.txt | grep '^RESULT' | awk '
  { for (i = 2; i <= NF; i++) { split($i, kv, "="); v[kv[1]] = kv[2] }
    runs++; frames += v["frames"]; mism += v["mismatch_frames"]; bad += v["mismatch_frames"] > 0
    gen += v["us_generated"] * v["frames"]; sh += v["us_shadow"] * v["frames"]; bare += v["us_readable_bare"] * v["frames"] }
  END { printf "wasm plain runs %d, frames %d, mismatching frames %d, runs with mismatches %d\n", runs, frames, mism, bad
        printf "wasm cost per frame (node, mean over frames): generated %.3f us, shadow %.3f us (readable step %.3f us + comparison %.3f us)\n",
               gen / frames, sh / frames, bare / frames, (sh - bare) / frames }'
for kind in fcu fcu.power1 fadec fac fac.power1; do
  cat ./*.$kind.txt 2>/dev/null | grep '^RESULT' | awk -v label="wasm $kind" '
    { delete v; for (i = 2; i <= NF; i++) { split($i, kv, "="); v[kv[1]] = kv[2] }
      runs++; frames += v["frames"]; mism += v["mismatch_frames"]; bad += v["mismatch_frames"] > 0
      if ("us_generated" in v) { timed += v["frames"]; gen += v["us_generated"] * v["frames"]; sh += v["us_shadow"] * v["frames"];
                                 bare += v["us_readable_bare"] * v["frames"] } }
    END { if (runs == 0) exit
          printf "%s: runs %d, frames %d, mismatching frames %d, runs with mismatches %d\n", label, runs, frames, mism, bad
          if (timed > 0) printf "%s: cost per unit and frame (node): generated %.3f us, shadow %.3f us (readable step %.3f us + comparison %.3f us)\n",
                                label, gen / timed, sh / timed, bare / timed, (sh - bare) / timed }'
done
grep -h 'error=\|Error' ./*.txt
grep -E '^(PASS|FAIL|SELF-TEST)' self-test.txt
for t in fcu-self-test fadec-self-test fac-self-test; do
  [ -f $t.txt ] && grep -E '^(PASS|FAIL|SELF-TEST)' $t.txt | sed "s/^/$t: /"
done
grep -h '^STACK' stack-generated.txt stack-shadow.txt
status=0
grep -q 'mismatch_frames=[1-9]\|error=' ./*.txt && status=1
grep -q '^SELF-TEST PASSED' self-test.txt || status=1
for t in fcu-self-test fadec-self-test fac-self-test; do
  [ -f $t.txt ] && { grep -q '^SELF-TEST PASSED' $t.txt || status=1; }
done
echo "results: $repo/$out"
exit $status
