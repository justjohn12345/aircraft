#!/bin/bash
# The shadow check of scripts/check-shadow-corpus.sh, but in wasm: build/wasm/fmgc-shadow-test.wasm
# (scripts/build-shadow-wasm.sh: MSFS wasi-sysroot libc/libm, fbw.wasm's compiler flags) run with node's WASI in the dev
# container, the corpus mounted read-only. Every recording, FMGC 1 and 2, with --time and with --power 1; the self-test;
# the stack depth of the generated model alone and with the shadow. Logs go to build/check-shadow-wasm/.
# Usage: [JOBS=n] [CORPUS=dir] scripts/check-shadow-wasm-corpus.sh
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
corpus=$(cd "${CORPUS:-$HOME/fdr-corpus/raw}" && pwd)
image=$(grep -m1 '^IMAGE=' "$repo/scripts/dev-env/run.sh" | cut -d'"' -f2)
wasm=tools/fmgc-lab/build/wasm/fmgc-shadow-test.wasm
[ -f "$repo/$wasm" ] || { echo "error: $wasm missing (scripts/build-shadow-wasm.sh)" >&2; exit 2; }
out=tools/fmgc-lab/build/check-shadow-wasm
rm -rf "${repo:?}/$out"

docker run --rm -v "$repo":/external -v "$corpus":/corpus:ro -w /external "$image" bash -c "
  set -u
  mkdir -p $out/self-test
  run() {
    local name=\$1
    shift
    while [ \"\$(jobs -r | wc -l)\" -ge ${JOBS:-8} ]; do wait -n; done
    node tools/fmgc-lab/scripts/wasi-run.mjs /external/$wasm \"\$@\" > $out/\$name.txt 2>&1 &
  }
  files=(/corpus/*.fdr)
  for path in \"\${files[@]}\"; do
    f=\$(basename \"\$path\" .fdr)
    for unit in 1 2; do
      run \$f.unit\$unit --unit \$unit --time --log /external/$out/\$f.unit\$unit.log \"\$path\"
      run \$f.unit\$unit.power1 --unit \$unit --power 1 --log /external/$out/\$f.unit\$unit.power1.log \"\$path\"
    done
  done
  run self-test --self-test --log-dir /external/$out/self-test \"\${files[0]}\"
  run stack-generated --no-shadow --stack \"\${files[@]}\"
  run stack-shadow --stack \"\${files[@]}\"
  wait
  chmod -R a+rwX $out
"

cd "$repo/$out" || exit 2
cat ./*.power1.txt | grep '^RESULT' | awk '
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
grep -h 'error=\|Error' ./*.txt
grep -E '^(PASS|FAIL|SELF-TEST)' self-test.txt
grep -h '^STACK' stack-generated.txt stack-shadow.txt
status=0
grep -q 'mismatch_frames=[1-9]\|error=' ./*.unit*.txt && status=1
grep -q '^SELF-TEST PASSED' self-test.txt || status=1
echo "results: $repo/$out"
exit $status
