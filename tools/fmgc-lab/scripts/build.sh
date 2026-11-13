#!/bin/bash
# Builds one static replay binary per model variant, in the FBW dev container (no host compiler needed).
# Variants: name=git-ref pairs. The binaries in build/ run on the host.
# Usage: scripts/build.sh [name=ref ...]
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
variants=("$@")
if [ ${#variants[@]} -eq 0 ]; then
  # stock = upstream master; cst = + ALT CST* sequencing patch; cst_athr = + FINAL A/THR patch (test/combined)
  variants=(stock=master cst=personal/fg-alt-cst-reversion cst_athr=test/combined)
fi
list=""
for v in "${variants[@]}"; do
  name=${v%%=*}
  ref=${v#*=}
  "$here/scripts/extract-model.sh" "$ref" "$name"
  list="$list;$name=/external/tools/fmgc-lab/build/models/$name"
done
list=${list#;}
cd "$repo"
./scripts/dev-env/run.sh bash -c "cd tools/fmgc-lab && cmake -S . -B build/cmake -G Ninja -DMODEL_VARIANTS='$list' >/dev/null && cmake --build build/cmake && cp build/cmake/fmgc-replay-* build/cmake/fmgc-dump-params-* build/cmake/fmgc-blocks-test-* build/cmake/fmgc-probe-* build/"
