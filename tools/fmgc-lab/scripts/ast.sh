#!/bin/bash
# Dumps the clang AST (JSON) of one function of a generated model, for scripts/slice.py. Runs clang in the dev container.
# Usage: scripts/ast.sh <variant> <FmgcComputer|FmgcOuterLoops>   ->  build/ast/<variant>-<model>.json
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
variant=$1
model=$2
dir="tools/fmgc-lab/build/models/$variant"
if [ ! -f "$repo/$dir/$model.cpp" ]; then
  echo "no $dir/$model.cpp (run scripts/build.sh or scripts/extract-model.sh first)" >&2
  exit 1
fi
mkdir -p "$here/build/ast"
cd "$repo"
./scripts/dev-env/run.sh bash -c "clang++ -std=c++17 -fsyntax-only -Xclang -ast-dump=json \
  -Xclang -ast-dump-filter=$model::step -I $dir -I fbw-a32nx/src/wasm/fbw_a320/src $dir/$model.cpp \
  > tools/fmgc-lab/build/ast/$variant-$model.json"
echo "build/ast/$variant-$model.json"
