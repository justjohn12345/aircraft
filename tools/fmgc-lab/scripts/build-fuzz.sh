#!/bin/bash
# Builds the clang fuzzing/coverage tools (CMake FMGC_FUZZ block) for the stock model into build/cmake-clang, in the FBW
# dev container, and copies them to build/: fmgc-fuzz, fmgc-fuzz-run, fmgc-fuzz-bank, fmgc-cov-drive,
# fmgc-cov-fuzz-run (static binaries, run on the host). Also builds the gcc corpus driver (build/fmgc-drive-stock)
# in the regular gcc build dir. The stock model is extracted from master unless build/models/stock exists.
# FCU_ONLY=1: only the FCU coverage checker (build/fcu-cov-check, see scripts/check-fcu-corpus.sh).
# Usage: [JOBS=n] [REF=master] [FCU_ONLY=1] scripts/build-fuzz.sh
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
ref=${REF:-master}
if [ ! -f "$here/build/models/stock/FmgcComputer.cpp" ]; then
  "$here/scripts/extract-model.sh" "$ref" stock
fi
jobs=${JOBS:-4}
targets="fmgc-fuzz fmgc-fuzz-run fmgc-fuzz-bank fmgc-cov-drive fmgc-cov-fuzz-run fcu-cov-check"
if [ -n "${FCU_ONLY:-}" ]; then
  cd "$repo"
  ./scripts/dev-env/run.sh bash -c "set -e; cd tools/fmgc-lab && \
    cmake -S . -B build/cmake-clang -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DFMGC_FUZZ=ON \
      -DMODEL_VARIANTS='stock=/external/tools/fmgc-lab/build/models/stock' >/dev/null && \
    cmake --build build/cmake-clang -j $jobs --target fcu-cov-check && cp build/cmake-clang/fcu-cov-check build/"
  exit 0
fi
cd "$repo"
./scripts/dev-env/run.sh bash -c "set -e; cd tools/fmgc-lab && \
  cmake -S . -B build/cmake-clang -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DFMGC_FUZZ=ON \
    -DMODEL_VARIANTS='stock=/external/tools/fmgc-lab/build/models/stock' >/dev/null && \
  cmake --build build/cmake-clang -j $jobs --target $targets && \
  for t in $targets; do cp build/cmake-clang/\$t build/; done && \
  if [ -f build/cmake/CMakeCache.txt ]; then cmake -S . -B build/cmake >/dev/null; else \
    cmake -S . -B build/cmake -G Ninja -DMODEL_VARIANTS='stock=/external/tools/fmgc-lab/build/models/stock' >/dev/null; fi && \
  cmake --build build/cmake -j $jobs --target fmgc-drive-stock && cp build/cmake/fmgc-drive-stock build/"
