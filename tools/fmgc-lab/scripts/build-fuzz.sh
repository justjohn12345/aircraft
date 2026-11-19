#!/bin/bash
# Builds the clang fuzzing/coverage tools (CMake FMGC_FUZZ block) for the stock model into build/cmake-clang, in the FBW
# dev container, and copies them to build/: fmgc-fuzz, fmgc-fuzz-run, fmgc-fuzz-bank, fmgc-cov-drive,
# fmgc-cov-fuzz-run (static binaries, run on the host). Also builds the gcc corpus driver (build/fmgc-drive-stock)
# in the regular gcc build dir. The stock model is extracted from master unless build/models/stock exists.
# FCU_ONLY=1: only the FCU coverage checker (build/fcu-cov-check, see scripts/check-fcu-corpus.sh).
# FADEC_ONLY=1: only the FADEC coverage checker (build/fadec-cov-check, see scripts/check-fadec-corpus.sh).
# FAC_ONLY=1: only the FAC coverage checker (build/fac-cov-check, see scripts/check-fac-corpus.sh).
# PITCH_ONLY=1: only the EFCS pitch law coverage checker (build/efcs-pitch-law-cov-check, scripts/pitch-law-coverage.sh).
# EFCS_ONLY=1: only the ELAC and SEC coverage checkers (build/elac-cov-check, build/sec-cov-check,
# scripts/check-efcs-corpus.sh with COVERAGE=1).
# Usage: [JOBS=n] [REF=master] [FCU_ONLY=1 | FADEC_ONLY=1 | FAC_ONLY=1 | PITCH_ONLY=1 | EFCS_ONLY=1] scripts/build-fuzz.sh
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
ref=${REF:-master}
if [ ! -f "$here/build/models/stock/FmgcComputer.cpp" ]; then
  "$here/scripts/extract-model.sh" "$ref" stock
fi
jobs=${JOBS:-4}
targets="fmgc-fuzz fmgc-fuzz-run fmgc-fuzz-bank fmgc-cov-drive fmgc-cov-fuzz-run fcu-cov-check fadec-cov-check fac-cov-check"
only=""
[ -n "${FCU_ONLY:-}" ] && only=fcu-cov-check
[ -n "${FADEC_ONLY:-}" ] && only=fadec-cov-check
[ -n "${FAC_ONLY:-}" ] && only=fac-cov-check
[ -n "${PITCH_ONLY:-}" ] && only=efcs-pitch-law-cov-check
[ -n "${EFCS_ONLY:-}" ] && only="elac-cov-check sec-cov-check"
if [ -n "$only" ]; then
  cd "$repo"
  ./scripts/dev-env/run.sh bash -c "set -e; cd tools/fmgc-lab && \
    cmake -S . -B build/cmake-clang -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DFMGC_FUZZ=ON \
      -DMODEL_VARIANTS='stock=/external/tools/fmgc-lab/build/models/stock' >/dev/null && \
    cmake --build build/cmake-clang -j $jobs --target $only && for t in $only; do cp build/cmake-clang/\$t build/; done"
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
