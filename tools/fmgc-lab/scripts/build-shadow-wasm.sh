#!/bin/bash
# Builds build/wasm/fmgc-shadow-test.wasm: the shadow test driver (src/shadow_test.cpp) with the sim's FmgcShadow, the
# readable FMGC and the generated FMGC as a wasm32-wasi command module, with the MSFS SDK's wasi-sysroot (the libc, libm
# and libc++ fbw.wasm links) and the compiler flags of fbw-a32nx/src/wasm/fbw_a320/build.sh (-flto -O2; the generated
# model as in its main compile, the readable FMGC and the shadow as in its shadow compile). Runs in the dev container.
# Run it with scripts/check-shadow-wasm-corpus.sh (node's WASI). Needs build/cmake/gen-stock/fields.gen.h
# (scripts/build.sh stock=...).
# Usage: scripts/build-shadow-wasm.sh
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"

if [ "${1:-}" != "--in-container" ]; then
  cd "$repo"
  exec ./scripts/dev-env/run.sh bash -c "tools/fmgc-lab/scripts/build-shadow-wasm.sh --in-container"
fi

cd "$repo"
lab=tools/fmgc-lab
wasm_src=fbw-a32nx/src/wasm/fbw_a320/src
zlib=fbw-common/src/wasm/fbw_common/src/zlib
gen=$lab/build/cmake/gen-stock
out=$lab/build/wasm
[ -f "$gen/fields.gen.h" ] || { echo "error: $gen/fields.gen.h missing (run scripts/build.sh stock=HEAD)" >&2; exit 2; }
rm -rf "$out/obj" && mkdir -p "$out/obj/zlib" "$out/obj/model" "$out/obj/shadow"
sysroot="$MSFS_SDK/WASM/wasi-sysroot"
flags=(-flto -O2 -DNDEBUG -target wasm32-unknown-wasi --sysroot "$sysroot" -D_MSFS_WASM=1 -D__wasi__ -D_LARGEFILE64_SOURCE
  -D_LIBCPP_HAS_NO_THREADS -D_WINDLL -D_MBCS -mthread-model single -fno-exceptions -fms-extensions -fvisibility=hidden
  -fdata-sections -fno-stack-protector -fstack-size-section -mbulk-memory -Werror=return-type
  -Wno-unused-command-line-argument)
cxx=(-std=c++20 -DNOMINMAX -Wno-ignored-attributes -Wno-macro-redefined)

(cd "$out/obj/zlib" && clang -c "${flags[@]}" -Wno-implicit-function-declaration -Wno-deprecated-non-prototype \
  -I "$repo/$zlib" "$repo/$zlib"/{adler32,crc32,deflate,gzclose,gzlib,gzread,gzwrite,infback,inffast,inflate,inftrees,trees,zutil}.c)

# the generated model, as fbw_a320/build.sh's main compile
(cd "$out/obj/model" && clang++ -c "${flags[@]}" "${cxx[@]}" -I "$repo/$wasm_src/model" \
  "$repo/$wasm_src/model"/{FmgcComputer,FmgcComputer_data,FmgcOuterLoops,look1_binlxpw,look1_iflf_binlxpw,look2_binlxpw,look2_pbinlxpw,rt_modd}.cpp)

# the readable FMGC and the shadow, as fbw_a320/build.sh's shadow compile; the test driver with them
readable=()
for source in "$repo/$lab"/readable/*.cpp; do
  [[ "$source" == *_test.cpp ]] || readable+=("$source")
done
(cd "$out/obj/shadow" && clang++ -c "${flags[@]}" "${cxx[@]}" -ffp-contract=off -Wno-deprecated-enum-float-conversion \
  -I "$repo/$wasm_src/model" -I "$repo/$lab" -I "$repo/$wasm_src/fmgc" -I "$repo/$wasm_src" -I "$repo/$gen" \
  -I "$repo/$lab/src" -I "$repo/$zlib" \
  "$repo/$wasm_src/fmgc/FmgcShadow.cpp" "${readable[@]}" "$repo/$lab/src/shadow_test.cpp")

# 8 MiB stack for the harness (fbw.wasm has wasm-ld's default 64 KiB; --stack reports the depth actually used)
lib="$sysroot/lib/wasm32-wasi"
wasm-ld -O2 --lto-O2 --strip-debug --gc-sections -z stack-size=8388608 -L "$lib" "$lib/crt1-command.o" \
  "$out"/obj/zlib/*.o "$out"/obj/model/*.o "$out"/obj/shadow/*.o -lc++ -lc++abi -lc "$lib/libclang_rt.builtins-wasm32.a" \
  -o "$out/fmgc-shadow-test.wasm"
ls -l "$out/fmgc-shadow-test.wasm"
