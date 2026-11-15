# Native (host) build of the generated A32NX FMGC with a flight data recorder replay harness.
# Build in the dev container: tools/fmgc-lab/scripts/build.sh
cmake_minimum_required(VERSION 3.20)
project(fmgc_lab CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(WASM_SRC ${REPO_ROOT}/fbw-a32nx/src/wasm/fbw_a320/src)

set(ZLIB_USE_STATIC_LIBS ON)
find_package(ZLIB REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# One replay executable per model variant: MODEL_VARIANTS is a list of name=dir pairs, where dir holds the generated
# model sources (e.g. extracted from a git ref by scripts/extract-model.sh). Default: the working tree.
if(NOT MODEL_VARIANTS)
  set(MODEL_VARIANTS "worktree=${WASM_SRC}/model")
endif()

foreach(variant ${MODEL_VARIANTS})
  string(REPLACE "=" ";" parts ${variant})
  list(GET parts 0 name)
  list(GET parts 1 dir)

  set(gen_dir ${CMAKE_CURRENT_BINARY_DIR}/gen-${name})
  add_custom_command(
    OUTPUT ${gen_dir}/fields.gen.h
    COMMAND ${CMAKE_COMMAND} -E make_directory ${gen_dir}
    COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_fields.py ${dir}/FmgcComputer_types.h ${gen_dir}/fields.gen.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_fields.py ${dir}/FmgcComputer_types.h)

  add_library(fmgc_model_${name} STATIC
    ${dir}/FmgcComputer.cpp
    ${dir}/FmgcComputer_data.cpp
    ${dir}/FmgcOuterLoops.cpp
    ${dir}/look1_binlxpw.cpp
    ${dir}/look1_iflf_binlxpw.cpp
    ${dir}/look2_binlxpw.cpp
    ${dir}/look2_pbinlxpw.cpp
    ${dir}/rt_modd.cpp)
  target_include_directories(fmgc_model_${name} PUBLIC ${dir})
  # Keep IEEE semantics close to the wasm build (no fast-math, no FMA contraction).
  target_compile_options(fmgc_model_${name} PRIVATE -O2 -ffp-contract=off)

  add_executable(fmgc-replay-${name} src/replay.cpp ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-replay-${name} PRIVATE ${gen_dir} ${WASM_SRC} src)
  target_compile_options(fmgc-replay-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-replay-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)

  # Parameter dictionary (block name = value) and the readable block library's differential test.
  add_custom_command(
    OUTPUT ${gen_dir}/params.gen.h
    COMMAND ${CMAKE_COMMAND} -E make_directory ${gen_dir}
    COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_params.py ${gen_dir}/params.gen.h
            FmgcComputer=${dir}/FmgcComputer.h FmgcOuterLoops=${dir}/FmgcOuterLoops.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_params.py ${dir}/FmgcComputer.h ${dir}/FmgcOuterLoops.h)

  add_executable(fmgc-dump-params-${name} src/dump_params.cpp ${gen_dir}/params.gen.h)
  target_include_directories(fmgc-dump-params-${name} PRIVATE ${gen_dir} ${WASM_SRC})
  target_link_libraries(fmgc-dump-params-${name} PRIVATE fmgc_model_${name} -static)

  add_executable(fmgc-probe-zero-vs-${name} src/probe_zero_vs.cpp)
  target_include_directories(fmgc-probe-zero-vs-${name} PRIVATE ${WASM_SRC} src)
  target_compile_options(fmgc-probe-zero-vs-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-probe-zero-vs-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)

  add_executable(fmgc-probe-alt-jump-${name} src/probe_alt_jump.cpp)
  target_include_directories(fmgc-probe-alt-jump-${name} PRIVATE ${WASM_SRC} src)
  target_compile_options(fmgc-probe-alt-jump-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-probe-alt-jump-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)

  add_executable(fmgc-probe-alt-capture-${name} src/probe_alt_capture.cpp)
  target_include_directories(fmgc-probe-alt-capture-${name} PRIVATE ${WASM_SRC} src)
  target_compile_options(fmgc-probe-alt-capture-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-probe-alt-capture-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)

  add_executable(fmgc-check-vertical-${name} src/check_vertical.cpp readable/vertical.cpp readable/vertical_alt.cpp
    readable/vertical_vs_tcas.cpp readable/vertical_climb_descent.cpp readable/vertical_approach.cpp readable/lateral_modes.cpp
    readable/targets.cpp ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-check-vertical-${name} PRIVATE ${gen_dir} ${WASM_SRC} src readable)
  target_compile_options(fmgc-check-vertical-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-check-vertical-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)

  add_executable(fmgc-check-athr-${name} src/check_athr.cpp readable/athr.cpp readable/athr_modes.cpp
    readable/athr_n1.cpp ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-check-athr-${name} PRIVATE ${gen_dir} ${WASM_SRC} src readable)
  target_compile_options(fmgc-check-athr-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-check-athr-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)
  # AP / FD / A/THR engagement check (Phase 5c)
  add_executable(fmgc-check-engagement-${name} src/check_engagement.cpp readable/engagement.cpp
    readable/engagement_land.cpp ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-check-engagement-${name} PRIVATE ${gen_dir} ${WASM_SRC} src readable)
  target_compile_options(fmgc-check-engagement-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-check-engagement-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)

  add_executable(fmgc-blocks-test-${name} readable/blocks_test.cpp ${gen_dir}/params.gen.h)
  target_include_directories(fmgc-blocks-test-${name} PRIVATE ${gen_dir} ${WASM_SRC} readable)
  target_compile_options(fmgc-blocks-test-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-blocks-test-${name} PRIVATE fmgc_model_${name} -static)
endforeach()

# Targets block check (Phase 5d): HDG/TRK preset, selected/managed speed, Mach, speed targets, ALT soft mode.
foreach(variant ${MODEL_VARIANTS})
  string(REPLACE "=" ";" parts ${variant})
  list(GET parts 0 name)
  set(gen_dir ${CMAKE_CURRENT_BINARY_DIR}/gen-${name})
  add_executable(fmgc-check-targets-${name} src/check_targets.cpp readable/targets.cpp ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-check-targets-${name} PRIVATE ${gen_dir} ${WASM_SRC} src readable)
  target_compile_options(fmgc-check-targets-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-check-targets-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)
endforeach()

# Input conditioning check (Phase 5e): source selection, voting and failure flags (the logic bus).
# Output packing check (Phase 5f): FMGC A/B buses, discrete words, discrete outputs, fmgc_outputs assembly and the
# end-of-step delays.
foreach(variant ${MODEL_VARIANTS})
  string(REPLACE "=" ";" parts ${variant})
  list(GET parts 0 name)
  set(gen_dir ${CMAKE_CURRENT_BINARY_DIR}/gen-${name})
  add_executable(fmgc-check-conditioning-${name} src/check_conditioning.cpp readable/conditioning.cpp
    ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-check-conditioning-${name} PRIVATE ${gen_dir} ${WASM_SRC} src readable)
  target_compile_options(fmgc-check-conditioning-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-check-conditioning-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)
  add_executable(fmgc-check-output-${name} src/check_output.cpp readable/output_packing.cpp ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-check-output-${name} PRIVATE ${gen_dir} ${WASM_SRC} src readable)
  target_compile_options(fmgc-check-output-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-check-output-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)
endforeach()

# The readable computer (Phase 5 integration): every readable stage chained into readable/fmgc.cpp, and its
# whole-computer free-running differential check against the generated model.
set(READABLE_COMPUTER_SOURCES
  readable/fmgc.cpp readable/conditioning.cpp readable/engagement.cpp readable/engagement_land.cpp readable/tcas.cpp
  readable/vertical.cpp readable/vertical_alt.cpp readable/vertical_vs_tcas.cpp readable/vertical_climb_descent.cpp
  readable/vertical_approach.cpp readable/lateral_modes.cpp readable/targets.cpp readable/law_selection.cpp
  readable/outer_loop_inputs.cpp readable/laws_output.cpp readable/laws_alt.cpp readable/laws_vs_fpa.cpp
  readable/laws_speed.cpp readable/laws_path.cpp readable/laws_lateral.cpp readable/athr.cpp readable/athr_modes.cpp
  readable/athr_n1.cpp readable/output_packing.cpp)
foreach(variant ${MODEL_VARIANTS})
  string(REPLACE "=" ";" parts ${variant})
  list(GET parts 0 name)
  list(GET parts 1 dir)
  set(gen_dir ${CMAKE_CURRENT_BINARY_DIR}/gen-${name})
  add_library(fmgc_readable_${name} STATIC ${READABLE_COMPUTER_SOURCES})
  target_include_directories(fmgc_readable_${name} PUBLIC ${dir} ${WASM_SRC} readable)
  target_compile_options(fmgc_readable_${name} PRIVATE -O2 -ffp-contract=off)
  add_executable(fmgc-check-fmgc-${name} src/check_fmgc.cpp ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-check-fmgc-${name} PRIVATE ${gen_dir} ${WASM_SRC} src readable)
  target_compile_options(fmgc-check-fmgc-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-check-fmgc-${name} PRIVATE fmgc_readable_${name} fmgc_model_${name} ZLIB::ZLIB -static)
endforeach()

# Control-law checks (Phase 3): an instrumented copy of the stock model (outer-loop call hooks and probes), never the
# repo's model/ folder. Built when a variant named "stock" exists.
foreach(variant ${MODEL_VARIANTS})
  string(REPLACE "=" ";" parts ${variant})
  list(GET parts 0 name)
  list(GET parts 1 dir)
  if(NOT name STREQUAL "stock")
    continue()
  endif()
  set(hooked_dir ${CMAKE_CURRENT_BINARY_DIR}/hooked-stock)
  file(GLOB probe_files ${CMAKE_CURRENT_SOURCE_DIR}/src/probes/*.txt)
  add_custom_command(
    OUTPUT ${hooked_dir}/FmgcComputer.cpp ${hooked_dir}/FmgcOuterLoops.cpp ${hooked_dir}/outer_io.gen.h
    COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_hooked_model.py ${dir} ${hooked_dir} ${probe_files}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_hooked_model.py ${probe_files} ${dir}/FmgcComputer.cpp
            ${dir}/FmgcOuterLoops.cpp)
  add_library(fmgc_model_stock_hooked STATIC
    ${hooked_dir}/FmgcComputer.cpp
    ${dir}/FmgcComputer_data.cpp
    ${hooked_dir}/FmgcOuterLoops.cpp
    ${dir}/look1_binlxpw.cpp
    ${dir}/look1_iflf_binlxpw.cpp
    ${dir}/look2_binlxpw.cpp
    ${dir}/look2_pbinlxpw.cpp
    ${dir}/rt_modd.cpp)
  target_include_directories(fmgc_model_stock_hooked PUBLIC ${dir} ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_options(fmgc_model_stock_hooked PRIVATE -O2 -ffp-contract=off)

  add_executable(fmgc-check-laws src/check_laws.cpp readable/laws_output.cpp readable/laws_alt.cpp
    readable/laws_vs_fpa.cpp readable/laws_speed.cpp readable/laws_path.cpp readable/laws_lateral.cpp
    ${hooked_dir}/outer_io.gen.h
    ${CMAKE_CURRENT_BINARY_DIR}/gen-stock/fields.gen.h)
  target_include_directories(fmgc-check-laws PRIVATE ${hooked_dir} ${CMAKE_CURRENT_BINARY_DIR}/gen-stock ${WASM_SRC} src
    readable)
  target_compile_options(fmgc-check-laws PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-check-laws PRIVATE fmgc_model_stock_hooked ZLIB::ZLIB -static)
endforeach()

# ---- Phase 4: corpus driver, fuzzing and coverage ---------------------------------------------------------------------
# The fuzz oracles use the readable computer library (fmgc_readable_<variant>): the output packing (stateless) and
# A/THR (resync) differential checks, and the whole-computer differential (FMGC_WHOLE_COMPUTER=1, src/fuzz_oracle.h).
set(FUZZ_DEFINES FMGC_WHOLE_COMPUTER=1)

# Corpus driver (any compiler): replay + scenario injectors + oracles (properties, differential checks).
foreach(variant ${MODEL_VARIANTS})
  string(REPLACE "=" ";" parts ${variant})
  list(GET parts 0 name)
  set(gen_dir ${CMAKE_CURRENT_BINARY_DIR}/gen-${name})
  add_executable(fmgc-drive-${name} src/drive.cpp ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-drive-${name} PRIVATE ${gen_dir} ${WASM_SRC} src readable)
  target_compile_definitions(fmgc-drive-${name} PRIVATE ${FUZZ_DEFINES})
  target_compile_options(fmgc-drive-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-drive-${name} PRIVATE fmgc_readable_${name} fmgc_model_${name} ZLIB::ZLIB -static)
endforeach()

# Clang-only targets (a separate build dir configured with -DFMGC_FUZZ=ON and clang++, see scripts/build-fuzz.sh):
#   fmgc-cov-drive, fmgc-cov-fuzz-run  source-based coverage (-fprofile-instr-generate -fcoverage-mapping) of
#                                      FmgcComputer.cpp and FmgcOuterLoops.cpp only
#   fmgc-fuzz                          libFuzzer harness; only the same two files carry the coverage instrumentation
#                                      (-fsanitize=fuzzer-no-link); the harness and the readable code are not instrumented
#   fmgc-fuzz-run, fmgc-fuzz-bank      standalone runner/minimiser/seed generator and the snapshot bank builder
# All model code keeps -O2 -ffp-contract=off. The model variant used is the one named "stock" (else the first).
option(FMGC_FUZZ "Build the clang fuzzing and coverage targets" OFF)
if(FMGC_FUZZ)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "FMGC_FUZZ needs clang (configure with -DCMAKE_CXX_COMPILER=clang++)")
  endif()
  set(fuzz_name "")
  foreach(variant ${MODEL_VARIANTS})
    string(REPLACE "=" ";" parts ${variant})
    list(GET parts 0 name)
    list(GET parts 1 dir)
    if(fuzz_name STREQUAL "" OR name STREQUAL "stock")
      set(fuzz_name ${name})
      set(fuzz_dir ${dir})
    endif()
  endforeach()
  set(fuzz_gen ${CMAKE_CURRENT_BINARY_DIR}/gen-${fuzz_name})
  set(model_core ${fuzz_dir}/FmgcComputer.cpp ${fuzz_dir}/FmgcOuterLoops.cpp)
  set(model_rest ${fuzz_dir}/FmgcComputer_data.cpp ${fuzz_dir}/look1_binlxpw.cpp ${fuzz_dir}/look1_iflf_binlxpw.cpp
    ${fuzz_dir}/look2_binlxpw.cpp ${fuzz_dir}/look2_pbinlxpw.cpp ${fuzz_dir}/rt_modd.cpp)

  # The two generated files compiled once per instrumentation (separate targets, so per-target flags apply).
  add_library(fmgc_model_rest OBJECT ${model_rest})
  target_include_directories(fmgc_model_rest PUBLIC ${fuzz_dir})
  target_compile_options(fmgc_model_rest PRIVATE -O2 -ffp-contract=off)
  add_library(fmgc_model_cov STATIC ${model_core} $<TARGET_OBJECTS:fmgc_model_rest>)
  target_include_directories(fmgc_model_cov PUBLIC ${fuzz_dir})
  target_compile_options(fmgc_model_cov PRIVATE -O2 -ffp-contract=off -fprofile-instr-generate -fcoverage-mapping)
  add_library(fmgc_model_fuzz STATIC ${model_core} $<TARGET_OBJECTS:fmgc_model_rest>)
  target_include_directories(fmgc_model_fuzz PUBLIC ${fuzz_dir})
  target_compile_options(fmgc_model_fuzz PRIVATE -O2 -ffp-contract=off -fsanitize=fuzzer-no-link)

  set(fuzz_inc ${fuzz_gen} ${WASM_SRC} src readable)
  macro(fuzz_tool target main)
    add_executable(${target} ${main} ${fuzz_gen}/fields.gen.h)
    target_include_directories(${target} PRIVATE ${fuzz_inc})
    target_compile_definitions(${target} PRIVATE ${FUZZ_DEFINES})
    target_compile_options(${target} PRIVATE -O2 -ffp-contract=off)
  endmacro()

  fuzz_tool(fmgc-cov-drive src/drive.cpp)
  target_link_libraries(fmgc-cov-drive PRIVATE fmgc_readable_${fuzz_name} fmgc_model_cov ZLIB::ZLIB -fprofile-instr-generate -static)

  fuzz_tool(fmgc-cov-fuzz-run src/fuzz_fmgc.cpp)
  target_compile_definitions(fmgc-cov-fuzz-run PRIVATE FMGC_FUZZ_STANDALONE)
  target_link_libraries(fmgc-cov-fuzz-run PRIVATE fmgc_readable_${fuzz_name} fmgc_model_cov -fprofile-instr-generate -static)

  fuzz_tool(fmgc-fuzz src/fuzz_fmgc.cpp)
  target_compile_options(fmgc-fuzz PRIVATE -g)
  target_link_libraries(fmgc-fuzz PRIVATE fmgc_readable_${fuzz_name} fmgc_model_fuzz -fsanitize=fuzzer -static-libstdc++ -static-libgcc)

  fuzz_tool(fmgc-fuzz-run src/fuzz_fmgc.cpp)
  target_compile_definitions(fmgc-fuzz-run PRIVATE FMGC_FUZZ_STANDALONE)
  target_link_libraries(fmgc-fuzz-run PRIVATE fmgc_readable_${fuzz_name} fmgc_model_${fuzz_name} -static)

  fuzz_tool(fmgc-fuzz-bank src/fuzz_bank.cpp)
  target_link_libraries(fmgc-fuzz-bank PRIVATE fmgc_readable_${fuzz_name} fmgc_model_${fuzz_name} ZLIB::ZLIB -static)
endif()
