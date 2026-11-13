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

  add_executable(fmgc-probe-alt-capture-${name} src/probe_alt_capture.cpp)
  target_include_directories(fmgc-probe-alt-capture-${name} PRIVATE ${WASM_SRC} src)
  target_compile_options(fmgc-probe-alt-capture-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-probe-alt-capture-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)

  add_executable(fmgc-check-vertical-${name} src/check_vertical.cpp readable/vertical.cpp readable/vertical_alt.cpp
    readable/vertical_vs_tcas.cpp readable/vertical_climb_descent.cpp readable/vertical_approach.cpp
    ${gen_dir}/fields.gen.h)
  target_include_directories(fmgc-check-vertical-${name} PRIVATE ${gen_dir} ${WASM_SRC} src readable)
  target_compile_options(fmgc-check-vertical-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-check-vertical-${name} PRIVATE fmgc_model_${name} ZLIB::ZLIB -static)

  add_executable(fmgc-blocks-test-${name} readable/blocks_test.cpp ${gen_dir}/params.gen.h)
  target_include_directories(fmgc-blocks-test-${name} PRIVATE ${gen_dir} ${WASM_SRC} readable)
  target_compile_options(fmgc-blocks-test-${name} PRIVATE -O2 -ffp-contract=off)
  target_link_libraries(fmgc-blocks-test-${name} PRIVATE fmgc_model_${name} -static)
endforeach()
