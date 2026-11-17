cmake_minimum_required(VERSION 3.20)
project(BallisticDynamics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BD_BUILD_TESTS "Build Ballistic Dynamics tests" ON)
option(BD_BUILD_TOOLS "Build the bdemo tool" ON)
option(BD_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

if(MSVC)
  set(BD_WARNINGS /W4 /permissive-)
  if(BD_WARNINGS_AS_ERRORS)
    list(APPEND BD_WARNINGS /WX)
  endif()
  # Determinism (ADR 0004): strict IEEE semantics, no contraction into FMA.
  set(BD_FP_FLAGS /fp:precise)
else()
  set(BD_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(BD_WARNINGS_AS_ERRORS)
    list(APPEND BD_WARNINGS -Werror)
  endif()
  # Determinism (ADR 0004): no fused multiply-add (clang contracts by
  # default on arm64), no fast-math.
  set(BD_FP_FLAGS -ffp-contract=off -fno-fast-math)
endif()

# ---- Engine-independent projectile core (no engine/renderer headers) ----
add_library(bd_core
  src/core/src/dmath.cpp
  src/core/src/json.cpp
  src/core/src/library.cpp
  src/core/src/air.cpp
  src/core/src/determinism.cpp
  src/core/src/drag.cpp
  src/core/src/point_mass.cpp
  src/core/src/solver.cpp
  src/core/src/spin.cpp
  src/core/src/wake.cpp
)
add_library(bd::core ALIAS bd_core)
target_include_directories(bd_core PUBLIC src/core/include)
target_compile_options(bd_core PRIVATE ${BD_WARNINGS} PUBLIC ${BD_FP_FLAGS})
target_compile_definitions(bd_core PUBLIC BD_DATA_DIR="${PROJECT_SOURCE_DIR}/data")

# ---- The air: Aviation Dynamics' atmosphere and weather (ADR 0003) ----
# Aviation's own sources, compiled unchanged from the pinned submodule.
set(AVD_DIR ${PROJECT_SOURCE_DIR}/extern/aviation-dynamics)
if(NOT EXISTS ${AVD_DIR}/src/core/src/atmosphere.cpp)
  message(FATAL_ERROR "extern/aviation-dynamics is missing: run `git submodule update --init`")
endif()
add_library(bd_avd_air STATIC
  ${AVD_DIR}/src/core/src/atmosphere.cpp
  ${AVD_DIR}/src/core/src/wind.cpp
  ${AVD_DIR}/src/core/src/table.cpp
)
target_include_directories(bd_avd_air PUBLIC ${AVD_DIR}/src/core/include)
target_compile_options(bd_avd_air PRIVATE ${BD_FP_FLAGS})

add_library(bd_aviation src/aviation/src/aviation_air.cpp)
add_library(bd::aviation ALIAS bd_aviation)
target_include_directories(bd_aviation PUBLIC src/aviation/include)
target_link_libraries(bd_aviation PUBLIC bd_core bd_avd_air)
target_compile_options(bd_aviation PRIVATE ${BD_WARNINGS})

if(BD_BUILD_TOOLS)
  add_executable(bdemo tools/bdemo/main.cpp)
  target_link_libraries(bdemo PRIVATE bd_core bd_aviation)
  target_compile_options(bdemo PRIVATE ${BD_WARNINGS})
endif()

if(BD_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
