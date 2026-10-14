cmake_minimum_required(VERSION 3.20)
project(AviationDynamics VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(AVD_BUILD_TESTS "Build Aviation Dynamics tests" ON)
option(AVD_BUILD_TOOLS "Build the avdemo tool" ON)
option(AVD_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

if(MSVC)
  set(AVD_WARNINGS /W4 /permissive-)
  if(AVD_WARNINGS_AS_ERRORS)
    list(APPEND AVD_WARNINGS /WX)
  endif()
else()
  set(AVD_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(AVD_WARNINGS_AS_ERRORS)
    list(APPEND AVD_WARNINGS -Werror)
  endif()
endif()

# ---- Engine-independent flight simulation core (no engine/renderer headers) ----
add_library(avd_core
  src/core/src/table.cpp
  src/core/src/geodesy.cpp
  src/core/src/atmosphere.cpp
  src/core/src/mass.cpp
  src/core/src/aero.cpp
  src/core/src/propulsion.cpp
  src/core/src/ground.cpp
  src/core/src/systems.cpp
  src/core/src/instruments.cpp
  src/core/src/aircraft_def.cpp
  src/core/src/json.cpp
  src/core/src/aircraft_loader.cpp
  src/core/src/aircraft.cpp
  src/core/src/assists.cpp
  src/core/src/procedures.cpp
  src/core/src/telemetry.cpp
  src/core/src/navigation.cpp
)
add_library(avd::core ALIAS avd_core)
target_include_directories(avd_core PUBLIC src/core/include)
target_compile_options(avd_core PRIVATE ${AVD_WARNINGS})

# ---- Scenarios, test pilot, airports (shared by tests and avdemo) ----
add_library(avd_scenarios src/scenarios/scenarios.cpp src/scenarios/test_pilot.cpp src/scenarios/airports.cpp)
target_include_directories(avd_scenarios PUBLIC src/scenarios)
target_link_libraries(avd_scenarios PUBLIC avd_core)
target_compile_options(avd_scenarios PRIVATE ${AVD_WARNINGS})
target_compile_definitions(avd_scenarios PUBLIC AVD_DATA_DIR="${PROJECT_SOURCE_DIR}/data")

if(AVD_BUILD_TOOLS)
  add_executable(avdemo tools/avdemo/main.cpp tools/avdemo/demos.cpp)
  target_link_libraries(avdemo PRIVATE avd_scenarios)
  target_compile_options(avdemo PRIVATE ${AVD_WARNINGS})
endif()

if(AVD_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
