set(AVD_TEST_SOURCES
  test_main.cpp
  test_foundations.cpp
  test_loader.cpp
  test_components.cpp
  test_flight.cpp
  test_systems_scenarios.cpp
  test_validation.cpp
  test_instruments.cpp
)
add_executable(avd_tests ${AVD_TEST_SOURCES})
target_link_libraries(avd_tests PRIVATE avd_scenarios)
target_compile_definitions(avd_tests PRIVATE AVD_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
target_compile_options(avd_tests PRIVATE ${AVD_WARNINGS})

# The runner prints per-case results; pass a substring to run a subset: avd_tests "atmosphere:"
add_test(NAME avd_tests COMMAND avd_tests)
