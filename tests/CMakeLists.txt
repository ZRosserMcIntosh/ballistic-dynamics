set(BD_TEST_SOURCES
  test_main.cpp
  test_foundations.cpp
  test_air.cpp
  test_data.cpp
  test_point_mass.cpp
  test_spin.cpp
)
add_executable(bd_tests ${BD_TEST_SOURCES})
target_link_libraries(bd_tests PRIVATE bd_core bd_aviation)
target_compile_options(bd_tests PRIVATE ${BD_WARNINGS})
target_compile_definitions(bd_tests PRIVATE BD_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# The runner prints per-case results; pass a substring to run a subset: bd_tests "dmath:"
add_test(NAME bd_tests COMMAND bd_tests)
