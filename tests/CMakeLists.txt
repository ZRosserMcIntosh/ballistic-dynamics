set(BD_TEST_SOURCES
  test_main.cpp
  test_foundations.cpp
  test_air.cpp
  test_data.cpp
)
add_executable(bd_tests ${BD_TEST_SOURCES})
target_link_libraries(bd_tests PRIVATE bd_core bd_aviation)
target_compile_options(bd_tests PRIVATE ${BD_WARNINGS})

# The runner prints per-case results; pass a substring to run a subset: bd_tests "dmath:"
add_test(NAME bd_tests COMMAND bd_tests)
