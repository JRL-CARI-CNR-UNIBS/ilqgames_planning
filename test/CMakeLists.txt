# If testing is enabled, build all tests.
if (BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)

  # Set the local gtest include directory.
  include_directories(${GTEST_INCLUDE_DIRS})

  # Gtest bug fix.
  add_definitions(-DGTEST_HAS_TR1_TUPLE=0)

  # Set a name for the output binary and testing arguments.
  set(test_target run_tests)
  set(test_args --gtest_shuffle)

  # Compile all tests into a single executable and link to ilqgames_planning and gtest.
  file(GLOB test_srcs ${CMAKE_SOURCE_DIR}/test/*.cpp)
  foreach(test ${test_srcs})
    get_filename_component(test_no_ext ${test} NAME_WE)
    message("Including test   \"${BoldBlue}${test_no_ext}${ColorReset}\".")
  endforeach()
  add_executable(${test_target} ${test_srcs})
  target_link_libraries(${test_target} ${PROJECT_NAME} ${ilqgames_LIBRARIES} gtest)
  add_test(${test_target} ${test_target})

  # Set the runtime directory for test_target.
  set_target_properties(${test_target} PROPERTIES
                        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

  # Make "make check" run the tests.
  add_custom_target(check COMMAND "${PROJECT_BINARY_DIR}/${test_target}")
  add_dependencies(check ${test_target})

  # Define a variable storing the path to test data.
  add_definitions(-DILQGAMES_PLANNING_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test/test_data")

  # If you want a test to generate temporary files (ie, testing file io) then store
  # them in this ignored folder
  add_definitions(-DGENERATED_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test/test_data/generated")

endif(BUILD_TESTS)
