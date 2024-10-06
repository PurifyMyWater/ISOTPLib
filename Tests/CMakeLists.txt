project(Google_tests)
add_subdirectory(googletest)
include_directories("${gtest_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTestUtils")

file(GLOB_RECURSE TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTests/*.cpp")
file(GLOB_RECURSE TEST_UTILS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTestUtils/*.cpp")
message(STATUS "TEST_SOURCES contents: ${TEST_SOURCES}")
message(STATUS "TEST_UTILS_SOURCES contents: ${TEST_UTILS_SOURCES}")
# adding the Google_Tests_run target
add_executable(Google_Tests_run ${TEST_SOURCES} ${TEST_UTILS_SOURCES})
target_include_directories(Google_Tests_run PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# linking Google_Tests_run with DoCANCppLib which will be tested
target_link_libraries(Google_Tests_run DoCANCppLib)

target_link_libraries(Google_Tests_run gtest gtest_main)
