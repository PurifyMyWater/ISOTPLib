if (NOT ESP_PLATFORM) # Only configure tests if we are building in a computer.

    project(DoCANCppLib_GoogleTests)
    add_subdirectory(LinuxOSShim)
    if (NOT TARGET gtest)
        add_subdirectory(googletest)
    endif ()
    include_directories("${gtest_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTestUtils")

    file(GLOB_RECURSE TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTests/*.cpp")
    file(GLOB_RECURSE TEST_UTILS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTestUtils/*.cpp")

    # adding the Google_Tests_run target
    add_executable(DoCANCppLib_GoogleTestsExe ${TEST_SOURCES} ${TEST_UTILS_SOURCES})
    target_include_directories(DoCANCppLib_GoogleTestsExe PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTestUtils")

    # linking Google_Tests_run with DoCANCppLib which will be tested
    target_link_libraries(DoCANCppLib_GoogleTestsExe DoCANCppLib LinuxOSShim)

    target_link_libraries(DoCANCppLib_GoogleTestsExe gtest gtest_main)
endif ()
