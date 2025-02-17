if (NOT ESP_PLATFORM) # Only configure tests if we are building in a computer.

    project(DoCANCppLib_GoogleTests)

    include(FetchContent)
    set(FETCHCONTENT_QUIET OFF)

    if(DEFINED ENV{GITHUB_ACTION})
        set(GIT_REPO https://vacmg:$ENV{GH_PAT}@github.com/vacmg/LinuxOSInterface.git)
    else()
        set(GIT_REPO git@github.com:vacmg/LinuxOSInterface.git)
    endif()

    FetchContent_Declare(
            LinuxOSInterface
            GIT_REPOSITORY  ${GIT_REPO}
            GIT_TAG         main
    )

    FetchContent_Declare(
            googletest
            GIT_REPOSITORY  https://github.com/google/googletest.git
            GIT_TAG         main
    )

    FetchContent_MakeAvailable(LinuxOSInterface)
    FetchContent_MakeAvailable(googletest)

    include_directories("${gtest_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTestUtils")

    file(GLOB_RECURSE TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTests/*.cpp")
    file(GLOB_RECURSE TEST_UTILS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTestUtils/*.cpp")

    # adding the Google_Tests_run target
    add_executable(DoCANCppLib_GoogleTestsExe ${TEST_SOURCES} ${TEST_UTILS_SOURCES})
    target_include_directories(DoCANCppLib_GoogleTestsExe PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/DoCANCppLibTestUtils")

    # linking Google_Tests_run with DoCANCppLib which will be tested
    target_link_libraries(DoCANCppLib_GoogleTestsExe DoCANCppLib LinuxOSInterface)

    target_link_libraries(DoCANCppLib_GoogleTestsExe gtest gtest_main)
endif ()
