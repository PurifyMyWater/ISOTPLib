if (NOT ESP_PLATFORM) # Only configure tests if we are building in a computer.

    project(ISOTPLib_GoogleTests)

    include(FetchContent)
    set(FETCHCONTENT_QUIET OFF)

    FetchContent_Declare(
            LinuxOSInterface
            GIT_REPOSITORY  git@github.com:purifymywater/LinuxOSInterface.git
            GIT_TAG         v1.1.1
    )

    FetchContent_Declare(
            googletest
            GIT_REPOSITORY  https://github.com/google/googletest.git
            GIT_TAG         v1.17.0
    )

    FetchContent_Declare(
            LocalCANNetwork
            GIT_REPOSITORY  git@github.com:PurifyMyWater/LocalCANNetwork.git
            GIT_TAG         v1.0.0
    )

    FetchContent_MakeAvailable(LinuxOSInterface)
    FetchContent_MakeAvailable(googletest)
    FetchContent_MakeAvailable(LocalCANNetwork)

    include_directories("${gtest_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/ISOTPLibTestUtils")

    file(GLOB_RECURSE TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ISOTPLibTests/*.cpp")
    file(GLOB_RECURSE TEST_UTILS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ISOTPLibTestUtils/*.cpp")

    # adding the Google_Tests_run target
    add_executable(ISOTPLib_GoogleTestsExe ${TEST_SOURCES} ${TEST_UTILS_SOURCES})
    target_include_directories(ISOTPLib_GoogleTestsExe PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/ISOTPLibTestUtils")

    # linking Google_Tests_run with ISOTPLib which will be tested
    target_link_libraries(ISOTPLib_GoogleTestsExe ISOTPLib LinuxOSInterface LocalCANNetwork)

    target_link_libraries(ISOTPLib_GoogleTestsExe gtest gtest_main)
endif ()
