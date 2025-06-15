add_library(DoCANCpp STATIC)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
        OSInterface
        GIT_REPOSITORY  git@github.com:vacmg/OSInterface.git
        GIT_TAG         v1.1.0
)
FetchContent_MakeAvailable(OSInterface)

file(GLOB DoCANCpp_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

target_sources(DoCANCpp PRIVATE ${DoCANCpp_SOURCES})
target_include_directories(DoCANCpp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_compile_options(DoCANCpp PRIVATE -Wall -Wextra -Werror)
target_link_libraries(DoCANCpp CANInterface OSInterface)
