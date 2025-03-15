add_library(DoCANCpp STATIC)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

if(DEFINED ENV{GITHUB_ACTION})
    set(GIT_REPO https://vacmg:$ENV{GH_PAT}@github.com/vacmg/OSInterface.git)
else()
    set(GIT_REPO git@github.com:vacmg/OSInterface.git)
endif()

FetchContent_Declare(
        OSInterface
        GIT_REPOSITORY  ${GIT_REPO}
        GIT_TAG         main
)
FetchContent_MakeAvailable(OSInterface)

file(GLOB DoCANCpp_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

target_sources(DoCANCpp PRIVATE ${DoCANCpp_SOURCES})
target_include_directories(DoCANCpp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

#target_compile_options(DoCANCpp PRIVATE -Wall -Wextra -Werror) // TODO: Rollback to this before merging this PR
target_compile_options(DoCANCpp PRIVATE -Wall -Wextra)
target_link_libraries(DoCANCpp CANInterface OSInterface)
