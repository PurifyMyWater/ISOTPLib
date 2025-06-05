add_library(DoCANCpp STATIC)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
        OSInterface
        GIT_REPOSITORY  git@github.com:vacmg/OSInterface.git
        GIT_TAG         main
)
FetchContent_MakeAvailable(OSInterface)

file(GLOB DoCANCpp_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

target_sources(DoCANCpp PRIVATE ${DoCANCpp_SOURCES})
target_include_directories(DoCANCpp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

if((NOT DEFINED DOCANCPP_USE_DEBUG_TIMEOUTS) OR (DOCANCPP_USE_DEBUG_TIMEOUTS STREQUAL ""))
    set(DOCANCPP_USE_DEBUG_TIMEOUTS 0)
endif()
message(STATUS "Using DOCANCPP_USE_DEBUG_TIMEOUTS = ${DOCANCPP_USE_DEBUG_TIMEOUTS}")

target_compile_definitions(DoCANCpp PUBLIC DOCANCPP_USE_DEBUG_TIMEOUTS=${DOCANCPP_USE_DEBUG_TIMEOUTS})
target_compile_options(DoCANCpp PRIVATE -Wall -Wextra -Werror)
target_link_libraries(DoCANCpp CANInterface OSInterface)
