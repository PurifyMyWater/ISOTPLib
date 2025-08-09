add_library(ISOTP STATIC)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
        OSInterface
        GIT_REPOSITORY  git@github.com:vacmg/OSInterface.git
        GIT_TAG         v1.1.0
)
FetchContent_MakeAvailable(OSInterface)

file(GLOB ISOTP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

target_sources(ISOTP PRIVATE ${ISOTP_SOURCES})
target_include_directories(ISOTP PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_compile_options(ISOTP PRIVATE -Wall -Wextra -Werror)
target_link_libraries(ISOTP CANInterface OSInterface)
