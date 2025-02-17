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

add_subdirectory(CANInterface)
add_subdirectory(DoCANCpp)

# Create a combined library
add_library(DoCANCppLib INTERFACE)

# Link the sub-libraries to the combined library
target_link_libraries(DoCANCppLib INTERFACE OSInterface CANInterface DoCANCpp)
