add_library(ISOTPLib STATIC)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
        OSInterface
        GIT_REPOSITORY  git@github.com:purifymywater/OSInterface.git
        GIT_TAG         v1.1.1
)
FetchContent_Declare(
        CANInterface
        GIT_REPOSITORY  git@github.com:PurifyMyWater/CANInterface.git
        GIT_TAG         164d834f0d662bb648b980c83d357693f8800713
)


FetchContent_MakeAvailable(OSInterface)
FetchContent_MakeAvailable(CANInterface)

file(GLOB ISOTP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

target_sources(ISOTPLib PRIVATE ${ISOTP_SOURCES})
target_include_directories(ISOTPLib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_compile_options(ISOTPLib PRIVATE -Wall -Wextra -Werror -fno-rtti)
target_link_libraries(ISOTPLib CANInterface OSInterface)
