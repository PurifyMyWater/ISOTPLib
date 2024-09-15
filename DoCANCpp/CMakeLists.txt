add_library(DoCANCpp STATIC)

file(GLOB DoCANCpp_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

target_sources(DoCANCpp PRIVATE ${DoCANCpp_SOURCES})
target_include_directories(DoCANCpp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(DoCANCpp CANShim OSShim)