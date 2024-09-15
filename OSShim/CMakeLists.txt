add_library(OSShim STATIC)

file(GLOB OSShim_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

target_sources(OSShim PRIVATE ${OSShim_SOURCES})
target_include_directories(OSShim PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")