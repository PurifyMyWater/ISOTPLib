add_library(CANShim STATIC)

file(GLOB CANShim_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

target_sources(CANShim PRIVATE ${CANShim_SOURCES})
target_include_directories(CANShim PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")