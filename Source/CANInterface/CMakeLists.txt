add_library(CANInterface STATIC)
target_sources(CANInterface PRIVATE "CANInterface.cpp")
target_include_directories(CANInterface PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
