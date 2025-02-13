add_subdirectory(OSShim)
add_subdirectory(CANShim)
add_subdirectory(DoCANCpp)

# Create a combined library
add_library(DoCANCppLib INTERFACE)

# Link the sub-libraries to the combined library
target_link_libraries(DoCANCppLib INTERFACE OSShim CANShim DoCANCpp)
