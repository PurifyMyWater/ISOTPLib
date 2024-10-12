Project(DoCANCppLib)
set(CMAKE_CXX_STANDARD 20)

# Include the subdirectories
add_subdirectory(OSShim)
add_subdirectory(CANShim)
add_subdirectory(DoCANCpp)
add_subdirectory(Tests)

# Create a combined library
add_library(DoCANCppLib INTERFACE)

# Link the sub-libraries to the combined library
target_link_libraries(DoCANCppLib INTERFACE OSShim CANShim DoCANCpp)