cmake_minimum_required(VERSION 3.29)

Project(DoCANCppLib)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include the subdirectories
add_subdirectory(Source)
add_subdirectory(Tests)
