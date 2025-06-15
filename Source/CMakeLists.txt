add_subdirectory(CANInterface)
add_subdirectory(ISOTP)

# Create a combined library
add_library(ISOTPLib INTERFACE)

# Link the sub-libraries to the combined library
target_link_libraries(ISOTPLib INTERFACE OSInterface CANInterface ISOTP)
