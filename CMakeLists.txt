cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

# Add this line - REQUIRED
project(rpi_pico_w C CXX ASM)

# Initialize the SDK
pico_sdk_init()

add_subdirectory(advertising)
add_subdirectory(connection)
add_subdirectory(data_streaming)
add_subdirectory(i2c_throughput)
