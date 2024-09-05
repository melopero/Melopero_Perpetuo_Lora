cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(../pico_sdk_import.cmake)
include(../pico_extras_import_optional.cmake)


# Specify the project name
project(MeloperoPerpetuoExamples C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# Add subdirectories for each example
add_subdirectory(examples/lora_emb_tx_test)
add_subdirectory(examples/lora_emb_rx_test)


# Add the src directory
add_subdirectory(src)
