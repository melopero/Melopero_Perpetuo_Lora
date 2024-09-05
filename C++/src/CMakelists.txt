# Create a library target for MeloperoPerpetuo
add_library(MeloperoPerpetuo STATIC
    MeloperoPerpetuo.cpp
)

# Generate the PIO header from the ws2812.pio file
pico_generate_pio_header(MeloperoPerpetuo ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)



# Link necessary libraries
target_link_libraries(MeloperoPerpetuo
    pico_stdlib
    hardware_i2c
    hardware_uart
    hardware_pio
)

# Include directories for the library
target_include_directories(MeloperoPerpetuo PUBLIC ${CMAKE_CURRENT_LIST_DIR})
