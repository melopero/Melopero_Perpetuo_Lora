# Create an executable target for the receiver test
add_executable(lora_emb_rx_test
    lora_emb_rx_test.cpp
)

# Link the MeloperoPerpetuo library and other dependencies
target_link_libraries(lora_emb_rx_test
    MeloperoPerpetuo
    pico_stdlib
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(lora_emb_rx_test 1)
pico_enable_stdio_uart(lora_emb_rx_test 0)

# Add this line to make the executable output as a UF2 file
pico_add_extra_outputs(lora_emb_rx_test)