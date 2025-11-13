# Create an executable target for the transmitter test
add_executable(lorawan_tx_test
    lorawan_tx_test.cpp
)

# Link the MeloperoPerpetuo library and other dependencies
target_link_libraries(lorawan_tx_test
    MeloperoPerpetuo
    pico_stdlib
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(lorawan_tx_test 1)
pico_enable_stdio_uart(lorawan_tx_test 0)


# Add this line to make the executable output as a UF2 file
pico_add_extra_outputs(lorawan_tx_test)