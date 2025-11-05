#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "MeloperoPerpetuo.h"

int main() {
    stdio_init_all();  // Initialize all standard IO

    MeloperoPerpetuo melopero;
    melopero.init();  // Initialize board peripherals

    melopero.led_init();
    melopero.blink_led(2, 500);

    // RGB LED test sequence
    melopero.enablelWs2812(true);
    melopero.setWs2812Color(255, 0, 0, 0.2);
    sleep_ms(300);
    melopero.setWs2812Color(0, 255, 0, 0.2);
    sleep_ms(300);
    melopero.setWs2812Color(0, 0, 255, 0.2);
    sleep_ms(300);
    melopero.enablelWs2812(false);

    // Request device ID
    melopero.sendCmd(0x01);
    sleep_ms(300);
    printf("Response to device ID:\n");
    melopero.printResponse();

    // LoRa EMB configuration (must match the transmitter)
    auto cfg = melopero.getEMBConfig();
    cfg.power    = 0x0A;                   // TX power step
    cfg.channel  = 1;                      // Channel index
    cfg.sf       = SPREADING_FACTOR_7;     // Spreading factor (SF7..SF12)
    cfg.bw       = BANDWIDTH_125;          // Bandwidth (125 or 250 kHz)
    cfg.cr       = CODING_RATE_4_5;        // Coding rate (4/5..4/8)
    cfg.net_addr = 0x1234;                 // Network address (must match TX)
    cfg.energy   = ENERGY_SAVE_MODE_ALWAYS_ON;  // Receiver always active
    melopero.setEMBConfig(cfg);

    if (melopero.startLoRaEMB(false) != TxStatus::Ok) {
        printf("Failed to start LoRa EMB (invalid configuration)\n");
        return 1;
    }

    printf("LoRa EMB RX started. Waiting for incoming packets...\n");

    // Optional battery status check
    if (melopero.isCharging()) {
        printf("Battery is charging.\n");
    } else if (melopero.isFullyCharged()) {
        printf("Battery is fully charged.\n");
    }

    // Main receive loop
    while (true) {
        if (melopero.checkRxFifo(1000)) {
            printf("Data received (%u bytes):\n", (unsigned)melopero.responseLen);

            // Print the raw frame content as hexadecimal bytes
            for (size_t i = 0; i < melopero.responseLen; ++i) {
                printf("0x%02X ", melopero.response[i]);
            }
            printf("\n");

            // Optional: check for multiple consecutive frames
            while (melopero.checkRxFifo(50)) {
                printf("Additional data received:\n");
                for (size_t i = 0; i < melopero.responseLen; ++i) {
                    printf("0x%02X ", melopero.response[i]);
                }
                printf("\n");
            }
        } else {
            printf("No data in RX FIFO\n");
        }

        sleep_ms(500);
    }

    return 0;
}
