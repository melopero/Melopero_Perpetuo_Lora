#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "MeloperoPerpetuo.h"



// Main function
int main() {
    stdio_init_all();  // Initialize all standard IO

    MeloperoPerpetuo melopero;
    melopero.init();  // Initialize the board and peripherals

    melopero.led_init();
    melopero.blink_led(2, 500);

    melopero.enablelWs2812(true);

    melopero.setWs2812Color(255, 0, 0, 0.2);  
    sleep_ms(500);  

    melopero.setWs2812Color(0, 255, 0, 0.2);  
    sleep_ms(500);  

    melopero.setWs2812Color(0, 0, 255, 0.2);  
    sleep_ms(500);  

    melopero.enablelWs2812(false);

    // Prepare LoRa EMB configuration (optional; defaults are valid).
auto cfg = melopero.getEMBConfig();
cfg.power    = 0x0A;                  // TX power step
cfg.channel  = 1;                     // logical channel
cfg.sf       = SPREADING_FACTOR_7;    // SF7..SF12
cfg.bw       = BANDWIDTH_125;         // 125/250 kHz
cfg.cr       = CODING_RATE_4_5;       // 4/5..4/8
cfg.net_addr = 0x1234;                // network address
// cfg.net_id   = network_id;          // optional network ID pointer
// cfg.net_id_len = sizeof(network_id);
cfg.energy   = ENERGY_SAVE_MODE_TX_ONLY; // Always-on/RX-window/TX-only
melopero.setEMBConfig(cfg);

// Start EMB mode (applies stored configuration atomically).
if (melopero.startLoRaEMB(false) != TxStatus::Ok) {
    printf("Failed to start LoRa EMB (invalid configuration)\n");
    return 1;
}

// Transmit loop: broadcast a small payload.
while (true) {
    const uint8_t data[] = {0x70, 0x71, 0x72, 0x73};
    melopero.transmitEMB(data, sizeof(data));   // default dest=0xFFFF, options=0x0000
    melopero.processExecStatus();               // prints execution status (and extras if present)
    sleep_ms(6000);
}


    return 0;
}
