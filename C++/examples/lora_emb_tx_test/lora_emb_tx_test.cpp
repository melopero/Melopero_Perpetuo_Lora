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

    // LoRaEMB operating mode configuration sequence:
    // Stop Network
    // Set Network preferences 
    // Set Output power
    // Set Operating channel
    // Set Network address
    // Set Network ID
    // Set Energy save mode (the default value is TX_ONLY, set RX_ALWAYS)
    // Start Network

    melopero.stopNetwork();
    sleep_ms(500);
    printf("response to stopNetwork\n");
    melopero.printResponse();

    melopero.setNetworkPreferences(false, false, false);
    sleep_ms(500);
    printf("response to setNetworkPreferences\n");
    melopero.printResponse();

    melopero.setOutputPower(0x0a);  // Example power level
    sleep_ms(500);
    printf("response to setOutputPower\n");
    melopero.printResponse();
    
    melopero.setOperatingChannel(1, SPREADING_FACTOR_7, BANDWIDTH_125, CODING_RATE_4_5);  // Example channel
    sleep_ms(500);
    printf("response to setOperatingChannel\n");
    melopero.printResponse();

    melopero.setNetworkAddress(0x1234);  // Example network address
    sleep_ms(500);
    printf("response to setNetworkAddress\n");
    melopero.printResponse();

    uint8_t network_id[] = {0x00, 0x01};  // Example network ID
    melopero.setNetworkId(network_id, sizeof(network_id));
    sleep_ms(500);
        printf("response to setNetworkId\n");
        melopero.printResponse();

     melopero.setEnergySaveMode(ENERGY_SAVE_MODE_TX_ONLY);
    sleep_ms(500);
    printf("response to setEnergySaveModeRxAlways\n");
    melopero.printResponse();

    //melopero.setEnergySaveModeRxAlways();
    melopero.startNetwork();
      sleep_ms(500);
        printf("response to startNetwork\n");
        melopero.printResponse();



    while (1) {
        uint8_t data[] = {0x00, 0x00, 0xFF, 0xFF, 0x70, 0x71, 0x72, 0x73};
        melopero.transmitData(data, sizeof(data));
         sleep_ms(500);
        printf("response to transmitData\n");
        melopero.printResponse();
        sleep_ms(6000);
    }

    return 0;
}
