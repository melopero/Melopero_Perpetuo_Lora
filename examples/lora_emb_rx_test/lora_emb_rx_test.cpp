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
    
    // Turn off the RGB LED
    melopero.enablelWs2812(false);

    melopero.sendCmd(0x01);
    sleep_ms(500);
    printf("response to deviceId\n");
    melopero.printResponse();

    
    // LoRaEMB operating mode configuration
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
    
    melopero.setNetworkAddress(0x1235);  // Example network address
    sleep_ms(500);
    printf("response to setNetworkAddress\n");
    melopero.printResponse();

    uint8_t network_id[] = {0x00, 0x01};  // Example network ID
    melopero.setNetworkId(network_id, sizeof(network_id));
    sleep_ms(500);
    printf("response to setNetworkId\n");
    melopero.printResponse();

    melopero.setEnergySaveMode(ENERGY_SAVE_MODE_ALWAYS_ON);
    sleep_ms(500);
    printf("response to setEnergySaveModeRxAlways\n");
    melopero.printResponse();

    melopero.startNetwork();
    sleep_ms(500);
    printf("response to startNetwork\n");
    melopero.printResponse();


//A battery must be connected; otherwise, 
//the status will alternate between charging and fully charged.    
int status = melopero.getChargerStatus();
    if (melopero.isCharging()) { 
        printf("The battery is charging.\n");
    } 
    else if (melopero.isFullyCharged()) {
        printf("The battery is fully charged.\n");
    } 
    
    while (1) {
        

        if(melopero.checkRxFifo(500)) {melopero.printResponse();}
        else {printf("nothing in the rx fifo\n");}

        sleep_ms(1000);
    }

    return 0;
}
