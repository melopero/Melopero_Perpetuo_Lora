#include <cstdio>
#include "pico/stdlib.h"
#include "MeloperoPerpetuo.h"

// Helpers for printing EUI/key 
static inline void print_hex_msb(const char* name, const uint8_t* v, size_t n) {
    printf("%s (MSB): ", name);
    for (size_t i = 0; i < n; ++i) printf("%02X", v[i]);
    printf("\n");
}
static inline void print_hex_lsb(const char* name, const uint8_t* v, size_t n) {
    printf("%s (LSB): ", name);
    for (size_t i = 0; i < n; ++i) printf("%02X", v[n - 1 - i]);
    printf("\n");
}


int main() {
    stdio_init_all();

    MeloperoPerpetuo m;
    m.init();
    
    // Print firmware version
    sleep_ms(5000);

        // --- Force full reset of LoRaWAN module before starting configuration ---
    printf("Forcing LoRaWAN NVM reset...\n");
    m.reset();          // CMD_RESET = 0x05
    sleep_ms(1500);     // give it time to reboot fully
    m.stopNetwork();    // extra safety to clear any pending state
    sleep_ms(500);
    printf("Reset complete. Continuing with configuration...\n");

    m.printFirmwareVersion();

    // LoRaWAN configuration (example: EU868, Class A, OTAA).
    LoRaWANConfig lw = m.getLoRaWANConfig();
    lw.region     = 0x00;  // module-specific mapping (e.g., EU868)
    lw.klass      = 0x01;  // 0x01 = Class A, 0x00 = Class C
    lw.adr        = true;
    lw.auto_join  = true;
    lw.use_otaa   = true;

    // OTAA credentials generated from the TTN console (LoRaWAN 1.0.2, EU868).
    static const uint8_t JOINEUI[8] = { 0x00, 0xC4, 0x5E, 0x72, 0x3A, 0x00, 0x79, 0x42 };
    static const uint8_t DEVEUI[8]  = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x3F, 0x08 };
    static const uint8_t APPKEY[16] = {
        0xF2, 0x0E, 0xA1, 0xBE, 
        0x2A, 0xFC, 0xAE, 0x4C, 
        0x17, 0xCA, 0x9D, 0xFB, 
        0xFC, 0xE8, 0xB9, 0xAA 
    };

    print_hex_msb("DevEUI", DEVEUI, 8);
    print_hex_lsb("DevEUI", DEVEUI, 8);
    print_hex_msb("JoinEUI", JOINEUI, 8);
    print_hex_lsb("JoinEUI", JOINEUI, 8);
    print_hex_msb("AppKey", APPKEY, 16);

    lw.join_eui = JOINEUI;  lw.join_eui_len = 8;
    lw.dev_eui  = DEVEUI;   lw.dev_eui_len  = 8;
    lw.app_key  = APPKEY;   lw.app_key_len  = 16;

    lw.default_fport     = 6;
    lw.default_confirmed = true;

    m.setLoRaWANConfig(lw);

    

    // Starts LoRaWAN; performs auto-join if enabled.
    if (m.startLoRaWAN(true) != TxStatus::Ok) {
        printf("LoRaWAN start failed (invalid configuration or setup)\n");
        return 1;
    }
    
    const uint8_t app[] = { 0xD0, 0xD1, 0xD2, 0xD3 };
    while (true) {
        m.transmitLoRaWAN(app, sizeof(app), 6, 1);
        m.processExecStatus();
        sleep_ms(60000);
    }

}


