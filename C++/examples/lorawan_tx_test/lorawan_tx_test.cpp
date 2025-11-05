#include <cstdio>
#include "pico/stdlib.h"
#include "MeloperoPerpetuo.h"

int main() {
    stdio_init_all();

    MeloperoPerpetuo m;
    m.init();

    // LoRaWAN configuration (example: EU868, Class A, OTAA).
    LoRaWANConfig lw = m.getLoRaWANConfig();
    lw.region     = 0x00;  // module-specific mapping (e.g., EU868)
    lw.klass      = 0x01;  // 0x01 = Class A, 0x00 = Class C
    lw.adr        = true;
    lw.auto_join  = true;
    lw.use_otaa   = true;

    // Fill OTAA credentials (replace with real values).
    static const uint8_t JOINEUI[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
    static const uint8_t DEVEUI[8]  = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
    static const uint8_t APPKEY[16] = {
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
    };
    lw.join_eui = JOINEUI;  lw.join_eui_len = 8;
    lw.dev_eui  = DEVEUI;   lw.dev_eui_len  = 8;
    lw.app_key  = APPKEY;   lw.app_key_len  = 16;

    lw.default_fport     = 10;
    lw.default_confirmed = false;

    m.setLoRaWANConfig(lw);

    // Starts LoRaWAN; performs auto-join if enabled.
    if (m.startLoRaWAN(false) != TxStatus::Ok) {
        printf("LoRaWAN start failed (invalid configuration or setup)\n");
        return 1;
    }

    // Uplink loop.
    while (true) {
        const uint8_t app[] = { 0x01, 0x02, 0x03 };
        m.transmitLoRaWAN(app, sizeof(app), /*fport_override*/-1, /*confirmed_override*/-1);
        m.processExecStatus();  // prints execution status and optional fields if present
        sleep_ms(5000);
    }
}
