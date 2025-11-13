#ifndef MELOPEROPERPETUO_H
#define MELOPEROPERPETUO_H

#include <cstdint>
#include <cstdio>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <cstring>

// PIO program for WS2812 (part of Raspberry Pi Pico SDK examples)
#include "ws2812.pio.h"

// Command Definitions
#define CMD_RESET 0x05
#define CMD_STOP_NETWORK 0x30
#define CMD_START_NETWORK 0x31
#define CMD_SET_NETWORK_PREFERENCES 0x25
#define CMD_SET_OUTPUT_POWER 0x10
#define CMD_SET_OPERATING_CHANNEL 0x11
#define CMD_SET_NETWORK_ADDRESS 0x21
#define CMD_SET_NETWORK_ID 0x22
#define CMD_SET_ENERGY_SAVE_MODE 0x13
#define CMD_SEND_DATA 0x50
#define CMD_GET_NETWORK_STATUS        0x19


// LoRaWAN-specific commands (from ebi_lora_rev_2.1)
#define CMD_SET_PHYSICAL_ADDRESS   0x20  // payload: [JoinEUI(8)][DevEUI(8)]
#define CMD_SET_NETWORK_SECURITY   0x26  // payload: [selector][value...]; 0x01=AppKey, 0x00=NwkKey

// Device / firmware info (EBI-LoRa rev1.0.1-3)
#define CMD_FIRMWARE_VERSION 0x06  // response: 0x86 + 4B version
// Device / firmware info (fallback if 0x06 unsupported)
#define CMD_DEVICE_INFO 0x01  // response: 0x81 + payload

// Buffer sizes
#define CMD_SIZE 64
#define MAX_PACKET_SIZE 256


// // Energy Save Modes
// ALWAYS ON: The reception is always enabled and it is possible to transmit and receive
// data anytime.
// RX WINDOW: The module opens a “reception window” for an amount of time after each
// transmission before going in low power mode. It allows to receive a response to the
// packet just sent.
// TX WINDOW: The module is in mono-directional transmission mode, after each
// transmission the radio goes automatically in sleep-mode.

#define ENERGY_SAVE_MODE_ALWAYS_ON 0x00
#define ENERGY_SAVE_MODE_RX_WINDOW 0x01
#define ENERGY_SAVE_MODE_TX_ONLY 0x02

// LoRa Configuration Constants
#define SPREADING_FACTOR_7 0x07
#define SPREADING_FACTOR_8 0x08
#define SPREADING_FACTOR_9 0x09
#define SPREADING_FACTOR_10 0x0A
#define SPREADING_FACTOR_11 0x0B
#define SPREADING_FACTOR_12 0x0C

#define BANDWIDTH_125 0x00
#define BANDWIDTH_250 0x01

#define CODING_RATE_4_5 0x01
#define CODING_RATE_4_6 0x02
#define CODING_RATE_4_7 0x03
#define CODING_RATE_4_8 0x04

// Network operating mode.
enum class NetworkMode { None, LoRaEMB, LoRaWAN };


// High-level transmission result codes used by EMB and LoRaWAN helpers.
enum class TxStatus : uint8_t {
    Ok = 0,
    InvalidArgs,
    Error,
    Timeout,
    Unsupported,
    ChannelBusy,
    DutyCycle,
    NoResponse
};


// LoRa EMB runtime configuration. Defaults are valid at boot.
struct EMBConfig {
    uint8_t  power    = 0x0A;                  // TX power step (module-specific range)
    uint8_t  channel  = 1;                     // Logical channel index
    uint8_t  sf       = SPREADING_FACTOR_7;    // Spreading factor (SF7..SF12)
    uint8_t  bw       = BANDWIDTH_125;         // Bandwidth (125/250 kHz)
    uint8_t  cr       = CODING_RATE_4_5;       // Coding rate (4/5..4/8)
    uint16_t net_addr = 0x1234;                // Network address (0x0000..0xFFFF)
    const uint8_t* net_id = nullptr;           // Optional network ID pointer
    size_t   net_id_len   = 0;                 // Optional network ID length
    uint8_t  energy  = ENERGY_SAVE_MODE_TX_ONLY; // Energy save mode
};

// LoRaWAN runtime configuration. Defaults are usable but require credentials.
struct LoRaWANConfig {
    // Operation mode
    bool     use_otaa        = true;   // true=OTAA, false=ABP
    bool     adr             = true;   // Adaptive Data Rate
    bool     auto_join       = true;   // OTAA: perform join automatically on start
    uint8_t  klass           = 0x01;   // 0x01=Class A, 0x00=Class C
    uint8_t  region          = 0x00;   // e.g., 0x00=EU868 (module-specific mapping)

    // OTAA credentials (8+8+16 bytes). Pointers may be null when unused.
    const uint8_t* join_eui  = nullptr; size_t join_eui_len = 0; // 8
    const uint8_t* dev_eui   = nullptr; size_t dev_eui_len  = 0; // 8
    const uint8_t* app_key   = nullptr; size_t app_key_len  = 0; // 16

    // ABP credentials (DevAddr + 16+16 bytes). Keys may be null when unused.
    uint32_t       dev_addr  = 0;
    const uint8_t* nwk_skey  = nullptr; size_t nwk_skey_len = 0; // 16
    const uint8_t* app_skey  = nullptr; size_t app_skey_len = 0; // 16

    // Default uplink behavior
    uint8_t  default_fport      = 1;    // Application port (1..223)
    bool     default_confirmed  = false; // true=confirmed uplink, false=unconfirmed
};



class MeloperoPerpetuo {
public:
    MeloperoPerpetuo();
    ~MeloperoPerpetuo();

    // Initialization
    void init();

    // LoRa Module Functions
    void sendCmd(uint8_t command, uint8_t* payload = nullptr, size_t payloadLen = 0);
    void sendCmdTimeout(uint8_t command, uint8_t* payload, size_t payloadLen, uint32_t timeout_ms);
    
    // Sends user data over LoRa EMB using the current configuration.
    // The header format is [options_H][options_L][addr_H][addr_L][payload...].
    // By default, 'dest_addr' = 0xFFFF (broadcast) and 'options' = 0x0000.
    void transmitEMB(const uint8_t* data,
                    size_t length,
                    uint16_t dest_addr = 0xFFFF,
                    uint16_t options   = 0x0000);

    // Sends an uplink over LoRaWAN using the stored defaults unless overridden.
    // 'fport_override' < 0 uses default_fport; 'confirmed_override' < 0 uses default_confirmed.
    // Returns Ok when the frame has been queued to the module.
    TxStatus transmitLoRaWAN(const uint8_t* data, size_t len,
                            int fport_override = -1,
                            int confirmed_override = -1);

    void reset();
    void stopNetwork();
    void startNetwork();
    void setNetworkPreferences(bool useLoRaWan, bool enableAutoJoining, bool enableADR);
    void setOutputPower(uint8_t power);
    void setOperatingChannel(uint8_t channel, uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate);
    void setNetworkAddress(uint16_t address);
    void setNetworkId(uint8_t* id, size_t idLen);
    void setEnergySaveMode(uint8_t save_mode);
    bool checkRxFifo(uint32_t timeoutMs);
    bool readRxFifo(uint8_t* response, size_t* responseLen, size_t maxBufferSize);

    // Get firmware version as 4 bytes; returns true on success.
    bool getFirmwareVersion(uint8_t out[4]);

    // Print firmware version as "FW: XX XX XX XX".
    void printFirmwareVersion();

    // rx buffer
    uint8_t response[256];  // Response buffer
    size_t responseLen;     // Length of the response

    // Returns the current network operating mode.
    NetworkMode getMode() const;

    // Stores the given EMB configuration without applying it to the module.
    void setEMBConfig(const EMBConfig& cfg);

    // Returns the currently stored EMB configuration.
    EMBConfig getEMBConfig() const;

    // Validates the provided EMB configuration values.
    bool validateEMBConfig(const EMBConfig& cfg) const;

 

    // Starts LoRa EMB mode. When 'force' is true, the stored configuration is
    // reapplied even if it is already in sync. The sequence performs:
    // Stop -> Select EMB preferences -> Apply stored EMBConfig -> Start.
    TxStatus startLoRaEMB(bool force = false);

    // Starts LoRaWAN mode. When 'force' is true, the stored configuration is
    // reapplied even if it is already in sync. The sequence performs:
    // Stop -> Select LoRaWAN preferences -> Apply stored LoRaWANConfig -> Start.
    TxStatus startLoRaWAN(bool force = false);




    // Stores the given LoRaWAN configuration without applying it to the module.
    void setLoRaWANConfig(const LoRaWANConfig& cfg);

    // Returns the currently stored LoRaWAN configuration.
    LoRaWANConfig getLoRaWANConfig() const;

    // Validates the provided LoRaWAN configuration values.
    bool validateLoRaWANConfig(const LoRaWANConfig& cfg) const;





    // Charger Status Functions
    int getChargerStatus();
    bool isCharging();
    bool isFullyCharged();
    bool hasRecoverableFault();
    bool hasNonRecoverableFault();

    // WS2812 Functions
    void enablelWs2812(bool enable);
    void setWs2812Color(uint8_t r, uint8_t g, uint8_t b, double brightness=0.2);

    void led_init();
    void blink_led(uint8_t times=1, uint32_t delay_ms=500);

    void enableVsen();
    void disableVsen();

    void printResponse();

    // Returns the execution status byte from the last response.
    // Returns 0xFF if unavailable (e.g., response too short).
    uint8_t getExecStatus() const;

    // Prints the execution status and basic optional fields when present.
    // Intended as a quick diagnostic after commands such as CMD_SEND_DATA.
    void processExecStatus() const;


private:
    // LoRa Private Methods
    uint8_t calculateChecksum(uint8_t* data, size_t length);
    void buildPacket(uint8_t messageId, uint8_t* payload, size_t payloadLen, uint8_t* packet, size_t* packetLen);
    
    void enableFlowControl(uint32_t baudRate, bool enable);

    // Tracks the current network mode and whether the module is running.
    NetworkMode mode = NetworkMode::None;
    bool network_running = false;

    // Last known EMB configuration (defaults at boot). Marked pending until applied.
    EMBConfig emb_config{};
    bool emb_config_pending = true;

    // Last known LoRaWAN configuration. Marked pending until applied.
    LoRaWANConfig lorawan_config{};
    bool lorawan_config_pending = true;




    // Pins and Ports
    static const int I2C_SDA_PIN = 24;
    static const int I2C_SCL_PIN = 25;
    
    static const int TX_PIN = 8;
    static const int RX_PIN = 9;

    // Charger Status Pins
    static const int STAT1_PIN = 1;
    static const int STAT2_PIN = 2;

    // WS2812 LED Pin
    static const int ENABLE_WS2812 = 14;
    static const int WS2812_PIN = 15;

    // PIO instance for WS2812
    PIO pio;
    uint sm;
};

#endif // MELOPEROPERPETUO_H
