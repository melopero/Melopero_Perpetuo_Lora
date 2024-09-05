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

class MeloperoPerpetuo {
public:
    MeloperoPerpetuo();
    ~MeloperoPerpetuo();

    // Initialization
    void init();

    // LoRa Module Functions
    void sendCmd(uint8_t command, uint8_t* payload = nullptr, size_t payloadLen = 0);
    void transmitData(uint8_t* data, size_t length);


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

    // rx buffer
    uint8_t response[256];  // Response buffer
    size_t responseLen;     // Length of the response

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

private:
    // LoRa Private Methods
    uint8_t calculateChecksum(uint8_t* data, size_t length);
    void buildPacket(uint8_t messageId, uint8_t* payload, size_t payloadLen, uint8_t* packet, size_t* packetLen);
    
    
    void enableFlowControl(uint32_t baudRate, bool enable);

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
