#include "MeloperoPerpetuo.h"

#define UART_PORT uart1



// Constructor
MeloperoPerpetuo::MeloperoPerpetuo() {
}

// Destructor
MeloperoPerpetuo::~MeloperoPerpetuo() {
    // Cleanup if needed
}

void MeloperoPerpetuo::init() {
    //initialize I2C0 on GPIO 24 an 25
    i2c_init(i2c0, 400 * 1000); // 400 kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Initialize UART for LoRa Module
    uart_init(UART_PORT, 9600);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);

    // Initialize Charger Status Pins
    gpio_init(STAT1_PIN);
    gpio_set_dir(STAT1_PIN, GPIO_IN);
    gpio_pull_up(STAT1_PIN);

    gpio_init(STAT2_PIN);
    gpio_set_dir(STAT2_PIN, GPIO_IN);
    gpio_pull_up(STAT2_PIN);


    //initialize GPIO0 to control VSEN
    gpio_init(0);
    gpio_set_dir(0, GPIO_OUT);
}

// LoRa Module Functions

void MeloperoPerpetuo::sendCmd(uint8_t command, uint8_t* payload, size_t payloadLen) {

    size_t totalPacketSize = payloadLen + 4; // 2 bytes for length, 1 byte for message ID, 1 byte for checksum
    uint8_t packetBuffer[totalPacketSize];
    size_t packetLen;

    // Build the packet with command and optional payload data
    buildPacket(command, payload, payloadLen, packetBuffer, &packetLen);

    // Transmit the packet via UART
    uart_write_blocking(UART_PORT, packetBuffer, packetLen);

    checkRxFifo(200);
}

  


void MeloperoPerpetuo::transmitData(uint8_t* data, size_t length) {
    size_t totalPacketSize = length + 4; // 2 bytes for length, 1 byte for message ID, 1 byte for checksum
    uint8_t packetBuffer[totalPacketSize];
    size_t packetLen;

    // Build the packet with data
    buildPacket(CMD_SEND_DATA, data, length, packetBuffer, &packetLen);

    // Transmit the packet via UART
    uart_write_blocking(UART_PORT, packetBuffer, packetLen);

    // Call checkRxFifo to update response and responseLen
    checkRxFifo(100);
}

// LoRa Helper Functions

uint8_t MeloperoPerpetuo::calculateChecksum(uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    
    }
    
    return checksum & 0xFF;
}

void MeloperoPerpetuo::buildPacket(uint8_t messageId, uint8_t* payload, size_t payloadLen, uint8_t* packet, size_t* packetLen) {
    // Calculate total packet size
    size_t totalPacketSize = payloadLen + 4; // Packet format: 2 bytes for length, 1 byte for message ID, variable payload, 1 byte for checksum

    // Set packet length
    packet[0] = (totalPacketSize >> 8) & 0xFF; // High byte of packet length
    packet[1] = totalPacketSize & 0xFF;        // Low byte of packet length

    // Set message ID
    packet[2] = messageId;

    // Copy payload data only if payloadLen is greater than 0
    if (payloadLen > 0 && payload != nullptr) {
        memcpy(&packet[3], payload, payloadLen);
    }
    

    // Calculate and set checksum
    packet[totalPacketSize - 1] = calculateChecksum(packet, totalPacketSize - 1);
    
    // Set packet length output parameter
    *packetLen = totalPacketSize;
}


// LoRa Module Command Functions

void MeloperoPerpetuo::reset() {
    uint8_t command = CMD_RESET;
    sendCmd(command);
}

void MeloperoPerpetuo::stopNetwork() {
    uint8_t command = CMD_STOP_NETWORK;
    sendCmd(command);
}

void MeloperoPerpetuo::startNetwork() {
    uint8_t command = CMD_START_NETWORK;
    sendCmd(command);
}

void MeloperoPerpetuo::setNetworkPreferences(bool useLoRaWan, bool enableAutoJoining, bool enableADR) {
    uint8_t command = CMD_SET_NETWORK_PREFERENCES;
    uint8_t payload = 0x00;

    // Set the network protocol (bit 7)
    if (useLoRaWan) {
        payload |= (1 << 7);  // Set bit 7 for LoRaWAN
    }

    // Set the Auto Joining option (bit 6)
    if (enableAutoJoining) {
        payload |= (1 << 6);  // Set bit 6 for Auto Joining
    }

    // Set the ADR option (bit 5)
    if (enableADR) {
        payload |= (1 << 5);  // Set bit 5 for ADR
    }

    // Send the command with the constructed payload
    sendCmd(command, &payload, sizeof(payload));
}


void MeloperoPerpetuo::setOutputPower(uint8_t power) {
    uint8_t command = CMD_SET_OUTPUT_POWER;
    uint8_t payload[1] = {power};
    sendCmd(command, payload, sizeof(payload));
}

void MeloperoPerpetuo::setOperatingChannel(uint8_t channel, uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate) {
    uint8_t command = CMD_SET_OPERATING_CHANNEL;
    uint8_t payload[4] = {channel, spreadingFactor, bandwidth, codingRate};
    sendCmd(command, payload, sizeof(payload));
}

void MeloperoPerpetuo::setNetworkAddress(uint16_t address) {
    uint8_t command = CMD_SET_NETWORK_ADDRESS;
    uint8_t payload[2] = {static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address & 0xFF)};
    sendCmd(command, payload, sizeof(payload));
}

void MeloperoPerpetuo::setNetworkId(uint8_t* id, size_t idLen) {
    uint8_t command = CMD_SET_NETWORK_ID;
    sendCmd(command, id, idLen);
}



void MeloperoPerpetuo::setEnergySaveMode(uint8_t save_mode) {
    uint8_t command = CMD_SET_ENERGY_SAVE_MODE;
    uint8_t payload[1] = {save_mode};
    sendCmd(command, payload, sizeof(payload));
}


// UART Communication Functions

bool MeloperoPerpetuo::checkRxFifo(uint32_t timeoutMs) {
    responseLen = 0;  // Reset response length
    memset(response, 0, sizeof(response));  // Clear the response buffer
    uint32_t startTime = to_ms_since_boot(get_absolute_time());  // Get the current time

    while ((to_ms_since_boot(get_absolute_time()) - startTime) < timeoutMs) {
        if (uart_is_readable(UART_PORT)) {
            return readRxFifo(response, &responseLen, sizeof(response));
        }
    }

    return false;  // Return false if no data was read before the timeout
}



bool MeloperoPerpetuo::readRxFifo(uint8_t* response, size_t* responseLen, size_t maxBufferSize) {
    size_t bytesRead = 0;

    // Read as long as there is data in the UART FIFO and space in the buffer
    while (uart_is_readable(UART_PORT) && *responseLen < maxBufferSize) {
        uint8_t byte = uart_getc(UART_PORT);  // Read one byte at a time
        response[*responseLen] = byte;        // Store the byte in the response buffer
        (*responseLen)++;                     // Increment the total byte count
        bytesRead++;
    }

    return bytesRead > 0;  // Return true if any bytes were read
}




void MeloperoPerpetuo::enableFlowControl(uint32_t baudRate, bool enable) {
    uart_set_hw_flow(UART_PORT, enable, enable);
    uart_set_baudrate(UART_PORT, baudRate);
}


// Charger Status Functions

int MeloperoPerpetuo::getChargerStatus() {
    bool stat1 = gpio_get(STAT1_PIN);
    bool stat2 = gpio_get(STAT2_PIN);

    if (!stat1 && !stat2) return 0; // Non-recoverable or latch-off fault
    if (!stat1 && stat2) return 1; // Recoverable fault
    if (stat1 && !stat2) return 2; // Charge in progress
    if (stat1 && stat2) return 3; // Charge completed

    return -1; // Undefined or error state
}

bool MeloperoPerpetuo::isCharging() {
    return getChargerStatus() == 2;
}

bool MeloperoPerpetuo::isFullyCharged() {
    return getChargerStatus() == 3;
}

bool MeloperoPerpetuo::hasRecoverableFault() {
    return getChargerStatus() == 1;
}

bool MeloperoPerpetuo::hasNonRecoverableFault() {
    return getChargerStatus() == 0;
}

void MeloperoPerpetuo::enablelWs2812(bool enable) {
    

    if(enable){
        // todo get free sm
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, 15, 800000, false);

    gpio_init(14);
    gpio_set_dir(14, GPIO_OUT);
    gpio_put(14, 1);
    
    }
    else{

    gpio_deinit(14);
    gpio_put(14, 0);
    }
}

void MeloperoPerpetuo::setWs2812Color(uint8_t r, uint8_t g, uint8_t b, double brightness) {
    r *= brightness;
    g *= brightness;
    b *= brightness;

    uint32_t pixel_grb = ((uint32_t) (r) << 8) |
                         ((uint32_t) (g) << 16) |
                         (uint32_t) (b);

    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u); 
            
    sleep_us(50);  // Short delay to ensure the LED receives the data
}


// Initialize the LED
void MeloperoPerpetuo::led_init() {
    gpio_init(23);
    gpio_set_dir(23, GPIO_OUT);
}

// Blink the LED a certain number of times
void MeloperoPerpetuo::blink_led(uint8_t times, uint32_t delay_ms) {
    for (uint8_t i = 0; i < times; i++) {
        gpio_put(23, 1);
        sleep_ms(delay_ms);
        gpio_put(23, 0);
        sleep_ms(delay_ms);
    }
}

 void MeloperoPerpetuo::enableVsen(){

    gpio_put(0, 1);
 }
    
void MeloperoPerpetuo::disableVsen(){
    
    gpio_put(0, 1);
}

void MeloperoPerpetuo::printResponse() {
    while (checkRxFifo(500)) {  // Keep checking the FIFO for new data
        for (size_t i = 0; i < responseLen; i++) {
            printf("0x%02X ", response[i]);
        }
        printf("\n");
    }
}
