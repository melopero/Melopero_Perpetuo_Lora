#include "MeloperoPerpetuo.h"

#define UART_PORT uart1



// Constructor
MeloperoPerpetuo::MeloperoPerpetuo() {

    // Initializes network state.
    mode = NetworkMode::None;
    network_running = false;

    // Marks default EMB configuration as pending until first apply.
    emb_config_pending = true;

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



void MeloperoPerpetuo::transmitEMB(const uint8_t* data,
                                   size_t length,
                                   uint16_t dest_addr,
                                   uint16_t options) {
    // Builds the EMB packet header and sends it through UART.
    // Payload layout: [options_H][options_L][addr_H][addr_L][user_data...]

    if (length + 4 > MAX_PACKET_SIZE) {
        length = MAX_PACKET_SIZE - 4;  // Prevents overflow of internal buffer.
    }

    uint8_t payload[MAX_PACKET_SIZE];
    size_t idx = 0;

    // Add "options" field (big-endian).
    payload[idx++] = static_cast<uint8_t>((options >> 8) & 0xFF);
    payload[idx++] = static_cast<uint8_t>(options & 0xFF);

    // Add destination address (big-endian).
    payload[idx++] = static_cast<uint8_t>((dest_addr >> 8) & 0xFF);
    payload[idx++] = static_cast<uint8_t>(dest_addr & 0xFF);

    // Copy user data after the 4-byte header.
    if (data && length > 0) {
        memcpy(&payload[idx], data, length);
        idx += length;
    }

    // Sends the assembled frame to the module.
    sendCmd(CMD_SEND_DATA, payload, idx);
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

    // Marks network as stopped and clears the active mode.
    network_running = false;
    mode = NetworkMode::None;
}

void MeloperoPerpetuo::startNetwork() {
    uint8_t command = CMD_START_NETWORK;
    sendCmd(command);

    // Marks network as running. The concrete mode is set by startLoRaEMB/LoRaWAN.
    network_running = true;
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

NetworkMode MeloperoPerpetuo::getMode() const {
    // Returns the last known operating mode.
    return mode;
}

void MeloperoPerpetuo::setLoRaWANConfig(const LoRaWANConfig& cfg) {
    // Stores configuration (application occurs on startLoRaWAN()).
    lorawan_config = cfg;
    lorawan_config_pending = true;  // Marks configuration as pending.
}

LoRaWANConfig MeloperoPerpetuo::getLoRaWANConfig() const {
    // Returns a copy of the stored configuration.
    return lorawan_config;
}

bool MeloperoPerpetuo::validateLoRaWANConfig(const LoRaWANConfig& cfg) const {
    // Region range check (module-specific; adjust mapping as required).
    if (cfg.region > 0x02) return false; // example: 0x00=EU868, 0x01=US915, 0x02=2.4GHz

    // Class check: accepts 0x01 (A) or 0x00 (C).
    if (cfg.klass != 0x01 && cfg.klass != 0x00) return false;

    // FPort range (LoRaWAN spec: 1..223 for application traffic).
    if (cfg.default_fport == 0 || cfg.default_fport > 223) return false;

    if (cfg.use_otaa) {
        // OTAA requires JoinEUI(8), DevEUI(8), AppKey(16).
        if (cfg.join_eui_len != 8 || cfg.dev_eui_len != 8 || cfg.app_key_len != 16) return false;
        if (!cfg.join_eui || !cfg.dev_eui || !cfg.app_key) return false;
    } else {
        // ABP requires NwkSKey(16) and AppSKey(16). DevAddr may be 0 only if assigned later.
        if (cfg.nwk_skey_len != 16 || cfg.app_skey_len != 16) return false;
        if (!cfg.nwk_skey || !cfg.app_skey) return false;
        // No strict check on dev_addr here; leave to application policy or region rules.
    }

    return true;
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
    
    gpio_put(0, 0);
}

void MeloperoPerpetuo::printResponse() {
    while (checkRxFifo(500)) {  // Keep checking the FIFO for new data
        for (size_t i = 0; i < responseLen; i++) {
            printf("0x%02X ", response[i]);
        }
        printf("\n");
    }
}

// Maps an execution status byte to a human-readable description.
static const char* exec_status_str(uint8_t s) {
    switch (s) {
        case 0x00: return "OK";
        case 0x01: return "Generic error (network not started?)";
        case 0x02: return "Invalid parameter";
        case 0x03: return "Timeout (no ACK)";
        case 0x04: return "No memory (reserved)";
        case 0x05: return "Unsupported option";
        case 0x06: return "Busy (channel activity, denied)";
        case 0x07: return "Duty-cycle limit";
        default:   return "Unknown status";
    }
}

// Returns the first payload byte (execution status) if present; 0xFF otherwise.
uint8_t MeloperoPerpetuo::getExecStatus() const {
    if (responseLen >= 4) return response[3];
    return 0xFF;
}

// Prints the execution status and, when available, retries and ACK RSSI fields.
// This function assumes the last response follows the standard EBI frame layout:
// [len_H][len_L][resp_id][status][optional...][checksum].
void MeloperoPerpetuo::processExecStatus() const {
    if (responseLen < 4) {
        printf("No valid response (len=%u)\n", (unsigned)responseLen);
        return;
    }

    const uint8_t status = response[3];
    printf("Execution status: 0x%02X (%s)\n", status, exec_status_str(status));

    // Best-effort parse of optional fields when present:
    // - retries: 1 byte at payload index 1 (overall index 4)
    // - ACK RSSI: 2 bytes (signed) at payload index 2..3 (overall 5..6)
    size_t idx = 4;
    if (idx < responseLen - 1) {
        const uint8_t retries = response[idx++];
        printf("Retries: %u\n", retries);
    }
    if (idx + 1 < responseLen - 1) {
        const int16_t ack_rssi = (int16_t)((response[idx] << 8) | response[idx + 1]);
        // idx += 2; // advance if more fields are parsed in the future
        printf("ACK RSSI: %d dBm\n", ack_rssi);
    }
}

void MeloperoPerpetuo::setEMBConfig(const EMBConfig& cfg) {
    // Stores configuration (application occurs on startLoRaEMB()).
    emb_config = cfg;
    emb_config_pending = true;  // Marks configuration as pending.
}

EMBConfig MeloperoPerpetuo::getEMBConfig() const {
    // Returns a copy of the stored configuration.
    return emb_config;
}

bool MeloperoPerpetuo::validateEMBConfig(const EMBConfig& cfg) const {
    // TX power: generic safe ceiling (adjust to module limits if needed).
    if (cfg.power > 0x14) return false;

    // Channel: generic 1..16 (adjust if your module provides a different map).
    if (cfg.channel < 1 || cfg.channel > 16) return false;

    // Spreading Factor: SF7..SF12.
    if (cfg.sf < SPREADING_FACTOR_7 || cfg.sf > SPREADING_FACTOR_12) return false;

    // Bandwidth: 125 or 250 kHz.
    if (cfg.bw != BANDWIDTH_125 && cfg.bw != BANDWIDTH_250) return false;

    // Coding rate: 4/5..4/8.
    if (cfg.cr < CODING_RATE_4_5 || cfg.cr > CODING_RATE_4_8) return false;

    // Optional Network ID coherence.
    if (cfg.net_id_len > 0 && cfg.net_id == nullptr) return false;

    // Energy save mode: 0..2 (ALWAYS_ON, RX_WINDOW, TX_ONLY).
    if (cfg.energy > ENERGY_SAVE_MODE_TX_ONLY) return false;

    return true;
}

TxStatus MeloperoPerpetuo::startLoRaEMB(bool force) {
    // Skips unnecessary restart when already in EMB mode, configuration is applied,
    // and no forced reapply is requested.
    if (mode == NetworkMode::LoRaEMB && emb_config_pending == false && !force) {
        return TxStatus::Ok;
    }

    // Validates the stored configuration before applying.
    if (!validateEMBConfig(emb_config)) {
        return TxStatus::InvalidArgs;
    }

    // Stops current network (required before changing radio options).
    stopNetwork();

    // Selects EMB as operating protocol (no LoRaWAN, no auto-join, no ADR).
    setNetworkPreferences(false, false, false);

    // Applies the stored EMB configuration (no start/stop inside setters).
    setOutputPower(emb_config.power);
    setOperatingChannel(emb_config.channel, emb_config.sf, emb_config.bw, emb_config.cr);
    setNetworkAddress(emb_config.net_addr);
    if (emb_config.net_id && emb_config.net_id_len) {
        setNetworkId((uint8_t*)emb_config.net_id, emb_config.net_id_len);
    }
    setEnergySaveMode(emb_config.energy);

    // Starts the network and marks configuration as synchronized.
    startNetwork();
    emb_config_pending = false;
    mode = NetworkMode::LoRaEMB;

    return TxStatus::Ok;
}



