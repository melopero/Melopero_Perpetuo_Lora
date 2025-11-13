#include "MeloperoPerpetuo.h"

#define UART_PORT uart1

// Forward declarations of helper functions
static uint8_t send_prefs_try(MeloperoPerpetuo& m, uint8_t flags, int variant, uint8_t klass, uint8_t region);




// Constructor
MeloperoPerpetuo::MeloperoPerpetuo() {

    // Initializes network state.
    mode = NetworkMode::None;
    network_running = false;

    // Marks default EMB configuration as pending until first apply.
    emb_config_pending = true;

    // Marks default LoRaWAN configuration as pending until first apply.
    lorawan_config_pending = true;

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

    // DEBUG: dump del frame in uscita
    printf("[TX] %02X bytes: ", (unsigned)packetLen);
    for (size_t i = 0; i < packetLen; ++i) printf("%02X ", packetBuffer[i]);
    printf("\n");

    // Transmit the packet via UART
    uart_write_blocking(UART_PORT, packetBuffer, packetLen);

    checkRxFifo(1000);
}

static inline void uart_flush_rx() {
    while (uart_is_readable(UART_PORT)) (void)uart_getc(UART_PORT);
}

void MeloperoPerpetuo::sendCmdTimeout(uint8_t command, uint8_t* payload, size_t payloadLen, uint32_t timeout_ms) {
    size_t totalPacketSize = payloadLen + 4;
    uint8_t packetBuffer[totalPacketSize];
    size_t packetLen;

    buildPacket(command, payload, payloadLen, packetBuffer, &packetLen);

    // DEBUG: dump del frame in uscita (con timeout)
    printf("[TX][T=%u ms] %02X bytes: ", (unsigned)timeout_ms, (unsigned)packetLen);
    for (size_t i = 0; i < packetLen; ++i) printf("%02X ", packetBuffer[i]);
    printf("\n");

    uart_flush_rx();
    uart_write_blocking(UART_PORT, packetBuffer, packetLen);

    checkRxFifo(timeout_ms);
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

TxStatus MeloperoPerpetuo::transmitLoRaWAN(const uint8_t* data, size_t len,
                                           int fport_override,
                                           int confirmed_override)
{
    if (mode != NetworkMode::LoRaWAN) return TxStatus::InvalidArgs;

    const uint8_t fport = (fport_override >= 0) ? (uint8_t)fport_override
                                                : lorawan_config.default_fport;
    const bool confirmed = (confirmed_override >= 0)
                         ? (bool)confirmed_override
                         : lorawan_config.default_confirmed;

    // Options: 0x0C00 = confirmed uplink, 0x0000 = unconfirmed.
    const uint16_t options = confirmed ? 0x0C00 : 0x0000;

    // Cap payload length to avoid overrunning the internal buffer.
    if (len + 3 > MAX_PACKET_SIZE) len = MAX_PACKET_SIZE - 3;

    // Payload layout: [opt_H][opt_L][FPort][app_data...]
    uint8_t payload[MAX_PACKET_SIZE];
    size_t idx = 0;
    payload[idx++] = (uint8_t)((options >> 8) & 0xFF); // 0x0C
    payload[idx++] = (uint8_t)(options & 0xFF);        // 0x00
    payload[idx++] = fport;                            // es. 0x01
    if (data && len) { memcpy(&payload[idx], data, len); idx += len; }

    // Log TX in a human-readable form.
    printf("[LoRaWAN] TX (confirmed=%d, fport=%u, bytes=%u)\n",
           confirmed ? 1 : 0, fport, (unsigned)len);

    // Use a long timeout (up to ~65 s) as the first TX may involve additional overhead.
    sendCmdTimeout(CMD_SEND_DATA, payload, idx, 65000);

    // The response frame may be fragmented or interleaved with other events:
    // if it is not valid yet, try to collect more data for a few extra iterations.
    auto is_send_resp = [this]() {
        return (this->responseLen >= 4) && (this->response[2] == 0xD0);
    };

    if (!is_send_resp()) {
        // extra attempts to collect the response
        for (int i = 0; i < 6 && !is_send_resp(); ++i) {
            // 6 * 1000ms = ~6s extra
            checkRxFifo(1000);
        }
    }

    // Dump raw response
    printf("[LoRaWAN] SEND_DATA resp (%u bytes): ", (unsigned)this->responseLen);
    for (size_t i = 0; i < this->responseLen; ++i) printf("%02X ", this->response[i]);
    printf("\n");

    

// Expected layout: [len_H][len_L][0xD0][status][...][checksum]
if (this->responseLen < 4 || this->response[2] != 0xD0) {
    printf("No valid SEND_DATA response (id != 0xD0)\n");
    return TxStatus::NoResponse;
}

const uint8_t status = this->response[3];
// Optional fields (if any)
int yy_present = (this->responseLen >= 5);
int zz_present = (this->responseLen >= 6);
uint8_t yy = yy_present ? this->response[4] : 0xFF;
uint8_t zz = zz_present ? this->response[5] : 0xFF;

printf("Execution status: 0x%02X", status);
if (yy_present) printf(", YY=%u", (unsigned)yy);
if (zz_present) printf(", ZZ=%s", (zz == 0x00 ? "ACK OK" : "ACK MISS/NA"));
printf("\n");

// Status Map -> TxStatus
switch (status) {
    case 0x00: return TxStatus::Ok;          // success
    case 0x01: return TxStatus::Error;       // generic error
    case 0x02: return TxStatus::InvalidArgs; // invalid parameter
    case 0x03: return TxStatus::Timeout;     // timeout / no ACK received
    case 0x05: return TxStatus::Unsupported; // unsupported option
    case 0x06: return TxStatus::ChannelBusy; // busy / CAD denied
    case 0x07: return TxStatus::DutyCycle;   // duty-cycle limit reached
    default:   return TxStatus::Error;       // unknown status code
}
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
    

    uint8_t flags = 0;
    flags |= (1<<7); // LoRaWAN
    if (lorawan_config.auto_join) flags |= (1<<6);
    if (lorawan_config.adr)       flags |= (1<<5);

    uint8_t prefs_payload[3] = { flags, lorawan_config.region /*0x00 EU868?*/, lorawan_config.klass /*0x01 A, 0x00 C*/ };
    sendCmd(CMD_SET_NETWORK_PREFERENCES, prefs_payload, sizeof(prefs_payload));
    printf("[LoRaWAN] Set prefs(flags,region,class) -> status=0x%02X\n", getExecStatus());
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
    responseLen = 0;
    memset(response, 0, sizeof(response));
    uint32_t start = to_ms_since_boot(get_absolute_time());

    // Wait until at least one byte is available or timeout expires.
    while ((to_ms_since_boot(get_absolute_time()) - start) < timeoutMs) {
        if (uart_is_readable(UART_PORT)) break;
        tight_loop_contents();
    }
    if (!uart_is_readable(UART_PORT)) return false;

    // Read the 2-byte length header.
    while (uart_is_readable(UART_PORT) && responseLen < 2) {
        response[responseLen++] = uart_getc(UART_PORT);
    }
    if (responseLen < 2) return false;

    const uint16_t total_len = ((uint16_t)response[0] << 8) | response[1];
    if (total_len < 4 || total_len > sizeof(response)) return false;

    while (responseLen < total_len) {
        if (uart_is_readable(UART_PORT)) {
            response[responseLen++] = uart_getc(UART_PORT);
        } else if ((to_ms_since_boot(get_absolute_time()) - start) > timeoutMs) {
            break;
        }
    }
    return (responseLen == total_len);
}



bool MeloperoPerpetuo::readRxFifo(uint8_t* response, size_t* responseLen, size_t maxBufferSize) {
    size_t bytesRead = 0;
    while (uart_is_readable(UART_PORT) && *responseLen < maxBufferSize) {
        uint8_t byte = uart_getc(UART_PORT);
        response[*responseLen] = byte;
        (*responseLen)++;
        bytesRead++;
    }
    return bytesRead > 0;
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

bool MeloperoPerpetuo::getFirmwareVersion(uint8_t out[4]) {
    
    // Try the dedicated firmware version command (may not be implemented on all revisions).
    sendCmd(CMD_FIRMWARE_VERSION, nullptr, 0);

    if (responseLen >= 8 && response[2] == 0x86) {
        out[0] = response[3];
        out[1] = response[4];
        out[2] = response[5];
        out[3] = response[6];
        return true;
    }

    // If the command is not supported or the payload is too short, report FW as unavailable.
    return false;
}


void MeloperoPerpetuo::printFirmwareVersion() {
    uint8_t v[4] = {0};
    if (getFirmwareVersion(v)) {
        printf("FW: %02X %02X %02X %02X\n", v[0], v[1], v[2], v[3]);
    } else {
        printf("FW: <unavailable>\n");
    }
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

TxStatus MeloperoPerpetuo::startLoRaWAN(bool force) {
    // Skip restart if already configured and no force requested.
    if (mode == NetworkMode::LoRaWAN && !lorawan_config_pending && !force)
        return TxStatus::Ok;

    // Validate configuration.
    if (!validateLoRaWANConfig(lorawan_config))
        return TxStatus::InvalidArgs;

    // Put module in a clean state.
    reset();
    sleep_ms(300);
    stopNetwork();
    sleep_ms(100);

    // --- PREFS (0x25) ---
    uint8_t flags = 0;
    flags |= (1 << 7);
    if (lorawan_config.auto_join) flags |= (1 << 6);
    if (lorawan_config.adr)       flags |= (1 << 5);

    uint8_t prefs_ok = 0xFF;

    const int variants[3] = {0, 1, 2};
    const uint8_t classes_try[2] = { (uint8_t)0x01, (uint8_t)0x00 };
    const uint8_t regions_try[4] = { lorawan_config.region, (uint8_t)0x00, (uint8_t)0x01, (uint8_t)0x02 };

    for (int vi = 0; vi < 3; ++vi) {
        int variant = variants[vi];
        for (int ci = 0; ci < 2; ++ci) {
            uint8_t klass_try = classes_try[ci];
            for (int ri = 0; ri < 4; ++ri) {
                uint8_t region_try = regions_try[ri];
                uint8_t st = send_prefs_try(*this, flags, variant, klass_try, region_try);
                sleep_ms(120);
                if (st == 0x00) {
                    printf("[PREFS] accepted variant=%d class=0x%02X region=0x%02X\n", variant, klass_try, region_try);
                    prefs_ok = 0x00;
                    goto PREFS_DONE;
                }
            }
        }
    }
    PREFS_DONE:


    // --- ENERGY (0x13) ---
    const uint8_t energy = (lorawan_config.klass == 0x01) ?
                        ENERGY_SAVE_MODE_RX_WINDOW : ENERGY_SAVE_MODE_ALWAYS_ON;
    setEnergySaveMode(energy);
    printf("[LoRaWAN] Set energy -> 0x%02X\n", getExecStatus());
    sleep_ms(120);

    // --- KEYS (0x26): AppKey (0x01) e NwkKey (0x00), MSB order ---
    if (lorawan_config.use_otaa && lorawan_config.app_key && lorawan_config.app_key_len == 16) {

        // AppKey (selector 0x01)
        {
            uint8_t pl[1 + 16];
            pl[0] = 0x01;
            memcpy(&pl[1], lorawan_config.app_key, 16);
            sendCmd(CMD_SET_NETWORK_SECURITY, pl, sizeof(pl));
            printf("[KEY sel=0x01] AppKey MSB -> status=0x%02X\n", getExecStatus());
        }

        // NwkKey (selector 0x00)
        {
            uint8_t pl[1 + 16];
            pl[0] = 0x00;
            memcpy(&pl[1], lorawan_config.app_key, 16);
            sendCmd(CMD_SET_NETWORK_SECURITY, pl, sizeof(pl));
            printf("[KEY sel=0x00] NwkKey MSB -> status=0x%02X\n", getExecStatus());
        }

        sleep_ms(200);
    }





    // --- PHYSICAL ADDRESS (0x20): JoinEUI(8) then DevEUI(8), already in LSB order ---
    if (lorawan_config.use_otaa &&
        lorawan_config.join_eui && lorawan_config.dev_eui &&
        lorawan_config.join_eui_len == 8 && lorawan_config.dev_eui_len == 8) {

        uint8_t phy[16];

        // Values are passed as-is; the application must provide them in the format
        // expected by the module (no implicit byte swapping is performed here).
        memcpy(phy, lorawan_config.join_eui, 8);      // JoinEUI
        memcpy(phy + 8, lorawan_config.dev_eui, 8);   // DevEUI

        // Debug: print values as sent to the module.
        printf("[DEBUG] JoinEUI direct: ");
        for (int i = 0; i < 8; ++i) printf("%02X", phy[i]);
        printf("\n");

        printf("[DEBUG] DevEUI direct: ");
        for (int i = 8; i < 16; ++i) printf("%02X", phy[i]);
        printf("\n");

        sendCmd(CMD_SET_PHYSICAL_ADDRESS, phy, sizeof(phy));
        printf("[PHY 0x20] JoinEUI|DevEUI direct -> status=0x%02X\n", getExecStatus());
        sleep_ms(200);
    }







    // Start the network (auto-join if enabled).
    startNetwork();
    printf("[LoRaWAN] Start network -> status=0x%02X\n", getExecStatus());
    sleep_ms(100);

    // --- TEST: ask for network status ---
    sendCmd(CMD_GET_NETWORK_STATUS, nullptr, 0);
    printf("[LoRaWAN] Network status resp (%u bytes): ", (unsigned)this->responseLen);
    for (size_t i = 0; i < this->responseLen; i++) printf("%02X ", this->response[i]);
    printf("\n");

    // Mark configuration as synced.
    lorawan_config_pending = false;
    mode = NetworkMode::LoRaWAN;
    return TxStatus::Ok;
}

// ------------------
// Helper SET_NETWORK_PREFERENCES (0x25)
// ------------------
uint8_t send_prefs_try(MeloperoPerpetuo& m, uint8_t flags, int variant, uint8_t klass, uint8_t region) {
    if (variant == 0) {
        m.sendCmd(CMD_SET_NETWORK_PREFERENCES, &flags, 1);
        printf("[PREFS] flags-only -> 0x%02X\n", m.getExecStatus());
        return m.getExecStatus();
    } else if (variant == 1) {
        uint8_t p[3] = { flags, klass, region };
        m.sendCmd(CMD_SET_NETWORK_PREFERENCES, p, sizeof(p));
        printf("[PREFS] flags,class,region (class=0x%02X,region=0x%02X) -> 0x%02X\n",
               klass, region, m.getExecStatus());
        return m.getExecStatus();
    } else {
        uint8_t p[3] = { flags, region, klass };
        m.sendCmd(CMD_SET_NETWORK_PREFERENCES, p, sizeof(p));
        printf("[PREFS] flags,region,class (region=0x%02X,class=0x%02X) -> 0x%02X\n",
               region, klass, m.getExecStatus());
        return m.getExecStatus();
    }
}

