## [Unreleased]

### Added
- Extended `TxStatus` enumeration to cover additional LoRaWAN execution results
  (generic error, timeout, unsupported option, channel busy, duty cycle limit, no response).
- High-level `transmitLoRaWAN()` helper that builds the `0x50` SEND_DATA frame
  as `[options_H][options_L][FPort][payload...]` and interprets the module response.
- `sendCmdTimeout()` helper to send commands with a configurable timeout and
  pre-TX RX FIFO flush.
- Optional firmware query via `CMD_FIRMWARE_VERSION (0x06)` and
  `printFirmwareVersion()` helper for diagnostics.
- Debug logging for TX frames and raw responses to simplify serial tracing.

### Changed
- `checkRxFifo()` now assembles complete, length-prefixed frames
  (`len_H`, `len_L`, payload, checksum) instead of reading raw bytes until timeout,
  improving robustness when multiple responses are queued.
- `startLoRaWAN()` now performs a complete configuration sequence:
  - Sets network preferences (`0x25`) using flags, class and region.
  - Sets energy save mode (`0x13`) according to LoRaWAN class.
  - Programs security keys (`0x26`) and physical address (`0x20`)
    from `LoRaWANConfig` (JoinEUI, DevEUI, AppKey).
  - Starts the network (`0x31`) and queries network status (`0x19`).
- `main.cpp` example updated to:
  - Use JoinEUI, DevEUI and AppKey as provisioned in the TTN console.
  - Configure LoRaWAN (EU868, Class A, OTAA, ADR, auto-join).
  - Periodically send an uplink on the configured FPort and log execution status.

### Fixed
- Improved handling of SEND_DATA responses by checking the response ID and
  mapping the execution status byte to `TxStatus`.
- Reduced risk of mixing frames in the UART RX path thanks to length-based
  frame reconstruction and RX FIFO flushing before long-timeout commands.


