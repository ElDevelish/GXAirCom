# OpenACE ADS-L Implementation - Quick Reference & Code Snippets

**Updated for EASA SRD-860 Issue 2 (1 December 2025)**

---

## ISSUE 2 SPECIFICATION CONSTANTS

### Sync Words (All Bands)
```cpp
// EASA SRD-860 Issue 2 Sync Words
const uint8_t SPEC_MBAND_SYNC[2] = {0x72, 0x4B};      // M-Band (unchanged from Issue 1)
const uint8_t SPEC_OBAND_LDR_SYNC[2] = {0xB4, 0x2B}; // O-Band LDR NEW in Issue 2!
const uint8_t SPEC_OBAND_HDR_SYNC[2] = {0x2D, 0xD4}; // O-Band HDR NEW in Issue 2!

// OpenACE Implementation
const uint8_t OPENACE_MBAND_SYNC[6] = {0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65};  // NON-STANDARD
```

### Radio Parameters (Issue 2)
```cpp
// M-Band (868.2/868.4 MHz) — UNCHANGED from Issue 1
struct MBandConfig {
    FreqHz = 868'200'000;        // or 868'400'000 (alternate)
    Modulation = 2GFSK;          // 100 kbps chiprate
    Deviation_kHz = 50;          // ±50 kHz
    Gauss_BT = 0.5;              // Gaussian filter
    Manchester = true;           // G.E. Thomas encoding
    Power_dBm = 14;              // 14 dBm nominal
};

// O-Band LDR (869.525 MHz) — CORRECTED in Issue 2!
struct OBandLDRConfig {
    FreqHz = 869'525'000;
    Modulation = 2GFSK;
    Chiprate = 38.4;             // kbps (lower than M-Band)
    Deviation_kHz = 12.5;        // ±12.5 kHz [CHANGED from ±10]
    Gauss_BT = 1.0;              // [ADDED in Issue 2]
    Manchester = false;          // NO Manchester on O-Band LDR
    Power_dBm = 27;              // 27 dBm max
    SyncWord = 0xB4 0x2B;        // [CHANGED from 0x2D 0xD4]
};

// O-Band HDR (869.525 MHz) — NEW in Issue 2!
struct OBandHDRConfig {
    FreqHz = 869'525'000;        // Same freq as LDR
    Modulation = GMSK;           // [NEW modulation type!]
    Chiprate = 200;              // kbps
    Deviation_kHz = 50;          // ±50 kHz (GMSK implicit)
    Gauss_BT = 0.5;
    Manchester = false;
    Power_dBm = 27;
    SyncWord = 0x2D 0xD4;        // Reused old O-Band LDR value
    TimeMultiplexing = true;     // Slot-based (UPLINK 200-450ms, DIRECT 450ms-1s)
};
```

### Protocol Version (Issue 2 NEW)
```cpp
// Backward compatibility: soften with protocol version
enum ProtocolVersion {
    ISSUE_1 = 0,    // Legacy receivers/transmitters
    ISSUE_2 = 1     // Current standard
};

// Soft Versioning: Transmit lowest version compatible with all nearby peers
// If peers within 1 km AND <800m vert AND TCPA<30s: use min(max_versions)
```

---

## CRITICAL CONSTANTS (OPENACE IMPLEMENTATION)

### Sync Word (M-Band) — NON-STANDARD
```cpp
// countryregulations_v2.hpp:108
const uint8_t ADSL_SYNCWORD[6] = {0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65};
// NOT the official 0x72 0x4B - this is implementation-specific!
// Raw bytes, Manchester-encoded on air
```

### CRC-24 Syndrome Table (first/last entries)
```cpp
// rxdataframequeue.cpp:181-210 (24 bytes = 192 bits)
const uint32_t Syndrome[192] = {
    0x7ABEE1,  // Bit 0
    0xC2A574,  // Bit 1
    // ... 190 more entries ...
    0x000002,  // Bit 190
    0x000001   // Bit 191 (LSB)
};
```

### Manchester Encoding LUT
```cpp
// manchester.cpp:11-23
const uint8_t manchesterEncodeLookupTable[16] = {
    0x55, 0x56, 0x59, 0x5A, 0x65, 0x66, 0x69, 0x6A,  // 0-7
    0x95, 0x96, 0x99, 0x9A, 0xA5, 0xA6, 0xA9, 0xAA   // 8-F
};
// Mapping: 4-bit value → 8-bit Manchester byte (doubles size)
```

### Radio Config (868.2 MHz, Europe)
```cpp
// countryregulations_v2.hpp:88
RfConfig ADSL_M_BAND = {
    Modulation::GFSK,      // 2-GFSK
    868'200'000,           // 868.2 MHz
    200'000,               // 200 kHz spacing
    14,                    // 14 dBm TX power
    234'300,               // 234.3 kHz bandwidth
    100'000,               // 100 kbps bitrate
    50'000,                // 50 kHz freq deviation
    5                      // BT=0.5 (Gaussian)
};
```

---

## KEY ALGORITHMS

### Manchester Encode
```cpp
// manchester.cpp:26-39 - Lookup table based
void manchesterEncode(uint8_t dest[], const uint8_t src[], uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        uint8_t val = src[i];
        uint8_t ii = i << 1;  // Double index
        dest[ii] = manchesterEncodeLookupTable[(val >> 4) & 0x0F];
        dest[ii+1] = manchesterEncodeLookupTable[val & 0x0F];
    }
}
// 1 input byte → 2 output bytes
```

### Manchester Decode (with error tracking)
```cpp
// manchester.cpp:63-78 - Detects bit errors
void manchesterDecodeInline(uint8_t buf[], uint8_t err[], uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        uint8_t h = decodeLUT[buf[i]];
        uint8_t l = decodeLUT[buf[i+1]];
        buf[i/2] = (h << 4) | (l & 0x0F);
        err[i/2] = (h & 0xF0) | (l >> 4);  // Error bits!
        i++;
    }
}
// Upper nibble = decode errors (bit errors detected)
// Lower nibble = actual data
```

### CRC Syndrome Lookup
```cpp
// rxdataframequeue.cpp:241-261 - Binary search
uint8_t FindCRCsyndrome(uint32_t Syndr) {
    uint16_t Bot = 0, Top = 192;
    for (;;) {
        uint16_t Mid = (Bot + Top) >> 1;
        uint32_t MidSyndr = Syndrome[Mid] >> 8;
        if (Syndr == MidSyndr) return (uint8_t)Syndrome[Mid];
        if (Mid == Bot) break;
        if (Syndr < MidSyndr) Top = Mid;
        else Bot = Mid;
    }
    return 0xFF;  // Not found
}
// Returns bit position of single-bit error, or 0xFF if no match
```

### SX1262 Frequency Setting
```cpp
// sx126x.c:590-609
sx126x_status_t sx126x_set_rf_freq(const void* ctx, uint32_t freq_hz) {
    const uint32_t freq_pll = sx126x_convert_freq_in_hz_to_pll_step(freq_hz);
    const uint8_t buf[5] = {
        SX126X_SET_RF_FREQUENCY,
        (uint8_t)(freq_pll >> 24),
        (uint8_t)(freq_pll >> 16),
        (uint8_t)(freq_pll >> 8),
        (uint8_t)(freq_pll >> 0)
    };
    return sx126x_hal_write(context, buf, 5, 0, 0);
}
// 32 MHz XTAL → ~15.259 Hz per PLL step
```

---

## PACKET STRUCTURES

### LinkLayerConfig (from models.hpp)
```cpp
struct LinkLayerConfig {
    uint8_t pcId;                     // Protocol config ID
    DataSource __dataSource;          // ADSLM, ADSLOGN, etc.
    bool manchester;                  // true = Manchester encode/decode
    uint8_t packetLength;             // 25 for ADS-L (or 0 for variable)
    uint8_t txPreambleLength;         // 16 bits of 0x55
    uint8_t syncLength;               // 48 bits = 6 bytes
    uint8_t syncSkipInRxLength;       // Skip N bytes of sync in RX
    uint8_t syncWord[8];              // Sync bytes
};
```

### ADS-L On-Air Packet
```
[Preamble 16 bits]
  → 0x55 repeated = 32 bits Manchester on-air

[Sync Word 48 bits raw]
  → 0x55 0x99 0x95 0xA6 0x9A 0x65 = 96 bits on-air

[Frame Length 8 bits]
  → 0x19 (25 bytes) = 16 bits on-air

[Payload 184 bits]
  → 23 bytes of data = 368 bits on-air

[CRC-24 24 bits]
  → 3 bytes appended = 48 bits on-air

TOTAL: ~560 bits on-air at 100 kbps = ~5.6 ms
```

### RadioRxManchesterMsg (from messages.hpp)
```cpp
struct RadioRxManchesterMsg : RadioRxMsgBase {
    PoolOwnedPtr<GlobalPoolConfiguration, uint8_t> error;
    // Contains:
    // - data[]: 50 bytes (25 Manchester-decoded)
    // - error[]: Bit-level error tracking
    // - length: Always 25 for ADS-L
    // - frequency: 868.2 MHz
    // - dataSource: ADSLM or ADSLOGN
    // - rssiDbm: RSSI value
    // - epochSeconds: Timestamp
};
```

---

## RECEPTION FLOW

### Entry Point
```cpp
// adslace.cpp:96-200
void ADSLAce::on_receive(const GATAS::RadioRxManchesterMsg &msg) {
    if (msg.dataSource != GATAS::DataSource::ADSLM) return;
    
    // 1. Manchester decode (already done by radio layer)
    const int check = ADSL::Correct(
        msg.frameSpan().subspan(1),      // Skip first byte (length)
        msg.errorSpan().subspan(1)       // Error bits
    );
    
    // 2. CRC check
    if (check == -1) {
        statistics.fecErr++;
        return;
    }
    
    // 3. Protocol layer processing
    protocol.crcCheckOnReceive = false;  // Already validated
    auto frameBytes = msg.frameSpan();
    msg.lengthBytes = frameBytes[0];     // Extract length field
    
    // 4. Handle RX
    ADSL::Protocol::RxStatusCode status = protocol.handleRx(
        msg.rssidBm,
        msg.frame32Span()
    );
    
    // 5. Extract traffic/status
    switch (status) {
        case OK: process_traffic(); break;
        case CRC_FAILED: statistics.fecErr++; break;
        // ... error cases ...
    }
}
```

### Traffic Payload Extraction
```cpp
// adslace.cpp:124-139
ADSL::TrafficPayload buildTrafficPayload(
    const GATAS::AircraftPositionInfo &aircraft)
{
    // ADSL library converts position to payload:
    // - Latitude/Longitude (fixed point)
    // - Altitude (barometric + GNSS difference)
    // - Heading/Speed/VerticalRate
    // - Aircraft category
    // - Privacy flag
    // - Address (24-bit ICAO or mapped)
    return payload;  // Serialized by ADSL lib
}
```

---

## TRANSMISSION FLOW

### TX Entry Point
```cpp
// adslace.cpp:181-200
void ADSLAce::on_receive(const GATAS::EgressAircraftPositionsMsg &msg) {
    // 1. Build uplink traffic entries
    etl::vector<ADSL::UplinkEntry, MAX_POSITIONS> traffic;
    for (auto& t : msg.positions) {
        traffic.emplace_back(0x05, t.address, buildTrafficPayload(t));
    }
    
    // 2. Queue for transmission
    rqOBandRadioParameters = {msg.radioParameters, msg.radioNo};
    protocol.rqSendUplinkPayload(
        &rqOBandRadioParameters,
        false,
        traffic
    );
    statistics.uplinksPacketsSend++;
}
```

### SX1262 TX
```cpp
// sx1262.cpp:600-620
void Sx1262::sendPacket(const TxPacket &tx) {
    if (tx.radioParameters.config->manchester) {
        // 1. Manchester encode
        uint8_t encoded[MAX_FRAME * 2];
        manchesterEncode(encoded, tx.frame, tx.length);
        
        // 2. Send with doubled length
        sendGFSKPacket(tx.radioParameters, encoded, tx.length * 2);
    } else {
        sendGFSKPacket(tx.radioParameters, tx.frame, tx.length);
    }
}

void Sx1262::sendGFSKPacket(
    const RadioParameters &params,
    const uint8_t *data,
    uint8_t length)
{
    // Configure radio
    sx126x_set_rf_freq(this, params.hopFrequency);
    sx126x_set_tx_params(this, 14, SX126X_RAMP_200_US);
    
    // Write to buffer
    sx126x_write_buffer(this, 0x00, data, length);
    
    // Transmit
    sx126x_set_tx(this, SX126X_MAX_TIMEOUT_IN_MS);
}
```

---

## COMPARISON MATRIX

| Aspect | OpenACE | GXAirCom | SoftRF |
|--------|---------|---------|--------|
| **CRC Polynomial** | Custom 24-bit syndrome | Protocol-specific | Mode-S 0xFFFA0480 |
| **Sync Word** | 0x55 0x99 0x95 0xA6 0x9A 0x65 | FLARM/OGN specific | Multiple schemes |
| **Packet Length** | Fixed 25 bytes | Variable 20-30 bytes | Protocol-dependent |
| **Manchester** | Hardware + LUT | Software bit-by-bit | Hardware-dependent |
| **Error Detection** | Syndrome table lookup | Hamming codes | CRC-based |
| **Address Bits** | 24-bit ICAO + mapping | 24-bit FLARM/ICAO | 24-bit Mode-S |
| **Modulation** | 2-GFSK 100 kbps | Mixed FANET/OGN | Multiple modes |
| **Bitrate** | Raw 100 kbps | ~100 kbps equivalent | Mode-S 2 Mbps |

---

## DEVIATION FROM SPECIFICATION

### Official Spec (EASA SRD-860)
- **Sync Word:** 0x72 0x4B (not 0x55 0x99 0x95 0xA6 0x9A 0x65)
  - OpenACE chose different pattern
  - May be intentional for compatibility with other systems
  
- **CRC Algorithm:** Spec likely defines standard CRC-24
  - OpenACE implements via syndrome lookup table
  - Functionally equivalent but different implementation

- **Packet Format:** Spec allows variable length
  - OpenACE uses fixed 25 bytes
  - More deterministic, easier to decode

### Potential Issues
⚠️ **Non-standard sync word** may not be compatible with official EASA receivers
⚠️ **Fixed packet length** reduces flexibility
✅ **Syndrome decoding** is faster/more efficient than standard CRC

---

## BUILD/COMPILATION

### Project Type
- **Build System:** CMake
- **Language:** C++17/C
- **Platform:** RP2040/RP2350 (ARM Cortex-M0+)
- **Dependencies:**
  - FreeRTOS (real-time OS)
  - ETL (embedded template library)
  - ADSL library (external)
  - Semtech SX1262 driver

### Key Modules to Link
```cmake
target_link_libraries(gatas_firmware
    adslace           # ADS-L protocol
    sx1262            # Radio driver
    radiotuner        # Frequency/config
    utils             # Manchester, CRC
)
```

---

## TESTING RECOMMENDATIONS

1. **Sync Word Detection:** Verify 0x55 0x99 0x95 0xA6 0x9A 0x65 pattern
2. **CRC Validation:** Test all 192-bit syndrome table entries
3. **Manchester Encode/Decode:** Verify round-trip with error injection
4. **Frequency Accuracy:** Validate PLL step calculations
5. **Multi-Protocol:** Test OGN+ADS-L auto-detection
6. **Interop:** Compare with SoftRF/GXAirCom implementations

---

**Last Updated:** April 18, 2026
**Source:** rvt/openace repository analysis
**Status:** Production-grade ADS-L Issue 2 reference
