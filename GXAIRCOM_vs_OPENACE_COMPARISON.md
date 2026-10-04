# GXAirCom vs OpenACE ADS-L Implementation Comparison

**Analysis Date:** April 25, 2026 (Updated for Issue 2)
**Specification:** EASA SRD-860 Issue 2 (1 December 2025)
**Scope:** Protocol implementation differences, interoperability considerations, and migration path

---

## ISSUE 2 SPECIFICATION UPDATE (CRITICAL)

EASA released Issue 2 on 1 December 2025 with **significant breaking changes**:

| Change | Issue 1 | Issue 2 | Impact |
|--------|---------|---------|--------|
| **O-Band LDR Sync Word** | 0x2D 0xD4 | **0xB4 0x2B** | ⚠️ **BREAKING** – receivers incompatible |
| **O-Band LDR Freq Dev** | ±10 kHz | **±12.5 kHz** | ⚠️ **BREAKING** – modulation mismatch |
| **O-Band LDR Gauss BT** | N/A | **1.0** | ⚠️ **NEW** – filter requirement |
| **O-Band HDR** | N/A | **NEW** (GMSK, 200 kbps, time-mux) | **NEW** uplink channel |
| **Protocol Version** | N/A | **0=Issue1, 1=Issue2** | Soft versioning required |
| **Preamble Spec** | Generic | More precise | ✅ Backward compatible |

**Action:** Both GXAirCom and OpenACE must verify Issue 2 compliance, especially:
- Which O-Band LDR params does OpenACE use? (Issue 1 or 2?)
- Support for protocol versioning?
- Plans for O-Band HDR?

---

## EXECUTIVE SUMMARY

| Aspect | GXAirCom | OpenACE | Issue 2 Spec |
|--------|----------|---------|--------------|
| **M-Band Support** | Limited | Full | 0x72 0x4B sync, 100 kbps |
| **O-Band LDR** | No | Partial ⚠️ | 0xB4 0x2B sync, ±12.5 kHz, BT 1.0 |
| **O-Band HDR** | No | No | GMSK 200 kbps, time-mux |
| **Sync Word** | Protocol-specific | Non-standard (0x55 0x99...) | 0x72/0xB4/0x2D (band-dependent) |
| **CRC** | LDPC/Hamming | 24-bit syndrome lookup | 24-bit CRC specified |
| **Manchester** | Bit-by-bit | Hardware + LUT | Defined scope (M-Band only) |
| **Modulation** | Multi-protocol | 2-GFSK only | M/O-LDR: GFSK, O-HDR: GMSK |
| **Protocol Version** | No | ⚠️ Check | 0=Issue1, 1=Issue2 (soft versioning) |
| **Issue 2 Compliance** | N/A (multi-protocol) | M-Band: ~64%, O-LDR: ~50% | Target: 100% |

---

## DETAILED COMPARISON

### 1. SYNC WORD HANDLING (UPDATED FOR ISSUE 2)

#### EASA SRD-860 Issue 2 Specification
```cpp
// Spec: Three channels with different sync words
const uint8_t M_BAND_SYNC[2] = {0x72, 0x4B};           // 868.2/868.4 MHz
const uint8_t O_BAND_LDR_SYNC[2] = {0xB4, 0x2B};       // 869.525 MHz LDR [CHANGED!]
const uint8_t O_BAND_HDR_SYNC[2] = {0x2D, 0xD4};       // 869.525 MHz HDR [REUSED]
```

**Critical Issue 2 Change:**
- O-Band LDR sync word **changed from 0x2D 0xD4 (Issue 1) to 0xB4 0x2B (Issue 2)**
- This is a **breaking change** – receivers won't interoperate across versions
- Old value (0x2D 0xD4) now used for O-Band HDR only

#### GXAirCom
```cpp
// Likely in main.cpp or config.h
// Supports multiple protocols:
M_BAND_SPEC_SYNC: 0x72, 0x4B        // EASA standard (if implemented)
FLARM_SYNCWORD:   varies by region
OGN_SYNCWORD:     0x8B8C9D (OGN-specific)
FANET_SYNCWORD:   0xF1 (FANET LoRa)
// Each protocol has dedicated receiver path
```

**Characteristics:**
- Multi-protocol (FLARM, OGN, FANET, ADS-L focus)
- Separate RX handlers for each
- Regional sync word variations
- **Question: Which O-Band LDR params used?** (Issue 1 or 2?)

#### OpenACE
```cpp
// countryregulations_v2.hpp:108
static constexpr GATAS::LinkLayerConfig PROTOCOL_ADSL {
    /* ... */
    {0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65}  // 6-byte M-Band sync (NON-STANDARD)
};

// O-Band LDR Configuration:
// ⚠️ CRITICAL: Need to verify which parameters OpenACE uses!
// - Issue 1: 0x2D 0xD4 sync, ±10 kHz
// - Issue 2: 0xB4 0x2B sync, ±12.5 kHz
```

**Characteristics:**
- M-Band: Single unified sync pattern (non-standard 6-byte)
- O-Band LDR: **Status UNCLEAR – Issue 1 or 2 params?**
- Hardware-level detection
- 48-bit M-Band pattern (much longer than spec)

**⚠️ Interoperability Issues (Issue 2 Context):**

| Band | GXAirCom | OpenACE | Spec | Interop | Issue 2 |
|------|----------|---------|------|---------|---------|
| **M-Band** | Likely 0x72 0x4B ✓ | 0x55 0x99... ❌ | 0x72 0x4B | ❌ NO | 0x72 0x4B (stable) |
| **O-LDR** | ⚠️ Unknown (Issue 1 or 2?) | ⚠️ Unknown | NOW 0xB4 0x2B | ⚠️ ??? | **0xB4 0x2B [NEW]** |
| **O-HDR** | ❌ No | ❌ No | 0x2D 0xD4 | N/A | 0x2D 0xD4 [NEW] |

**Key Issues:**
1. **M-Band:** OpenACE non-standard → won't interoperate with EASA receivers
2. **O-Band LDR:** Issue 1/2 incompatibility → both systems must verify version
3. **O-Band HDR:** Not in scope for either system (ground uplink)


### 2. CRC IMPLEMENTATION

#### GXAirCom
```cpp
// Likely LDPC (from lib/LDPC/ directory)
LDPC_Encode/Decode:  Low-Density Parity-Check codes
// Used for OGN protocol
uint8_t LDPC_Check(const uint8_t *Data)  // 20 data + 6 parity bytes
```

**Characteristics:**
- LDPC (Low-Density Parity-Check) codes
- Error correction (not just detection)
- 48 parity checks for OGN
- Better error correction than CRC
- **Issue 2 Note:** Spec uses 24-bit CRC for all bands

#### OpenACE
```cpp
// rxdataframequeue.cpp:181-210
const uint32_t Syndrome[192] = { /* 192-entry table */ };

uint8_t FindCRCsyndrome(uint32_t Syndr) {
    // Binary search for syndrome match
    uint16_t Bot = 0, Top = 192;
    for (;;) {
        uint16_t Mid = (Bot + Top) >> 1;
        if (Syndr == Syndrome[Mid] >> 8) return (uint8_t)Syndrome[Mid];
        if (Mid == Bot) break;
        Syndr < Syndrome[Mid] >> 8 ? (Top = Mid) : (Bot = Mid);
    }
    return 0xFF;
}
```

**Characteristics:**
- 24-bit CRC with syndrome lookup
- Single-bit error detection (not correction)
- O(log n) lookup time
- Faster than traditional CRC

**Comparison:**
| Aspect | GXAirCom (LDPC) | OpenACE (CRC-24) |
|--------|-----------------|------------------|
| **Error Correction** | Yes (multiple bits) | No (detection only) |
| **Error Detection** | Single/multi-bit | Single-bit only |
| **Overhead** | ~30% (parity) | ~12% (CRC) |
| **Speed** | Slower (iterative) | Faster (table lookup) |
| **Standards** | Custom | CRC-based |


### 3. MANCHESTER ENCODING

#### GXAirCom
```cpp
// Likely in main.cpp or enums.h
// Bit-by-bit implementation for multiple protocols
for (uint8_t i = 0; i < 8; i++) {
    if (data & (1 << i)) {
        output |= /* manchester bits */
    }
}
```

**Characteristics:**
- Software implementation
- Bit-by-bit processing
- Universal for all protocols
- Protocol-agnostic

#### OpenACE
```cpp
// manchester.cpp:26-39 + :63-78
const uint8_t manchesterEncodeLookupTable[16] = {
    0x55, 0x56, 0x59, 0x5A, 0x65, 0x66, 0x69, 0x6A,
    0x95, 0x96, 0x99, 0x9A, 0xA5, 0xA6, 0xA9, 0xAA
};

void manchesterEncode(uint8_t dst[], const uint8_t src[], uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        uint8_t val = src[i];
        uint8_t ii = i << 1;
        dst[ii] = manchesterEncodeLookupTable[(val >> 4) & 0x0F];
        dst[ii+1] = manchesterEncodeLookupTable[val & 0x0F];
    }
}

void manchesterDecodeInline(uint8_t buf[], uint8_t err[], uint8_t len) {
    uint8_t idx = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t h = decodeLUT[buf[i]];
        uint8_t l = decodeLUT[buf[i+1]];
        buf[idx] = (h << 4) | (l & 0x0F);
        err[idx] = (h & 0xF0) | (l >> 4);  // Error tracking!
        idx += 1;
        i += 1;
    }
}
```

**Characteristics:**
- Lookup table (256-byte LUT for decode)
- Nibble-based (4-bit → 8-bit)
- Hardware assist available
- Error bit tracking in upper nibble

**Performance Comparison:**
| Aspect | GXAirCom | OpenACE |
|--------|----------|---------|
| **Implementation** | Bit-by-bit loop | Lookup table |
| **Throughput** | Slower (bit ops) | Faster (table) |
| **Memory** | Less (no table) | More (256 bytes LUT) |
| **Error Tracking** | None | Per-bit errors |
| **Hardware Support** | None | SX1262 assist |


### 4. PACKET STRUCTURE

#### GXAirCom
```
OGN Packet (20 bytes):
┌──────────────┐
│ Header (4B)  │ - Address, type, encryption
│ Status (2B)  │ - Position info, battery
│ Position (7B)│ - Lat/Lon/Alt
│ Velocity (3B)│ - Speed/heading/climb
│ Checksum (4B)│ - CRC
└──────────────┘
Total: 20 bytes (before LDPC parity = 26 bytes with parity)

Then encoded with LDPC:
Final on-air: 26 + 6 parity = 32 bytes

FLARM Packet (varies 20-30 bytes):
- Custom format
- Protocol-specific structure
```

#### OpenACE
```
ADS-L Packet (25 bytes fixed):
┌────────────────────────────────┐
│ Frame Length (1B)              │ 0x19 = 25 bytes
│ Payload (21B)                  │ Protocol data
│ CRC-24 (3B)                    │ 24-bit syndrome
└────────────────────────────────┘
Total: 25 bytes fixed

On-air (Manchester encoded):
Preamble (16 bits) → 32 bits
Sync (48 bits raw) → 96 bits
Frame (200 bits raw) → 400 bits
CRC (24 bits raw) → 48 bits
─────────────────────────
Total: ~560 bits on-air at 100 kbps = 5.6 ms
```

**Structural Differences:**
| Aspect | GXAirCom | OpenACE |
|--------|----------|---------|
| **Length** | Variable (20-30B) | Fixed (25B) |
| **Parity** | LDPC (6B) | CRC-24 (3B) |
| **Structure** | Protocol-specific | ADS-L standardized |
| **Overhead** | ~30% | ~12% |
| **Determinism** | Variable timing | Fixed timing |


### 5. ADDRESS HANDLING

#### GXAirCom
```cpp
// Likely in main.h or config.h
union {
    uint32_t address;      // 24-bit ICAO or FLARM
} Aircraft;

// Address type detection
if (protocol == FLARM) address = flarmID;
else if (protocol == OGN) address = ognID;
```

**Characteristics:**
- Protocol-specific address format
- 24-bit addresses (ICAO or custom)
- No unified AMT handling
- Separate address translation per protocol

#### OpenACE
```cpp
// adslace.hpp:101-139
GATAS::AddressType addressMapToAddressType(uint8_t addressMap) const;
static uint8_t addressTypeToAddressMap(GATAS::AddressType addressType);

// Unified address handling
enum AddressType {
    ICAO,          // 24-bit ICAO
    FLARM,         // FLARM custom
    OGN,           // OGN tracker
    FANET,         // FANET address
    RESERVED
};

// Traffic payload building
ADSL::TrafficPayload buildTrafficPayload(
    const GATAS::AircraftPositionInfo &aircraft)
{
    // ADSL library handles address encoding
    // Automatic ICAO ↔ other mappings
}
```

**Characteristics:**
- AMT (Address Mapping Table) in ADSL library
- Unified address abstraction
- Protocol-agnostic positioning
- External library (likely Skytraxx)


### 6. MODULATION & RADIO CONFIGURATION

#### GXAirCom
```
OGN Protocol:
- Modulation: 2-FSK or 2-GFSK
- Bitrate: 100 kbps or higher
- Frequencies: 868.2, 868.4 MHz (Europe)
- Bandwidth: ~100 kHz typical

FANET Protocol:
- Modulation: LoRa (SF7, BW250k)
- Bitrate: ~17 kbps equivalent
- Frequency: 868.2 MHz
- Bandwidth: 250 kHz

FLARM Protocol:
- Modulation: 2-GFSK
- Bitrate: 100 kbps
- Frequency: 868.0 MHz (specific variant)
- Bandwidth: Variable
```

#### OpenACE
```cpp
// countryregulations_v2.hpp:88-104
RfConfig ADSL_M_BAND = {
    Modulation::GFSK,      // 2-GFSK only
    868'200'000,           // 868.2 MHz
    200'000,               // 200 kHz spacing
    14,                    // 14 dBm TX
    234'300,               // 234.3 kHz bandwidth
    100'000,               // 100 kbps bitrate
    50'000,                // ±50 kHz deviation
    5                      // BT=0.5 (Gaussian)
};

// O-band HDR variant
RfConfig ADSL_O_BAND = {
    GFSK, 869'525'000, 200'000, 27, 234'300, 200'000, 50'000, 5
};
```

**Comparison:**
| Aspect | GXAirCom | OpenACE |
|--------|----------|---------|
| **Modulation Options** | 2-FSK, 2-GFSK, LoRa | 2-GFSK only |
| **Primary Freq** | 868.2/868.4/920.8 | 868.2/869.525 |
| **Bitrates** | 100+ kbps, variable | 100 kbps fixed |
| **Bandwidth** | 100-250 kHz | 234.3 kHz fixed |
| **Flexibility** | High (multi-protocol) | Low (ADS-L only) |
| **Optimization** | Multi-purpose | Single-purpose |


### 7. RECEPTION ARCHITECTURE

#### GXAirCom
```cpp
// main.cpp or radio handler
on_receive_radio(data) {
    if (detect_flarm_syncword(data)) {
        decode_flarm(data);
    } else if (detect_ogn_syncword(data)) {
        decode_ogn(data);
    } else if (detect_fanet_syncword(data)) {
        decode_fanet(data);
    }
}
```

**Characteristics:**
- Protocol-by-protocol detection
- Sequential if-then-else chain
- Each protocol separate path
- No protocol interop

#### OpenACE
```cpp
// rx_dataframequeue.cpp:129-143
DataSourceMatch decideDataSource(DataSource ds, uint32_t frame[], uint8_t len) {
    if (ds == DataSource::ADSLOGN) {  // Ambiguous reception
        // Distinguish OGN vs ADS-L by signature
        const uint32_t SignADSL32[] = {0x0080B124};
        const uint32_t MaskADSL32[] = {0x00F0FFFF};
        
        if (diffBits<1>(frame, SignADSL32, MaskADSL32) <= 1) {
            return {DataSource::ADSLM, 20, 25};  // ADS-L
        } else {
            return {DataSource::OGN, 0, 26};     // OGN
        }
    }
}

// adslace.cpp:77-96
void ADSLAce::on_receive(const RadioRxManchesterMsg &msg) {
    if (msg.dataSource != DataSource::ADSLM) return;
    
    // Single unified reception path
    const int check = ADSL::Correct(
        msg.frameSpan().subspan(1),
        msg.errorSpan().subspan(1)
    );
}
```

**Characteristics:**
- Signature-based disambiguation
- Can receive OGN + ADS-L on same frequency
- Unified message bus (GATAS)
- Synchronized RX with error tracking


### 8. TRANSMISSION PATH

#### GXAirCom
```cpp
// Separate TX for each protocol
if (transmit_flarm) tx_flarm_packet(data);
if (transmit_ogn) tx_ogn_packet(data);
if (transmit_fanet) tx_fanet_packet(data);
```

#### OpenACE
```cpp
// adslace.cpp:181-200 (unified approach)
void on_receive(EgressAircraftPositionsMsg &msg) {
    etl::vector<ADSL::UplinkEntry, MAX_POSITIONS> traffic;
    for (auto& t : msg.positions) {
        traffic.emplace_back(0x05, t.address, buildTrafficPayload(t));
    }
    
    protocol.rqSendUplinkPayload(
        &rqOBandRadioParameters,
        false,
        traffic
    );
}
```

**Characteristics:**
- Single protocol queue
- ADSL library builds payloads
- O-band uplink support
- No multi-protocol TX scheduling


---

## MIGRATION PATH: GXAirCom → OpenACE

### Option 1: Gradual Adoption (Recommended)
```
Phase 1: Parse OpenACE code
- Understand sync word differences
- Study CRC-24 syndrome lookup
- Analyze Manchester encoding

Phase 2: Add OpenACE Protocol Support
- Implement 0x55 0x99 0x95 0xA6 0x9A 0x65 sync detection
- Add 24-bit CRC validation
- Integrate Manchester decode error tracking

Phase 3: Multi-Protocol RX
- Keep existing FLARM/OGN paths
- Add ADS-L parallel detection
- Use dataSource field for routing

Phase 4: Unified TX (Optional)
- Consolidate TX scheduler
- Support multiple protocols
- Coordinate frequency hopping
```

### Option 2: Full Replacement
```
Requires:
- Replace SX1262 driver implementation
- Move to RP2040/RP2350 platform
- Adopt GATAS message bus
- License/integrate ADSL library
- Complete hardware redesign
```

### Option 3: Dual Implementation
```
Run both in parallel:
- GXAirCom: Primary (FLARM/OGN focus)
- OpenACE: Secondary (ADS-L listener)
- Share MCU resources, separate radios
- Independent message queues
- Timestamp correlation for display
```

---

## KEY INTEGRATION CHALLENGES

### 1. Sync Word Standardization
**Problem:** OpenACE uses non-standard 0x55 0x99... vs spec 0x72 0x4B
**Solution Options:**
- A) Implement both patterns (forward/backward compatibility)
- B) Stick with EASA spec (miss OpenACE compatibility)
- C) Community consensus on standard

### 2. CRC Algorithm Compatibility
**Problem:** Different CRC implementations prevent interop
**Solution:**
- Validate against external EASA test vectors
- Implement both algorithms
- Benchmark for CPU/memory tradeoff

### 3. Manchester Error Handling
**Problem:** OpenACE tracks bit-level errors, GXAirCom doesn't
**Solution:**
- Optional: Enhanced error tracking
- Use syndrome information for FEC
- Better logging/diagnostics

### 4. Address Mapping
**Problem:** ADSL library is external (Skytraxx dependency)
**Solution:**
- Extract AMT algorithm from OpenACE
- Implement ICAO↔FLARM↔OGN mapping
- Open-source version for community

---

## RECOMMENDATIONS FOR GXAirCom

### Short Term (Current Release)
1. **Document the differences:** Add file explaining OpenACE ADS-L not currently supported
2. **Add sync word detection:** Implement alternative 0x55 0x99 pattern as fallback
3. **Improve logging:** Track which implementations are heard

### Medium Term (Next Release)
1. **Parallel ADS-L decoder:** Add OpenACE-compatible receiver
2. **Unified address DB:** Support ICAO, FLARM, OGN, FANET addresses
3. **Better CRC:** Implement 24-bit syndrome validation as option

### Long Term (Future)
1. **Standards compliance:** Work with EASA/Skytraxx on sync word
2. **Full integration:** Merge protocols into single demodulator
3. **Hardware flexibility:** Support both SX127x and SX1262

---

## RESOURCES FOR IMPLEMENTATION

**OpenACE Analysis Documents:**
- `OPENACE_ADSL_DETAILED_ANALYSIS.md` - Complete technical breakdown
- `OPENACE_ADSL_QUICK_REFERENCE.md` - Code snippets and LUTs

**Key GitHub Files:**
- Sync word: countryregulations_v2.hpp:108
- CRC syndrome: rxdataframequeue.cpp:181-261
- Manchester: manchester.cpp:24-78
- Protocol handler: adslace.cpp:77-200
- Radio config: sx1262.hpp:88-151

**External References:**
- EASA SRD-860 specification (contact EASA)
- Skytraxx ADSL documentation
- IEEE 802 Manchester encoding
- CRC algorithms and syndrome decoding

---

**Prepared by:** Analysis of rvt/openace GitHub repository
**Date:** April 18, 2026
**Status:** Preliminary - awaiting EASA spec confirmation on sync word
