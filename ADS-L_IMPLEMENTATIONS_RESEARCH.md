# ADS-L Protocol Open-Source Implementations Research Report

**Research Date:** April 18, 2026  
**Specification Focus:** EASA ADS-L 4 SRD-860 & Related Aviation Protocols  
**Scope:** Open-source implementations with actual protocol compliance claims

---

## Executive Summary

The search for open-source ADS-L implementations claiming explicit EASA SRD-860 compliance revealed:

- **Limited pure ADS-L implementations** exist as standalone projects on GitHub
- **Most aviation conspicuity systems** support multiple protocols (FLARM, OGN, FANET, ADS-L) rather than ADS-L exclusively
- **GATAS (openace)** is the **primary production implementation** with explicit ADS-L Issue 2 support and ground station capabilities
- **SoftRF** has broad protocol support but treats ADS-L as a supported protocol rather than primary focus
- **OGN-based projects** implement protocol reception but focus on APRS forwarding rather than direct ADS-L specification compliance
- **No official EASA/FLARM/Skytraxx reference implementations** found as open-source repositories

---

## Primary Production Implementations

### 1. **GATAS Conspicuity Device (openace)**
- **Repository:** [rvt/openace](https://github.com/rvt/openace)
- **Language:** C++/C (80.9% C++, 13.2% C)
- **Last Update:** 14 hours ago (April 2026)
- **Status:** Active development, v2.0.0 released
- **License:** GPL-3.0
- **Stars:** 17

#### ADS-L Compliance:
- **ADS-L Issue 2 Implementation** ✓ (Ground Station Mode & Traffic Uplink - NEW in v2.0.0)
- **TX/RX:** Both transmit and receive
- **Spec Compliance:** Implements Issue 2 (latest variant)
- **Key Features:**
  - Ground station mode with traffic uplink/relay (max 10 aircraft)
  - Multi-protocol support: ADS-L, OGN, FLARM (2024), FANET, PAW (2026 ADS-L version)
  - Time-sharing radio implementation (multiple protocols on single transceiver)
  - Adaptive Protocol Prioritisation (intelligent listening allocation)
  - GDL90 communication protocol over WiFi/Bluetooth
  - Tested with SkyDemon EFB

#### Hardware Support:
- **Primary:** Raspberry Pi RP2040/RP2350
- **Transceivers:** Dual or single configuration
- **Plugins:** Waveshare modules for plug-and-play setup
- **Battery:** Li-Ion with USB-C charger, 6-10 hour runtime

#### Code Quality Indicators:
- GitHub Actions CI/CD pipeline
- Active issue tracking and feature development
- Well-documented via wiki: [rvt.github.io/gatas-doc](https://rvt.github.io/gatas-doc)
- Sponsored project with community support
- External library integration (FreeRTOS, LWiP, ArduinoJSON, libcrc, minmea, Catch2)

**Assessment:** **HIGHEST CONFIDENCE** - This is the most actively maintained, production-ready ADS-L implementation found.

---

### 2. **SoftRF - Multi-Protocol Aviation Proximity System**
- **Repository:** [lyusupov/SoftRF](https://github.com/lyusupov/SoftRF)
- **Language:** C (84.7%), C++ (14.2%)
- **Last Update:** 2 days ago (April 2026)
- **Status:** Highly active development, Release 1.8 (Dec 2025)
- **License:** GPL-3.0
- **Stars:** 969

#### ADS-L Compliance:
- **ADS-L Support:** Listed in compatibility table as "SRD 860 ADS-L"
- **TX/RX:** Limited (appears to be RX-focused, TX capability unclear from documentation)
- **Spec Compliance:** Minimal/subset implementation (per project philosophy: "implements only a reasonable minimum of the protocols specs")
- **Note:** ADS-L row in compatibility table shows mostly empty cells

#### Supported Protocols (Complete):
- FLARM AIR V7
- OGNTP (OGN Tracking Protocol)
- P3I (PilotAware)
- FANET+
- 978 UAT ADS-B
- 1090 ES ADS-B
- **SRD 860 ADS-L** (listed but incomplete)
- APRS
- Remote ID
- GDL90 (Garmin datalink)

#### Hardware Support:
**Extensive:** 50+ hardware variations including:
- **MCUs:** ESP8266, ESP32, nRF52840, STM32, CC1310, Raspberry Pi, Nordic nRF54L15, Rockchip RK3506
- **Radio Chips:** nRF905, SX1276, SX1262, CC1310, LR1110, LR1121, SA868, and more
- **GNSS Chips:** u-blox 6/7/8/9/10, AT6558, CXD5603, MT3339, UC6580, AG3335
- **Edition Variants:** 40+ different physical form factors (Prime, Badge, Card, Handheld, Pocket, Solaris, etc.)

#### Code Quality Indicators:
- **Contributors:** 4 main contributors
- **Forks:** 252
- **Very active release history** (20 releases shown)
- **Documentation:** Extensive wiki and multiple PDFs
- **CI/CD:** GitHub Actions workflows
- **Language Mix:** C/C++/Python/Shell/Makefile

#### Architecture Notes:
- Supports simultaneous operation of multiple protocols (Octave Concept)
- Includes flight recorder functionality
- Web-based configuration UI
- NMEA, GDL90, Dump1090 data outputs

**Assessment:** **MEDIUM-HIGH CONFIDENCE** - Extensive platform but ADS-L appears to be a secondary protocol with incomplete implementation. Focus is on breadth rather than ADS-L specification depth.

---

### 3. **GXAirCom - LoRa-Based Aviation Communication**
- **Repository:** [gereic/GXAirCom](https://github.com/gereic/GXAirCom)
- **Language:** C (66.5%), C++ (22.3%), Python (7.9%)
- **Last Update:** 1 month ago (March 2026)
- **Status:** Active development
- **License:** Not specified in repo
- **Stars:** 156

#### Protocol Support:
- **Primary:** FANET+ / FLARM protocols
- **Secondary:** OGN support (broadcasts received FANET to OGN network)
- **ADS-L:** No explicit mention in descriptions
- **Type:** LoRa-based communication device for free-flying sports

#### Features:
- Complete open-source FANET+ (Fanet + Flarm) protocol implementation
- Acts as FANET ground station
- Bluetooth interface to mobile phones
- OGN gateway capability
- Variometer integration
- Messaging functionality

#### Hardware:
- Multiple LoRa module support
- Paragliding/UAV focus
- T-Beam, Heltec variants supported

#### Code Quality:
- 15 contributors
- Extensive wiki documentation
- PDF guides and technical documentation
- Video tutorials available
- Related to Skytraxx FANET reference implementation

**Assessment:** **LOW-MEDIUM RELEVANCE** - Focused on FANET/FLARM, not specifically ADS-L compliant. Related to sports aviation rather than general aviation conspicuity.

---

## Secondary/Specialized Implementations

### 4. **FLARE - Demodulator/Decoder Library**
- **Repository:** [lyusupov/flare](https://github.com/lyusupov/flare)
- **Language:** C/C++ (63.5% C++, 36% C)
- **Last Update:** 6 years ago (2019)
- **Status:** Archived/Unmaintained
- **License:** GPL-3.0
- **Focus:** Signal demodulation and decoding

#### Implements:
- nRF905 demodulator
- FLARM decoder
- OGNTP demodulator/decoder
- PilotAware (P3I) demodulator/decoder

**Assessment:** **LOW RELEVANCE FOR ADS-L** - Historical reference implementation, outdated. Focuses on signal layer rather than protocol compliance.

---

### 5. **OGN Project (Multiple Repositories)**
- **Organization:** [glidernet](https://github.com/glidernet)
- **Repository Count:** 29+ repositories tagged with OGN
- **Primary Focus:** Open Glider Network (APRS-based tracking)
- **Related to ADS-L:** Indirect (protocols receive ADS-L and forward via APRS)

#### Notable OGN Repositories:
- **[glidernet/ogn-aprs-protocol](https://github.com/glidernet/ogn-aprs-protocol)** - Protocol documentation
- **[snip/OGN-receiver-RPI-image](https://github.com/snip/OGN-receiver-RPI-image)** - Raspberry Pi receiver image
- Multiple receiver implementations, web gateways, data parsers

**Assessment:** **INDIRECT RELEVANCE** - OGN receives and processes multiple aviation protocols including FLARM/OGN signals. Not a direct ADS-L specification implementation but part of the ecosystem.

---

### 6. **Aviation Protocol Support Libraries**

#### FANET Library (rvt/FANET)
- **Repository:** [rvt/FANET](https://github.com/rvt/FANET)
- **Language:** C++ (90.6%)
- **Status:** Active, updated Oct 2025
- **Type:** Reusable C++ header-only library
- **Scope:** FANET protocol implementation for embedded systems

#### go-fanet
- **Repository:** [twpayne/go-fanet](https://github.com/twpayne/go-fanet)
- **Language:** Go
- **Function:** FANET sentence generation and parsing

**Assessment:** **NOT ADS-L SPECIFIC** - These are FANET implementations. FANET is a different protocol for paragliding/UAV coordination.

---

## Search Results Summary

### Direct ADS-L Searches (GitHub):
- **"ADS-L protocol"** - 1 result (adsb.lol-mcp, ADS-B focused, not ADS-L)
- **"EASA ADS-L"** - 0 repository results (8 issues, 1 discussion, 6 wikis)
- **"SRD-860 specification"** - 0 results
- **"FLARM protocol implementation"** - 0 standalone results (scattered in issues)
- **"LMIC ADS-L"** - 0 results
- **"SkyTraxx ADS-L"** - 0 results

### Related Protocol Searches:
- **BASICMAC (LoRaWAN)** - 119 results, but oriented toward LoRaWAN, not aviation-specific
- **FANET** - 11 repositories, mostly simulation/research or paragliding-focused
- **OGN** - 29 repositories with mixed protocol support
- **FLARM** - Scattered references across multiple repositories

### Ecosystem Observation:
The absence of direct repository matches for "ADS-L" or "SRD-860" suggests:
1. ADS-L implementations may be proprietary (FLARM, Skytraxx)
2. Open-source implementations exist but are labeled under parent projects (SoftRF, openace)
3. ADS-L development may be internal to commercial entities
4. EASA specification documentation may not be publicly available on GitHub

---

## Key Findings

### 1. **Limited True ADS-L Implementations**
- Only **GATAS (openace)** explicitly documents ADS-L Issue 2 support as a primary feature
- **SoftRF** lists ADS-L in compatibility but with minimal feature completeness
- Most other projects focus on related protocols (FLARM, OGN, FANET, ADS-B)

### 2. **ADS-L Integration Pattern**
The dominant pattern is **multi-protocol conspicuity systems** that support:
- 868 MHz aviation band (FLARM, OGN, ADS-L, FANET)
- Simultaneous protocol reception
- Protocol forwarding to backend services (OGN, APRS)

### 3. **Specification Documentation**
- **Official EASA ADS-L SRD-860 spec:** Not found as open-source
- **Reference implementations:** Proprietary to FLARM/Skytraxx
- **Implementation guidance:** Scattered in GitHub issues, project wikis
- **Protocol.txt files:** Found in FANET implementations but not standalone ADS-L

### 4. **Development Ecosystem**
- **Skytraxx FANET** is the reference for paragliding/UAV protocols
- **GXAirCom** derives from Skytraxx's FANET reference
- **GATAS** implements broader aviation conspicuity including ADS-L
- **SoftRF** provides maximum hardware breadth with protocol subset implementations

### 5. **Hardware Focus Areas**
- **General Aviation (GA):** GATAS (RP2040/RP2350), SoftRF (ESP32, nRF52840)
- **Paragliding/Sports:** GXAirCom (LoRa modules), Skytraxx (proprietary)
- **Ground Stations:** Multiple OGN receiver implementations
- **Universal:** SoftRF with 50+ hardware variants

---

## Specification Compliance Assessment

### GATAS (openace) - ADS-L Issue 2
**Claimed Compliance:** ✓ **YES - Explicit**
- Implements Issue 2 features (ground station mode, traffic uplink)
- Active development with recent updates
- Documented features align with ADS-L specification
- **Confidence Level:** HIGH (v2.0.0 release notes confirm)

### SoftRF - ADS-L/SRD-860
**Claimed Compliance:** ✓ **PARTIAL**
- Lists ADS-L in compatibility matrix
- Acknowledged philosophy: "implements only a reasonable minimum"
- ADS-L column shows mostly empty cells
- **Confidence Level:** MEDIUM (acknowledged as subset implementation)

### OGN Projects - ADS-L
**Claimed Compliance:** ~ **INDIRECT**
- Receives and processes ADS-L signals
- Forwards to APRS network
- Not a spec-conformant implementation, but protocol-aware
- **Confidence Level:** LOW (protocol agnostic reception)

### Others (GXAirCom, FANET, etc.)
**Claimed Compliance:** ✗ **NO**
- Focus on FANET, FLARM, or OGN
- Not positioned as ADS-L implementations
- May include ADS-L reception passively

---

## Recommended Resources for ADS-L Development

### Official Sources
1. **EASA Document:** SRD-860 specification (access required - likely restricted)
2. **FLARM:** Proprietary documentation (not open-source)
3. **Skytraxx:** FANET reference but not ADS-L specific

### Open-Source Reference Points
1. **GATAS (openace):** Best production reference for ADS-L Issue 2
   - GitHub: https://github.com/rvt/openace
   - Documentation: https://rvt.github.io/gatas-doc

2. **SoftRF:** Multi-protocol approach for breadth
   - GitHub: https://github.com/lyusupov/SoftRF
   - Wiki: Extensive documentation

3. **OGN Project:** Protocol reception architecture
   - GitHub: https://github.com/glidernet
   - Forum: https://groups.google.com/forum/#!forum/openglidernetwork

### Protocol Documentation
1. **FANET Protocol.txt:** Reference for 868 MHz band protocols
   - Source: https://github.com/3s1d/fanet-stm32
   - Demonstrates protocol structure patterns

2. **GXAirCom Documentation:** FANET+ specification interpretation
   - Wiki: https://github.com/gereic/GXAirCom/wiki
   - PDFs: Technical documentation included

### Development Communities
1. **OGN Discussion Group:** https://groups.google.com/forum/#!forum/openglidernetwork
2. **GitHub Issues:** GATAS, SoftRF, GXAirCom projects have active issue discussions
3. **Aviation Forums:** XCSoar project, SkyDemon community forums

---

## Implementation Features Comparison

| Feature | GATAS | SoftRF | GXAirCom | OGN |
|---------|-------|--------|----------|-----|
| **ADS-L TX** | ✓ (Issue 2) | ? | ✗ | ✗ |
| **ADS-L RX** | ✓ (Issue 2) | ✓ | ✗ | ✓ |
| **FLARM Support** | ✓ | ✓ | ✓ | ✓ |
| **OGN Support** | ✓ | ✓ | ✓ (gateway) | ✓ |
| **FANET Support** | ✓ | ✓ | ✓ | ✗ |
| **Multi-Protocol** | ✓ | ✓ | ✓ | ✓ |
| **Ground Station** | ✓ | ✗ | ✓ | ✓ |
| **Traffic Uplink** | ✓ | ✗ | ✗ | ✗ |
| **EFB Integration** | ✓ (GDL90) | ✓ (GDL90) | ✓ (BLE) | ✗ |
| **Last Update** | Recent | 2 days ago | 1 month | Various |
| **Active Development** | ✓ | ✓ | ✓ | ✓ |

---

## Conclusions

### Primary Recommendation
**For EASA ADS-L 4 SRD-860 compliant implementation:**
→ **GATAS (rvt/openace)** is the only production-ready open-source implementation with explicit Issue 2 compliance and active development.

### Alternative for Breadth
**For multi-protocol aviation conspicuity:**
→ **SoftRF** provides the widest hardware/protocol support but treats ADS-L as secondary.

### Learning Resource
**For understanding the aviation protocol ecosystem:**
→ **GXAirCom** + **OGN projects** provide good examples of protocol integration patterns and community standards.

### Important Note
**No pure ADS-L specification implementation** exists in readily available open-source form. The EASA SRD-860 specification itself appears to be restricted or not publicly documented on GitHub. All implementations are either:
1. Subset/partial implementations (SoftRF)
2. Multi-protocol systems with ADS-L as one component (GATAS, OGN)
3. Proprietary (FLARM, Skytraxx)

---

## Research Limitations

1. **GitHub-only search:** Specification documents may exist on other platforms (GitLab, academic sites, aviation forums)
2. **Commercial confidentiality:** FLARM and Skytraxx specifications likely proprietary
3. **Specification access:** EASA SRD-860 may require official purchase/membership
4. **Indirect implementations:** Some ADS-L support may be embedded without explicit labeling
5. **Documentation currency:** Some project documentation may be outdated relative to code

---

## Appendix: Repository URLs

### ADS-L Relevant
- GATAS/OpenACE: https://github.com/rvt/openace
- SoftRF: https://github.com/lyusupov/SoftRF
- GXAirCom: https://github.com/gereic/GXAirCom

### Protocol Support Libraries
- FANET Library: https://github.com/rvt/FANET
- FLARE (Decoders): https://github.com/lyusupov/flare
- OGN Projects: https://github.com/glidernet

### Related Projects
- OGN Official: https://www.glidernet.org
- FLARE RTL-SDR: https://github.com/creaktive/flare
- XCSoar: https://xcsoar.org
- Skytraxx FANET: https://github.com/3s1d/fanet-stm32

---

**Report Generated:** April 18, 2026  
**Research Scope:** Comprehensive GitHub and open-source aviation project search  
**Confidence Levels:** Based on explicit documentation and active development status
