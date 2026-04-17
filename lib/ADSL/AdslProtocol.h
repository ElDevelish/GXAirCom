/*!
 * @file AdslProtocol.h
 * @brief ADS-L SRD860 M-Band protocol encoder / decoder for GXAirCom
 *
 * Implements EASA ADS-L 4 SRD-860 Issue 1/2 specification
 * M-Band: 868.2 / 868.4 MHz, 2-GFSK, 50 kbps net (100 kbps raw w/ Manchester)
 *
 * Packet structure (all bytes after Manchester decode):
 * [Length 1B][Net Header 4B][Pres Header 5B][iConspicuity 15B][CRC16 2B]
 * = 27 bytes total on-air (excl. preamble + syncword)
 *
 * Scrambling: XOR with 24-bit LFSR seeded from source address,
 * covers Presentation Header + Application Payload (20 bytes).
 * CRC-16:     Poly 0x1021, Init 0xFFFF (CRC-16/CCITT-FALSE),
 * over [Length + Net Header + Pres Header + Payload].
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// ────────────────────────────────────────────────────────────────────────────
// Physical layer constants
// ────────────────────────────────────────────────────────────────────────────
#define ADSL_FREQ_MBAND_1       868200000UL   ///< 868.2 MHz (M-Band chan 1)
#define ADSL_FREQ_MBAND_2       868400000UL   ///< 868.4 MHz (M-Band chan 2)

// SX1276 FSK bitrate register value for 100 kbps raw (50 kbps + Manchester)
// FXOSC / BitRate = 32e6 / 100000 = 320
#define ADSL_BITRATE_RAW        100000UL      ///< raw on-air bitrate [bps]

// ────────────────────────────────────────────────────────────────────────────
// Packet size constants
// ────────────────────────────────────────────────────────────────────────────
#define ADSL_LEN_FIELD_SIZE     1             ///< Length byte
#define ADSL_NET_HDR_SIZE       4             ///< Protocol byte + 3-byte address
#define ADSL_PRES_HDR_SIZE      5             ///< Type(1) + Address(3) + Privacy(1)
#define ADSL_ICONSPICUITY_SIZE  15            ///< 117 payload bits → 15 bytes
#define ADSL_CRC_SIZE           2             ///< CRC-16


/// Total bytes covered by CRC (excludes CRC itself but includes length byte)
#define ADSL_CRC_COVER_SIZE     (ADSL_LEN_FIELD_SIZE + ADSL_NET_HDR_SIZE + \
                                 ADSL_PRES_HDR_SIZE + ADSL_ICONSPICUITY_SIZE)

/// Total packet size written to radio TX buffer
#define ADSL_PACKET_SIZE        (ADSL_CRC_COVER_SIZE + ADSL_CRC_SIZE)

/// Number of bytes that are scrambled (Pres Header + Payload)
#define ADSL_SCRAMBLE_SIZE      (ADSL_PRES_HDR_SIZE + ADSL_ICONSPICUITY_SIZE)

/// Offset of first scrambled byte inside packet buffer (0-indexed)
#define ADSL_SCRAMBLE_OFFSET    (ADSL_LEN_FIELD_SIZE + ADSL_NET_HDR_SIZE)

// ────────────────────────────────────────────────────────────────────────────
// Protocol constants
// ────────────────────────────────────────────────────────────────────────────
#define ADSL_PROTOCOL_VERSION           1     ///< ADS-L protocol version
#define ADSL_PAYLOAD_TYPE_ICONSPICUITY  0x00  ///< Presentation layer type ID

/// Transmission interval [ms] — spec mandates alternation each TX
#define ADSL_TX_INTERVAL_MS     4000

extern uint32_t adsl_rx_count;
extern uint32_t adsl_tx_count;

// ────────────────────────────────────────────────────────────────────────────
// ADS-L Aircraft Category  (spec section G.1.3, 5-bit field)
// ────────────────────────────────────────────────────────────────────────────
typedef enum adsl_aircraft_category {
    ADSL_CAT_UNKNOWN        = 0,
    ADSL_CAT_GLIDER         = 1,  ///< Glider / motor-glider
    ADSL_CAT_TOW_PLANE      = 2,  ///< Tow plane
    ADSL_CAT_HELICOPTER     = 3,
    ADSL_CAT_SKYDIVER       = 4,
    ADSL_CAT_DROP_PLANE     = 5,
    ADSL_CAT_HANG_GLIDER    = 6,
    ADSL_CAT_PARA_GLIDER    = 7,
    ADSL_CAT_RECIP_ENGINE   = 8,  ///< Reciprocating engine aircraft
    ADSL_CAT_JET_ENGINE     = 9,
    ADSL_CAT_UFO            = 10,
    ADSL_CAT_BALLOON        = 11,
    ADSL_CAT_AIRSHIP        = 12,
    ADSL_CAT_UAV            = 13,
    ADSL_CAT_GROUND_SUPPORT = 14,
    ADSL_CAT_STATIC_OBJECT  = 15,
} adsl_aircraft_category_t;

// ────────────────────────────────────────────────────────────────────────────
// ADS-L Flight State  (spec section G.1.2, 2-bit field)
// ────────────────────────────────────────────────────────────────────────────
typedef enum adsl_flight_state {
    ADSL_STATE_UNKNOWN  = 0,
    ADSL_STATE_GROUND   = 1,
    ADSL_STATE_AIRBORNE = 2,
    ADSL_STATE_RESERVED = 3,
} adsl_flight_state_t;

// ────────────────────────────────────────────────────────────────────────────
// ADS-L Emergency Status  (spec section G.1.4, 3-bit field)
// ────────────────────────────────────────────────────────────────────────────
typedef enum adsl_emergency {
    ADSL_EMERG_NONE         = 0,
    ADSL_EMERG_GENERAL      = 1,
    ADSL_EMERG_FUEL         = 2,
    ADSL_EMERG_COMMS_FAIL   = 3,
    ADSL_EMERG_MIN_FUEL     = 4,
    ADSL_EMERG_HI_TEMP      = 5,
    ADSL_EMERG_OWNSHIP      = 6,
    ADSL_EMERG_RESERVED     = 7,
} adsl_emergency_t;

// ────────────────────────────────────────────────────────────────────────────
// iConspicuity data structure
// ────────────────────────────────────────────────────────────────────────────
typedef struct adsl_iconspicuity {
    // Identity
    uint32_t                address;          ///< 24-bit device address (LSB-first)

    // Position (WGS-84)
    float                   latitude;         ///< decimal degrees [-90 .. +90]
    float                   longitude;        ///< decimal degrees [-180 .. +180]
    float                   altitude_wgs84_m; ///< WGS-84 ellipsoid height [m]

    // Kinematics
    float                   speed_ms;         ///< ground speed [m/s]
    float                   vertical_rate_ms; ///< climb rate [m/s], + = up
    float                   track_deg;        ///< ground track [deg true, 0=N, 90=E]

    // Status
    uint8_t                 timestamp_s;      ///< seconds within 64-sec cycle (6 bit)
    adsl_flight_state_t     flight_state;
    adsl_aircraft_category_t aircraft_category;
    adsl_emergency_t        emergency;

    // Quality / accuracy indicators (set conservatively for DIY)
    uint8_t                 sil;              ///< Source Integrity Level [0..3]
    uint8_t                 design_assurance; ///< Design Assurance Level [0..3]
    uint8_t                 nav_integrity;    ///< Navigation Integrity [0..15]
    uint8_t                 h_accuracy;       ///< Horiz. Position Accuracy [0..15]
    uint8_t                 v_accuracy;       ///< Vert. Position Accuracy [0..7]
    uint8_t                 vel_accuracy;     ///< Velocity Accuracy [0..7]

    // Privacy
    bool                    privacy_mode;     ///< true → receiver shall not store pos
} adsl_iconspicuity_t;

// ────────────────────────────────────────────────────────────────────────────
// Function declarations
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief Encode a complete ADS-L M-Band packet into buf[ADSL_PACKET_SIZE].
 *
 * Assembles network header, presentation header, iConspicuity payload,
 * applies data scrambling and appends CRC-16.  The caller passes the result
 * directly to the radio TX function.
 *
 * @param buf       Output buffer, must be at least ADSL_PACKET_SIZE bytes.
 * @param data      Filled adsl_iconspicuity_t describing own aircraft.
 * @return true on success, false if data pointer is NULL.
 */
bool adsl_encode_packet(uint8_t *buf, const adsl_iconspicuity_t *data);

/**
 * @brief Decode an ADS-L M-Band packet (after CRC check + descramble).
 *
 * @param buf       Raw packet bytes as received from radio (ADSL_PACKET_SIZE).
 * @param len       Number of bytes in buf (must equal ADSL_PACKET_SIZE).
 * @param data      Output structure filled on success.
 * @return true if CRC is valid and decoding succeeded.
 */
bool adsl_decode_packet(const uint8_t *buf, int len, adsl_iconspicuity_t *data);

/**
 * @brief CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection.
 */
uint16_t adsl_crc16(const uint8_t *data, int len);

/**
 * @brief XOR data with 24-bit LFSR pseudo-random stream derived from address.
 * Applied in-place.  Covers Presentation Header + Application Payload.
 *
 * @param buf       Pointer to first scrambled byte (index ADSL_SCRAMBLE_OFFSET).
 * @param len       Number of bytes to scramble (ADSL_SCRAMBLE_SIZE).
 * @param address   24-bit source address used as LFSR seed.
 */
void adsl_scramble(uint8_t *buf, int len, uint32_t address);

// Software Manchester Encoder
void manchester_encode(const uint8_t *in, uint8_t *out, int in_len);

/**
 * @brief Map a FANET aircraft_t to the nearest ADS-L aircraft category.
 *
 * @param fanet_type  Value from FanetLora::aircraft_t (or legacy 0x80+N).
 * @return adsl_aircraft_category_t
 */
adsl_aircraft_category_t adsl_category_from_fanet(uint8_t fanet_type);