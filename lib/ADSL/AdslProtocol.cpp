/*!
 * @file AdslProtocol.cpp
 * @brief ADS-L SRD860 M-Band protocol encoder / decoder for GXAirCom
 *
 * Reference: EASA ADS-L 4 SRD-860 Issue 1 (20 December 2022),
 * pjalocha/ogn-tracker ADS-L library (OGN source).
 */

#include "AdslProtocol.h"
#include <string.h>

uint32_t adsl_rx_count = 0;
uint32_t adsl_tx_count = 0;

// ────────────────────────────────────────────────────────────────────────────
// Internal bit-packing helpers
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief Write @p bits LSBs of @p value into @p buf starting at bit offset
 * @p bit_offset (MSB-first, i.e. bit 0 of offset is the MSB of buf[0]).
 */
static void pack_bits(uint8_t *buf, int bit_offset, int bits, uint32_t value) {
    for (int i = bits - 1; i >= 0; i--) {
        int global_bit = bit_offset + (bits - 1 - i);
        if (value & (1u << i)) {
            buf[global_bit >> 3] |= (uint8_t)(0x80u >> (global_bit & 7));
        }
    }
}

/**
 * @brief Read @p bits from @p buf starting at bit offset @p bit_offset.
 * Returns unsigned value (caller casts to signed if needed).
 */
static uint32_t unpack_bits(const uint8_t *buf, int bit_offset, int bits) {
    uint32_t value = 0;
    for (int i = bits - 1; i >= 0; i--) {
        int global_bit = bit_offset + (bits - 1 - i);
        if (buf[global_bit >> 3] & (uint8_t)(0x80u >> (global_bit & 7))) {
            value |= (1u << i);
        }
    }
    return value;
}

/**
 * @brief Sign-extend a 2's complement value of @p bits width to int32_t.
 */
static int32_t sign_extend(uint32_t value, int bits) {
    uint32_t sign_bit = 1u << (bits - 1);
    if (value & sign_bit) {
        return (int32_t)(value | (~((sign_bit << 1) - 1)));
    }
    return (int32_t)value;
}

// ────────────────────────────────────────────────────────────────────────────
// CRC-16/CCITT-FALSE  (poly=0x1021, init=0xFFFF, no bit-reversal)
// ────────────────────────────────────────────────────────────────────────────
uint16_t adsl_crc16(const uint8_t *data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ────────────────────────────────────────────────────────────────────────────
// Data scrambling — 24-bit Galois LFSR, polynomial x^24+x^23+x^22+x^17+1
// Seed = 24-bit source address.  Each clock advances the LFSR; each byte of
// the scramble region is XORed with the low byte of the current LFSR state.
// ────────────────────────────────────────────────────────────────────────────
void adsl_scramble(uint8_t *buf, int len, uint32_t address) {
    uint32_t lfsr = address & 0x00FFFFFFu;
    if (lfsr == 0) lfsr = 0x00AAAAAAu;   // fallback non-zero seed

    for (int i = 0; i < len; i++) {
        // Advance LFSR 8 times (one byte worth)
        for (int b = 0; b < 8; b++) {
            // Taps at bits 23, 22, 21, 16 (0-indexed from LSB)
            uint32_t feedback = ((lfsr >> 23) ^ (lfsr >> 22) ^
                                 (lfsr >> 21) ^ (lfsr >> 16)) & 1u;
            lfsr = ((lfsr << 1) | feedback) & 0x00FFFFFFu;
        }
        buf[i] ^= (uint8_t)(lfsr & 0xFFu);
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Aircraft type mapping: FANET → ADS-L category
// ────────────────────────────────────────────────────────────────────────────
adsl_aircraft_category_t adsl_category_from_fanet(uint8_t fanet_type) {
    // Strip "legacy" prefix (0x80 = FLARM aircraft type)
    uint8_t t = (fanet_type >= 0x80) ? (fanet_type - 0x80) : fanet_type;

    switch (t) {
        case 0:  return ADSL_CAT_UNKNOWN;       // otherAircraft
        case 1:  return ADSL_CAT_PARA_GLIDER;   // paraglider
        case 2:  return ADSL_CAT_HANG_GLIDER;   // hangglider
        case 3:  return ADSL_CAT_BALLOON;        // balloon
        case 4:  return ADSL_CAT_GLIDER;         // glider / motor-glider
        case 5:  return ADSL_CAT_RECIP_ENGINE;   // poweredAircraft
        case 6:  return ADSL_CAT_HELICOPTER;     // helicopter
        case 7:  return ADSL_CAT_UAV;            // uav
        // Legacy FLARM types (0x80+N, stripped above)
        case 8:  return ADSL_CAT_RECIP_ENGINE;   // aircraft reciprocating
        case 9:  return ADSL_CAT_JET_ENGINE;     // jet engine
        case 10: return ADSL_CAT_UFO;
        case 11: return ADSL_CAT_BALLOON;
        case 12: return ADSL_CAT_AIRSHIP;
        case 13: return ADSL_CAT_UAV;
        case 14: return ADSL_CAT_GROUND_SUPPORT;
        case 15: return ADSL_CAT_STATIC_OBJECT;
        default: return ADSL_CAT_UNKNOWN;
    }
}

// ────────────────────────────────────────────────────────────────────────────
// iConspicuity payload encoder
// Packs 117 bits into payload[15].  Output buffer must be zeroed first.
// ────────────────────────────────────────────────────────────────────────────
static void encode_iconspicuity(uint8_t *payload, const adsl_iconspicuity_t *d) {
    memset(payload, 0, ADSL_ICONSPICUITY_SIZE);

    // --- Timestamp (6 bit) ---
    pack_bits(payload, 0, 6, d->timestamp_s & 0x3Fu);

    // --- Flight State (2 bit) ---
    pack_bits(payload, 6, 2, (uint8_t)d->flight_state & 0x03u);

    // --- Aircraft Category (5 bit) ---
    pack_bits(payload, 8, 5, (uint8_t)d->aircraft_category & 0x1Fu);

    // --- Emergency Status (3 bit) ---
    pack_bits(payload, 13, 3, (uint8_t)d->emergency & 0x07u);

    // --- Latitude (22 bit signed) ---
    int32_t lat_i = (int32_t)roundf(d->latitude * (float)(1 << 21) / 180.0f);
    lat_i = (lat_i < -(1 << 21)) ? -(1 << 21) : lat_i;
    lat_i = (lat_i >  (1 << 21) - 1) ? (1 << 21) - 1 : lat_i;
    pack_bits(payload, 16, 22, (uint32_t)(lat_i & 0x3FFFFFu));

    // --- Longitude (23 bit signed) ---
    int32_t lon_i = (int32_t)roundf(d->longitude * (float)(1 << 22) / 360.0f);
    lon_i = (lon_i < -(1 << 22)) ? -(1 << 22) : lon_i;
    lon_i = (lon_i >  (1 << 22) - 1) ? (1 << 22) - 1 : lon_i;
    pack_bits(payload, 38, 23, (uint32_t)(lon_i & 0x7FFFFFu));

    // --- Altitude WGS-84 (14 bit unsigned, 2m steps, offset -512m) ---
    int32_t alt_raw = (int32_t)((d->altitude_wgs84_m + 512.0f) / 2.0f);
    if (alt_raw < 0)        alt_raw = 0;
    if (alt_raw > 0x3FFF)   alt_raw = 0x3FFF;
    pack_bits(payload, 61, 14, (uint32_t)alt_raw);

    // --- Ground Speed (8 bit unsigned, 0.5 m/s steps) ---
    uint32_t spd_raw = (uint32_t)(d->speed_ms / 0.5f + 0.5f);
    if (spd_raw > 0xFF) spd_raw = 0xFF;
    pack_bits(payload, 75, 8, spd_raw);

    // --- Vertical Rate (7 bit signed, 0.5 m/s steps) ---
    int32_t vr_raw = (int32_t)(d->vertical_rate_ms / 0.5f);
    if (vr_raw < -64) vr_raw = -64;
    if (vr_raw >  63) vr_raw =  63;
    pack_bits(payload, 83, 7, (uint32_t)(vr_raw & 0x7Fu));

    // --- Ground Track (9 bit unsigned, 360/512 deg/step) ---
    float trk = d->track_deg;
    while (trk <   0.0f) trk += 360.0f;
    while (trk >= 360.0f) trk -= 360.0f;
    uint32_t trk_raw = (uint32_t)(trk * 512.0f / 360.0f + 0.5f) & 0x1FFu;
    pack_bits(payload, 90, 9, trk_raw);

    pack_bits(payload, 99,  2, d->sil & 0x03u);
    pack_bits(payload, 101, 2, d->design_assurance & 0x03u);
    pack_bits(payload, 103, 4, d->nav_integrity & 0x0Fu);
    pack_bits(payload, 107, 4, d->h_accuracy & 0x0Fu);
    pack_bits(payload, 111, 3, d->v_accuracy & 0x07u);
    pack_bits(payload, 114, 3, d->vel_accuracy & 0x07u);
}

// ────────────────────────────────────────────────────────────────────────────
// iConspicuity payload decoder
// ────────────────────────────────────────────────────────────────────────────
static void decode_iconspicuity(const uint8_t *payload, adsl_iconspicuity_t *d) {
    d->timestamp_s        = (uint8_t)unpack_bits(payload, 0, 6);
    d->flight_state       = (adsl_flight_state_t)unpack_bits(payload, 6, 2);
    d->aircraft_category  = (adsl_aircraft_category_t)unpack_bits(payload, 8, 5);
    d->emergency          = (adsl_emergency_t)unpack_bits(payload, 13, 3);

    int32_t lat_i = sign_extend(unpack_bits(payload, 16, 22), 22);
    d->latitude = (float)lat_i * 180.0f / (float)(1 << 21);

    int32_t lon_i = sign_extend(unpack_bits(payload, 38, 23), 23);
    d->longitude = (float)lon_i * 360.0f / (float)(1 << 22);

    d->altitude_wgs84_m = (float)unpack_bits(payload, 61, 14) * 2.0f - 512.0f;
    d->speed_ms = (float)unpack_bits(payload, 75, 8) * 0.5f;

    int32_t vr_i = sign_extend(unpack_bits(payload, 83, 7), 7);
    d->vertical_rate_ms = (float)vr_i * 0.5f;

    d->track_deg = (float)unpack_bits(payload, 90, 9) * 360.0f / 512.0f;

    d->sil              = (uint8_t)unpack_bits(payload, 99,  2);
    d->design_assurance = (uint8_t)unpack_bits(payload, 101, 2);
    d->nav_integrity    = (uint8_t)unpack_bits(payload, 103, 4);
    d->h_accuracy       = (uint8_t)unpack_bits(payload, 107, 4);
    d->v_accuracy       = (uint8_t)unpack_bits(payload, 111, 3);
    d->vel_accuracy     = (uint8_t)unpack_bits(payload, 114, 3);
}

void manchester_encode(const uint8_t *in, uint8_t *out, int in_len) {
    for (int i = 0; i < in_len; i++) {
        uint8_t in_byte = in[i];
        uint16_t out_word = 0;
        
        // Encode each bit of the byte
        for (int b = 7; b >= 0; b--) {
            out_word <<= 2;
            if (in_byte & (1 << b)) {
                out_word |= 0b10; // '1' becomes '10'
            } else {
                out_word |= 0b01; // '0' becomes '01'
            }
        }
        
        // Store the 16-bit encoded word as two bytes
        out[i * 2] = (out_word >> 8) & 0xFF;
        out[i * 2 + 1] = out_word & 0xFF;
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Public: Full packet encoder
// ────────────────────────────────────────────────────────────────────────────
bool adsl_encode_packet(uint8_t *buf, const adsl_iconspicuity_t *data) {
    if (!buf || !data) return false;

    memset(buf, 0, ADSL_PACKET_SIZE);

    uint32_t addr = data->address & 0x00FFFFFFu;

    buf[0] = (uint8_t)(ADSL_NET_HDR_SIZE + ADSL_PRES_HDR_SIZE +
                       ADSL_ICONSPICUITY_SIZE);  // = 24

    buf[1] = (uint8_t)((ADSL_PROTOCOL_VERSION << 4) | 0x00u);
    buf[2] = (uint8_t)(addr & 0xFFu);
    buf[3] = (uint8_t)((addr >> 8)  & 0xFFu);
    buf[4] = (uint8_t)((addr >> 16) & 0xFFu);

    buf[5] = ADSL_PAYLOAD_TYPE_ICONSPICUITY;
    buf[6] = (uint8_t)(addr & 0xFFu);
    buf[7] = (uint8_t)((addr >> 8)  & 0xFFu);
    buf[8] = (uint8_t)((addr >> 16) & 0xFFu);
    buf[9] = data->privacy_mode ? 0x01u : 0x00u;

    encode_iconspicuity(&buf[10], data);

    adsl_scramble(&buf[ADSL_SCRAMBLE_OFFSET], ADSL_SCRAMBLE_SIZE, addr);

    uint16_t crc = adsl_crc16(buf, ADSL_CRC_COVER_SIZE);
    buf[ADSL_CRC_COVER_SIZE]     = (uint8_t)(crc >> 8);
    buf[ADSL_CRC_COVER_SIZE + 1] = (uint8_t)(crc & 0xFFu);
    adsl_tx_count++;
    return true;
}

// ────────────────────────────────────────────────────────────────────────────
// Public: Full packet decoder
// ────────────────────────────────────────────────────────────────────────────
bool adsl_decode_packet(const uint8_t *buf, int len, adsl_iconspicuity_t *data) {
    if (!buf || !data) return false;
    if (len < ADSL_PACKET_SIZE) return false;

    uint16_t crc_calc = adsl_crc16(buf, ADSL_CRC_COVER_SIZE);
    uint16_t crc_recv = ((uint16_t)buf[ADSL_CRC_COVER_SIZE] << 8) |
                                   buf[ADSL_CRC_COVER_SIZE + 1];
    if (crc_calc != crc_recv) return false;

    uint32_t addr = (uint32_t)buf[2] |
                    ((uint32_t)buf[3] << 8) |
                    ((uint32_t)buf[4] << 16);
    data->address = addr;

    uint8_t tmp[ADSL_SCRAMBLE_SIZE];
    memcpy(tmp, &buf[ADSL_SCRAMBLE_OFFSET], ADSL_SCRAMBLE_SIZE);
    adsl_scramble(tmp, ADSL_SCRAMBLE_SIZE, addr);

    if (tmp[0] != ADSL_PAYLOAD_TYPE_ICONSPICUITY) return false;

    data->privacy_mode = (tmp[4] & 0x01u) != 0;

    decode_iconspicuity(&tmp[5], data);
    adsl_rx_count++; // Increment on valid received packet
    return true;
}