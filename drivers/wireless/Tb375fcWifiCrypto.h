/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/*
 * Tb375fcWifiCrypto.h — Clean-Room Self-Contained WPA2-PSK (CCMP-128) Crypto
 *
 * Implements:
 *   - SHA-1, HMAC-SHA1, PBKDF2-HMAC-SHA1 (IEEE 802.11i PMK derivation)
 *   - IEEE 802.11i PRF-384 (PTK = KCK[16] || KEK[16] || TK[16])
 *   - SHA-256, HMAC-SHA256, IEEE 802.11 KDF-SHA256-384 (AKM 00-0F-AC:6)
 *   - AES-128 block encrypt/decrypt, AES-128-CMAC (RFC 4493),
 *     AES-128 Key Unwrap (RFC 3394, for EAPOL M3 / G1 GTK decryption)
 *
 * Zero external dependencies; usable in both XNU kernel and host unit tests.
 */
#ifndef TB375FC_WIFI_CRYPTO_H
#define TB375FC_WIFI_CRYPTO_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace tb375fc_wcrypto {

static inline uint32_t rotl32(uint32_t x, unsigned n) {
    return (x << n) | (x >> (32U - n));
}

static inline uint32_t rotr32(uint32_t x, unsigned n) {
    return (x >> n) | (x << (32U - n));
}

static inline uint32_t load_be32(const uint8_t *p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
           (uint32_t(p[2]) << 8)  | uint32_t(p[3]);
}

static inline void store_be32(uint8_t *p, uint32_t v) {
    p[0] = uint8_t(v >> 24);
    p[1] = uint8_t(v >> 16);
    p[2] = uint8_t(v >> 8);
    p[3] = uint8_t(v);
}

static inline void store_be64(uint8_t *p, uint64_t v) {
    store_be32(p, uint32_t(v >> 32));
    store_be32(p + 4, uint32_t(v));
}

/* =========================================================================
 * SHA-1 (FIPS 180-4) & HMAC-SHA1 (RFC 2104)
 * ========================================================================= */

struct Sha1Ctx {
    uint32_t h[5];
    uint64_t total_bits;
    uint8_t  buf[64];
    size_t   buf_len;
};

static inline void sha1_init(Sha1Ctx *c) {
    c->h[0] = 0x67452301U;
    c->h[1] = 0xEFCDAB89U;
    c->h[2] = 0x98BADCFEU;
    c->h[3] = 0x10325476U;
    c->h[4] = 0xC3D2E1F0U;
    c->total_bits = 0;
    c->buf_len = 0;
}

static void sha1_compress(Sha1Ctx *c, const uint8_t blk[64]) {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) w[i] = load_be32(blk + i * 4);
    for (int i = 16; i < 80; ++i) w[i] = rotl32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

    uint32_t a = c->h[0], b = c->h[1], cc = c->h[2], d = c->h[3], e = c->h[4];
    for (int i = 0; i < 80; ++i) {
        uint32_t f, k;
        if (i < 20)      { f = (b & cc) | ((~b) & d);           k = 0x5A827999U; }
        else if (i < 40) { f = b ^ cc ^ d;                      k = 0x6ED9EBA1U; }
        else if (i < 60) { f = (b & cc) | (b & d) | (cc & d);   k = 0x8F1BBCDCU; }
        else             { f = b ^ cc ^ d;                      k = 0xCA62C1D6U; }
        uint32_t tmp = rotl32(a, 5) + f + e + k + w[i];
        e = d; d = cc; cc = rotl32(b, 30); b = a; a = tmp;
    }
    c->h[0] += a; c->h[1] += b; c->h[2] += cc; c->h[3] += d; c->h[4] += e;
}

static void sha1_update(Sha1Ctx *c, const uint8_t *data, size_t len) {
    c->total_bits += uint64_t(len) * 8ULL;
    while (len > 0) {
        if (c->buf_len == 0 && len >= 64) {
            sha1_compress(c, data);
            data += 64;
            len -= 64;
        } else {
            size_t take = 64 - c->buf_len;
            if (take > len) take = len;
            memcpy(c->buf + c->buf_len, data, take);
            c->buf_len += take;
            data += take;
            len -= take;
            if (c->buf_len == 64) {
                sha1_compress(c, c->buf);
                c->buf_len = 0;
            }
        }
    }
}

static void sha1_final(Sha1Ctx *c, uint8_t out[20]) {
    uint64_t bits = c->total_bits;
    uint8_t pad = 0x80;
    sha1_update(c, &pad, 1);
    uint8_t z = 0;
    while (c->buf_len != 56) sha1_update(c, &z, 1);
    uint8_t len_be[8];
    store_be64(len_be, bits);
    sha1_update(c, len_be, 8);
    for (int i = 0; i < 5; ++i) store_be32(out + i * 4, c->h[i]);
}

static void hmac_sha1(const uint8_t *key, size_t key_len,
                      const uint8_t *msg, size_t msg_len,
                      uint8_t out[20]) {
    uint8_t kblk[64];
    memset(kblk, 0, sizeof(kblk));
    if (key_len > 64) {
        Sha1Ctx kc; sha1_init(&kc); sha1_update(&kc, key, key_len); sha1_final(&kc, kblk);
    } else {
        memcpy(kblk, key, key_len);
    }
    uint8_t ipad[64], opad[64];
    for (int i = 0; i < 64; ++i) { ipad[i] = kblk[i] ^ 0x36; opad[i] = kblk[i] ^ 0x5C; }
    uint8_t inner[20];
    Sha1Ctx ci; sha1_init(&ci); sha1_update(&ci, ipad, 64); sha1_update(&ci, msg, msg_len); sha1_final(&ci, inner);
    Sha1Ctx co; sha1_init(&co); sha1_update(&co, opad, 64); sha1_update(&co, inner, 20); sha1_final(&co, out);
}

static void pbkdf2_sha1(const uint8_t *pass, size_t pass_len,
                        const uint8_t *salt, size_t salt_len,
                        uint32_t iters, uint8_t *out, size_t out_len) {
    uint8_t kblk[64];
    memset(kblk, 0, sizeof(kblk));
    if (pass_len > 64) {
        Sha1Ctx kc; sha1_init(&kc); sha1_update(&kc, pass, pass_len); sha1_final(&kc, kblk);
    } else {
        memcpy(kblk, pass, pass_len);
    }
    uint8_t ipad[64], opad[64];
    for (int i = 0; i < 64; ++i) { ipad[i] = kblk[i] ^ 0x36; opad[i] = kblk[i] ^ 0x5C; }
    Sha1Ctx base_i, base_o;
    sha1_init(&base_i); sha1_update(&base_i, ipad, 64);
    sha1_init(&base_o); sha1_update(&base_o, opad, 64);

    uint32_t blk_idx = 1;
    size_t pos = 0;
    while (pos < out_len) {
        uint8_t u[20], t[20], be_idx[4];
        store_be32(be_idx, blk_idx++);
        Sha1Ctx ci = base_i;
        sha1_update(&ci, salt, salt_len);
        sha1_update(&ci, be_idx, 4);
        sha1_final(&ci, u);
        Sha1Ctx co = base_o;
        sha1_update(&co, u, 20);
        sha1_final(&co, u);
        memcpy(t, u, 20);

        for (uint32_t it = 1; it < iters; ++it) {
            ci = base_i; sha1_update(&ci, u, 20); sha1_final(&ci, u);
            co = base_o; sha1_update(&co, u, 20); sha1_final(&co, u);
            for (int j = 0; j < 20; ++j) t[j] ^= u[j];
        }
        size_t take = out_len - pos;
        if (take > 20) take = 20;
        memcpy(out + pos, t, take);
        pos += take;
    }
}

/* IEEE 802.11i PRF-384 (produces 48 bytes: KCK[16] | KEK[16] | TK[16]) */
static void prf_sha1_384(const uint8_t pmk[32], const uint8_t data[76], uint8_t out[48]) {
    static const char label[] = "Pairwise key expansion";
    uint8_t buf[23 + 76 + 1];
    memcpy(buf, label, 23); /* includes trailing NUL */
    memcpy(buf + 23, data, 76);
    uint8_t block[60];
    for (uint8_t ctr = 0; ctr < 3; ++ctr) {
        buf[23 + 76] = ctr;
        hmac_sha1(pmk, 32, buf, sizeof(buf), block + ctr * 20);
    }
    memcpy(out, block, 48);
}

/* =========================================================================
 * SHA-256 (FIPS 180-4) & HMAC-SHA256 & IEEE 802.11 KDF-SHA256-384
 * ========================================================================= */

struct Sha256Ctx {
    uint32_t h[8];
    uint64_t total_bits;
    uint8_t  buf[64];
    size_t   buf_len;
};

static inline void sha256_init(Sha256Ctx *c) {
    c->h[0] = 0x6A09E667U; c->h[1] = 0xBB67AE85U; c->h[2] = 0x3C6EF372U; c->h[3] = 0xA54FF53AU;
    c->h[4] = 0x510E527FU; c->h[5] = 0x9B05688CU; c->h[6] = 0x1F83D9ABU; c->h[7] = 0x5BE0CD19U;
    c->total_bits = 0;
    c->buf_len = 0;
}

static void sha256_compress(Sha256Ctx *c, const uint8_t blk[64]) {
    static const uint32_t K[64] = {
        0x428A2F98U,0x71374491U,0xB5C0FBCFU,0xE9B5DBA5U,0x3956C25BU,0x59F111F1U,0x923F82A4U,0xAB1C5ED5U,
        0xD807AA98U,0x12835B01U,0x243185BEU,0x550C7DC3U,0x72BE5D74U,0x80DEB1FEU,0x9BDC06A7U,0xC19BF174U,
        0xE49B69C1U,0xEFBE4786U,0x0FC19DC6U,0x240CA1CCU,0x2DE92C6FU,0x4A7484AAU,0x5CB0A9DCU,0x76F988DAU,
        0x983E5152U,0xA831C66DU,0xB00327C8U,0xBF597FC7U,0xC6E00BF3U,0xD5A79147U,0x06CA6351U,0x14292967U,
        0x27B70A85U,0x2E1B2138U,0x4D2C6DFCU,0x53380D13U,0x650A7354U,0x766A0ABBU,0x81C2C92EU,0x92722C85U,
        0xA2BFE8A1U,0xA81A664BU,0xC24B8B70U,0xC76C51A3U,0xD192E819U,0xD6990624U,0xF40E3585U,0x106AA070U,
        0x19A4C116U,0x1E376C08U,0x2748774CU,0x34B0BCB5U,0x391C0CB3U,0x4ED8AA4AU,0x5B9CCA4FU,0x682E6FF3U,
        0x748F82EEU,0x78A5636FU,0x84C87814U,0x8CC70208U,0x90BEFFFAU,0xA4506CEBU,0xBEF9A3F7U,0xC67178F2U
    };
    uint32_t w[64];
    for (int i = 0; i < 16; ++i) w[i] = load_be32(blk + i * 4);
    for (int i = 16; i < 64; ++i) {
        uint32_t s0 = rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a=c->h[0], b=c->h[1], cc=c->h[2], d=c->h[3], e=c->h[4], f=c->h[5], g=c->h[6], h=c->h[7];
    for (int i = 0; i < 64; ++i) {
        uint32_t S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
        uint32_t ch = (e & f) ^ ((~e) & g);
        uint32_t t1 = h + S1 + ch + K[i] + w[i];
        uint32_t S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
        uint32_t maj = (a & b) ^ (a & cc) ^ (b & cc);
        uint32_t t2 = S0 + maj;
        h = g; g = f; f = e; e = d + t1;
        d = cc; cc = b; b = a; a = t1 + t2;
    }
    c->h[0]+=a; c->h[1]+=b; c->h[2]+=cc; c->h[3]+=d;
    c->h[4]+=e; c->h[5]+=f; c->h[6]+=g;  c->h[7]+=h;
}

static void sha256_update(Sha256Ctx *c, const uint8_t *data, size_t len) {
    c->total_bits += uint64_t(len) * 8ULL;
    while (len > 0) {
        if (c->buf_len == 0 && len >= 64) {
            sha256_compress(c, data);
            data += 64; len -= 64;
        } else {
            size_t take = 64 - c->buf_len;
            if (take > len) take = len;
            memcpy(c->buf + c->buf_len, data, take);
            c->buf_len += take; data += take; len -= take;
            if (c->buf_len == 64) { sha256_compress(c, c->buf); c->buf_len = 0; }
        }
    }
}

static void sha256_final(Sha256Ctx *c, uint8_t out[32]) {
    uint64_t bits = c->total_bits;
    uint8_t pad = 0x80;
    sha256_update(c, &pad, 1);
    uint8_t z = 0;
    while (c->buf_len != 56) sha256_update(c, &z, 1);
    uint8_t len_be[8];
    store_be64(len_be, bits);
    sha256_update(c, len_be, 8);
    for (int i = 0; i < 8; ++i) store_be32(out + i * 4, c->h[i]);
}

static void hmac_sha256(const uint8_t *key, size_t key_len,
                        const uint8_t *msg, size_t msg_len,
                        uint8_t out[32]) {
    uint8_t kblk[64];
    memset(kblk, 0, sizeof(kblk));
    if (key_len > 64) {
        Sha256Ctx kc; sha256_init(&kc); sha256_update(&kc, key, key_len); sha256_final(&kc, kblk);
    } else {
        memcpy(kblk, key, key_len);
    }
    uint8_t ipad[64], opad[64];
    for (int i = 0; i < 64; ++i) { ipad[i] = kblk[i] ^ 0x36; opad[i] = kblk[i] ^ 0x5C; }
    uint8_t inner[32];
    Sha256Ctx ci; sha256_init(&ci); sha256_update(&ci, ipad, 64); sha256_update(&ci, msg, msg_len); sha256_final(&ci, inner);
    Sha256Ctx co; sha256_init(&co); sha256_update(&co, opad, 64); sha256_update(&co, inner, 32); sha256_final(&co, out);
}

/* IEEE 802.11 KDF-SHA256-384 (for AKM 00-0F-AC:6 WPA2-PSK-SHA256) */
static void kdf_sha256_384(const uint8_t pmk[32], const uint8_t data[76], uint8_t out[48]) {
    static const char label[] = "Pairwise key expansion";
    uint8_t buf[2 + 22 + 76 + 2];
    memcpy(buf + 2, label, 22); /* no NUL in 802.11w KDF! */
    memcpy(buf + 24, data, 76);
    buf[100] = 0x80; buf[101] = 0x01; /* Length = 384 (0x0180) little-endian */
    uint8_t block[64];
    for (uint16_t ctr = 1; ctr <= 2; ++ctr) {
        buf[0] = uint8_t(ctr & 0xFF);
        buf[1] = uint8_t(ctr >> 8);
        hmac_sha256(pmk, 32, buf, sizeof(buf), block + (ctr - 1) * 32);
    }
    memcpy(out, block, 48);
}

/* =========================================================================
 * AES-128 (FIPS 197), RFC 3394 AES Key Unwrap, RFC 4493 AES-128-CMAC
 * ========================================================================= */

static const uint8_t AES_SBOX[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static const uint8_t AES_INV_SBOX[256] = {
    0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
    0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
    0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
    0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
    0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
    0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
    0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
    0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
    0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
    0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
    0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
    0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
    0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
    0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
    0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
    0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d
};

static inline uint8_t gf_mul2(uint8_t x) {
    return uint8_t((x << 1) ^ ((x & 0x80U) ? 0x1BU : 0U));
}
static inline uint8_t gf_mul(uint8_t a, uint8_t b) {
    uint8_t r = 0;
    for (int i = 0; i < 8; ++i) {
        if (b & 1U) r ^= a;
        a = gf_mul2(a);
        b >>= 1;
    }
    return r;
}

static void aes128_expand_key(const uint8_t key[16], uint8_t rk[176]) {
    static const uint8_t RCON[10] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1B,0x36};
    memcpy(rk, key, 16);
    for (int i = 4; i < 44; ++i) {
        uint8_t t[4];
        memcpy(t, rk + (i - 1) * 4, 4);
        if ((i & 3) == 0) {
            uint8_t tmp = t[0];
            t[0] = AES_SBOX[t[1]] ^ RCON[i / 4 - 1];
            t[1] = AES_SBOX[t[2]];
            t[2] = AES_SBOX[t[3]];
            t[3] = AES_SBOX[tmp];
        }
        for (int j = 0; j < 4; ++j) rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ t[j];
    }
}

static void aes128_encrypt_block(const uint8_t rk[176], const uint8_t in[16], uint8_t out[16]) {
    uint8_t s[16];
    for (int i = 0; i < 16; ++i) s[i] = in[i] ^ rk[i];
    for (int r = 1; r <= 10; ++r) {
        for (int i = 0; i < 16; ++i) s[i] = AES_SBOX[s[i]];
        uint8_t t;
        t = s[1]; s[1] = s[5]; s[5] = s[9];  s[9]  = s[13]; s[13] = t;
        t = s[2]; s[2] = s[10]; s[10] = t; t = s[6]; s[6] = s[14]; s[14] = t;
        t = s[15]; s[15] = s[11]; s[11] = s[7]; s[7] = s[3]; s[3] = t;
        if (r < 10) {
            for (int c = 0; c < 4; ++c) {
                uint8_t a0 = s[c*4+0], a1 = s[c*4+1], a2 = s[c*4+2], a3 = s[c*4+3];
                uint8_t x = a0 ^ a1 ^ a2 ^ a3;
                s[c*4+0] ^= x ^ gf_mul2(a0 ^ a1);
                s[c*4+1] ^= x ^ gf_mul2(a1 ^ a2);
                s[c*4+2] ^= x ^ gf_mul2(a2 ^ a3);
                s[c*4+3] ^= x ^ gf_mul2(a3 ^ a0);
            }
        }
        for (int i = 0; i < 16; ++i) s[i] ^= rk[r * 16 + i];
    }
    memcpy(out, s, 16);
}

static void aes128_decrypt_block(const uint8_t rk[176], const uint8_t in[16], uint8_t out[16]) {
    uint8_t s[16];
    for (int i = 0; i < 16; ++i) s[i] = in[i] ^ rk[160 + i];
    for (int r = 9; r >= 0; --r) {
        uint8_t t;
        t = s[13]; s[13] = s[9]; s[9] = s[5]; s[5] = s[1]; s[1] = t;
        t = s[2]; s[2] = s[10]; s[10] = t; t = s[6]; s[6] = s[14]; s[14] = t;
        t = s[3]; s[3] = s[7]; s[7] = s[11]; s[11] = s[15]; s[15] = t;
        for (int i = 0; i < 16; ++i) s[i] = AES_INV_SBOX[s[i]] ^ rk[r * 16 + i];
        if (r > 0) {
            for (int c = 0; c < 4; ++c) {
                uint8_t a0 = s[c*4+0], a1 = s[c*4+1], a2 = s[c*4+2], a3 = s[c*4+3];
                s[c*4+0] = gf_mul(a0,0x0e)^gf_mul(a1,0x0b)^gf_mul(a2,0x0d)^gf_mul(a3,0x09);
                s[c*4+1] = gf_mul(a0,0x09)^gf_mul(a1,0x0e)^gf_mul(a2,0x0b)^gf_mul(a3,0x0d);
                s[c*4+2] = gf_mul(a0,0x0d)^gf_mul(a1,0x09)^gf_mul(a2,0x0e)^gf_mul(a3,0x0b);
                s[c*4+3] = gf_mul(a0,0x0b)^gf_mul(a1,0x0d)^gf_mul(a2,0x09)^gf_mul(a3,0x0e);
            }
        }
    }
    memcpy(out, s, 16);
}

/* RFC 3394 AES-128 Key Unwrap: ciphertext_len = (n + 1) * 8, plaintext_len = n * 8 */
static bool aes128_key_unwrap(const uint8_t kek[16], const uint8_t *ct, size_t ct_len, uint8_t *pt) {
    if (ct_len < 24 || (ct_len & 7U) != 0) return false;
    size_t n = (ct_len / 8U) - 1U;
    uint8_t rk[176];
    aes128_expand_key(kek, rk);
    uint8_t a[8];
    memcpy(a, ct, 8);
    memcpy(pt, ct + 8, n * 8U);
    for (int j = 5; j >= 0; --j) {
        for (size_t i = n; i >= 1; --i) {
            uint64_t t = uint64_t(n) * uint64_t(j) + uint64_t(i);
            uint8_t b[16];
            memcpy(b, a, 8);
            for (int k = 0; k < 8; ++k) b[7 - k] ^= uint8_t((t >> (k * 8)) & 0xFFU);
            memcpy(b + 8, pt + (i - 1) * 8U, 8);
            aes128_decrypt_block(rk, b, b);
            memcpy(a, b, 8);
            memcpy(pt + (i - 1) * 8U, b + 8, 8);
        }
    }
    for (int k = 0; k < 8; ++k) {
        if (a[k] != 0xA6U) return false;
    }
    return true;
}

/* RFC 4493 AES-128-CMAC (for EAPOL Key Descriptor Version 3 / WPA2-PSK-SHA256) */
static void aes128_cmac(const uint8_t key[16], const uint8_t *msg, size_t len, uint8_t mac[16]) {
    uint8_t rk[176], l[16], k1[16], k2[16], z[16] = {};
    aes128_expand_key(key, rk);
    aes128_encrypt_block(rk, z, l);
    auto dbl = [](const uint8_t in[16], uint8_t out[16]) {
        uint8_t carry = (in[0] & 0x80U) ? 0x87U : 0U;
        for (int i = 0; i < 15; ++i) out[i] = uint8_t((in[i] << 1) | (in[i + 1] >> 7));
        out[15] = uint8_t((in[15] << 1) ^ carry);
    };
    dbl(l, k1);
    dbl(k1, k2);

    size_t n = (len + 15U) / 16U;
    bool complete = (len > 0 && (len & 15U) == 0);
    if (n == 0) { n = 1; complete = false; }

    uint8_t x[16] = {}, y[16];
    for (size_t i = 0; i + 1 < n; ++i) {
        for (int j = 0; j < 16; ++j) y[j] = x[j] ^ msg[i * 16 + j];
        aes128_encrypt_block(rk, y, x);
    }
    uint8_t last[16] = {};
    size_t rem = len - (n - 1) * 16U;
    if (rem > 0) memcpy(last, msg + (n - 1) * 16U, rem);
    if (complete) {
        for (int j = 0; j < 16; ++j) y[j] = x[j] ^ last[j] ^ k1[j];
    } else {
        last[rem] = 0x80U;
        for (int j = 0; j < 16; ++j) y[j] = x[j] ^ last[j] ^ k2[j];
    }
    aes128_encrypt_block(rk, y, mac);
}

/* Compute 48-byte PTK from PMK, AA (AP BSSID), SPA (our STA MAC), ANonce, SNonce */
static void derive_ptk(const uint8_t pmk[32],
                       const uint8_t aa[6], const uint8_t spa[6],
                       const uint8_t anonce[32], const uint8_t snonce[32],
                       bool sha256_kdf, uint8_t ptk[48]) {
    uint8_t data[76];
    if (memcmp(aa, spa, 6) < 0) {
        memcpy(data, aa, 6); memcpy(data + 6, spa, 6);
    } else {
        memcpy(data, spa, 6); memcpy(data + 6, aa, 6);
    }
    if (memcmp(anonce, snonce, 32) < 0) {
        memcpy(data + 12, anonce, 32); memcpy(data + 44, snonce, 32);
    } else {
        memcpy(data + 12, snonce, 32); memcpy(data + 44, anonce, 32);
    }
    if (sha256_kdf) kdf_sha256_384(pmk, data, ptk);
    else            prf_sha1_384(pmk, data, ptk);
}

} // namespace tb375fc_wcrypto

#endif /* TB375FC_WIFI_CRYPTO_H */
