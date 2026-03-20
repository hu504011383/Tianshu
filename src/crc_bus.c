#include "crc_bus.h"

// Tablice wspólne
static uint8_t  g_crc8_tbl[256]  __attribute__((aligned(4)));
static uint16_t g_crc16_tbl[256] __attribute__((aligned(4)));
static uint8_t  g_inited = 0;

void bus_crc_init(void)
{
    if (g_inited) return;
    g_inited = 1;

    for (uint32_t d = 0; d < 256; d++) {
        uint8_t r = (uint8_t)d;
        for (uint8_t b = 0; b < 8; b++) {
            if (r & 0x80u) r = (uint8_t)((uint8_t)(r << 1) ^ 0x39u);
            else          r = (uint8_t)(r << 1);
        }
        g_crc8_tbl[d] = r;
    }

    for (uint32_t d = 0; d < 256; d++) {
        uint16_t r = (uint16_t)(d << 8);
        for (uint8_t b = 0; b < 8; b++) {
            if (r & 0x8000u) r = (uint16_t)((uint16_t)(r << 1) ^ 0x1021u);
            else             r = (uint16_t)(r << 1);
        }
        g_crc16_tbl[d] = r;
    }
}

uint8_t bus_crc8(const uint8_t* data, uint32_t len)
{
    uint8_t r = 0x66u;
    for (uint32_t i = 0; i < len; i++) {
        r = g_crc8_tbl[(uint8_t)(r ^ data[i])];
    }
    return r;
}

uint16_t bus_crc16(const uint8_t* data, uint32_t len)
{
    uint16_t r = 0x913Du;
    for (uint32_t i = 0; i < len; i++) {
        const uint8_t idx = (uint8_t)((r >> 8) ^ data[i]);
        r = (uint16_t)(g_crc16_tbl[idx] ^ (uint16_t)(r << 8));
    }
    return r;
}
