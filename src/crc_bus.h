#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void     bus_crc_init(void);
uint8_t  bus_crc8(const uint8_t* data, uint32_t len);
uint16_t bus_crc16(const uint8_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif
