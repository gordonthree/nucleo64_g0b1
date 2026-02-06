/**
 * @file    node_id.h
 * @brief   Compute a CRC32-based Node ID from the STM32 Unique Device ID (UID).
 *
 * This module reads the factory UID words using HAL (HAL_GetUIDw0/1/2),
 * calculates CRC32 with the hardware CRC peripheral, and returns the result
 * either as a 4-byte array (MSB-first) or as a uint32_t.
 *
 * Requirements:
 *  - CRC peripheral enabled and initialized (MX_CRC_Init())
 *  - Valid CRC_HandleTypeDef hcrc (provided by CubeMX-generated code)
 */

#ifndef NODE_ID_H
#define NODE_ID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "canbus_project.h" /* my various CAN functions and structs */
#include "typedef.h"


/**
 * @brief Compute CRC32 over the STM32 Unique Device ID and return it as 4 bytes.
 *
 * @param[out] out  Pointer to a 4-byte buffer. Bytes are returned MSB-first:
 *                  out[0] = CRC[31:24], out[1] = CRC[23:16], out[2] = CRC[15:8], out[3] = CRC[7:0].
 *
 * @note  The function uses HAL_CRC_Calculate with InputDataFormat=WORDS (uint32_t).
 *        Ensure MX_CRC_Init() has been called before using this function.
 */
void NodeID_GetBytes(uint8_t out[NODE_ID_SIZE]);

/**
 * @brief Compute CRC32 over the STM32 Unique Device ID and return it as uint32_t.
 *
 * @return CRC32 value (IEEE polynomial 0x04C11DB7 with default HAL init value unless reconfigured).
 */
uint32_t NodeID_GetU32(void);

/**
 * @brief (Optional) Convert a uint32_t CRC into MSB-first bytes.
 *
 * @param[in]  crc  CRC32 value.
 * @param[out] out  4-byte buffer to receive MSB-first bytes.
 */
static inline void NodeID_U32ToBytes(uint32_t crc, uint8_t out[NODE_ID_SIZE])
{
    out[0] = (uint8_t)((crc >> 24) & 0xFFU);
    out[1] = (uint8_t)((crc >> 16) & 0xFFU);
    out[2] = (uint8_t)((crc >>  8) & 0xFFU);
    out[3] = (uint8_t)( crc        & 0xFFU);
}

#ifdef __cplusplus
}
#endif

#endif /* NODE_ID_H */