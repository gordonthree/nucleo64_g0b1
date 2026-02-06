/**
* @file    node_id.c
* @brief   Implementation of CRC32-based Node ID using STM32 HAL CRC peripheral.
*/

#include "node_id.h"
#include "stm32g0xx_hal.h"
#include "crc.h"              // CubeMX-generated, declares CRC_HandleTypeDef hcrc
// #include "stm32g0xx_hal_msp.h"  // replace fxxx with your family (e.g., stm32f4xx_hal.h)

/* If you want compile-time debug prints, define NODE_ID_DEBUG and provide printf(). */
/* #define NODE_ID_DEBUG */

/**
 * @brief Compute and return the 4-byte Node ID based on the STM32 Unique Device ID.
 *
 * This function reads the factory unique device ID (96 bits as three 32-bit words),
 * calculates the CRC32 using the hardware CRC peripheral, and returns the result
 * as 4 bytes in MSB-first order.
 *
 * @param[out] out  Pointer to a 4-byte buffer to receive the Node ID.
 */

void NodeID_GetBytes(uint8_t out[NODE_ID_SIZE])
{
    /* Read the factory unique device ID (96 bits as three 32-bit words). */
    uint32_t uid_words[3];
    uid_words[0] = HAL_GetUIDw0();
    uid_words[1] = HAL_GetUIDw1();
    uid_words[2] = HAL_GetUIDw2();

#ifdef NODE_ID_DEBUG
    /* Use your preferred output (ITM/SWO/UART); printf shown for illustration. */
    printf("UID: %08lX:%08lX:%08lX\r\n", uid_words[0], uid_words[1], uid_words[2]);
#endif

    /* Calculate CRC32 using the hardware CRC peripheral.
       InputDataFormat must match WORDS in MX_CRC_Init(). */
    uint32_t crc = HAL_CRC_Calculate(&hcrc, uid_words, 3U);

#ifdef NODE_ID_DEBUG
    printf("CRC32: %08lX\r\n", crc);
#endif

    /* Return bytes MSB-first (matches your original Arduino shifts). */
    out[0] = (uint8_t)((crc >> 24) & 0xFFU);
    out[1] = (uint8_t)((crc >> 16) & 0xFFU);
    out[2] = (uint8_t)((crc >>  8) & 0xFFU);
    out[3] = (uint8_t)( crc        & 0xFFU);
}

/**
 * @brief Compute and return the 32-bit CRC over the STM32 Unique Device ID words.
 *
 * This function reads the factory unique device ID (96 bits as three 32-bit words)
 * and calculates CRC32 using the hardware CRC peripheral.
 *
 * @return 32-bit CRC32 value (IEEE polynomial 0x04C11DB7 with default HAL init value unless reconfigured).
 */
uint32_t NodeID_GetU32(void)
{
    uint32_t uid_words[3];
    uid_words[0] = HAL_GetUIDw0();
    uid_words[1] = HAL_GetUIDw1();
    uid_words[2] = HAL_GetUIDw2();

    /* Compute and return the 32-bit CRC over the UID words. */
    return HAL_CRC_Calculate(&hcrc, uid_words, 3U);
}
