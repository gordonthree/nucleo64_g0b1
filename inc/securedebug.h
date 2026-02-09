/* inc/securedebug.h */
#ifndef SECUREDEBUG_H_
#define SECUREDEBUG_H_

#include <string.h>
#include "stm32g0xx_hal.h"  // For UART_HandleTypeDef
#include "cmsis_os.h"       // For osMutexId, osOK, osWaitForever

// Function Prototypes
void SecureDebug(const char* buffer);

// If you want to use Getters (optional, but keep types consistent)
UART_HandleTypeDef* GetUART2Handle(void);
osMutexId GetUART2MutexHandle(void);

#endif