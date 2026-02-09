/* src/securedebug.c */
#include "securedebug.h"

// We reference these from main.c
extern osMutexId uartMutexHandle;
extern UART_HandleTypeDef huart2;

// If you want to use the Getters defined in your header:
UART_HandleTypeDef* GetUART2Handle(void) {
    return &huart2;
}

osMutexId GetUART2MutexHandle(void) {
    return uartMutexHandle;
}

void SecureDebug(const char* buffer) {
    // Note: use the handle directly or your getter
    if (osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
        osMutexRelease(uartMutexHandle);
    }
}