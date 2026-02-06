/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>   // For sprintf and snprintf
#include <stdlib.h>  // For abs
#include <string.h>  // For memset (if you use it)
#include "canbus_project.h" /* my various CAN functions and structs */
#include "typedef.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "typedef.h"
#include "crc.h"
#include "adc.h"
#include "fdcan.h"
#include "dma.h"
#include "gpio.h"
#include "rtc.h"
#include "securedebug.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// Board ID (calculated in DefaultTask, we'll make it global)
uint32_t board_crc;

osMutexId uartMutexHandle; // CMSIS-RTOS handle
osMutexDef(uartMutex);

extern uint16_t adc_buffer[2];           // From main.c or adc.c
extern FDCAN_RxHeaderTypeDef rxHeader;   // From main.c or fdcan.c
extern uint8_t rxData[8];                // From main.c or fdcan.c
 
osThreadId defaultTaskHandle;
osThreadId lcdTaskHandle;
osThreadId blinkTaskHandle;

osThreadId canTxTaskHandle;
osThreadId canRxTaskHandle;

osMessageQId canTxQueueHandle;


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartLCDTask(void const * argument);
void StartBlinkTask(void const * argument);
void StartCanRxTask(void const * argument);
void StartCanTxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of canTxQueue */
  osMessageQDef(canTxQueue, 10, CAN_Msg_t);
  canTxQueueHandle = osMessageCreate(osMessageQ(canTxQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of lcdTask */
  osThreadDef(lcdTask, StartLCDTask, osPriorityLow, 0, 768);
  lcdTaskHandle = osThreadCreate(osThread(lcdTask), NULL);

  /* definition and creation of blinkTask */
  osThreadDef(blinkTask, StartBlinkTask, osPriorityLow, 0, 128);
  blinkTaskHandle = osThreadCreate(osThread(blinkTask), NULL);

  /* definition and creation of canRxTask */
  osThreadDef(canRxTask, StartCanRxTask, osPriorityHigh, 0, 256);
  canRxTaskHandle = osThreadCreate(osThread(canRxTask), NULL);

  /* definition and creation of canTxTask */
  osThreadDef(canTxTask, StartCanTxTask, osPriorityNormal, 0, 256);
  canTxTaskHandle = osThreadCreate(osThread(canTxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  static char msg[256]; // Buffer to hold the string
  static uint32_t uid[3]; // Buffer to hold the UID
  uid[0] = HAL_GetUIDw0();
  uid[1] = HAL_GetUIDw1();
  uid[2] = HAL_GetUIDw2();

  // We use HAL_CRC_Calculate which handles the peripheral state and returns the 32-bit result
  board_crc = HAL_CRC_Calculate(&hcrc, uid, 3);

  // 2. Identify which binary we are running using the PIO build flag
  // We use a default value (0) in case the flag isn't defined
  #ifndef BOARD_ID
    #define BOARD_ID 0
  #endif

  osDelay(1000);

  snprintf(msg, sizeof(msg), 
          "\r\n--- DEVICE IDENTITY ---\r\n"
          "PIO ENV ID: %d\r\n"
          "Chip UID: %08X-%08X-%08X\r\n"
          "UNIQUE CRC:  0x%08X\r\n"
          "-----------------------\r\n\r\n", 
          (int)BOARD_ID, (unsigned int)uid[0], (unsigned int)uid[1], (unsigned int)uid[2], (unsigned int)board_crc);

  SecureDebug(msg); // Print to UART2

  osDelay(2000);

  // 2. Signal the LCD task to start
  // We use the handle 'lcdTaskHandle' created by the CMSIS-RTOS wrapper
  xTaskNotifyGive(lcdTaskHandle);

  /* Infinite loop */
  for(;;)
  {
    // 1. Get the current tick value (milliseconds since boot)
    uint16_t tick = (HAL_GetTick() / 1000);

    // 2. Format the string
    // %lu is used for unsigned long (uint32_t)
    snprintf(msg, sizeof(msg), "Uptime: %u sec\r\n", tick);

    // 3. Transmit over UART2
    SecureDebug(msg); // Print to UART2

    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLCDTask */
/**
* @brief Function implementing the lcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void const * argument)
{
  /* USER CODE BEGIN StartLCDTask */
  static char msg[64] __attribute__((aligned(8)));

  // What are the factory ADC Calibration Constants for STM32G0?
  uint16_t *TS_CAL1 = (uint16_t *)(0x1FFF75A8); // Note: Address differs from F1/F3
  uint16_t *TS_CAL2 = (uint16_t *)(0x1FFF75CA); 

  // Wait indefinitely for a notification from the Default Task
  // This puts the task into a 'Blocked' state (0% CPU usage)
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // 1. Start Calibration
  HAL_ADCEx_Calibration_Start(&hadc1);
  // osDelay(50);

  /* Manually ensure internal paths are enabled */
  ADC1_COMMON->CCR |= ADC_CCR_TSEN;   // Temperature Sensor Enable
  ADC1_COMMON->CCR |= ADC_CCR_VREFEN; // Vrefint Enable

  // 2. Start ADC in DMA Mode
  // This tells the hardware: "Convert 2 channels and put them in adc_buffer"
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);

  // osDelay(1000); 
  /* Infinite loop */
  for(;;)
  {
    // 3. No Polling! The data is already in the buffer.
    uint32_t rawAn1 = adc_buffer[0];
    uint32_t rawTemp = adc_buffer[1];

    /* --- Corrected Calculation --- */
    // 1. Adjust rawTemp to a 3.0V reference (assuming your VCC is 3.3V)
    // 918 * 3300 / 3000 = ~1010
    int32_t rawTemp_30V = (int32_t)rawTemp * 3300 / 3000;

    // 2. Calculate the difference from the 30C calibration point
    int32_t delta_ADC = rawTemp_30V - (int32_t)(*TS_CAL1);

    // 3. Scale and divide
    // (110 - 30) = 80. We multiply by 10 to get one decimal place (800)
    int32_t temp_scaled = (delta_ADC * 800) / (int32_t)(*TS_CAL2 - *TS_CAL1) + 300;

    int t_int = temp_scaled / 10;
    int t_dec = abs(temp_scaled % 10);

    /* --- Printing --- */
    snprintf(msg, sizeof(msg), "\r\nA1:%u T:%d.%dC\r\n", 
              (unsigned int)rawAn1, t_int, t_dec);

    SecureDebug(msg); // Print to UART2

    osDelay(1000); // The DMA updates much faster than this loop
  }
  /* USER CODE END StartLCDTask */
}

/* USER CODE BEGIN Header_StartBlinkTask */
/**
* @brief Function implementing the blinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkTask */
void StartBlinkTask(void const * argument)
{
  /* USER CODE BEGIN StartBlinkTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  /* USER CODE END StartBlinkTask */
}


void StartCanRxTask(void const * argument) {
    for(;;) {
        // Wait for notification from the ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
            // 1. Extract Remote ID (Bytes 0-3 as per your CSV)
            uint32_t msgRemoteId = (rxData[0]) | (rxData[1] << 8) | (rxData[2] << 16) | (rxData[3] << 24);

            // 2. Check if this message is intended for this board
            if (msgRemoteId == board_crc) {
                // uint8_t switchId = rxData[4]; // Byte 4 is switch id in CSV

                switch (rxHeader.Identifier) {
                    case SW_SET_ON:
                        // Example: Toggle LED or Load based on switchId
                        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
                        SecureDebug("CAN: Load ON\r\n");
                        break;

                    case SW_SET_OFF:
                        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
                        SecureDebug("CAN: Load OFF\r\n");
                        break;
                        
                    default:
                        break;
                }
            }
        }
    }
}

void StartCanTxTask(void const * argument) {
    // We don't need the local txMsg buffer anymore because 
    // we are getting a pointer from the queue event.
    FDCAN_TxHeaderTypeDef txHeader;

    for(;;) {
        // 1. Capture the event in a variable so we can access its value
        osEvent event = osMessageGet(canTxQueueHandle, osWaitForever);

        if (event.status == osEventMessage) {
            // 2. Cast the value to our pointer type
            // Note: This assumes you sent a pointer to the queue 
            // OR the struct is small enough to fit in the value field.
            CAN_Msg_t *ptrMsg = (CAN_Msg_t*)event.value.p;

            // 3. Assign Header values using the GLOBAL FDCAN constants
            txHeader.Identifier = ptrMsg->Identifier;
            txHeader.IdType = FDCAN_STANDARD_ID;      // Hardware constant
            txHeader.TxFrameType = FDCAN_DATA_FRAME;   // Hardware constant
            
            // Set DLC. Since ptrMsg has a DLC field, you can use it, 
            // but FDCAN uses specific macros for the field.
            txHeader.DataLength = FDCAN_DLC_BYTES_8; 
            
            txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            txHeader.BitRateSwitch = FDCAN_BRS_OFF;
            txHeader.FDFormat = FDCAN_CLASSIC_CAN;
            txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            txHeader.MessageMarker = 0;

            // 4. Send the message using the data from the pointer
            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, ptrMsg->Data);
        }
    }
}

