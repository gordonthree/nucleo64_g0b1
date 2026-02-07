/* Includes ------------------------------------------------------------------*/
#include <stdio.h>   // For sprintf and snprintf
#include <stdlib.h>  // For abs
#include <string.h>  // For memset (if you use it)
#include "canbus_project.h" /* my various CAN functions and structs */

#include "FreeRTOS.h"
#include "main.h"


/* Private variables ---------------------------------------------------------*/
uint8_t introMsgPtr; /**< pointer for intro messages */

extern uint32_t board_crc; 

osMutexId uartMutexHandle; // CMSIS-RTOS handle
osMutexDef(uartMutex);

extern uint16_t adc_buffer[2];           
extern FDCAN_RxHeaderTypeDef rxHeader;   
extern uint8_t rxData[8];                

struct canNodeInfo nodeInfo; /**< Store information about this node */

/* Create the handles for RTOS tasks*/
osThreadId defaultTaskHandle;
osThreadId canTxTaskHandle;
osThreadId canRxTaskHandle;

/* Define the message pool: 16 blocks of type CAN_Msg_t */
osPoolId canMsgPoolHandle;
osPoolDef(canMsgPool, 16, CAN_Msg_t); 

/* Define the TX pointer queue: 16 slots, each holding a POINTER (uint32_t) */
osMessageQId canTxQueueHandle;
osMessageQDef(canTxQueue, 16, uint32_t);

/* Define the RX pointer queue: 16 slots, each holding a POINTER (uint32_t) */
osMessageQId canRxQueueHandle;
osMessageQDef(canRxQueue, 16, uint32_t);

/* Global function prototypes -----------------------------------------------*/

void StartDefaultTask(void const * argument);
void StartCanRxTask(void const * argument);
void StartCanTxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Private function prototypes -----------------------------------------------*/

static void txIntroduction(void); /**< Function to send data about this node to the can bus */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  // Calculate Board ID here to ensure it is ready before any task starts.
  board_crc = NodeID_GetU32();


  /* BEGIN RTOS_MUTEX */
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));

  /* Create the Memory Pool */
  canMsgPoolHandle = osPoolCreate(osPool(canMsgPool));

  /* Create the Message Queue */
  canTxQueueHandle = osMessageCreate(osMessageQ(canTxQueue), NULL);
  canRxQueueHandle = osMessageCreate(osMessageQ(canRxQueue), NULL);
  /* END RTOS_MUTEX */

  /* Create the thread(s) */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of canRxTask */
  osThreadDef(canRxTask, StartCanRxTask, osPriorityHigh, 0, 256);
  canRxTaskHandle = osThreadCreate(osThread(canRxTask), NULL);

  /* definition and creation of canTxTask */
  osThreadDef(canTxTask, StartCanTxTask, osPriorityNormal, 0, 256);
  canTxTaskHandle = osThreadCreate(osThread(canTxTask), NULL);
}


/* --- Node Introduction Logic --- */
static void txIntroduction(void) {
    if (FLAG_SEND_INTRODUCTION == false) return; /**< If we're not supposed to send an intro, don't */

    char msg[64] __attribute__((aligned(8)));
    CAN_Msg_t *pNew = (CAN_Msg_t*)osPoolAlloc(canMsgPoolHandle); /* Allocate from Pool (Non-blocking) */
    if (pNew == NULL) {
      SecureDebug(("TX INTRO: Pool Error\r\n"));
      return;
    }
    uint16_t txMsgID = 0;
    uint8_t payload[2] = {0};

    // --- STEP 0: Introduce the Node itself ---
    if (introMsgPtr == 0) {
        txMsgID = nodeInfo.nodeTypeMsg; // Retrieve node type aka can message ID
        if (txMsgID == 0) {
            osPoolFree(canMsgPoolHandle, pNew); /* Release unused pool block */
            return;
        }
        snprintf(msg, sizeof(msg), "TX INTRO: NODE INTRO Type %03x\r\n", txMsgID);
        SecureDebug(msg);
        // SecureDebug("TX: NODE INTRO Type %03x\r\n", txMsgID);
        
        // Send the feature mask
        payload[0] = nodeInfo.featureMask[0]; 
        payload[1] = nodeInfo.featureMask[1];    
    } 
    // --- STEP 1+: Introduce Sub-Modules ---
    else {
        uint8_t modIdx = (uint8_t)(introMsgPtr - 1); // Sub-modules start at ptr 1
        
        if (modIdx >= nodeInfo.subModCnt) {
            osPoolFree(canMsgPoolHandle, pNew); /* Release unused pool block */
            FLAG_SEND_INTRODUCTION = false; // We've reached the end
            return;
        }

        txMsgID = nodeInfo.subModules[modIdx].modTypeMsg; // Retrieve module type aka can message ID
        
        if (txMsgID == 0) {
            osPoolFree(canMsgPoolHandle, pNew); /* Release unused pool block */
            introMsgPtr++; /* Error condition, skip empty slot and move to next */
            return;
        }
        snprintf(msg, sizeof(msg), "TX INTRO: MOD INTRO Type %03x at Idx %i\r\n", txMsgID, modIdx);
        SecureDebug(msg);
        // SecureDebug("TX: MOD INTRO Type %03x at Idx %i\r\n", txMsgID, modIdx);
        
        if (nodeInfo.subModules[modIdx].sendFeatureMask) {
            // Send the feature mask
            payload[0] = nodeInfo.subModules[modIdx].featureMask[0];
            payload[1] = nodeInfo.subModules[modIdx].featureMask[1];
        }
    }

    /* Assemble the final 8-byte CAN Frame */
    pNew->canID = txMsgID;

    /* Get a pointer to the start of the payload as a byte array */
    uint8_t *pData = (uint8_t*)&pNew->payload; 

    /*  Bytes 0-3: Hardware Node ID (CRC32) */
    *(uint32_t*)&pData[0] = nodeInfo.nodeID;

    /* Bytes 4-5: Feature Mask */
    pData[4] = payload[0];
    pData[5] = payload[1];

    /* Bytes 6-7: Padding */
    pData[6] = 0;
    pData[7] = 0;

    pNew->DLC = 8;

    /* Push the POINTER to the queue */
    if (osMessagePut(canTxQueueHandle, (uint32_t)pNew, 0) != osOK) {
        osPoolFree(canMsgPoolHandle, pNew); /* Release unused pool block */
    }
}

/* USER CODE BEGIN RTOS_THREADS */


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
  static char msg[256]; /* Buffer to hold message strings */
  introMsgPtr = 0; /* Reset the pointer */
  FLAG_SEND_INTRODUCTION = true; /* Set flag to send the introduction*/

  /* Initialize nodeInfo */
  nodeInfo.nodeID = NodeID_GetU32(); 
  nodeInfo.nodeTypeMsg = BOX_MULTI_IO_ID;
  nodeInfo.subModCnt = 2;

  nodeInfo.subModules[0].modTypeMsg = INPUT_ANALOG_KNOB_ID;
  nodeInfo.subModules[0].modTypeDLC = INPUT_ANALOG_KNOB_DLC;
  nodeInfo.subModules[0].dataSize = 2; // 2 bytes
  nodeInfo.subModules[0].sendFeatureMask = false;

  nodeInfo.subModules[1].modTypeMsg = NODE_CPU_TEMP_ID;
  nodeInfo.subModules[1].modTypeDLC = NODE_CPU_TEMP_DLC;
  nodeInfo.subModules[1].dataSize = 2; // 2 bytes
  nodeInfo.subModules[1].sendFeatureMask = false;

  osDelay(2000);

  snprintf(msg, sizeof(msg), 
          "\r\n\r\n"
          "-----------------------\r\n"
          "--- DEVICE IDENTITY ---\r\n"
          "--NODE ID: 0x%08lX--\r\n"
          "-----------------------\r\n\r\n", 
          (uint32_t)nodeInfo.nodeID);

  SecureDebug(msg); // Print to UART2

  FLAG_SEND_INTRODUCTION = true;

  // 2. Signal the CAN transmitter task to start
  // We use the handle 'canTxTaskHandle' created by the CMSIS-RTOS wrapper
  xTaskNotifyGive(canTxTaskHandle);

  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

    // Handle the 1-second logic (nodeCheckStatus)
    // nodeCheckStatus();

    /* Keep the submodule data fresh, copying from the DMA buffer 
    *  TODO: Needs to be based on a build flag that defines what type of module the code 
    *  is running on. Not all modules will use the ADC */
    extern uint16_t adc_buffer[2]; 
    nodeInfo.subModules[0].data.i32Value = (uint32_t)adc_buffer[0]; // Example: VRef
    nodeInfo.subModules[1].data.i32Value = (uint32_t)adc_buffer[1]; // Example: Temp

    if (FLAG_SEND_INTRODUCTION) {
      /* We haven't received an INTRO_ACK for a while, re-send the current portion in case the master missed it */
      txIntroduction();
    }
    // This replaces the millis() check. 1000ms = 1Hz.
    osDelay(500);
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




/**
 * @brief Function implementing the canRxTask thread.
 * @details This function continuously waits for message pointers from the ISR
 *          and checks if the message is intended for this node. It then
 *          performs the respective actions based on the message type.
 * @param argument: Not used
 * @retval None
 */
void StartCanRxTask(void const * argument) {
    osEvent event;
    CAN_Msg_t *pRx;
    char dbg[64]; // Local stack message buffer

    for(;;) {
        /* 1. Wait for a message pointer from the ISR */
        event = osMessageGet(canRxQueueHandle, osWaitForever);

        if (event.status == osEventMessage) {
            pRx = (CAN_Msg_t*)event.value.p;

            /* 2. Extract Remote ID using little-endian pointer cast */
            uint32_t msgRemoteId = *(uint32_t*)&pRx->payload;

            /* 3. Logic: Is this intended for us? */
            if (msgRemoteId == nodeInfo.nodeID) {
                
                switch (pRx->canID) {
                    case REQ_NODE_INTRO_ID:
                        FLAG_BEGIN_NORMAL_OPER = false; 
                        FLAG_SEND_INTRODUCTION = true;
                        SecureDebug("CAN RX: Master Requested Intro\r\n");
                        break;

                    case ACK_INTRO_ID:
                        /* Increment pointer ONLY on hardware ACK */
                        introMsgPtr++; 
                        
                        if (introMsgPtr > nodeInfo.subModCnt) {
                            FLAG_SEND_INTRODUCTION = false;
                            FLAG_BEGIN_NORMAL_OPER = true;
                            SecureDebug("CAN RX: Intro Sequence Complete\r\n");
                        } else {
                            /* Trigger next intro packet immediately */
                            txIntroduction();
                        }
                        break;

                    case SW_SET_ON_ID:
                        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
                        SecureDebug("CAN RX: Remote Load ON\r\n");
                        break;

                    case SW_SET_OFF_ID:
                        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
                        SecureDebug("CAN RX: Remote Load OFF\r\n");
                        break;
                }
            }

            /* 4. ALWAYS free the pointer back to the pool */
            osPoolFree(canMsgPoolHandle, pRx);
        }
    }
}

/**
 * @brief  CAN Transmitter Task
 * @details  This task waits for pointers to CAN_Msg_t structures to be pushed into the canTxQueueHandle by other tasks.
 *             It then transmits the data over the CAN bus using the HAL_FDCAN library.
 *             The task is responsible for checking for hardware space and releasing memory back to the pool.
 * @param  argument: Unused
 * @retval None
 */
void StartCanTxTask(void const * argument) {
    osEvent event;
    CAN_Msg_t *pMsg;
    FDCAN_TxHeaderTypeDef txHeader;

    /* Initialize static header fields */
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    for(;;) {
        /* 1. Wait for a pointer to arrive in the queue */
        event = osMessageGet(canTxQueueHandle, osWaitForever);

        if (event.status == osEventMessage) {
            /* 2. Retrieve the pointer to the CAN_Msg_t in the pool */
            pMsg = (CAN_Msg_t*)event.value.p;

            if (pMsg != NULL) {
                txHeader.Identifier = pMsg->canID;

                /* 3. Check for hardware space and transmit */
                if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
                  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t*)&pMsg->payload) != HAL_OK) {
                      SecureDebug("CAN HW Error\r\n");
                  }
                } else {
                    SecureDebug("CAN Mailbox Full\r\n");
                }

                /* 4. IMPORTANT: Free the memory back to the pool so it can be reused */
                osPoolFree(canMsgPoolHandle, pMsg);
            }
        }
    }
}
