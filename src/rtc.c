/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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
#include "rtc.h"

/* USER CODE BEGIN 0 */

RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

uint32_t RTC_Get_Timestamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef RTC_Set_Timestamp(RTC_HandleTypeDef *hrtc, uint32_t timestamp);


/* USER CODE END 0 */


/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 0x2;
  sDate.Year = 0x26;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_RTCAPB_CLK_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include <time.h>

/* *
 * Converts current RTC time/date into a 32-bit Unix timestamp.
 * Useful for your CAN node data payloads.
 */
uint32_t RTC_Get_Timestamp(RTC_HandleTypeDef *hrtc) {
    static RTC_TimeTypeDef sTime;
    static RTC_DateTypeDef sDate;
    struct tm time_struct;

    HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN);

    time_struct.tm_year = sDate.Year + 100; // RTC Year is 0-99 (2000-2099)
    time_struct.tm_mon  = sDate.Month - 1;
    time_struct.tm_mday = sDate.Date;
    time_struct.tm_hour = sTime.Hours;
    time_struct.tm_min  = sTime.Minutes;
    time_struct.tm_sec  = sTime.Seconds;

    return (uint32_t)mktime(&time_struct);
}


/**
 * @brief Thread-safe version of RTC sync using gmtime_r.
 * @param hrtc: Pointer to the RTC handle
 * @param timestamp: Unix timestamp (seconds since Jan 1, 1970)
 * @return HAL_StatusTypeDef: HAL_OK if successful
 */
HAL_StatusTypeDef RTC_Set_Timestamp(RTC_HandleTypeDef *hrtc, uint32_t timestamp) {
    /* * By declaring 'result' on the stack, we ensure that even if multiple
     * FreeRTOS tasks call this function, each has its own private copy.
     */
    struct tm result;
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    time_t raw_time = (time_t)timestamp;

    /* * gmtime_r takes the source and a pointer to our local 'result' struct.
     * It returns NULL if the conversion fails.
     */
    if (gmtime_r(&raw_time, &result) == NULL) {
        return HAL_ERROR;
    }

    /* Safety check: STM32 RTC usually handles 2000-2099 */
    if (result.tm_year < 100 || result.tm_year > 199) {
        /* Optional: Handle out-of-range dates (pre-2000 or post-2099) */
        return HAL_ERROR;
    }

    /* Map components to RTC Time Structure */
    sTime.Hours   = (uint8_t)result.tm_hour;
    sTime.Minutes = (uint8_t)result.tm_min;
    sTime.Seconds = (uint8_t)result.tm_sec;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    /* Map components to RTC Date Structure */
    sDate.Year    = (uint8_t)(result.tm_year - 100); /* 2026 becomes 26 */
    sDate.Month   = (uint8_t)(result.tm_mon + 1);    /* 0-11 -> 1-12 */
    sDate.Date    = (uint8_t)result.tm_mday;
    sDate.WeekDay = (uint8_t)(result.tm_wday == 0 ? 7 : result.tm_wday);

    /* Update Hardware (Time then Date for Shadow Register sync) */
    if (HAL_RTC_SetTime(hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_RTC_SetDate(hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* USER CODE END 1 */
