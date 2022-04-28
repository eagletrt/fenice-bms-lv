/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "common.h"
#include "stdio.h"
#include "string.h"

static void _log_wlcm_header();

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
volatile static bool _is_log_on = true;

void UART_disable_log(void) {
    printl("Logging disabled", UART_HEADER);
    _is_log_on = false;
};

void UART_enable_log(void) {
    _is_log_on = true;
    printl("Logging enabled", UART_HEADER);
};
bool UART_is_log_on() {
    return _is_log_on;
};

void printl(char *txt, UART_HeaderTypeDef header_type) {
    if (_is_log_on) {
        char *header;
        switch (header_type) {
            case NORM_HEADER:
                header = "\r\n[" M_NAME_TO_STR(MCU_NAME) "@fenice]~> ";
                break;
            case ERR_HEADER:
                header = "\r\n[" M_NAME_TO_STR(MCU_NAME) "@fenice]~> {!ERROR!} ";
                break;
            case CAN_HEADER:
                header = "\r\n[" M_NAME_TO_STR(MCU_NAME) "@fenice]~> {CAN} ";
                break;
            case CAN_ERR_HEADER:
                header = "\r\n[" M_NAME_TO_STR(MCU_NAME) "@fenice]~> {CAN !ERROR!} ";
                break;
            case UART_HEADER:
                header = "\r\n[" M_NAME_TO_STR(MCU_NAME) "@fenice]~> {UART} ";
                break;
            case NO_HEADER:
                header = "\r\n";
                break;
            case VOLT_HEADER:
                header = "\r\n[" M_NAME_TO_STR(MCU_NAME) "@fenice]~> {VOLT} ";
                break;     
            case NOTHING:
            default:
                header = "";
                break;
        }
        char buf[200];
        sprintf(buf, "%s", header);
        strncat(buf, txt, 200 - strlen(buf) - 1);
        HAL_UART_Transmit(&LOG_HUART, (uint8_t *)buf, strlen(buf), 100);
    }
}

static void _log_wlcm_header() {
    HAL_UART_Transmit(
        &LOG_HUART, (uint8_t *)LOG_WLCM_HEADER_STR, M_STATIC_FIXED_STRING_STRLEN(LOG_WLCM_HEADER_STR), 100);
}
/* USER CODE END 1 */
