/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "common.h"
#include "stdio.h"
#include "string.h"

static void _log_wlcm_header();

/* USER CODE END 0 */

UART_HandleTypeDef huart4;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
    _log_wlcm_header();
  /* USER CODE END UART4_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA11     ------> UART4_RX
    PA12     ------> UART4_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA11     ------> UART4_RX
    PA12     ------> UART4_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
