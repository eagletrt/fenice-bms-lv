/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

/* USER CODE BEGIN Private defines */
#define LOG_WLCM_HEADER_STR                                                                                                                    \
    "\r\n\r\n"                                                                                                                                 \
    " ███████╗███████╗███╗   ██╗██╗ ██████╗███████╗\r\n"     \
    " ██╔════╝██╔════╝████╗  ██║██║██╔════╝██╔════╝\r\n" \
    " █████╗  █████╗  ██╔██╗ ██║██║██║     █████╗  \r\n"                     \
    " ██╔══╝  ██╔══╝  ██║╚██╗██║██║██║     ██╔══╝  \r\n"                   \
    " ██║     ███████╗██║ ╚████║██║╚██████╗███████╗\r\n"         \
    " ╚═╝     ╚══════╝╚═╝  ╚═══╝╚═╝ ╚═════╝╚══════╝\r\n"             \
    " >>>>>>>>>>>>>>>>> BMS LV >>>>>>>>>>>>>>>>>>>>\r\n"

#define LOG_HUART huart4  // Uart used for log

/**
 * @brief   Serial print a static string on a given serial interface handle
 * @details The input string must be defined on the stack and defined while
 * declared. If the string was output of sprintf do not use this macro
 *
 * @param   __UART_HANDLE__ Uart handle where data will be logged
 * @param   __FXD_STR__ A char[] defined on the stack where
 *          strlen() == sizeof()-1
 *
 * @return  HAL_StatusTypeDef HAL_OK if all went ok else something went wrong
 */
#define M_SPRINT_FIXED_STRING(__UART_HANDLE__, __FXD_STR__) \
    HAL_UART_Transmit(&__UART_HANDLE__, (uint8_t *)__FXD_STR__, M_STATIC_FIXED_STRING_STRLEN(__FXD_STR__), 10)

/**
 * @brief  Logs a string on the common logging uart port
 *
 * @param  __STR__ A char[] defined on the stack where strlen() == sizeof()-1
 *
 * @return HAL_StatusTypeDef HAL_OK if all went ok else something went wrong
 */
#define M_LOG_STRING(__STR__) M_SPRINT_FIXED_STRING(LOG_HUART, __STR__)

/* USER CODE END Private defines */

void MX_UART4_Init(void);

/* USER CODE BEGIN Prototypes */
typedef enum {
    NORM_HEADER = 0U,
    NO_HEADER,
    NOTHING,
    ERR_HEADER,
    CAN_HEADER,
    UART_HEADER,
    CAN_ERR_HEADER
} UART_HeaderTypeDef;

/**
 * @brief Print log messages over the serial UART, prefixed with header
 *        "[MCU_NAME]"
 * @param txt Pointer to input string without \r\n and null terminated
 * @param header_type Type of postfix appended to the header, can be a
 *        parameter of @arg UART_HeaderTypeDef
 * */
void printl(char *txt, UART_HeaderTypeDef header_type);

/**
 * @brief Disable logging messages
 */
void UART_disable_log(void);

/**
 * @brief Enable logging messages
 */
void UART_enable_log(void);

/**
 * @brief Check if logging is enabled or not
 *
 * @return true if logging enabled else otherwise
 */
bool UART_is_log_on();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
