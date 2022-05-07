/**
 * @file      common.h
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      2021-03-28
 * @updated
 * @ingroup
 * @prefix    COMM
 *
 * @brief     This file contains common entities for the whole mcu
 *
 *
 * @copyright Copyright (c) 2021 E-Agle TRT
 */
#ifndef _COMMON_H_
#define _COMMON_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief     Common Status structures definition
 */
typedef enum { COMM_OK = 0x00U, COMM_ERROR = 0x01U, COMM_BUSY = 0x02U, COMM_TIMEOUT = 0x03U } COMM_StatusTypeDef;

/**
 * @brief     Common lock structure definition
 */
typedef enum { COMM_UNLOCKED = 0x00U, COMM_LOCKED = 0x01U } COMM_LockTypeDef;

/* Exported constants --------------------------------------------------------*/
#define MCU_NAME BMS_LV
/* Exported macro ------------------------------------------------------------*/

/**
 * @brief Returns the size of a static fixed length string
 * @details The input string must be defined on the stack and initialized on the
 *          spot. If the string has been input of sprintf than DO NOT USE THIS
 *          MACRO
 * @param  __STATIC_FIXD_STRNG_PNTR__ A char[] defined on the stack
 * @return The length of the input string
 */
#define M_STATIC_FIXED_STRING_STRLEN(__STATIC_FIXD_STRNG_PNTR__) \
    (size_t)((sizeof(__STATIC_FIXD_STRNG_PNTR__) / sizeof(*(__STATIC_FIXD_STRNG_PNTR__))) - 1)

/**
 * @brief    Returns the in code string representation of the given parameter
 *
 * @param    _NAME_ Any valid C identifier
 *
 */
#define M_NAME_TO_STR(_NAME_) _STR(_NAME_)

/* Exported functions --------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros -----------------------------------------------------------*/

#define _STR(_STR_) #_STR_

#endif /*_COMMON_H_*/
