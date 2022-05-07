/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
extern ADC_ChannelConfTypeDef UserAdcConfig;
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief This functions is a wrapper HAL_ADC_Start_DMA(...)it, starts ADC in DMA mode 
 * @note  The necessity of this function was due to confine the array of ADC values into the ADC module.
 *        Proper getters will be used to acces the values retirived from the adc
 * @return see @ref HAL_ADC_Start_DMA
 */
HAL_StatusTypeDef ADC_start_dma_readings();
/* USER CODE END Prototypes */

/**
 * @brief Current sensor value getter
 * @return current sensor value
 */
uint16_t ADC_get_i_sensor_val();

/**
 * @brief Battery Temperature sensor #1 value getter
 * @return Battery Temperature sensor #1 value
 */
uint16_t ADC_get_t_batt1_val();

/**
 * @brief Battery Temperature sensor #2 value getter
 * @return Battery Temperature sensor #2 value
 */
uint16_t ADC_get_t_batt2_val();

/**
 * @brief DCDC 12v Temperature sensor value getter
 * @return DCDC 12v Temperature sensor value
 */
uint16_t ADC_get_t_dcdc12_val();

/**
 * @brief DCDC 24v Temperature sensor value getter
 * @return DCDC 24v Temperature sensor value
 */
uint16_t ADC_get_t_dcdc24_val();

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

