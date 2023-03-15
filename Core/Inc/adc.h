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

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

/**
 *  @brief Number of samples used to compute the value of HO_50S_SP33 current transducer
 *        HO_50S_SP33 value is calculated as the average of this samples
 */
#define HO_50S_SP33_SAMPLES_FOR_AVERAGE (10U)

extern ADC_ChannelConfTypeDef UserAdcConfig;
/* USER CODE END Private defines */

void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief  Get resoultion bits of the ADC
 * @param  adcHandle @ref ADC_HandleTypeDef
 * @return The adc bit-width resolution (12bit,10bit...) 
 */
uint8_t ADC_get_resolution_bits(ADC_HandleTypeDef *adcHandle);

/**
 * @brief  Get the ADC total number of voltage levels (range of resolution)
 * @param  adcHandle @ref ADC_HandleTypeDef
 * @return ADC voltage levels (2^resolution_bits) 
 */
uint32_t ADC_get_tot_voltage_levels(ADC_HandleTypeDef *adcHandle);

/**
 * @brief  Get the value in mV expressed by the output of value from the ADC
 * @param  adcHandle @ref ADC_HandleTypeDef
 * @param  value_from_adc output value from the adc
 * @return Value in mV expressed by the adc
 */
float ADC_get_value_mV(ADC_HandleTypeDef *adcHandle, uint32_t value_from_adc);

/**
 * @brief Start a reading of all ADC values in DMA mode.
 *        This functions is a wrapper HAL_ADC_Start_DMA(...)it, starts the ADC
 *        of all sensors in DMA mode 
 * @note  The necessity of this function was due to confine the array of ADC
 *        values into the ADC module. Proper getters will be used to acces the
 *        values retirived from the adc
 *         Since we did not configure the ADC in circular mode we need to restart
 *        the conversions each time Thus this function must be recalled in the
 *        code if we want new values (by SFW timer or inside the interrupt of a
 *        HW timer)
 */
void ADC_start_DMA_readings();

/**
 * @brief current transducer value getter (the value is the average over @ref HO_50S_SP33_SAMPLES_FOR_AVERAGE samples)
 * @return current transducer value
 */
uint16_t ADC_get_HO_50S_SP33_1106_sensor_val();

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
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

