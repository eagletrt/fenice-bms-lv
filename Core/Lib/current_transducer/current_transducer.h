/**
 * @file      current_transducer.h
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      Fri May 20 05:35:10 PM CEST 2022
 * 
 * @prefix    CT
 * 
 * @brief     Current Transducer Api
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CURRENT_TRANSDUCER_H__
#define __CURRENT_TRANSDUCER_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/**
 * @brief Length of the history of electric current data points
 */
#define CT_HISTORY_LENGTH 50

/**
 * @brief overcurrent threshold in milliAmpere
 */
#define CT_OVERCURRENT_THRESHOLD_MA (50000U)

/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief Return if an overcurrent is detected
 * 
 * @return true 
 * @return false 
 */
bool CT_is_overcurrent();

/**
 * @brief  Get electric current value currently flowing in the Current Transducer
 * @return Current in mA flowing inside the transducer
 *         if 0xFFFF then error
 */
float CT_get_electric_current_mA();

/**
 * @brief Get the average current over the last number_of_samples 
 * @param number_of_samples the number of samples over wich the average is calculated
 * @return float average current
 */
float CT_get_average_electric_current(uint8_t number_of_samples);

/**
 * @brief Over-Current Detection callback (put this in the correct ISR)
 * 
 * @param isOn value of the OCD pin: true if rising edge false if falling edge
 *             of OCD pin
 */
void CT_OCD_callback(bool isOn);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros -----------------------------------------------------------*/
#endif
