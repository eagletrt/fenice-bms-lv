/**
 * @file      thermocouple.h
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      Sun May 29 08:26:38 PM CEST 2022
 * 
 * @prefix    THC
 * 
 * @brief     Thermocouple Api for TI LM35D
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __THERMOCOUPLE_H__
#define __THERMOCOUPLE_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

/**
 * @brief Length of the history of current data points
 */
#define THC_HISTORY_LENGTH 40

/**
 * @brief overtemperature threshold in milliCelsius
 */
#define THC_OVERTEMPERATURE_THRESHOLD_MC (60000U)

/* Exported types ------------------------------------------------------------*/
typedef struct __THC_Handle THC_Handle_TD;

/* Exported macros -----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Get temperature reading in Celsius of the termocouple associated to hTHC
 * 
 * @param hTHC Thermocouple handle pointer
 * @return float temperature read by the thermocouple in Celsius
 */
float THC_get_temperature_C(THC_Handle_TD *hTHC);
float THC_get_average_temperature_C(THC_Handle_TD *hTHC);
/**
 * @brief Get average temperature reading in Celsius of the termocouple
 *        associated to hTHC over the last @param number_of_samples
 * 
 * @param hTHC Thermocouple handle pointer
 * @param number_of_samples the number of samples over wich the average is calculated
 * @return float average temperature read by the thermocouple in Celsius
 */
float THC_get_average_simia_temperature_C(THC_Handle_TD *hTHC, uint8_t number_of_samples);

/* Exported variables --------------------------------------------------------*/

/**
 * @brief Handle to the 12V-DCDC thermocouple sensor 
 * 
 */
extern THC_Handle_TD hTHC_DCDC12V;

/**
 * @brief Handle to the 24V-DCDC thermocouple sensor 
 * 
 */
extern THC_Handle_TD hTHC_DCDC24V;

/**
 * @brief Handle to the first Battery thermocouple sensor 
 * 
 */
extern THC_Handle_TD hTHC_BATT1;

/**
 * @brief Handle to the second Battery thermocouple sensor 
 * 
 */
extern THC_Handle_TD hTHC_BATT2;

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros -----------------------------------------------------------*/
#endif
