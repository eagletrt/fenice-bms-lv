/**
 * @file      thermocouple.c
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      Sun May 29 08:26:38 PM CEST 2022
 * 
 * @prefix    THC
 * 
 * @brief     Thermocouple Api for TI LM35D
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "thermocouple.h"

#include "adc.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
struct __THC_Handle_TD {
    ADC_HandleTypeDef *hadc;
    uint16_t history[THC_HISTORY_LENGTH];
    uint16_t (*ADC_raw_value_getter)(void);
};

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
THC_Handle_TD hTHC_DCDC12V = {
    .hadc                 = &T_SENS_DCDC12V_HADC,
    .history              = {},
    .ADC_raw_value_getter = &ADC_get_t_dcdc12_val};
THC_Handle_TD hTHC_DCDC24V = {
    .hadc                 = &T_SENS_DCDC24V_HADC,
    .history              = {},
    .ADC_raw_value_getter = &ADC_get_t_dcdc24_val};
THC_Handle_TD hTHC_BATT1 = {.hadc = &T_SENS_BATT1_HADC, .history = {}, .ADC_raw_value_getter = &ADC_get_t_batt1_val};
THC_Handle_TD hTHC_BATT2 = {.hadc = &T_SENS_BATT2_HADC, .history = {}, .ADC_raw_value_getter = &ADC_get_t_batt2_val};
/* Exported functions --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
