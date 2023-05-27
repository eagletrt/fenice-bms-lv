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
struct __THC_Handle {
    ADC_HandleTypeDef *hadc;
    uint16_t _history[THC_HISTORY_LENGTH];
    uint8_t _history_idx;
    uint16_t (*_ADC_raw_value_getter)(void);
    ADC_HandleTypeDef *_hadc;
    uint8_t _adc_channel;
};

/* Private define ------------------------------------------------------------*/

/** 
  * @brief Sensor Gain (average slope) expressed in mV/°C
  * Sensitivity: If in the thermocouple flows 1A of current then the output
  * voltage vould be Vout=(1000mA*Sensitivity)
  */
#define LM35D_SENSOR_GAIN (10.0f)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @param value electric current
 */

/**
 * @brief Push a temperature reading into the history
 * 
 * @param hTHC @ref struct __THC_Handle handle
 * @param value temperature reading in °C
 */
static void __push_into_history(struct __THC_Handle *hTHC, float value);

/**
 * @brief calculate the temperature in C from the raw value got by ADC
 * 
 * @param adc_raw_value ADC output
 * @param hTHC @ref struct __THC_Handle handle
 * @return temperature in C
 */
static float __calculate_temperature_C(struct __THC_Handle *hTHC, uint32_t adc_raw_value);

/* Exported variables --------------------------------------------------------*/
THC_Handle_TD hTHC_DCDC12V =
    {.hadc = NULL, ._history = {}, ._history_idx = 0, ._ADC_raw_value_getter = NULL, ._hadc = NULL, ._adc_channel = 0};
THC_Handle_TD hTHC_DCDC24V =
    {.hadc = NULL, ._history = {}, ._history_idx = 0, ._ADC_raw_value_getter = NULL, ._hadc = NULL, ._adc_channel = 0};
THC_Handle_TD hTHC_BATT1 =
    {.hadc = NULL, ._history = {}, ._history_idx = 0, ._ADC_raw_value_getter = NULL, ._hadc = NULL, ._adc_channel = 0};
THC_Handle_TD hTHC_BATT2 =
    {.hadc = NULL, ._history = {}, ._history_idx = 0, ._ADC_raw_value_getter = NULL, ._hadc = NULL, ._adc_channel = 0};

static void __push_into_history(struct __THC_Handle *hTHC, float value) {
    hTHC->_history_idx                 = (hTHC->_history_idx + 1) % THC_HISTORY_LENGTH;
    hTHC->_history[hTHC->_history_idx] = value;
}

static float __calculate_temperature_C(struct __THC_Handle *hTHC, uint32_t adc_raw_value) {
    float adc_val_mV = ADC_get_value_mV(hTHC->_hadc, adc_raw_value);

    // temperature [°C] = Vout[mV] / Sensor Gain [mV/°C]

    float current = adc_val_mV / LM35D_SENSOR_GAIN;
    return current;
}

float THC_get_temperature_C(THC_Handle_TD *hTHC) {
    uint32_t raw_value     = hTHC->_ADC_raw_value_getter();
    float temperature_in_C = __calculate_temperature_C(hTHC, raw_value);
    __push_into_history(hTHC, temperature_in_C);
    return temperature_in_C;
}

float THC_get_average_temperature_C(THC_Handle_TD *hTHC) {
    float accumulator = 0;
    for (int i = 0; i < THC_HISTORY_LENGTH; i++) {
        accumulator += hTHC->_history[i];
    }
    return accumulator / THC_HISTORY_LENGTH;
}

float THC_get_average_simia_temperature_C(THC_Handle_TD *hTHC, uint8_t number_of_samples) {
    // check out of range on numer_of_samples
    number_of_samples = number_of_samples > THC_HISTORY_LENGTH ? THC_HISTORY_LENGTH : number_of_samples;
    number_of_samples = number_of_samples < 1 ? 1 : number_of_samples;

    float accumulator = 0;
    uint8_t idx       = (((THC_HISTORY_LENGTH + hTHC->_history_idx) - number_of_samples) % THC_HISTORY_LENGTH) + 1;
    for (int i = 0; i < number_of_samples; i++) {
        accumulator += hTHC->_history[idx];
        idx = (idx + 1) % THC_HISTORY_LENGTH;
    }
    return accumulator / number_of_samples;
}
/* Exported functions --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
