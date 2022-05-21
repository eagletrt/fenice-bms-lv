/**
 * @file      current_transducer.c
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      Fri May 20 05:35:10 PM CEST 2022
 * 
 * @prefix    CT
 * 
 * @brief     Current Transducer Api
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "current_transducer.h"

#include "adc.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/** 
  * @brief Theoretical sensitivity expressed in mV/A
  * Sensitivity: If in the transducer flows 1A of current then the output
  * voltage vould be Vout=Vref+(1A*Sensitivity)
  */
#define HO_50S_SP33_THEORETICAL_SENSITIVITY (9.2f)  //
//
/**
 * @brief Voltage reference of the current tranducer
 * This value should not be static because the LEM HO 50S has an internal reference
 * calculated from the VDD. The internal reference can be read from the 4th pin (Vref)
 * and used to adjust the reading, but at the current time this output is not wired
 * in to a MCU pin. 
 * Therefore we assume the optimal value given Vdd = 3.3V
 * (PROJECT_FOLDER/Doc/ho-s_sp33-1106_series.pdf page 4)
 */
#define HO_50S_SP33_VREF_mV (1650U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static float electric_current_history[CT_HISTORY_LENGTH];
static uint32_t current_idx_in_array = 0;
static bool isOvercurrent            = false;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief Push an elctric current value into the history
 * 
 * @param value electric current
 */
void __push_into_history(float value);

/**
 * @brief calculate the electric current in mA from the raw value got by ADC
 * 
 * @param adc_raw_value ADC output
 * @return electric current in mA
 */
float __calculate_current_mA(uint32_t adc_raw_value);

/* Exported functions --------------------------------------------------------*/

float CT_get_electric_current_mA() {
    uint32_t raw_value  = ADC_get_HO_50S_SP33_sensor_val();
    float current_in_mA = __calculate_current_mA(raw_value);
    isOvercurrent       = (current_in_mA > CT_OVERCURRENT_THRESHOLD_MA);
    __push_into_history(current_in_mA);
    return current_in_mA;
}

void __push_into_history(float value) {
    current_idx_in_array                           = (current_idx_in_array + 1) % CT_HISTORY_LENGTH;
    electric_current_history[current_idx_in_array] = value;
}

float __calculate_current_mA(uint32_t adc_raw_value) {
    float adc_val_mV = ADC_get_value_mV(&CURRENT_TRANSDUCER_HADC, adc_raw_value);

    // current [mA] = ((Vadc-Vref)[mV] / Sensibility [mV/A])*1000

    float current = ((adc_val_mV - HO_50S_SP33_VREF_mV) / HO_50S_SP33_THEORETICAL_SENSITIVITY) * 1000;
    return current;

    //ADC_voltages = (uint32_t)((adcVAL * (Vcc * 1024 / RESOLUTION)) / 1024);
    //if (ADC_voltages >= REFERENCE_VOLTAGE)
    //    ADC_voltages = ADC_voltages - REFERENCE_VOLTAGE;
    //else
    //    ADC_voltages = REFERENCE_VOLTAGE - ADC_voltages;
    //return ADC_voltages = (uint32_t)(100 * ADC_voltages);
}

float CT_get_average_electric_current(uint8_t numer_of_samples) {
    // check out of range on numer_of_samples
    numer_of_samples = numer_of_samples > CT_HISTORY_LENGTH ? CT_HISTORY_LENGTH : numer_of_samples;
    numer_of_samples = numer_of_samples < 1 ? 1 : numer_of_samples;

    float accumulator = 0;
    uint8_t idx       = (((CT_HISTORY_LENGTH + current_idx_in_array) - numer_of_samples) % CT_HISTORY_LENGTH) + 1;
    for (int i = 0; i < numer_of_samples; i++) {
        accumulator += electric_current_history[idx];
        idx = (idx + 1) % CT_HISTORY_LENGTH;
    }
    return accumulator / numer_of_samples;
}

bool CT_is_overcurrent() {
    return isOvercurrent;
};

/* Private functions ---------------------------------------------------------*/
