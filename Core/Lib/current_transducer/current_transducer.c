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
#define HO_50_SP33_1106_THEORETICAL_SENSITIVITY (9.2f)

/**
 * @brief Voltage reference of the current tranducer
 * This value should not be static because the LEM HO 50S has an internal reference
 * calculated from the VDD. The internal reference can be read from the 4th pin (Vref)
 * and used to adjust the reading, but at the current time this output is not wired
 * in to a MCU pin. 
 * Therefore we assume the optimal value given Vdd = 3.3V
 * (PROJECT_FOLDER/Doc/ho-s_sp33-1106_series.pdf page 4)
 */
#define HO_50_SP33_1106_VREF_mV (1650U)

/**
 * @brief OverCurrentDetection multiplier value:
 *  OCD is on when the current flowing the current transducer is
 *  OCD_MULT*Ipn  (Ipn = primary nominal current)
 * 
 */
#define HO_50_SP33_1106_OCD_MULT (2.92f)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static bool isOvercurrent = false;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief calculate the electric current in mA from the raw value got by ADC
 * 
 * @param adc_raw_value ADC output
 * @return electric current in mA
 */
static float __calculate_current_mA(uint32_t adc_raw_value);

/* Exported functions --------------------------------------------------------*/

float CT_get_electric_current_mA() {
    uint32_t raw_value  = ADC_get_HO_50S_SP33_1106_sensor_val();
    float current_in_mA = __calculate_current_mA(raw_value);
    //float current_in_mA = CT_get_average_electric_current(128)
    isOvercurrent = (current_in_mA > CT_OVERCURRENT_THRESHOLD_MA);
    return current_in_mA;
}

static float __calculate_current_mA(uint32_t adc_raw_value) {
    float adc_val_mV = ADC_get_value_mV(&CURRENT_TRANSDUCER_HADC, adc_raw_value);

    // current [mA] = ((Vadc-Vref)[mV] / Sensibility [mV/A])*1000

    float current = ((adc_val_mV - HO_50_SP33_1106_VREF_mV) / HO_50_SP33_1106_THEORETICAL_SENSITIVITY) * 1000;
    return current;
}

bool CT_is_overcurrent() {
    return isOvercurrent;
};

void CT_OCD_callback(bool isOn) {
    if (isOn && !isOvercurrent) {
        // Do something here like toggle the LVMS or trigger a CAN MSG send
        isOvercurrent = true;
    } else if (isOn && isOvercurrent) {
        // Already in Overcurrent
    } else if (!isOn && isOvercurrent) {
        // Overcurrent ended
        isOvercurrent = false;
    } else if (!isOn && !isOvercurrent) {
        // impossible state: Nothing to do
    }
}