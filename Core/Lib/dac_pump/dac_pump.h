/**
 * @file dac_pump.c
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-05-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __DAC_PUMP_H
#define __DAC_PUMP_H

#include "dac.h"
#include "main.h"
#include "usart.h"

#define MAX_DAC_OUT       4096.0f
#define MAX_GPIO_OUT      3.3
#define AMPLIFICATOR_GAIN 1.5
#define MAX_OPAMP_OUT     MAX_GPIO_OUT *AMPLIFICATOR_GAIN
#define MIN_OPAMP_OUT     0.0

/**
 * @brief Note that with BMS LV rev 1.1
 * both pumps are controller by PUMP_L_CHANNEL
 * 
 */

/**
 * @brief Pump handle used with DAC pheripheral
 * 
 */
typedef struct {
    uint32_t last_analog_value_L;
    uint8_t is_L_on;
    uint32_t last_analog_value_R;
    uint8_t is_R_on;
    bool automatic_mode;
} DAC_Pump_Handle;

extern DAC_Pump_Handle hdac_pump;

/**
 * @brief Set analog output value based on digital input value, 
 * note that the output value corresponds to the amplifier output, so the AMPLIFICATOR GAIN parameter 
 * is considered
 * 
 * analog = digital * max dac out / max opamp out -> analog* maxopamp out / max dac out
 * @param val Desired digital value, not above MAX_OPAMP_OUT
 * @return uint32_t analog value
 */
inline uint32_t dac_pump_digital_volt_to_analog(float val) {
    return (val <= MAX_OPAMP_OUT) ? (uint32_t)((val * MAX_DAC_OUT) / (MAX_OPAMP_OUT)) : (uint32_t)((MAX_DAC_OUT));
}

inline float dac_pump_analog_to_digital(uint32_t analog) {
    return (float)analog * MAX_OPAMP_OUT / MAX_DAC_OUT;
}

/**
 * @brief Convert digital to analog value based on a given proportional parameter
 * 
 * @param percentage Escursion percentage of MAX_OPAMP_OUT [min: 0.0, max: 1.0]
 * @return uint32_t Analog value needed to DAC Handle
 * @example dac_pump_dt_to_analog(0.5) -> 0.5 * MAX_OPAMP_OUT
 */
inline uint32_t dac_pump_proportional_to_analog(float percentage) {
    if (percentage <= 1) {
        return dac_pump_digital_volt_to_analog(percentage * MAX_OPAMP_OUT);
    } else {
        return dac_pump_digital_volt_to_analog(MAX_OPAMP_OUT);  // if duty cycle it's not
    }
}
// store: stores value into the struct
// set:   set output value using DAC pheripheral
void dac_pump_handle_init(DAC_Pump_Handle *hdp, float pump_l_volt, float pump_r_volt);
uint8_t dac_pump_set_value_on_both_channels(DAC_Pump_Handle *hdp);
uint8_t dac_pump_store_and_set_value_on_both_channels(DAC_Pump_Handle *hdp, float pump_l_volt, float pump_r_volt);
uint8_t dac_pump_store_and_set_value_on_single_channel(DAC_Pump_Handle *hdp, uint32_t channel, float digital_value);
uint8_t dac_pump_store_and_set_proportional_on_single_channel(
    DAC_Pump_Handle *hdp,
    uint32_t channel,
    float proportional);
uint8_t dac_pump_store_and_set_proportional_on_both_channels(
    DAC_Pump_Handle *hdp,
    float proportional_l,
    float proportional_r);
uint8_t dac_pump_break_single(DAC_Pump_Handle *hdp, uint32_t channel);
uint8_t dac_pump_break_both(DAC_Pump_Handle *hdp);
void dac_pump_sample_test(DAC_Pump_Handle *hdp);
float dac_pump_get_voltage(float temp);
#endif