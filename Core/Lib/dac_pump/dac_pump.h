#ifndef __DAC_PUMP_H
#define __DAC_PUMP_H

#include "main.h"
#include "dac.h"
#include "usart.h"

#define MAX_DAC_OUT 4096
#define MAX_GPIO_OUT 3.3
#define AMPLIFICATOR_GAIN 1.5
#define MAX_OPAMP_OUT MAX_GPIO_OUT * AMPLIFICATOR_GAIN

/**
 * @brief Pump handle used with DAC pheripheral
 * 
 */
typedef struct{
    uint32_t last_analog_value_L;
    uint8_t  is_L_on;
    uint32_t last_analog_value_R;
    uint8_t  is_R_on;
} DAC_Pump_Handle;

extern DAC_Pump_Handle hdac_pump;

/**
 * @brief Set analog output value based on digital input value, 
 * note that the output value corresponds to the amplifier output, so the AMPLIFICATOR GAIN parameter 
 * is considered
 * 
 * 
 * @param val Desired digital value
 * @return uint32_t analog value
 */
inline uint32_t dac_pump_digital_volt_to_analog (float val){
    return (uint32_t)((val * MAX_DAC_OUT) / (MAX_OPAMP_OUT));
}

/**
 * @brief Convert digital to analog value based on a given duty cyle
 * 
 * @param dt Escursion percentage of MAX_OPAMP_OUT
 * @return uint32_t Analog value needed to DAC Handle
 * @example dac_pump_dt_to_analog(0.5) -> 0.5 * MAX_OPAMP_OUT
 */
inline uint32_t dac_pump_dt_to_analog(float dt){
    if( dt <= 1){
        return dac_pump_digital_volt_to_analog(dt * MAX_OPAMP_OUT);
    }else{
        return dac_pump_digital_volt_to_analog(MAX_OPAMP_OUT); // if duty cycle it's not
    }
}
// store: stores value into the struct
// set:   set output value using DAC pheripheral  
void dac_pump_handle_init(DAC_Pump_Handle *hdp, float pump_l_volt, float pump_r_volt);
uint8_t dac_pump_set_value_on_both_channels(DAC_Pump_Handle *hdp);
uint8_t dac_pump_store_and_set_value_on_both_channels(DAC_Pump_Handle *hdp, float pump_l_volt, float pump_r_volt);
uint8_t dac_pump_set_value_on_single_channel(DAC_Pump_Handle *hdp, uint32_t channel, float digital_value);
uint8_t dac_pump_break_single(DAC_Pump_Handle *hdp, uint32_t channel);
uint8_t dac_pump_break_both(DAC_Pump_Handle *hdp);
void dac_pump_sample_test(DAC_Pump_Handle *hdp);
#endif