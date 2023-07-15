/**
 * @file radiator.h
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-05-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __RADIATORS_H
#define __RADIATORS_H

#include "main.h"

/**
 * @brief Fans in fail safe configuration works with a negative duty cycle, 
 * time_on it's based on low output value instead of tipical high output value
 * 
 * @param dt 
 * @return float 
 */
inline float duty_cycle_to_fan_duty_cycle(float dt) {
    return (dt <= 1.0) ? 1.0 - dt : 1.0;
}

typedef struct {
    float duty_cycle_l;
    float duty_cycle_r;
    uint8_t left_is_on;
    uint8_t right_is_on;
    bool automatic_mode;
    bool update_value;
    // Wheter automatic mode is false tue radiators will be controlled by the steer,
    // otherwise the radiators will be under the bms_lv_control as are designed to be
} radiator_t;

extern radiator_t radiator_handle;

void radiator_init();
void start_radiator(TIM_HandleTypeDef *rad_tim, uint8_t channel);
void stop_radiator(TIM_HandleTypeDef *rad_tim, uint8_t channel);
void start_both_radiator(TIM_HandleTypeDef *rad_tim, uint8_t channel1, uint8_t channel2);
void set_radiator_dt(TIM_HandleTypeDef *rad_tim, uint8_t channel, float duty_cycle);
float get_radiator_dt(float temp);

#endif