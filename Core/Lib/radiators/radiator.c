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

#include "radiator.h"

#include "pwm.h"
#include "tim.h"

radiator_t radiator_handle;

void set_radiator_struct_channel_on(uint8_t channel) {
    if (channel == RAD_L_PWM_TIM_CHNL) {
        radiator_handle.left_is_on = 1;
    } else if (channel == RAD_R_PWM_TIM_CHNL) {
        radiator_handle.right_is_on = 1;
    }
}
void set_radiator_struct_channel_off(uint8_t channel) {
    if (channel == RAD_L_PWM_TIM_CHNL) {
        radiator_handle.left_is_on = 0;
    } else if (channel == RAD_R_PWM_TIM_CHNL) {
        radiator_handle.right_is_on = 0;
    }
}

void set_radiator_struct_dt(uint8_t channel, float dt) {
    if (channel == RAD_L_PWM_TIM_CHNL) {
        radiator_handle.duty_cycle_l = dt;
        (radiator_handle.duty_cycle_l == 0.0) ? set_radiator_struct_channel_off(channel)
                                              : set_radiator_struct_channel_on(channel);

    } else if (channel == RAD_R_PWM_TIM_CHNL) {
        radiator_handle.duty_cycle_r = dt;
        (radiator_handle.duty_cycle_r == 0.0) ? set_radiator_struct_channel_off(channel)
                                              : set_radiator_struct_channel_on(channel);
    }
}

/**
 * @brief Set radiator frequency to 25 kHz and then set pwm duty cycle to 1.0 = off
 * 
 */
void radiator_init() {
    pwm_set_period(&RAD_L_HTIM, 0.04);  //Set frequency to 25kHz
    pwm_set_duty_cicle(&RAD_L_HTIM, RAD_L_PWM_TIM_CHNL, 1.0);
    pwm_set_duty_cicle(&RAD_R_HTIM, RAD_R_PWM_TIM_CHNL, 1.0);
    start_both_radiator(&RAD_L_HTIM, RAD_L_PWM_TIM_CHNL, RAD_R_PWM_TIM_CHNL);
    radiator_handle.duty_cycle_l   = 0.0;
    radiator_handle.duty_cycle_r   = 0.0;
    radiator_handle.automatic_mode = true;
}

/**
 * @brief Start pwm on radiator channel
 * 
 * @param rad_tim TIM_HandleTypeDef
 * @param channel Radiator pwm channel
 */
void start_radiator(TIM_HandleTypeDef *rad_tim, uint8_t channel) {
    pwm_start_channel(rad_tim, channel);
    set_radiator_struct_channel_on(channel);
}

/**
 * @brief Stop pwm on radiator channel
 * 
 * @param rad_tim TIM_HandleTypeDef
 * @param channel Radiator pwm channel
 */
void stop_radiator(TIM_HandleTypeDef *rad_tim, uint8_t channel) {
    pwm_stop_channel(rad_tim, channel);
    set_radiator_struct_channel_off(channel);
}

/**
 * @brief Start pwm on both radiators
 * 
 * @param rad_tim TIM_HandleTypeDef
 * @param channel1 Radiator1 pwm channel
 * @param channel2 Radiator2 pwm channel
 */
void start_both_radiator(TIM_HandleTypeDef *rad_tim, uint8_t channel1, uint8_t channel2) {
    pwm_start_channel(rad_tim, channel1);
    pwm_start_channel(rad_tim, channel2);
    (radiator_handle.duty_cycle_l == 0.0) ? set_radiator_struct_channel_off(channel1)
                                          : set_radiator_struct_channel_on(channel1);
    (radiator_handle.duty_cycle_r == 0.0) ? set_radiator_struct_channel_off(channel2)
                                          : set_radiator_struct_channel_on(channel2);
}

/**
 * @brief Set the radiator dt object
 * 
 * @param rad_tim TIM_HandleTypeDef
 * @param channel Radiator pwm channel
 * @param duty_cycle Duty cycle expressed as float (eg: 0.5 equals 50%)
 */
void set_radiator_dt(TIM_HandleTypeDef *rad_tim, uint8_t channel, float duty_cycle) {
    pwm_set_duty_cicle(rad_tim, channel, duty_cycle_to_fan_duty_cycle(duty_cycle));
    set_radiator_struct_dt(channel, duty_cycle);
}
/**
 * @brief Get the a duty cycle for the radiators based on a given motor temperature
 * 
 * @param temp Motor temperature
 * @return float Optimal PWM Duty Cycle 
 */
float get_radiator_dt(float temp) {
    return (MAX_FAN_DUTY_CYCLE - MIN_FAN_DUTY_CYCLE) / (MAX_MOTOR_TEMP - MIN_MOTOR_TEMP) * (temp - MIN_MOTOR_TEMP);
}