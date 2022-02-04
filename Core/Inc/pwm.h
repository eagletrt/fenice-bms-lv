#ifndef _PWM_H_
#define _PWM_H_

#include "main.h"

#include <inttypes.h>

typedef struct pwm_struct {
    uint32_t value;

    uint32_t prev_value;

    TIM_HandleTypeDef *timer;
    uint32_t channel;

} pwm_struct;

void Init_pwm(pwm_struct *pwm, TIM_HandleTypeDef *timer, uint32_t channel);
void write_pwm_value(pwm_struct *pwm);

#endif