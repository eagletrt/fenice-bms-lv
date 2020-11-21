#include "pwm.h"

void write_pwm_value(pwm_struct *pwm)
{
    HAL_TIM_PWM_Stop(pwm->timer, pwm->channel);

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pwm->value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(pwm->timer, &sConfigOC, pwm->channel);
    HAL_TIM_PWM_Start(pwm->timer, pwm->channel);
}

void Init_pwm(pwm_struct *pwm, TIM_HandleTypeDef *timer, uint32_t channel)
{
    pwm->timer = timer;
    pwm->channel = channel;
    pwm->value = 0;
    pwm->prev_value = 0;

    HAL_TIM_PWM_Start(pwm->timer, pwm->channel);
}