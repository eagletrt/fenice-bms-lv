/* GOD BLESS ALEX SARTORI*/
#ifndef __NOTES_BUZZER_H__
#define __NOTES_BUZZER_H__

#include "main.h"

#define PWM_FANS_STANDARD_PERIOD 0.03846153846  //26kHz
void fans_init();
void fans_set_speed(float power_percentage);
void BUZ_sborati(TIM_HandleTypeDef *htim);

#endif