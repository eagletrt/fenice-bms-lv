#ifndef __CURRENT_SENSOR_H__
#define __CURRENT_SENSOR_H__

#include "stm32f7xx_hal.h"

#define REFERENCE_VOLTAGE 1650 //mV
#define VOLTAGE_OFFSET 1150 //mV
#define MAX_mA 50000 //mA
#define OFFSET REFERENCE_VOLTAGE - VOLTAGE_OFFSET
#define RESOLUTION 4096 //2^12 bit
#define Vcc 3300 //mV

uint32_t getCurrent(uint32_t adcVAL);
uint32_t mean_current();
void push_into_array(uint32_t val);

#endif
