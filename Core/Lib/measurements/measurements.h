/**
 * @file measurements.c
 * @author Federico Carbone, Tommaso Canova
 * @brief 
 * @version 0.1
 * @date 2022-05-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "tim.h"

/* MEASUREMENTS INTERVALS */
#define VOLT_MEASURE_INTERVAL_MS 200
#define CURRENT_MEASURE_INTERVAL_MS 500
#define TEMPERATURE_MEASURE_INTERVAL_MS 200
#define COOLING_STATUS_INTERVAL_MS 1000
#define LV_VERSION_INTERVAL_MS 1000

/* MEASUREMENT BITSET */
enum {
    MEAS_VOLTS_AND_TEMPS_READ_FLAG = 1,
    MEAS_COOLING_AND_LV_VERSION_READ_FLAG = 2,
    MEAS_CURRENT_READ_FLAG = 4
};

void measurements_init(TIM_HandleTypeDef *htim);
void measurements_flags_check();
void measurements_oc_handler(TIM_HandleTypeDef *htim);