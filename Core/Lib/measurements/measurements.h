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
#define OPEN_WIRE_MEASURE_INTERVAL_MS                   50
#define VOLT_MEASURE_INTERVAL_MS                        100
#define CURRENT_AND_INVERTER_STATUS_MEASURE_INTERVAL_MS 500
#define TEMPERATURE_MEASURE_INTERVAL_MS                 100
#define COOLING_STATUS_INTERVAL_MS                      1000
#define LV_VERSION_INTERVAL_MS                          1000

/* MEASUREMENT BITSET */
enum {
    MEAS_OPEN_WIRE                              = 1,
    MEAS_VOLTS_AND_TEMPS_READ_FLAG              = 2,
    MEAS_COOLING_AND_LV_VERSION_READ_FLAG       = 4,
    MEAS_CURRENT_AND_INVERTERS_STATUS_READ_FLAG = 8
};

void measurements_init(TIM_HandleTypeDef *htim);
void measurements_flags_check();
void measurements_oc_handler(TIM_HandleTypeDef *htim);