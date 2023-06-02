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

#define MEAS_OPEN_WIRE_INTERVAL_MS 50
/* @brief Used to convert current sensors (HALL MUX) and feedbacks (FB MUX)
AS_RELAY, LVMS_OUT, RELAY_OUT and BATT_OUT*/
#define MEAS_ALL_ANALOG_SIGNALS_INTERVAL_MS     300
#define MEAS_VOLTS_AND_TEMPS_INTERVAL_MS        200
#define MEAS_LV_VERSION_AND_COOLING_INTERVAL_MS 1000
#define MEAS_CELL_TEMPS_INTERVAL_MS             5

/* MEASUREMENT BITSET */
enum {
    MEAS_OPEN_WIRE_FLAG              = 1,
    MEAS_ALL_ANALOG_SIGNALS_FLAG     = 2,
    MEAS_VOLTS_AND_TEMPS_FLAG        = 4,
    MEAS_LV_VERSION_AND_COOLING_FLAG = 8,
    MEAS_CELL_TEMPS_FLAG             = 16
};

void measurements_init(TIM_HandleTypeDef *htim);
void measurements_flags_check();
void measurements_oc_handler(TIM_HandleTypeDef *htim);