/**
 * @file measurements.c
 * @author Federico Carbone, Tommaso Canova
 * @brief Measurements and can send messages handler
 * @version 0.1
 * @date 2022-05-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "measurements.h"

#include "../can-lib/lib/primary/c/ids.h"
#include "can_comm.h"
#include "cli_bms_lv.h"
#include "current_transducer.h"
#include "error.h"
#include "fenice-config.h"
#include "main.h"
#include "thermocouple.h"
#include "timer_utils.h"
#include "volt.h"

#ifdef MEAS_DEBUG
#include "usart.h"
#endif

uint8_t volatile flags;
uint8_t open_wire_check_status = 0;

void measurements_init(TIM_HandleTypeDef *htim) {
    __HAL_TIM_SetCompare(htim, COOLING_AND_LV_VERSION_TIMER_CHANNEL, TIM_MS_TO_TICKS(htim, COOLING_STATUS_INTERVAL_MS));
    __HAL_TIM_SetCompare(
        htim, CURRENT_TIMER_CHANNEL, TIM_MS_TO_TICKS(htim, CURRENT_AND_INVERTER_STATUS_MEASURE_INTERVAL_MS));
    __HAL_TIM_SetCompare(htim, VOLTAGE_AND_TEMPS_TIMER_CHANNEL, TIM_MS_TO_TICKS(htim, VOLT_MEASURE_INTERVAL_MS));
    __HAL_TIM_SetCompare(
        &OPEN_WIRE_MEASUREMENT_TIMER,
        OPEN_WIRE_TIMER_CHANNEL,
        TIM_MS_TO_TICKS(&OPEN_WIRE_MEASUREMENT_TIMER, OPEN_WIRE_MEASURE_INTERVAL_MS));

    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC3);
    __HAL_TIM_CLEAR_IT(&OPEN_WIRE_MEASUREMENT_TIMER, TIM_IT_CC2);

    HAL_TIM_OC_Start_IT(htim, COOLING_AND_LV_VERSION_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(htim, CURRENT_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(htim, VOLTAGE_AND_TEMPS_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(&OPEN_WIRE_MEASUREMENT_TIMER, OPEN_WIRE_TIMER_CHANNEL);

    flags = 0;
}

void check_overcurrent() {
    if (CT_get_electric_current_mA() > MAX_CURRENT_ALLOWED) {
        error_set(ERROR_OVER_CURRENT, 0, HAL_GetTick());
    } else {
        error_reset(ERROR_OVER_CURRENT, 0);
    }
}

void check_battery_temperatures() {
    if (THC_get_temperature_C(&hTHC_BATT1) > MAX_CELLS_ALLOWED_TEMP ||
        THC_get_temperature_C(&hTHC_BATT2) > MAX_CELLS_ALLOWED_TEMP) {
        error_set(ERROR_CELL_OVER_TEMPERATURE, 0, HAL_GetTick());
    } else if (
        THC_get_temperature_C(&hTHC_BATT1) < MIN_CELLS_ALLOWED_TEMP ||
        THC_get_temperature_C(&hTHC_BATT2) < MIN_CELLS_ALLOWED_TEMP) {
        error_set(ERROR_CELL_UNDER_TEMPERATURE, 0, HAL_GetTick());
    } else {
        error_reset(ERROR_CELL_OVER_TEMPERATURE, 0);
        error_reset(ERROR_CELL_UNDER_TEMPERATURE, 0);
    }
}

void check_dcdc_12_24_temperatures() {
    if (THC_get_temperature_C(&hTHC_DCDC12V) > MAX_DCDC12_ALLOWED_TEMP) {
        error_set(ERROR_DCDC12_OVER_TEMPERATURE, 0, HAL_GetTick());
    } else if (THC_get_temperature_C(&hTHC_DCDC12V) < MIN_DCDC12_ALLOWED_TEMP) {
        error_set(ERROR_DCDC12_UNDER_TEMPERATURE, 0, HAL_GetTick());
    } else {
        error_reset(ERROR_DCDC12_OVER_TEMPERATURE, 0);
        error_reset(ERROR_DCDC12_UNDER_TEMPERATURE, 0);
    }
    if (THC_get_temperature_C(&hTHC_DCDC24V) > MAX_DCDC24_ALLOWED_TEMP) {
        error_set(ERROR_DCDC24_OVER_TEMPERATURE, 0, HAL_GetTick());
    } else if (THC_get_temperature_C(&hTHC_DCDC24V) < MIN_DCDC24_ALLOWED_TEMP) {
        error_set(ERROR_DCDC24_UNDER_TEMPERATURE, 0, HAL_GetTick());
    } else {
        error_reset(ERROR_DCDC24_OVER_TEMPERATURE, 0);
        error_reset(ERROR_DCDC24_UNDER_TEMPERATURE, 0);
    }
}

void measurements_flags_check() {
    if (flags & MEAS_OPEN_WIRE) {
        // Kinda ugly solution i know :(
        // From here
        if (!is_bms_on_fault) {
#ifdef MEAS_DEBUG
            cli_bms_debug("OPEN WIRE", 9);
#endif
            volt_start_open_wire_check(open_wire_check_status);
            open_wire_check_status += 1;  // 1
            volt_start_open_wire_check(open_wire_check_status);
            open_wire_check_status += 1;                  // 2
            volt_read_open_wire(open_wire_check_status);  // read pup
            open_wire_check_status += 1;                  // 3
            volt_start_open_wire_check(open_wire_check_status);
            open_wire_check_status += 1;  // 4
            volt_start_open_wire_check(open_wire_check_status);
            volt_read_open_wire(open_wire_check_status);  // read pud
            volt_open_wire_check();
            open_wire_check_status += (open_wire_check_status + 1) % 5;
        }
        flags &= ~MEAS_OPEN_WIRE;
        // to here about 5/7 ms
    }
    if (flags & MEAS_VOLTS_AND_TEMPS_READ_FLAG) {
        if (volt_sample_and_read() != VOLT_ERR) {
            can_primary_send(primary_ID_LV_VOLTAGE);
            can_primary_send(primary_ID_LV_TOTAL_VOLTAGE);
        }
        can_primary_send(primary_ID_LV_TEMPERATURE);
#ifdef MEAS_DEBUG
        cli_bms_debug("VOLTS + TEMPS", 13);
#endif

        flags &= ~MEAS_VOLTS_AND_TEMPS_READ_FLAG;
    }
    if (flags & MEAS_COOLING_AND_LV_VERSION_READ_FLAG) {
        check_battery_temperatures();
        check_dcdc_12_24_temperatures();
        can_primary_send(primary_ID_COOLING_STATUS);
        can_primary_send(primary_ID_LV_VERSION);
        check_on_feedbacks();
#ifdef MEAS_DEBUG
        cli_bms_debug("COOLING + LV VERSION", 20);
#endif
        flags &= ~MEAS_COOLING_AND_LV_VERSION_READ_FLAG;
    }
    if (flags & MEAS_CURRENT_AND_INVERTERS_STATUS_READ_FLAG) {
        check_overcurrent();
        can_primary_send(primary_ID_LV_CURRENT);
        can_primary_send(primary_ID_INVERTER_CONNECTION_STATUS);
#ifdef MEAS_DEBUG
        cli_bms_debug("CURRENT", 7);
#endif
        flags &= ~MEAS_CURRENT_AND_INVERTERS_STATUS_READ_FLAG;
    }
}

void measurements_oc_handler(TIM_HandleTypeDef *htim) {
    uint32_t counter = __HAL_TIM_GetCounter(htim);
    if (htim->Instance == OPEN_WIRE_MEASUREMENT_TIMER.Instance) {
        switch (htim->Channel) {
            case OPEN_WIRE_TIMER_ACTIVE_CHANNEL:
                __HAL_TIM_SetCompare(
                    htim, OPEN_WIRE_TIMER_CHANNEL, counter + TIM_MS_TO_TICKS(htim, OPEN_WIRE_MEASURE_INTERVAL_MS));
                flags |= MEAS_OPEN_WIRE;
                break;
            default:
                break;
        }
    } else {
        switch (htim->Channel) {
            case COOLING_AND_LV_VERSION_TIMER_ACTIVE_CHANNEL:
                __HAL_TIM_SetCompare(
                    htim,
                    COOLING_AND_LV_VERSION_TIMER_CHANNEL,
                    counter + TIM_MS_TO_TICKS(htim, COOLING_STATUS_INTERVAL_MS));
                flags |= MEAS_COOLING_AND_LV_VERSION_READ_FLAG;
                break;
            case CURRENT_TIMER_ACTIVE_CHANNEL:
                __HAL_TIM_SetCompare(
                    htim,
                    CURRENT_TIMER_CHANNEL,
                    counter + TIM_MS_TO_TICKS(htim, CURRENT_AND_INVERTER_STATUS_MEASURE_INTERVAL_MS));
                flags |= MEAS_CURRENT_AND_INVERTERS_STATUS_READ_FLAG;
                break;
            case VOLTAGE_AND_TEMPS_TIMER_ACTIVE_CHANNEL:
                __HAL_TIM_SetCompare(
                    htim, VOLTAGE_AND_TEMPS_TIMER_CHANNEL, counter + TIM_MS_TO_TICKS(htim, VOLT_MEASURE_INTERVAL_MS));
                flags |= MEAS_VOLTS_AND_TEMPS_READ_FLAG;
                break;
            default:
                break;
        }
    }
}
