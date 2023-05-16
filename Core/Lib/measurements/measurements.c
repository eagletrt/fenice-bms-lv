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
#include "../can-lib/lib/primary/c/watchdog.h"
#include "adc.h"
#include "can_comm.h"
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
    __HAL_TIM_SetCompare(
        &OPEN_WIRE_MEASUREMENT_TIMER,
        OPEN_WIRE_TIMER_CHANNEL,
        TIM_MS_TO_TICKS(&OPEN_WIRE_MEASUREMENT_TIMER, MEAS_OPEN_WIRE_INTERVAL_MS));

    __HAL_TIM_SetCompare(
        htim, MEAS_ALL_ANALOG_SIGNALS_TIMER_CHANNEL, TIM_MS_TO_TICKS(htim, MEAS_ALL_ANALOG_SIGNALS_INTERVAL_MS));

    __HAL_TIM_SetCompare(
        htim, MEAS_LV_VERSION_AND_COOLING_TIMER_CHANNEL, TIM_MS_TO_TICKS(htim, MEAS_VOLTS_AND_TEMPS_INTERVAL_MS));

    __HAL_TIM_SetCompare(
        htim,
        MEAS_LV_VERSION_AND_COOLING_TIMER_CHANNEL,
        TIM_MS_TO_TICKS(htim, MEAS_LV_VERSION_AND_COOLING_INTERVAL_MS));

    __HAL_TIM_CLEAR_IT(&OPEN_WIRE_MEASUREMENT_TIMER, TIM_IT_CC2);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC3);

    HAL_TIM_OC_Start_IT(htim, MEAS_ALL_ANALOG_SIGNALS_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(htim, MEAS_VOLTS_AND_TEMPS_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(htim, MEAS_LV_VERSION_AND_COOLING_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(&OPEN_WIRE_MEASUREMENT_TIMER, OPEN_WIRE_TIMER_CHANNEL);

    flags = 0;
}

//TODO implement overcurrent check for the 3 hall sensors
void check_overcurrent() {
    /*if (CT_get_electric_current_mA() > MAX_CURRENT_ALLOWED_mA) {
        error_set(ERROR_OVER_CURRENT, 0, HAL_GetTick());
    } else {
        if (!is_bms_on_fault) {
            error_reset(ERROR_OVER_CURRENT, 0);
        }
    }*/
}

void measurements_flags_check() {
    if (flags & MEAS_OPEN_WIRE_FLAG) {
        // Kinda ugly solution i know :(
        // From here
        if (!is_bms_on_fault) {
// #ifdef MEAS_DEBUG
//             cli_bms_debug("OPEN WIRE", 9);
// #endif
#ifdef LTC_ON_BOARD
            volt_start_open_wire_check(open_wire_check_status);
            open_wire_check_status += 1;                  // 1
            volt_start_open_wire_check(open_wire_check_status);
            open_wire_check_status += 1;                  // 2
            volt_read_open_wire(open_wire_check_status);  // read pup
            open_wire_check_status += 1;                  // 3
            volt_start_open_wire_check(open_wire_check_status);
            open_wire_check_status += 1;                  // 4
            volt_start_open_wire_check(open_wire_check_status);
            volt_read_open_wire(open_wire_check_status);  // read pud
            volt_open_wire_check();
            open_wire_check_status += (open_wire_check_status + 1) % 5;
#else
            // TODO: Send high level open wire check
#endif
        }
        flags &= ~MEAS_OPEN_WIRE_FLAG;
        // to here about 5/7 ms
    }
    if (flags & MEAS_ALL_ANALOG_SIGNALS_FLAG) {
        //uint16_t raw = ADC_get_batt_fb_raw();
        // convert

        relay_out_conversion();
        lvms_out_conversion();
        as_computer_fb_conversion();
        mux_fb_conversion();
        mux_hall_conversion();
        batt_out_conversion();
        // cansend
        flags &= ~MEAS_ALL_ANALOG_SIGNALS_FLAG;
    }

    if (flags & MEAS_VOLTS_AND_TEMPS_FLAG) {
        flags &= ~MEAS_VOLTS_AND_TEMPS_FLAG;
    }

    if (flags & MEAS_LV_VERSION_AND_COOLING_FLAG) {
        flags &= ~MEAS_LV_VERSION_AND_COOLING_FLAG;
    }
}

void measurements_oc_handler(TIM_HandleTypeDef *htim) {
    uint32_t counter = __HAL_TIM_GetCounter(htim);
    if (htim->Instance == OPEN_WIRE_MEASUREMENT_TIMER.Instance) {
        switch (htim->Channel) {
            case OPEN_WIRE_TIMER_ACTIVE_CHANNEL:
                __HAL_TIM_SetCompare(
                    htim, OPEN_WIRE_TIMER_CHANNEL, counter + TIM_MS_TO_TICKS(htim, MEAS_OPEN_WIRE_INTERVAL_MS));
                flags |= MEAS_OPEN_WIRE_FLAG;
                break;
            default:
                break;
        }
    } else if (htim->Instance == MEASUREMENTS_TIMER.Instance) {
        switch (htim->Channel) {
            case MEAS_ALL_ANALOG_SIGNALS_TIMER_ACTIVE_CHANNEL:
                __HAL_TIM_SetCompare(
                    htim,
                    MEAS_ALL_ANALOG_SIGNALS_TIMER_CHANNEL,
                    counter + TIM_MS_TO_TICKS(htim, MEAS_ALL_ANALOG_SIGNALS_INTERVAL_MS));
                flags |= MEAS_ALL_ANALOG_SIGNALS_FLAG;
                break;

            case MEAS_VOLTS_AND_TEMPS_TIMER_ACTIVE_CHANNEL:
                __HAL_TIM_SetCompare(
                    htim,
                    MEAS_VOLTS_AND_TEMPS_TIMER_CHANNEL,
                    counter + TIM_MS_TO_TICKS(htim, MEAS_VOLTS_AND_TEMPS_INTERVAL_MS));
                flags |= MEAS_VOLTS_AND_TEMPS_FLAG;
                break;

            case MEAS_LV_VERSION_AND_COOLING_TIMER_ACTIVE_CHANNEL:
                __HAL_TIM_SetCompare(
                    htim,
                    MEAS_LV_VERSION_AND_COOLING_TIMER_CHANNEL,
                    counter + TIM_MS_TO_TICKS(htim, MEAS_LV_VERSION_AND_COOLING_INTERVAL_MS));
                flags |= MEAS_LV_VERSION_AND_COOLING_FLAG;
                break;

            default:
                break;
        }
    }
}
