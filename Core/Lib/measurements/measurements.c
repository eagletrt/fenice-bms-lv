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

#include "../can-lib/lib/primary/primary_network.h"
#include "adc.h"
#include "can_comm.h"
#include "current_transducer.h"
#include "error.h"
#include "fenice-config.h"
#include "health_signals.h"
#include "main.h"
#include "monitor_int.h"
#include "thermocouple.h"
#include "timer_utils.h"
#include "volt.h"

#ifdef MEAS_DEBUG
#include "usart.h"
#endif

uint8_t volatile flags;

void measurements_init(TIM_HandleTypeDef *htim) {
    __HAL_TIM_SetCompare(
        &OPEN_WIRE_MEASUREMENT_TIMER,
        OPEN_WIRE_TIMER_CHANNEL,
        TIM_MS_TO_TICKS(&OPEN_WIRE_MEASUREMENT_TIMER, MEAS_OPEN_WIRE_INTERVAL_MS));

    __HAL_TIM_SetCompare(
        htim, MEAS_ALL_ANALOG_SIGNALS_TIMER_CHANNEL, TIM_MS_TO_TICKS(htim, MEAS_ALL_ANALOG_SIGNALS_INTERVAL_MS));

    __HAL_TIM_SetCompare(
        htim, MEAS_VOLTS_AND_TEMPS_TIMER_CHANNEL, TIM_MS_TO_TICKS(htim, MEAS_VOLTS_AND_TEMPS_INTERVAL_MS));

    __HAL_TIM_SetCompare(
        htim,
        MEAS_LV_VERSION_AND_COOLING_TIMER_CHANNEL,
        TIM_MS_TO_TICKS(htim, MEAS_LV_VERSION_AND_COOLING_INTERVAL_MS));

    __HAL_TIM_SetCompare(htim, MEAS_CELL_TEMPS_TIMER_CHANNEL, TIM_MS_TO_TICKS(htim, MEAS_CELL_TEMPS_INTERVAL_MS));

    __HAL_TIM_CLEAR_IT(&OPEN_WIRE_MEASUREMENT_TIMER, TIM_IT_CC2);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC3);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC4);

    HAL_TIM_OC_Start_IT(htim, MEAS_ALL_ANALOG_SIGNALS_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(htim, MEAS_VOLTS_AND_TEMPS_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(htim, MEAS_LV_VERSION_AND_COOLING_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(htim, MEAS_CELL_TEMPS_TIMER_CHANNEL);
    HAL_TIM_OC_Start_IT(&OPEN_WIRE_MEASUREMENT_TIMER, OPEN_WIRE_TIMER_CHANNEL);

    flags = 0;
}

void check_overcurrent() {
    if (adcs_converted_values.mux_hall.S_HALL0 > MAX_CURRENT_ALLOWED_mA) {
        error_set(ERROR_OVER_CURRENT, 0);
    } else {
        if (!is_bms_on_fault) {
            error_reset(ERROR_OVER_CURRENT, 0);
        }
    }

    if (adcs_converted_values.mux_hall.S_HALL1 > MAX_CURRENT_ALLOWED_mA) {
        error_set(ERROR_OVER_CURRENT, 1);
    } else {
        if (!is_bms_on_fault) {
            error_reset(ERROR_OVER_CURRENT, 1);
        }
    }

    if (adcs_converted_values.mux_hall.S_HALL2 > MAX_CURRENT_ALLOWED_mA) {
        error_set(ERROR_OVER_CURRENT, 2);
    } else {
        if (!is_bms_on_fault) {
            error_reset(ERROR_OVER_CURRENT, 2);
        }
    }
}

void measurements_flags_check() {
    if (flags & MEAS_OPEN_WIRE_FLAG) {
        if (volt_open_wire(&monitor_handler, LTC6811_MD_7KHZ_3KHZ, LTC6811_DCP_DISABLED, 10) != 0) {
            error_set(ERROR_OPEN_WIRE, 0);
#ifdef NDEBUG_LTC
            printl("Wire successful", NO_HEADER);
#endif
        } else {
            error_reset(ERROR_OPEN_WIRE, 0);
#ifdef NDEBUG_LTC
            printl("Wire Error", NO_HEADER);
#endif
        }
        if (open_blt_status.is_flash_requested) {
            open_blt_status_update(&hs, &open_blt_status);
            if (open_blt_status.is_flash_available) {
                if (!open_blt_status.is_time_set_pin_on) {
                    // Charging the capacitor in order to keep the microcontroller alive after flashing
                    HAL_GPIO_WritePin(TIME_SET_GPIO_Port, TIME_SET_Pin, GPIO_PIN_SET);
                    open_blt_status.is_time_set_pin_on       = 1;
                    open_blt_status.time_set_initial_time_ms = HAL_GetTick();
                    open_blt_status.state = PRIMARY_LV_CAN_FLASH_ACK_RESPONSE_PREPARING_TO_FLASH_CHOICE;
                } else {
                    if (HAL_GetTick() - open_blt_status.time_set_initial_time_ms >
                        open_blt_status.time_set_timeout_ms) {
                        open_blt_status.charging_done = 1;
                        open_blt_status.state         = PRIMARY_LV_CAN_FLASH_ACK_RESPONSE_FLASH_CHOICE;
                    }
                }
            } else {
                open_blt_status.state = PRIMARY_LV_CAN_FLASH_ACK_RESPONSE_NO_FLASH_CHOICE;
            }
            // TODO: test if open wire still works
            can_primary_send(PRIMARY_LV_CAN_FLASH_ACK_FRAME_ID, 0);
        }
        flags &= ~MEAS_OPEN_WIRE_FLAG;
        // to here about 5/7 ms
    }

    if (flags & MEAS_CELL_TEMPS_FLAG) {
        monitor_read_temp();
        flags &= ~MEAS_CELL_TEMPS_FLAG;
    }

    if (flags & MEAS_ALL_ANALOG_SIGNALS_FLAG) {
        relay_out_conversion();
        lvms_out_conversion();
        as_computer_fb_conversion();
        mux_fb_conversion();
        mux_hall_conversion();
        check_overcurrent();
        batt_out_conversion();
        update_can_feedbacks();
        update_health_status(&hs, &adcs_converted_values);
        can_primary_send(PRIMARY_LV_CURRENTS_FRAME_ID, 0);
        can_primary_send(PRIMARY_LV_FEEDBACKS_FRAME_ID, 0);
        can_primary_send(PRIMARY_LV_ERRORS_FRAME_ID, 0);
        can_primary_send(PRIMARY_INVERTER_CONNECTION_STATUS_FRAME_ID, 0);
        //can_primary_send(PRIMARY_LV_HEALTH_SIGNALS_FRAME_ID, 0);
        // cansend
        flags &= ~MEAS_ALL_ANALOG_SIGNALS_FLAG;
    }

    if (flags & MEAS_VOLTS_AND_TEMPS_FLAG) {
        monitor_read_voltage();
        can_primary_send(PRIMARY_LV_CELLS_VOLTAGE_FRAME_ID, 0);
        can_primary_send(PRIMARY_LV_CELLS_VOLTAGE_FRAME_ID, 3);
        can_primary_send(PRIMARY_LV_TOTAL_VOLTAGE_FRAME_ID, 0);
        monitor_temp_conversion();
        can_primary_send(PRIMARY_LV_CELLS_TEMP_FRAME_ID, 0);
        can_primary_send(PRIMARY_LV_CELLS_TEMP_FRAME_ID, 1);
        flags &= ~MEAS_VOLTS_AND_TEMPS_FLAG;
    }

    if (flags & MEAS_LV_VERSION_AND_COOLING_FLAG) {
        can_primary_send(PRIMARY_LV_VERSION_FRAME_ID, 0);
        can_primary_send(PRIMARY_COOLING_STATUS_FRAME_ID, 0);
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

            case MEAS_CELL_TEMPS_TIMER_ACTIVE_CHANNEL:
                __HAL_TIM_SetCompare(
                    htim, MEAS_CELL_TEMPS_TIMER_CHANNEL, counter + TIM_MS_TO_TICKS(htim, MEAS_CELL_TEMPS_INTERVAL_MS));
                flags |= MEAS_CELL_TEMPS_FLAG;
                break;

            default:
                break;
        }
    }
}
