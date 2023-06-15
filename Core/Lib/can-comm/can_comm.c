/**
 * @file can_comm.c
 * @author Canova Tommaso (tommaso.canova@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-05-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "can_comm.h"

/* CAN LIB */
#include "../current_transducer/current_transducer.h"
#include "adc.h"
#include "dac_pump.h"
#include "error.h"
#include "fenice-config.h"
#include "inverters.h"
#include "main.h"
#include "monitor_int.h"
#include "radiator.h"
#include "thermocouple.h"

#define CAN_COMM_LV_VERSION 0x01

CAN_TxHeaderTypeDef tx_header;
CAN_RxHeaderTypeDef rx_header;

primary_set_radiator_speed_converted_t rads_speed_msg;
primary_set_pumps_speed_converted_t pumps_speed_msg;

void can_tx_header_init() {
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
}

void can_primary_init() {
    CAN_FilterTypeDef filter;
    filter.FilterMode       = CAN_FILTERMODE_IDMASK;
    filter.FilterIdLow      = 0 << 5;                 // Take all ids from 0
    filter.FilterIdHigh     = ((1U << 11) - 1) << 5;  // to 2^11 - 1
    filter.FilterMaskIdHigh = 0 << 5;                 // Don't care on can id bits
    filter.FilterMaskIdLow  = 0 << 5;                 // Don't care on can id bits
    /* HAL considers IdLow and IdHigh not as just the ID of the can message but
        as the combination of: 
        STDID + RTR + IDE + 4 most significant bits of EXTID
    */
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank           = 0;
    filter.FilterScale          = CAN_FILTERSCALE_16BIT;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = CAN_SLAVE_START_FILTER_BANK;

    HAL_CAN_ConfigFilter(&CANP, &filter);
    HAL_CAN_ActivateNotification(&CANP, CAN_IT_ERROR | CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&CANP);

    can_tx_header_init();
}
void can_secondary_init() {
    CAN_FilterTypeDef filter;
    filter.FilterMode       = CAN_FILTERMODE_IDMASK;
    filter.FilterIdLow      = 0 << 5;                 // Take all ids from 0
    filter.FilterIdHigh     = ((1U << 11) - 1) << 5;  // to 2^11 - 1
    filter.FilterMaskIdHigh = 0 << 5;                 // Don't care on can id bits
    filter.FilterMaskIdLow  = 0 << 5;                 // Don't care on can id bits
    /* HAL considers IdLow and IdHigh not as just the ID of the can message but
        as the combination of: 
        STDID + RTR + IDE + 4 most significant bits of EXTID
    */
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    filter.FilterBank           = 14;
    filter.FilterScale          = CAN_FILTERSCALE_16BIT;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = CAN_SLAVE_START_FILTER_BANK;

    HAL_CAN_ConfigFilter(&CANS, &filter);
    HAL_CAN_ActivateNotification(&CANS, CAN_IT_ERROR | CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&CANS);

    can_tx_header_init();
}

HAL_StatusTypeDef can_send(CAN_HandleTypeDef *hcan, uint8_t *buffer, CAN_TxHeaderTypeDef *header) {
    CAN_WAIT(hcan);

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, header, buffer, NULL);
    if (status != HAL_OK) {
        if (hcan->Instance == CANP.Instance) {
            error_set(ERROR_CAN, 0);
        } else if (hcan->Instance == CANS.Instance) {
            error_set(ERROR_CAN, 1);
        }
    } else {
        if (hcan->Instance == CANP.Instance) {
            error_reset(ERROR_CAN, 0);
        } else if (hcan->Instance == CANS.Instance) {
            error_reset(ERROR_CAN, 1);
        }
    }

    return status;
}

HAL_StatusTypeDef can_primary_send(uint16_t id, uint8_t optional_offset) {
    uint8_t buffer[CAN_MAX_PAYLOAD_LENGTH] = {0, 1, 0, 1, 0, 1, 0, 1};

    tx_header.StdId = id;
    tx_header.DLC   = 8;

    if (id == PRIMARY_LV_VERSION_FRAME_ID) {
        primary_lv_version_t version;
        version.component_version = CAN_COMM_LV_VERSION;
        version.canlib_build_time = CANLIB_BUILD_TIME;
        primary_lv_version_pack(buffer, &version, PRIMARY_LV_VERSION_BYTE_SIZE);
        tx_header.DLC = PRIMARY_LV_VERSION_BYTE_SIZE;
    } else if (id == PRIMARY_LV_CELLS_VOLTAGE_FRAME_ID) {
        primary_lv_cells_voltage_t raw_volts;
        raw_volts.start_index = optional_offset;
        raw_volts.voltage_0   = voltages[0 + optional_offset];
        raw_volts.voltage_1   = voltages[1 + optional_offset];
        raw_volts.voltage_2   = voltages[2 + optional_offset];
        primary_lv_cells_voltage_pack(buffer, &raw_volts, PRIMARY_LV_CELLS_VOLTAGE_BYTE_SIZE);
        tx_header.DLC = PRIMARY_LV_CELLS_VOLTAGE_BYTE_SIZE;
    } else if (id == PRIMARY_LV_TOTAL_VOLTAGE_FRAME_ID) {
        primary_lv_total_voltage_t raw_total_volt;
        primary_lv_total_voltage_converted_t conv_total_volts;
        conv_total_volts.total_voltage = total_voltage_on_board;  //*100
        primary_lv_total_voltage_conversion_to_raw_struct(&raw_total_volt, &conv_total_volts);
        primary_lv_total_voltage_pack(buffer, &raw_total_volt, PRIMARY_LV_TOTAL_VOLTAGE_BYTE_SIZE);
        // primary_serialize_LV_TOTAL_VOLTAGE(buffer, (uint16_t) (total_voltage_on_board * 100));
        tx_header.DLC = PRIMARY_LV_TOTAL_VOLTAGE_BYTE_SIZE;
    } else if (id == PRIMARY_LV_CURRENTS_FRAME_ID) {
        primary_lv_currents_t raw_msg;
        primary_lv_currents_converted_t conv_msg;
        //conv_msg.current = CT_get_electric_current_mA() / 1000.0;
        primary_lv_currents_conversion_to_raw_struct(&raw_msg, &conv_msg);
        primary_lv_currents_pack(buffer, &raw_msg, PRIMARY_LV_CURRENTS_BYTE_SIZE);
        tx_header.DLC = PRIMARY_LV_CURRENTS_BYTE_SIZE;
    } else if (id == PRIMARY_LV_CELLS_TEMP_FRAME_ID) {
        primary_lv_cells_temp_t raw_temps;
        primary_lv_cells_temp_converted_t conv_temps;
        raw_temps.start_index = optional_offset;
        raw_temps.temp0       = cell_temps[0 + optional_offset];
        raw_temps.temp1       = cell_temps[1 + optional_offset];
        raw_temps.temp2       = cell_temps[2 + optional_offset];
        primary_lv_cells_temp_conversion_to_raw_struct(&raw_temps, &conv_temps);
        primary_lv_cells_temp_pack(buffer, &raw_temps, PRIMARY_LV_CELLS_TEMP_BYTE_SIZE);
        //primary_serialize_LV_TEMPERATURE(buffer, ADC_get_t_batt1_val(), ADC_get_t_dcdc12_val());
        tx_header.DLC = PRIMARY_LV_CELLS_TEMP_BYTE_SIZE;
    } else if (id == PRIMARY_COOLING_STATUS_FRAME_ID) {
        primary_cooling_status_t raw_cooling;
        primary_cooling_status_converted_t conv_cooling;
        conv_cooling.radiators_speed = radiator_handle.duty_cycle_l;
        conv_cooling.pumps_speed = hdac_pump.last_analog_value_L > 0 ? hdac_pump.last_analog_value_L / MAX_DAC_OUT : 0;
        primary_cooling_status_conversion_to_raw_struct(&raw_cooling, &conv_cooling);
        primary_cooling_status_pack(buffer, &raw_cooling, PRIMARY_COOLING_STATUS_BYTE_SIZE);
        tx_header.DLC = PRIMARY_COOLING_STATUS_BYTE_SIZE;
    } else if (id == PRIMARY_INVERTER_CONNECTION_STATUS_FRAME_ID) {
        primary_inverter_connection_status_t car_inverters_conn;
        car_inverters_conn.status = car_inverters.comm_status;
        primary_inverter_connection_status_pack(
            buffer, &car_inverters_conn, PRIMARY_INVERTER_CONNECTION_STATUS_BYTE_SIZE);
    } else if (id == PRIMARY_LV_ERRORS_FRAME_ID) {
        // primary_LvErrors lv_warnings = 0;
        // primary_LvErrors lv_errors   = 0;
        // size_t error_count           = 0;
        // error_t error_array[ERROR_NUM_ERRORS];
        // error_dump(error_array, &error_count);

        // for (uint8_t i = 0; i < error_count; i++) {
        //     if (error_array[i].state == STATE_WARNING) {
        //         CANLIB_BITSET(lv_warnings, error_array[i].id);
        //     } else if (error_array[i].state == STATE_FATAL) {
        //         CANLIB_BITSET(lv_errors, error_array[i].id);
        //     }
        // }
        //tx_header.DLC = primary_serialize_LV_ERRORS(buffer, lv_warnings, lv_errors);
    }

    return can_send(&CANP, buffer, &tx_header);
}

HAL_StatusTypeDef can_secondary_send(uint16_t id) {
    uint8_t buffer[CAN_MAX_PAYLOAD_LENGTH] = {1, 2, 3, 4, 5, 6, 7, 8};
    tx_header.StdId                        = id;
    tx_header.DLC                          = 8;

    return can_send(&CANS, buffer, &tx_header);
}

// CAN Primary Network rx interrupt callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //volatile uint8_t x = 0;
    uint8_t rx_data[8] = {'\0'};
    error_toggle_check(HAL_CAN_GetRxMessage(&CANP, CAN_RX_FIFO0, &rx_header, rx_data), ERROR_CAN, 0);

    if (rx_header.StdId == PRIMARY_SET_RADIATOR_SPEED_FRAME_ID) {
        primary_set_radiator_speed_t raw;
        primary_set_radiator_speed_unpack(&raw, rx_data, PRIMARY_SET_RADIATOR_SPEED_BYTE_SIZE);
        primary_set_radiator_speed_raw_to_conversion_struct(&rads_speed_msg, &raw);
        if (rads_speed_msg.radiators_speed >= 0) {
            radiator_handle.automatic_mode = false;
        } else if (rads_speed_msg.radiators_speed < 0) {
            radiator_handle.automatic_mode = true;
        }
    } else if (rx_header.StdId == PRIMARY_SET_PUMPS_SPEED_FRAME_ID) {
        primary_set_pumps_speed_t raw;
        primary_set_pumps_speed_unpack(&raw, rx_data, PRIMARY_SET_PUMPS_SPEED_BYTE_SIZE);
        primary_set_pumps_speed_raw_to_conversion_struct(&pumps_speed_msg, &raw);
        if (pumps_speed_msg.pumps_speed >= 0) {
            hdac_pump.automatic_mode = false;
        } else if (pumps_speed_msg.pumps_speed < 0) {
            hdac_pump.automatic_mode = true;
        }
    } else if (rx_header.StdId == PRIMARY_SET_INVERTER_CONNECTION_STATUS_FRAME_ID) {
        primary_set_inverter_connection_status_t inverter_msg;
        primary_set_inverter_connection_status_unpack(
            &inverter_msg, rx_data, PRIMARY_SET_INVERTER_CONNECTION_STATUS_BYTE_SIZE);
        set_inverter_status(&car_inverters, inverter_msg.status);
    }
}
// CAN Secondary Network rx interrupt callback
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //volatile uint8_t x = 0;
    uint8_t rx_data[8] = {'\0'};
    error_toggle_check(HAL_CAN_GetRxMessage(&CANS, CAN_RX_FIFO1, &rx_header, rx_data), ERROR_CAN, 1);
    //uint8_t z = 0;
}