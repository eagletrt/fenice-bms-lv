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
#include "monitor_int.h"
#include "radiator.h"
#include "thermocouple.h"

#define CAN_COMM_LV_VERSION 0x01

CAN_TxHeaderTypeDef tx_header;
CAN_RxHeaderTypeDef rx_header;

primary_set_radiator_speed_converted_t rads_speed_msg;
primary_set_pumps_speed_converted_t pumps_speed_msg;

open_blt_status_t open_blt_status;

void open_blt_status_update(health_signals_t *hs, open_blt_status_t *obs) {
    if (obs->is_flash_requested) {
        if (hs->lvms_out == 1 && hs->charger_current == 1) {
            obs->is_flash_available = 0;
        } else {
            obs->is_flash_available = 1;
        }
    }
}

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

    open_blt_status.is_flash_requested              = 0;
    open_blt_status.is_flash_available              = 0;
    open_blt_status.is_time_set_pin_on              = 0;
    open_blt_status.is_time_set_pin_timeout_elapsed = 0;
    open_blt_status.time_set_initial_time_ms        = 0;
    open_blt_status.time_set_timeout_ms             = OPEN_BLT_TIME_SET_TIMEOUT_MS;
    open_blt_status.charging_done                   = 0;
    open_blt_status.state                           = PRIMARY_LV_CAN_FLASH_ACK_RESPONSE_NO_FLASH_CHOICE;
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
    } else if (id == PRIMARY_LV_ERRORS_FRAME_ID) {
        primary_lv_errors_t raw_errors;
        primary_lv_errors_conversion_to_raw_struct(&raw_errors, &primary_lv_errors);
        primary_lv_errors_pack(buffer, &raw_errors, PRIMARY_LV_ERRORS_BYTE_SIZE);
        tx_header.DLC = PRIMARY_LV_ERRORS_BYTE_SIZE;
    } else if (id == PRIMARY_LV_FEEDBACKS_FRAME_ID) {
        primary_lv_feedbacks_t raw_feedbacks;
        primary_lv_feedbacks_conversion_to_raw_struct(&raw_feedbacks, &primary_lv_feedbacks_converted);
        primary_lv_feedbacks_pack(buffer, &raw_feedbacks, PRIMARY_LV_FEEDBACKS_BYTE_SIZE);
        tx_header.DLC = PRIMARY_LV_FEEDBACKS_BYTE_SIZE;
    } else if (id == PRIMARY_LV_CELLS_VOLTAGE_FRAME_ID) {
        primary_lv_cells_voltage_t raw_volts;
        primary_lv_cells_voltage_converted_t conv_volts;
        conv_volts.start_index = optional_offset;

        conv_volts.voltage_0 = voltages[0 + optional_offset] / 1000;
        conv_volts.voltage_1 = voltages[1 + optional_offset] / 1000;
        conv_volts.voltage_2 = voltages[2 + optional_offset] / 1000;
        primary_lv_cells_voltage_conversion_to_raw_struct(&raw_volts, &conv_volts);
        primary_lv_cells_voltage_raw_to_conversion_struct(&conv_volts, &raw_volts);
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
    } else if (id == PRIMARY_LV_HEALTH_SIGNALS_FRAME_ID) {
        primary_lv_health_signals_t raw_health;
        raw_health.health_signals_battery_current      = hs.battery_current;
        raw_health.health_signals_battery_voltage_out  = hs.battery_voltage_out;
        raw_health.health_signals_charger_current      = hs.charger_current;
        raw_health.health_signals_lvms_out             = hs.lvms_out;
        raw_health.health_signals_relay_out            = hs.relay_out;
        raw_health.health_signals_sign_battery_current = hs.sign_battery_current;
        primary_lv_health_signals_pack(buffer, &raw_health, PRIMARY_LV_HEALTH_SIGNALS_BYTE_SIZE);
        tx_header.DLC = PRIMARY_LV_HEALTH_SIGNALS_BYTE_SIZE;
    } else if (id == PRIMARY_LV_CAN_FLASH_ACK_FRAME_ID) {
        primary_lv_can_flash_ack_t raw_ack;
        raw_ack.response = open_blt_status.state;
        primary_lv_can_flash_ack_pack(buffer, &raw_ack, PRIMARY_LV_CAN_FLASH_ACK_BYTE_SIZE);
        tx_header.DLC = PRIMARY_LV_CAN_FLASH_ACK_BYTE_SIZE;
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
    } else if (rx_header.StdId == PRIMARY_LV_CAN_FLASH_REQ_FRAME_ID) {
        open_blt_status.is_flash_requested = 1;
    } else if (rx_header.StdId == PRIMARY_BMS_LV_JMP_TO_BLT_FRAME_ID) {
        HAL_NVIC_SystemReset();
    }
}
// CAN Secondary Network rx interrupt callback
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //volatile uint8_t x = 0;
    uint8_t rx_data[8] = {'\0'};
    error_toggle_check(HAL_CAN_GetRxMessage(&CANS, CAN_RX_FIFO1, &rx_header, rx_data), ERROR_CAN, 1);
    //uint8_t z = 0;
}

void update_can_feedbacks() {
    // From ADC mux
    primary_lv_feedbacks_converted.feedbacks_bspd_fb = (adcs_converted_values.mux_fb.BSPD_FB > 1600.0f) ? 1 : 0;
    primary_lv_feedbacks_converted.feedbacks_imd_fb  = (adcs_converted_values.mux_fb.IMD_FB > 1600.0f) ? 1 : 0;
    primary_lv_feedbacks_converted.feedbacks_lvms_fb = (adcs_converted_values.mux_fb.LVMS_FB > 1600.0f) ? 1 : 0;
    primary_lv_feedbacks_converted.feedbacks_res_fb  = (adcs_converted_values.mux_fb.RES_FB > 1600.0f) ? 1 : 0;
    primary_lv_feedbacks_converted.feedbacks_lv_encl = (adcs_converted_values.mux_fb.LV_ENCL_1_FB > 1600.0f) ? 1 : 0;
    primary_lv_feedbacks_converted.feedbacks_hv_encl_1_fb  = (adcs_converted_values.mux_fb.HV_ENCL_1_FB > 1600.0f) ? 1
                                                                                                                   : 0;
    primary_lv_feedbacks_converted.feedbacks_hv_encl_2_fb  = (adcs_converted_values.mux_fb.HV_ENCL_2_FB > 1600.0f) ? 1
                                                                                                                   : 0;
    primary_lv_feedbacks_converted.feedbacks_back_plate_fb = (adcs_converted_values.mux_fb.BACK_PLATE_FB > 1600.0f) ? 1
                                                                                                                    : 0;
    primary_lv_feedbacks_converted.feedbacks_hvd_fb        = (adcs_converted_values.mux_fb.HVD_FB > 1600.0f) ? 1 : 0;
    primary_lv_feedbacks_converted.feedbacks_ams_fb        = (adcs_converted_values.mux_fb.AMS_FB > 1600.0f) ? 1 : 0;
    primary_lv_feedbacks_converted.feedbacks_asms_fb       = (adcs_converted_values.mux_fb.ASMS_FB > 1600.0f) ? 1 : 0;
    primary_lv_feedbacks_converted.feedbacks_interlock_fb =
        (adcs_converted_values.mux_fb.INTERLOCK_IMD_FB > 1600.0f) ? 1 : 0;
    primary_lv_feedbacks_converted.sd_start = adcs_converted_values.mux_fb.SD_START;
    primary_lv_feedbacks_converted.sd_end   = adcs_converted_values.mux_fb.SD_END;

    // From mcp23017
    primary_lv_feedbacks_converted.feedbacks_inverters_fb = mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_INVERTERS);
    primary_lv_feedbacks_converted.feedbacks_pcbs_fb      = mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_PCBS);
    primary_lv_feedbacks_converted.feedbacks_pumps_fb     = mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_PUMPS);
    primary_lv_feedbacks_converted.feedbacks_shutdown_fb  = mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_SHUTDOWN);
    primary_lv_feedbacks_converted.feedbacks_radiators_fb = mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_RADIATORS);
    primary_lv_feedbacks_converted.feedbacks_fan_fb       = mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_FAN);
    primary_lv_feedbacks_converted.feedbacks_as_actuation_fb =
        mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_AS_ACTUATION);
}