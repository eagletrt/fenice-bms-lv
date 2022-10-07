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

#define primary_NETWORK_IMPLEMENTATION

#include "can_comm.h"

#include "../can-lib/lib/primary/c/network.h"

/* CAN LIB */
#include "../current_transducer/current_transducer.h"
#include "adc.h"
#include "dac_pump.h"
#include "error.h"
#include "fenice-config.h"
#include "inverters.h"
#include "main.h"
#include "radiator.h"
#include "thermocouple.h"
#include "volt.h"

CAN_TxHeaderTypeDef tx_header;
CAN_RxHeaderTypeDef rx_header;

primary_message_SET_RADIATOR_SPEED_conversion rads_speed_msg;
primary_message_SET_PUMPS_SPEED_conversion pumps_speed_msg;

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
            error_set(ERROR_CAN, 0, HAL_GetTick());
        } else if (hcan->Instance == CANS.Instance) {
            error_set(ERROR_CAN, 1, HAL_GetTick());
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

HAL_StatusTypeDef can_primary_send(uint16_t id) {
    uint8_t buffer[CAN_MAX_PAYLOAD_LENGTH] = {0, 1, 0, 1, 0, 1, 0, 1};

    tx_header.StdId = id;
    tx_header.DLC   = 8;

    if (id == primary_ID_LV_VERSION) {
        primary_serialize_LV_VERSION(buffer, 1, CANLIB_BUILD_TIME);
        tx_header.DLC = primary_SIZE_LV_VERSION;
    } else if (id == primary_ID_LV_VOLTAGE) {
        primary_message_LV_VOLTAGE raw_volts;
        raw_volts.voltage_1 = voltages[0];
        raw_volts.voltage_2 = voltages[1];
        raw_volts.voltage_3 = voltages[2];
        raw_volts.voltage_4 = voltages[3];
        primary_serialize_struct_LV_VOLTAGE(buffer, &raw_volts);
        tx_header.DLC = primary_SIZE_LV_VOLTAGE;
    } else if (id == primary_ID_LV_TOTAL_VOLTAGE) {
        primary_message_LV_TOTAL_VOLTAGE raw_total_volt;
        primary_message_LV_TOTAL_VOLTAGE_conversion conv_total_volts;
        conv_total_volts.total_voltage = total_voltage_on_board;  //*100
        primary_conversion_to_raw_struct_LV_TOTAL_VOLTAGE(&raw_total_volt, &conv_total_volts);
        primary_serialize_struct_LV_TOTAL_VOLTAGE(buffer, &raw_total_volt);
        // primary_serialize_LV_TOTAL_VOLTAGE(buffer, (uint16_t) (total_voltage_on_board * 100));
        tx_header.DLC = primary_SIZE_LV_TOTAL_VOLTAGE;
    } else if (id == primary_ID_LV_CURRENT) {
        primary_message_LV_CURRENT raw_msg;
        primary_message_LV_CURRENT_conversion conv_msg;
        conv_msg.current = CT_get_electric_current_mA() / 1000.0;
        primary_conversion_to_raw_struct_LV_CURRENT(&raw_msg, &conv_msg);
        primary_serialize_struct_LV_CURRENT(buffer, &raw_msg);
        tx_header.DLC = primary_SIZE_LV_CURRENT;
    } else if (id == primary_ID_LV_TEMPERATURE) {
        primary_message_LV_TEMPERATURE raw_temps;
        primary_message_LV_TEMPERATURE_conversion conv_temps;
        conv_temps.bp_temperature_1   = THC_get_average_temperature_C(&hTHC_BATT1);
        conv_temps.bp_temperature_2   = THC_get_average_temperature_C(&hTHC_BATT2);
        conv_temps.dcdc12_temperature = THC_get_average_temperature_C(&hTHC_DCDC12V);
        conv_temps.dcdc24_temperature = THC_get_average_temperature_C(&hTHC_DCDC24V);
        primary_conversion_to_raw_struct_LV_TEMPERATURE(&raw_temps, &conv_temps);
        primary_serialize_struct_LV_TEMPERATURE(buffer, &raw_temps);
        //primary_serialize_LV_TEMPERATURE(buffer, ADC_get_t_batt1_val(), ADC_get_t_dcdc12_val());
        tx_header.DLC = primary_SIZE_LV_TEMPERATURE;
    } else if (id == primary_ID_COOLING_STATUS) {
        primary_message_COOLING_STATUS raw_cooling;
        primary_message_COOLING_STATUS_conversion conv_cooling;
        conv_cooling.radiators_speed = radiator_handle.duty_cycle_l;
        conv_cooling.pumps_speed = hdac_pump.last_analog_value_L > 0 ? hdac_pump.last_analog_value_L / MAX_DAC_OUT : 0;
        primary_conversion_to_raw_struct_COOLING_STATUS(&raw_cooling, &conv_cooling);
        primary_serialize_struct_COOLING_STATUS(buffer, &raw_cooling);
        tx_header.DLC = primary_SIZE_COOLING_STATUS;
    } else if (id == primary_ID_INVERTER_CONNECTION_STATUS) {
        primary_serialize_INVERTER_CONNECTION_STATUS(buffer, car_inverters.comm_status);
    } else if (id == primary_ID_LV_ERRORS) {
        primary_LvErrors lv_warnings = 0;
        primary_LvErrors lv_errors   = 0;
        error_t error_array[100];
        error_dump(error_array);

        for (uint8_t i = 0; i < error_count(); i++) {
            if (error_array[i].state == STATE_WARNING) {
                CANLIB_BITSET(lv_warnings, error_array[i].id);
            } else if (error_array[i].state == STATE_FATAL) {
                CANLIB_BITSET(lv_errors, error_array[i].id);
            }
        }
        tx_header.DLC = primary_serialize_LV_ERRORS(buffer, lv_warnings, lv_errors);
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

    if (rx_header.StdId == primary_ID_SET_RADIATOR_SPEED) {
        primary_message_SET_RADIATOR_SPEED raw;
        primary_deserialize_SET_RADIATOR_SPEED(&raw, rx_data);
        primary_raw_to_conversion_struct_SET_RADIATOR_SPEED(&rads_speed_msg, &raw);
        if (rads_speed_msg.radiators_speed >= 0) {
            radiator_handle.automatic_mode = false;
        } else if (rads_speed_msg.radiators_speed < 0) {
            radiator_handle.automatic_mode = true;
        }
    } else if (rx_header.StdId == primary_ID_SET_PUMPS_SPEED) {
        primary_message_SET_PUMPS_SPEED raw;
        primary_deserialize_SET_PUMPS_SPEED(&raw, rx_data);
        primary_raw_to_conversion_struct_SET_PUMPS_SPEED(&pumps_speed_msg, &raw);
        if (pumps_speed_msg.pumps_speed >= 0) {
            hdac_pump.automatic_mode = false;
        } else if (pumps_speed_msg.pumps_speed < 0) {
            hdac_pump.automatic_mode = true;
        }
    } else if (rx_header.StdId == primary_ID_SET_INVERTER_CONNECTION_STATUS) {
        primary_message_SET_INVERTER_CONNECTION_STATUS inverter_msg;
        primary_deserialize_SET_INVERTER_CONNECTION_STATUS(&inverter_msg, rx_data);
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