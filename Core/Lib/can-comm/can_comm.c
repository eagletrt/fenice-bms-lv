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
#define primary_IMPLEMENTATION


#include "../can-lib/lib/primary/c/ids.h"
#include "../can-lib/lib/primary/c/network.h"


#include "../current_transducer/current_transducer.h"
#include "adc.h"  // TODO: remove this eventually
#include "dac_pump.h"
#include "error.h"
#include "fenice-config.h"
#include "radiator.h"
#include "volt.h"
#include "main.h"

CAN_TxHeaderTypeDef tx_header;
CAN_RxHeaderTypeDef rx_header;

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
    tx_header.DLC = 8;

    if (id == primary_id_LV_VERSION) {
        primary_serialize_LV_VERSION(buffer, 1,1);
        tx_header.DLC = primary_LV_VERSION_SIZE;
    } else if (id == primary_id_LV_VOLTAGE) {
        primary_message_LV_VOLTAGE raw_volts;
        raw_volts.voltage_1 = voltages[0];
        raw_volts.voltage_2 = voltages[1];
        raw_volts.voltage_3 = voltages[2];
        raw_volts.voltage_4 = voltages[3];
        primary_serialize_struct_LV_VOLTAGE(buffer, &raw_volts);
        tx_header.DLC = primary_LV_VOLTAGE_SIZE;
    } else if (id == primary_id_LV_TOTAL_VOLTAGE) {
        primary_message_LV_TOTAL_VOLTAGE raw_total_volt;
        primary_message_LV_TOTAL_VOLTAGE_conversion conv_total_volts;
        conv_total_volts.total_voltage = total_voltage_on_board; //*100
        primary_conversion_to_raw_struct_LV_TOTAL_VOLTAGE(&raw_total_volt, &conv_total_volts);
        primary_serialize_struct_LV_TOTAL_VOLTAGE(buffer, &raw_total_volt);
       // primary_serialize_LV_TOTAL_VOLTAGE(buffer, (uint16_t) (total_voltage_on_board * 100));
        tx_header.DLC = primary_LV_TOTAL_VOLTAGE_SIZE;
    } else if (id == primary_id_LV_CURRENT) {
        primary_message_LV_CURRENT raw_msg;
        primary_message_LV_CURRENT_conversion conv_msg;
        conv_msg.current = CT_get_electric_current_mA();
        primary_conversion_to_raw_struct_LV_CURRENT(&raw_msg, &conv_msg);
        primary_serialize_struct_LV_CURRENT(buffer, &raw_msg);
        tx_header.DLC = primary_LV_CURRENT_SIZE;
    } else if (id == primary_id_LV_TEMPERATURE) {
        primary_message_LV_TEMPERATURE raw_temps;
        primary_message_LV_TEMPERATURE_conversion conv_temps;
        //TODO: hcnage temps into float
        conv_temps.bp_temperature_1 = ADC_get_t_batt1_val();
        conv_temps.bp_temperature_2 = ADC_get_t_batt2_val();
        conv_temps.dcdc12_temperature = ADC_get_t_dcdc12_val();
        conv_temps.dcdc24_temperature = ADC_get_t_dcdc24_val();
        primary_conversion_to_raw_struct_LV_TEMPERATURE(&raw_temps, &conv_temps);
        primary_serialize_struct_LV_TEMPERATURE(buffer, &raw_temps);
        //primary_serialize_LV_TEMPERATURE(buffer, ADC_get_t_batt1_val(), ADC_get_t_dcdc12_val());
        tx_header.DLC = primary_LV_TEMPERATURE_SIZE;
    } else if (id == primary_id_COOLING_STATUS) {
        primary_message_COOLING_STATUS raw_cooling;
        primary_message_COOLING_STATUS_conversion conv_cooling;
        conv_cooling.hv_fan_speed = bms_fan_duty_cycle * 100; //TODO: hv fan is inteded as it is?
        conv_cooling.lv_fan_speed = radiator_handle.duty_cycle_l*100;
        conv_cooling.pump_speed = hdac_pump.last_analog_value_L > 0 ?  hdac_pump.last_analog_value_L / MAX_DAC_OUT * 100 : 0;
        primary_conversion_to_raw_struct_COOLING_STATUS(&raw_cooling, &conv_cooling);
        primary_serialize_struct_COOLING_STATUS(buffer, &raw_cooling);
        // uint8_t volatile y = (uint8_t)(((float) hdac_pump.last_analog_value_L / MAX_DAC_OUT * 100));
        // // TODO: add duty cycle right as message
        // primary_serialize_COOLING_STATUS(
        //     buffer,
        //     (uint8_t)(radiator_handle.duty_cycle_l * 100),
        //     (uint8_t)(fan_duty_cycle * 100),
        //     (uint8_t)(hdac_pump.last_analog_value_L / MAX_DAC_OUT * 100));
        tx_header.DLC = primary_COOLING_STATUS_SIZE;
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
    
    if(rx_header.StdId == primary_id_SET_RADIATOR_SPEED){
        if(rx_data[0] == primary_Cooling_RADIATORS_MAX){
            radiator_handle.automatic_mode = false;
        }else if(rx_data[0] == primary_Cooling_RADIATORS_OFF){
            radiator_handle.automatic_mode = true;
        }
    }else if(rx_header.StdId == primary_id_SET_PUMPS_POWER){
        if(rx_data[0] == primary_Cooling_PUMPS_MAX){
            hdac_pump.automatic_mode = false;
        }else if(rx_data[0] == primary_Cooling_PUMPS_OFF){
            hdac_pump.automatic_mode = true;
        }
    }
}
// CAN Secondary Network rx interrupt callback
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //volatile uint8_t x = 0;
    uint8_t rx_data[8] = {'\0'};
    error_toggle_check(HAL_CAN_GetRxMessage(&CANS, CAN_RX_FIFO1, &rx_header, rx_data), ERROR_CAN, 1);
    //uint8_t z = 0;
}