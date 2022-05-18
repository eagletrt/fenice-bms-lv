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

#include "../can-cicd/includes_generator/primary/ids.h"
#include "../can-cicd/naked_generator/primary/c/primary.h"

#include "dac_pump.h"
#include "fenice-config.h"
#include "radiator.h"
#include "volt.h"
#include "adc.h"

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
        //error_set(ERROR_CAN, 0, HAL_GetTick());

    } else {
        //error_reset(ERROR_CAN, 0);
    }

    return status;
}

HAL_StatusTypeDef can_primary_send(uint16_t id) {
    uint8_t buffer[CAN_MAX_PAYLOAD_LENGTH] = {0, 0, 0, 0, 0, 0, 0, 0};

    tx_header.StdId = id;
    tx_header.DLC   = 8;  //TODO: remove

    if (id == PRIMARY_ID_LV_VERSION) {
        serialize_PrimaryLvVersion(buffer, 1, 1);
        tx_header.DLC = PRIMARY_LV_VERSION_SIZE;
    } else if (id == PRIMARY_ID_LV_VOLTAGE) {
        serialize_PrimaryLvVoltage(buffer, voltages[0], voltages[1], voltages[2], voltages[3]);
        tx_header.DLC = PRIMARY_LV_VOLTAGE_SIZE;
    } else if(id == PRIMARY_ID_LV_TOTAL_VOLTAGE){
        serialize_PrimaryLvTotalVoltage(buffer, (uint16_t)total_voltage_on_board * 100);
        tx_header.DLC = PRIMARY_LV_TOTAL_VOLTAGE_SIZE;
    } else if (id == PRIMARY_ID_LV_CURRENT) {
        serialize_PrimaryLvCurrent(buffer, ADC_get_i_sensor_val());
        tx_header.DLC = PRIMARY_LV_CURRENT_SIZE;
    } else if (id == PRIMARY_ID_LV_TEMPERATURE) {
        //TODO: check which one battery and which dcdc
        serialize_PrimaryLvTemperature(buffer, ADC_get_t_batt1_val(), ADC_get_t_dcdc12_val());
        tx_header.DLC = PRIMARY_LV_TEMPERATURE_SIZE;
    } else if (id == PRIMARY_ID_COOLING_STATUS) {
        serialize_PrimaryCoolingStatus(
            buffer,
            (uint8_t)(radiator_handle.duty_cycle_l * 100),
            (uint8_t)(fan_duty_cycle * 100),
            (uint8_t)(hdac_pump.last_analog_value_L / MAX_DAC_OUT * 100));
        tx_header.DLC = PRIMARY_COOLING_STATUS_SIZE;
    }

    return can_send(&CANP, buffer, &tx_header);
}

HAL_StatusTypeDef can_secondary_send(uint16_t id) {
    uint8_t buffer[CAN_MAX_PAYLOAD_LENGTH] = {1, 2, 3, 4, 5, 6, 7, 8};
    tx_header.StdId                        = id;
    tx_header.DLC                          = 8;

    return can_send(&CANS, buffer, &tx_header);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //volatile uint8_t x = 0;
    uint8_t rx_data[8] = {'\0'};
    HAL_CAN_GetRxMessage(&CANP, CAN_RX_FIFO0, &rx_header, rx_data);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    //volatile uint8_t x = 0;
    uint8_t rx_data[8] = {'\0'};
    HAL_CAN_GetRxMessage(&CANS, CAN_RX_FIFO1, &rx_header, rx_data);
}