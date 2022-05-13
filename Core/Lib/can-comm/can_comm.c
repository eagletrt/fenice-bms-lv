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
#include "volt.h"
#include "fenice-config.h"

CAN_TxHeaderTypeDef tx_header;
CAN_RxHeaderTypeDef rx_header;

void can_tx_header_init() {
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
}

void can_primary_init(){

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
void can_secondary_init(){    
    
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

HAL_StatusTypeDef can_primary_send(uint16_t id){
    uint8_t buffer[CAN_MAX_PAYLOAD_LENGTH] = {1,2,3,4,5,6,7,8};

    tx_header.StdId = id;
    

    if (id == PRIMARY_ID_LV_VOLTAGE){
        //tx_header.DLC = serialize_PrimaryLvVoltage();
    }
    

    return can_send(&CANP, buffer, &tx_header);
}

HAL_StatusTypeDef can_secondary_send(uint16_t id){
    uint8_t buffer[CAN_MAX_PAYLOAD_LENGTH] = {1,2,3,4,5,6,7,8};
    tx_header.StdId = id;
    tx_header.DLC = 8;

    return can_send(&CANS, buffer, &tx_header);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    //volatile uint8_t x = 0;
    HAL_CAN_GetRxMessage(&CANP, CAN_RX_FIFO0, &rx_header, 0x0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
    //volatile uint8_t x = 0;
    HAL_CAN_GetRxMessage(&CANS, CAN_RX_FIFO1, &rx_header, 0x0);
}