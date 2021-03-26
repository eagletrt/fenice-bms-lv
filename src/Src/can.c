/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "ltc.h"
#include "fenice-config.h"

canStruct                can1, can3;
extern CAN_HandleTypeDef hcan1;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan3;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */

    /* USER CODE END CAN1_Init 1 */
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 3;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_16TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = DISABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = DISABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */

    /* USER CODE END CAN1_Init 2 */
}
/* CAN3 init function */
void MX_CAN3_Init(void)
{

    /* USER CODE BEGIN CAN3_Init 0 */

    /* USER CODE END CAN3_Init 0 */

    /* USER CODE BEGIN CAN3_Init 1 */

    /* USER CODE END CAN3_Init 1 */
    hcan3.Instance                  = CAN3;
    hcan3.Init.Prescaler            = 3;
    hcan3.Init.Mode                 = CAN_MODE_NORMAL;
    hcan3.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan3.Init.TimeSeg1             = CAN_BS1_16TQ;
    hcan3.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan3.Init.TimeTriggeredMode    = DISABLE;
    hcan3.Init.AutoBusOff           = DISABLE;
    hcan3.Init.AutoWakeUp           = DISABLE;
    hcan3.Init.AutoRetransmission   = DISABLE;
    hcan3.Init.ReceiveFifoLocked    = DISABLE;
    hcan3.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN3_Init 2 */

    /* USER CODE END CAN3_Init 2 */
}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED = 0;

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (canHandle->Instance == CAN1)
    {
        /* USER CODE BEGIN CAN1_MspInit 0 */

        /* USER CODE END CAN1_MspInit 0 */
        /* CAN1 clock enable */
        HAL_RCC_CAN1_CLK_ENABLED++;
        if (HAL_RCC_CAN1_CLK_ENABLED == 1)
        {
            __HAL_RCC_CAN1_CLK_ENABLE();
        }

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**CAN1 GPIO Configuration
        PD0     ------> CAN1_RX
        PD1     ------> CAN1_TX
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN CAN1_MspInit 1 */

        /* USER CODE END CAN1_MspInit 1 */
    }
    else if (canHandle->Instance == CAN3)
    {
        /* USER CODE BEGIN CAN3_MspInit 0 */

        /* USER CODE END CAN3_MspInit 0 */
        /* CAN3 clock enable */
        __HAL_RCC_CAN3_CLK_ENABLE();
        __HAL_RCC_CAN2_CLK_ENABLE();
        HAL_RCC_CAN1_CLK_ENABLED++;
        if (HAL_RCC_CAN1_CLK_ENABLED == 1)
        {
            __HAL_RCC_CAN1_CLK_ENABLE();
        }

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**CAN3 GPIO Configuration
        PA8     ------> CAN3_RX
        PB4     ------> CAN3_TX
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_8;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_CAN3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_PIN_4;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_CAN3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN CAN3_MspInit 1 */

        /* USER CODE END CAN3_MspInit 1 */
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

    if (canHandle->Instance == CAN1)
    {
        /* USER CODE BEGIN CAN1_MspDeInit 0 */

        /* USER CODE END CAN1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CAN1_CLK_DISABLE();

        /**CAN1 GPIO Configuration
        PD0     ------> CAN1_RX
        PD1     ------> CAN1_TX
        */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);

        /* USER CODE BEGIN CAN1_MspDeInit 1 */

        /* USER CODE END CAN1_MspDeInit 1 */
    }
    else if (canHandle->Instance == CAN3)
    {
        /* USER CODE BEGIN CAN3_MspDeInit 0 */

        /* USER CODE END CAN3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CAN3_CLK_DISABLE();
        __HAL_RCC_CAN2_CLK_DISABLE();
        HAL_RCC_CAN1_CLK_ENABLED--;
        if (HAL_RCC_CAN1_CLK_ENABLED == 0)
        {
            __HAL_RCC_CAN1_CLK_DISABLE();
        }

        /**CAN3 GPIO Configuration
        PA8     ------> CAN3_RX
        PB4     ------> CAN3_TX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);

        /* USER CODE BEGIN CAN3_MspDeInit 1 */

        /* USER CODE END CAN3_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

bool can_init()
{
    bool ret = false;
    if (CAN_initialization(&can1))
    {
        ret = true;
        // report_error_can1();
    }
    /*
    if(CAN_initialization(&can3)){
        report_error_can3();
    }*/
    return ret;
}

bool CAN_initialization(canStruct *can)
{
    // CAN filter initialization
    can->canFilter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can->canFilter.FilterIdLow          = 0;
    can->canFilter.FilterIdHigh         = 0;
    can->canFilter.FilterMaskIdHigh     = 0;
    can->canFilter.FilterMaskIdLow      = 0;
    can->canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can->canFilter.FilterBank           = 0;
    can->canFilter.FilterScale          = CAN_FILTERSCALE_16BIT;
    can->canFilter.FilterActivation     = ENABLE;

    // CAN filter configuration
    can->configFilter_status = HAL_CAN_ConfigFilter(can->hcan, &can->canFilter);

    can->activateNotif_status
        = HAL_CAN_ActivateNotification(can->hcan, can->rx0_interrupt);
    can->activateNotif_status
        = HAL_CAN_ActivateNotification(can->hcan, can->tx_interrupt);

    can->fifo.rxHead = 0;
    can->fifo.rxTail = 0;
    can->fifo.txHead = 0;
    can->fifo.txTail = 0;

    // CAN start
    can->canStart_status = HAL_CAN_Start(can->hcan);

    if (can->configFilter_status == HAL_OK
        && can->activateNotif_status == HAL_OK
        && can->canStart_status == HAL_OK)
        return false; // no errors occurred
    else
        return true;
}

uint8_t CAN_Send(canStruct *can)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) != 0)
    {
        if (CAN_Send_IT(can) == 0)
        {
            return 0;
        }
    }
    else
    {
        if (can->hcan == &hcan1)
        {
            if (fifoTxDataCAN_push(can) == 0)
            {
                return 0;
            }
        }
        /*else{
            if(fifoTxDataCAN3_push(&fifoCAN3, &fifodata) == 0){
                //TODO: implementare errore
                return 0;
            }*/
        //}
    }

    return 1;
}

uint8_t CAN_Send_IT(canStruct *can)
{
    uint32_t mailbox = 0;
    // CAN_TxMailBox_TypeDef mailbox;
    // mailbox.TIR = 0; //set to mailbox 0

    for (int i = 0; i < 8; i++)
    {
        can->dataTxBck[i] = can->dataTx[i];
    }
    can->idBck   = can->tx_id;
    can->sizeBck = can->tx_size;

    uint8_t flag = 0; // error

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId              = can->tx_id;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = can->tx_size;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTx, &mailbox)
        == HAL_OK)
    {
        flag = 1; // ok
    }

    return flag;
}

uint8_t fifoRxDataCAN_pop(canStruct *can)
{
    if (can->fifo.rxHead == can->fifo.rxTail)
    {
        return 0;
    }
    else
    {
        can->rx_id   = can->fifo.rx[can->fifo.rxTail].id;
        can->rx_size = can->fifo.rx[can->fifo.rxTail].size;
        for (uint8_t i = 0; i < can->rx_size; i++)
        {
            can->dataRx[i] = can->fifo.rx[can->fifo.rxTail].data[i];
        }
        can->fifo.rxTail = (can->fifo.rxTail + 1) % fifoLength;
        return 1;
    }
}

uint8_t fifoRxDataCAN_push(canStruct *can)
{
    if ((can->fifo.rxHead + 1) % fifoLength == can->fifo.rxTail)
    {
        return 0;
    }
    else
    {
        can->fifo.rx[can->fifo.rxHead].id   = can->rx_id_int;
        can->fifo.rx[can->fifo.rxHead].size = can->rx_size_int;
        for (uint8_t i = 0; i < can->rx_size_int; i++)
        {
            can->fifo.rx[can->fifo.rxHead].data[i] = can->dataRX_int[i];
        }
        can->fifo.rxHead = (can->fifo.rxHead + 1) % fifoLength;
        return 1;
    }
}

uint8_t fifoTxDataCAN_pop(canStruct *can)
{
    if (can->fifo.txHead == can->fifo.txTail)
    {
        return 0;
    }
    else
    {
        can->tx_id   = can->fifo.tx[can->fifo.txTail].id;
        can->tx_size = can->fifo.tx[can->fifo.txTail].size;
        for (uint8_t i = 0; i < can->tx_size; i++)
        {
            can->dataTx[i] = can->fifo.tx[can->fifo.txTail].data[i];
        }
        can->fifo.txTail = (can->fifo.txTail + 1) % fifoLength;
        return 1;
    }
}

uint8_t fifoTxDataCAN_push(canStruct *can)
{
    if ((can->fifo.txHead + 1) % fifoLength == can->fifo.txTail)
    {
        return 0;
    }
    else
    {
        can->fifo.tx[can->fifo.txHead].id   = can->tx_id;
        can->fifo.tx[can->fifo.txHead].size = can->tx_size;
        for (uint8_t i = 0; i < can->tx_size; i++)
        {
            can->fifo.tx[can->fifo.txHead].data[i] = can->dataTx[i];
        }
        can->fifo.txHead = (can->fifo.txHead + 1) % fifoLength;
        return 1;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0)
        {
            CAN_RxHeaderTypeDef header;
            HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header,
                                 can1.dataRX_int);
            can1.rx_id_int   = header.StdId;
            can1.rx_size_int = header.DLC;
            fifoRxDataCAN_push(&can1);
        }
    }
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        if (fifoTxDataCAN_pop(&can1))
        {
            if (CAN_Send_IT(&can1) == 0)
            {
                // TODO: implementare errore
            }
        }
    }
}

bool CAN_send_data(uint32_t tick)
{
    bool message_sent = false;

    if (tick % 200 == 0)
    {
        can1.tx_id     = 0x97;
        can1.dataTx[0] = ltc.voltage[0] >> 8;
        can1.dataTx[1] = (uint8_t)ltc.voltage[1];
        can1.tx_size   = 2;
        CAN_Send(&can1);
        message_sent = true;
    }

    return message_sent;
}

bool CAN_Read_Message()
{
    if (fifoRxDataCAN_pop(&can1))
    {
        switch (can1.rx_id)
        {
            case INV_LEFT_ASK_ID:
                if (can1.dataRx[0] == 0x4A)
                { // Inverter left temperature
                  // invLeftTemp = can->dataRx[1] + (can->dataRx[2] << 8);
                  // inverterLeftTemp = (invLeftTemp - 15797) / 112.12;
                }
                else if (can1.dataRx[0] == 0x49)
                { // Motor left temperature
                  // motLeftTemp = can->dataRx[1] + (can->dataRx[2] << 8);
                  // motorLeftTemp = (motLeftTemp - 9394) / 55.10;
                }
                break;
            case INV_RIGHT_ASK_ID:
                if (can1.dataRx[0] == 0x4A)
                { // Inverter right temperature
                  // invRightTemp = can->dataRx[1] + (can->dataRx[2] << 8);
                  // inverterRightTemp = (invLeftTemp - 15797) / 112.12;
                }
                else if (can1.dataRx[0] == 0x49)
                { // Motor right temperature
                  // motRightTemp = can->dataRx[1] + (can->dataRx[2] << 8);
                  // motorRightTemp = (motLeftTemp - 9394) / 55.10;
                }
                break;
            case STEER_ASK_ID:
                if (can1.dataRx[0] == 0)
                {
                    // overridePID = RxData[1]; // 1 - override PID
                }
                else if (can1.dataRx[0] == 1)
                {
                    // overridePID = RxData[1];
                    // pumpRequest = RxData[2];
                }
                else if (can1.dataRx[0] == 2)
                {
                    // overridePID = RxData[1];
                    // fanRequest = RxData[2];
                }
                else if (can1.dataRx[0] == 3 && can1.dataRx[1] == 1)
                {
                    // overridePID = 2;
                }
                break;
            case ACC_TEMP_ASK_ID:
                if (can1.dataRx[0] == 6)
                { // Little endian
                  // tmpHvAvgTemp = can->dataRx[5] + (can->dataRx[4] << 8);
                  // hvAvgTemp = tmpHvAvgTemp / 100.0;
                }
                else if (can1.dataRx[0] == 6)
                {
                    // tmpHvMaxTemp = can->dataRx[7] + (can->dataRx[6] << 8);
                    // hvMaxTemp = tmpHvMaxTemp / 100.0;
                }
                break;
            case ECU_ASK_ID:
                break;
            case BMS_LV_ASK_ID:
                break;

            default:
                break;
        }
        return 0;
    }
    return 0;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
