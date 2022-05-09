/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "../Lib/can-cicd/includes_generator/Primary/ids.h"
#include "../Lib/can-cicd/includes_generator/Primary/utils.h"
#include "../Lib/can-cicd/includes_generator/Secondary/ids.h"
#include "../Lib/can-cicd/includes_generator/Secondary/utils.h"
#include "../Lib/can-cicd/naked_generator/Primary/c/Primary.h"
#include "../Lib/can-cicd/naked_generator/Secondary/c/Secondary.h"
#include "../Lib/micro-libs/priority-queue/priority_queue_fast_insert.h"
#include "assert.h"
#include "common.h"
#include "fenice-config.h"
#include "peripherals/buzzer.h"
#include "usart.h"
/* Private defines -----------------------------------------------------------*/
canStruct can1, can3;

/* Private variables ---------------------------------------------------------*/
PQFI_HandleTypeDef hpqfi_rx_pcan = NULL; /* PQFI handle for primary CAN in RX (receive messages) */
#ifndef NO_SECONDARY_CAN
PQFI_HandleTypeDef hpqfi_rx_scan = NULL; /* PQFI handle for secondary CAN in RX (receive messages) */
#endif

/* Private functions prototypes ----------------------------------------------*/
static void _can_apply_filter(CAN_HandleTypeDef *hcan);
static void _can_error_handler(CAN_HandleTypeDef *hcan);
static bool _pqfi_rx_pcan_pri_cmpr_function(PQFI_PriorityTypeDef a, PQFI_PriorityTypeDef b);

void _compose_can_message(CAN_MsgNameTypeDef can_msg_name, uint8_t msg_buff[8], uint8_t *buff_fin_size);

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void) {
    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */

    /* USER CODE END CAN1_Init 1 */
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 16;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_1TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_1TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = DISABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = DISABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */

    _can_apply_filter(&hcan1);

    /* USER CODE END CAN1_Init 2 */
}
/* CAN2 init function */
void MX_CAN2_Init(void) {
    /* USER CODE BEGIN CAN2_Init 0 */

    /* USER CODE END CAN2_Init 0 */

    /* USER CODE BEGIN CAN2_Init 1 */

    /* USER CODE END CAN2_Init 1 */
    hcan2.Instance                  = CAN2;
    hcan2.Init.Prescaler            = 16;
    hcan2.Init.Mode                 = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan2.Init.TimeSeg1             = CAN_BS1_1TQ;
    hcan2.Init.TimeSeg2             = CAN_BS2_1TQ;
    hcan2.Init.TimeTriggeredMode    = DISABLE;
    hcan2.Init.AutoBusOff           = DISABLE;
    hcan2.Init.AutoWakeUp           = DISABLE;
    hcan2.Init.AutoRetransmission   = DISABLE;
    hcan2.Init.ReceiveFifoLocked    = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN2_Init 2 */

    /* USER CODE END CAN2_Init 2 */
}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED = 0;

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (canHandle->Instance == CAN1) {
        /* USER CODE BEGIN CAN1_MspInit 0 */

        /* USER CODE END CAN1_MspInit 0 */
        /* CAN1 clock enable */
        HAL_RCC_CAN1_CLK_ENABLED++;
        if (HAL_RCC_CAN1_CLK_ENABLED == 1) {
            __HAL_RCC_CAN1_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN CAN1_MspInit 1 */

        /* USER CODE END CAN1_MspInit 1 */
    } else if (canHandle->Instance == CAN2) {
        /* USER CODE BEGIN CAN2_MspInit 0 */

        /* USER CODE END CAN2_MspInit 0 */
        /* CAN2 clock enable */
        __HAL_RCC_CAN2_CLK_ENABLE();
        HAL_RCC_CAN1_CLK_ENABLED++;
        if (HAL_RCC_CAN1_CLK_ENABLED == 1) {
            __HAL_RCC_CAN1_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
        GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN CAN2_MspInit 1 */

        /* USER CODE END CAN2_MspInit 1 */
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle) {
    if (canHandle->Instance == CAN1) {
        /* USER CODE BEGIN CAN1_MspDeInit 0 */

        /* USER CODE END CAN1_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_CAN1_CLK_ENABLED--;
        if (HAL_RCC_CAN1_CLK_ENABLED == 0) {
            __HAL_RCC_CAN1_CLK_DISABLE();
        }

        /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

        /* USER CODE BEGIN CAN1_MspDeInit 1 */

        /* USER CODE END CAN1_MspDeInit 1 */
    } else if (canHandle->Instance == CAN2) {
        /* USER CODE BEGIN CAN2_MspDeInit 0 */

        /* USER CODE END CAN2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CAN2_CLK_DISABLE();
        HAL_RCC_CAN1_CLK_ENABLED--;
        if (HAL_RCC_CAN1_CLK_ENABLED == 0) {
            __HAL_RCC_CAN1_CLK_DISABLE();
        }

        /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

        /* USER CODE BEGIN CAN2_MspDeInit 1 */

        /* USER CODE END CAN2_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* Exported functions --------------------------------------------------------*/

COMM_StatusTypeDef CAN_start_all() {
    COMM_StatusTypeDef ret = COMM_OK;
    /* Start primary Can*/

    if (HAL_CAN_ActivateNotification(&hcanP, CANP_IT_NOTI_SET) != HAL_OK) {
        printl("Failed to activate PRIMARY can interrupts", CAN_ERR_HEADER);
        ret = COMM_ERROR;
    }
    hpqfi_rx_pcan =
        PQFI_init(C_RX_FIFO_LENGTH, sizeof(CAN_RxPQFI_Data_TypeDef), &_pqfi_rx_pcan_pri_cmpr_function, NULL);
    if (hpqfi_rx_pcan == NULL) {
        printl("Priority queue RX in PRIMARY can not initialized, probably malloc errors", CAN_ERR_HEADER);
        ret = COMM_ERROR;
    }
    if (HAL_CAN_Start(&hcanP) != HAL_OK) {
        printl("Unable to start PRIMARY CAN", CAN_ERR_HEADER);

        /* Primary can error sequence*/
        BZZR_play_pulses(200, 2);
        HAL_Delay(500);
        BZZR_play_pulses(200, 1);
        ret = COMM_ERROR;
    }
    printl("Primary CAN ready", CAN_HEADER);

#ifndef NO_SECONDARY_CAN
    /* Start secondary Can*/
    if (HAL_CAN_ActivateNotification(&hcanS, CANS_IT_NOTI_SET) == HAL_ERROR) {
        printl("Failed to activate SECONDARY can interrupts", CAN_ERR_HEADER);
        ret = COMM_ERROR;
    }
    if (hpqfi_rx_pcan == NULL) {
        printl("Priority queue RX in SECONDARY can not initialized, probably malloc errors", CAN_ERR_HEADER);
        ret = COMM_ERROR;
    }
    if (HAL_CAN_Start(&hcanS) != HAL_OK) {
        printl("Unable to start SECONDARY CAN", CAN_ERR_HEADER);

        /* Secondary can error sequence*/
        BZZR_play_pulses(200, 2);
        HAL_Delay(500);
        BZZR_play_pulses(200, 2);
        ret = COMM_ERROR;
    }
    printl("Secondary CAN ready", CAN_HEADER);
#endif
    return ret;
}

HAL_StatusTypeDef CAN_send(CAN_HandleTypeDef *hcan, uint16_t msg_name) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        printl("No free mailboxes available", CAN_ERR_HEADER);
        _can_error_handler(hcan);
        return HAL_ERROR;
    }

    CAN_TxHeaderTypeDef header;
    uint8_t data[1] = {};
    HAL_StatusTypeDef status;
    uint32_t free_mailbox;

    /* Prepare TX header */
    header.StdId              = msg_name;
    header.IDE                = CAN_ID_STD;
    header.DLC                = 1;
    header.RTR                = CAN_RTR_DATA;
    header.TransmitGlobalTime = DISABLE;

    /* Send the message */
    status = HAL_CAN_AddTxMessage(hcan, &header, data, &free_mailbox);
    if (status != HAL_OK) {
        printl("AddTxMessage failed", CAN_ERR_HEADER);
        _can_error_handler(hcan);
    }

    printl("CAN_send", CAN_HEADER);
    return status;
}

COMM_StatusTypeDef CAN_get(CAN_HandleTypeDef *hcan, CAN_RxPQFI_Data_TypeDef *rx_data) {
    assert(hcan != NULL);
    assert(rx_data != NULL);
    PQFI_HandleTypeDef hpqfi_tmp;
    if (hcan == &hcanP) {
        hpqfi_tmp = hpqfi_rx_pcan;
    }
#ifndef NO_SECONDARY_CAN
    else if (hcan == hcanS) {
        hpqfi_tmp = hpqfi_rx_scan;
    }
#endif
    else {
        printl("Wrong can instance", CAN_HEADER);
        return COMM_ERROR;
    }

    return PQFI_pop(hpqfi_tmp, rx_data) ? COMM_OK : COMM_ERROR;
}
void CAN_print_rxdata(CAN_RxPQFI_Data_TypeDef *rx_data) {
    char buf[100]    = {};
    char id_name[50] = {};
#ifndef NO_SECONDARY_CAN
    hcan->Instance == CANP ? CAN_id_tostring(CAN_NET_PRIM, rxheader->StdId, id_name)
                           : CAN_id_tostring(CAN_NET_SEC, rxheader->StdId, id_name);
#else
    CAN_id_tostring(CAN_NET_PRIM, rx_data->rxheader.StdId, id_name);
#endif
    sprintf(buf, "ID: %lu (%s) [", rx_data->rxheader.StdId, id_name);
    printl(buf, CAN_HEADER);
    sprintf(
        buf,
        "x%02X x%02X x%02X x%02X], DLC: %lu, RTR: %lu",
        rx_data->payload[0] << 8 | rx_data->payload[1],
        rx_data->payload[2] << 8 | rx_data->payload[3],
        rx_data->payload[4] << 8 | rx_data->payload[5],
        rx_data->payload[6] << 8 | rx_data->payload[7],
        rx_data->rxheader.DLC,
        rx_data->rxheader.RTR);
    printl(buf, NOTHING);
}
void CAN_id_tostring(CAN_NetTypeDef net, uint32_t msg_id, char buf[static 50]) {
    switch (net) {
        case CAN_NET_PRIM:
            Primary_msgname_from_id(msg_id, buf);
            break;
#ifndef NO_SECONDARY_CAN
        case CAN_NET_SEC:
            Secondary_msgname_from_id(msg_id, buf);
            break;
#endif
        default:
            buf[0] = '\0';
            break;
    }
}
/**
 * @brief     Make a can message in function of the specified can_msg_name
 * and save the message inside msg_buffer
 *
 * @param can_msg_name  The name of the message to be sent.
 * @param msg_buff      An array of size 8 where the message will be saved.
 * @param size          pointer to a variable that will contain
 *                      the final length of the buffer after serialization.
 */
void _compose_can_message(CAN_MsgNameTypeDef can_msg_name, uint8_t msg_buff[8], uint8_t *buff_fin_size) {
    // TODO: check in testing that msg_buffer and size are not NULL
    // clear common buffer

#if 0
    switch (can_msg_name)
    {
        case ID_HV_VOLTAGE:

            HV_VOLTAGE_create(B, 42000, 41900, 38000, 37500);

            break;
        case ID_HV_CURRENT:
            HV_CURRENT_create(B, 2000, 69); // nice
            break;

        default:
            return HAL_ERROR;
    }
#endif
}

COMM_StatusTypeDef CAN_Send(CAN_MsgNameTypeDef can_msg_name) {
    uint8_t buffer[8];
    uint8_t buffer_final_size = 8;  // Init to an acceptable value
    _compose_can_message(can_msg_name, buffer, &buffer_final_size);

    CAN_TxHeaderTypeDef tx_header;
    uint32_t txUsedMailbox;
    tx_header.StdId = (uint32_t)can_msg_name;
    tx_header.ExtId = tx_header.StdId;  // Not used but just in case
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = buffer_final_size;

    // Try to send a message
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, buffer, &txUsedMailbox) != HAL_OK) {
        // Possible errors: can not initializes, someone else is already
        // listening or finally all mailboxes are full
        // TODO: manage full mailboxes.
        // +> approaches: - delete less important message
        //                - watch timestamp of stagnant messages
        //                - put message in sfw FIFO and retry sending via
        //                  interrupt notification
    } else {
    }

    return COMM_OK;
}

uint8_t old_CAN_Send(canStruct *can) {
    return 1;
    if (HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) != 0) {
        if (old_CAN_Send_IT(can) == 0) {
            return 0;
        }
    } else {
        if (can->hcan == &hcan1) {
            if (fifoTxDataCAN_push(can) == 0) {
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

uint8_t old_CAN_Send_IT(canStruct *can) {
    uint32_t mailbox = 0;
    // CAN_TxMailBox_TypeDef mailbox;
    // mailbox.TIR = 0; //set to mailbox 0

    for (int i = 0; i < 8; i++) {
        can->dataTxBck[i] = can->dataTx[i];
    }
    can->idBck   = can->tx_id;
    can->sizeBck = can->tx_size;

    uint8_t flag = 0;  // error

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId              = can->tx_id;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = can->tx_size;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTx, &mailbox) == HAL_OK) {
        flag = 1;  // ok
    }

    return flag;
}

uint8_t fifoRxDataCAN_pop(canStruct *can) {
    if (can->fifo.rxHead == can->fifo.rxTail) {
        return 0;
    } else {
        can->rx_id   = can->fifo.rx[can->fifo.rxTail].id;
        can->rx_size = can->fifo.rx[can->fifo.rxTail].size;
        for (uint8_t i = 0; i < can->rx_size; i++) {
            can->dataRx[i] = can->fifo.rx[can->fifo.rxTail].data[i];
        }
        can->fifo.rxTail = (can->fifo.rxTail + 1) % fifoLength;
        return 1;
    }
}

uint8_t fifoRxDataCAN_push(canStruct *can) {
    if ((can->fifo.rxHead + 1) % fifoLength == can->fifo.rxTail) {
        return 0;
    } else {
        can->fifo.rx[can->fifo.rxHead].id   = can->rx_id_int;
        can->fifo.rx[can->fifo.rxHead].size = can->rx_size_int;
        for (uint8_t i = 0; i < can->rx_size_int; i++) {
            can->fifo.rx[can->fifo.rxHead].data[i] = can->dataRX_int[i];
        }
        can->fifo.rxHead = (can->fifo.rxHead + 1) % fifoLength;
        return 1;
    }
}

uint8_t fifoTxDataCAN_pop(canStruct *can) {
    if (can->fifo.txHead == can->fifo.txTail) {
        return 0;
    } else {
        can->tx_id   = can->fifo.tx[can->fifo.txTail].id;
        can->tx_size = can->fifo.tx[can->fifo.txTail].size;
        for (uint8_t i = 0; i < can->tx_size; i++) {
            can->dataTx[i] = can->fifo.tx[can->fifo.txTail].data[i];
        }
        can->fifo.txTail = (can->fifo.txTail + 1) % fifoLength;
        return 1;
    }
}

uint8_t fifoTxDataCAN_push(canStruct *can) {
    if ((can->fifo.txHead + 1) % fifoLength == can->fifo.txTail) {
        return 0;
    } else {
        can->fifo.tx[can->fifo.txHead].id   = can->tx_id;
        can->fifo.tx[can->fifo.txHead].size = can->tx_size;
        for (uint8_t i = 0; i < can->tx_size; i++) {
            can->fifo.tx[can->fifo.txHead].data[i] = can->dataTx[i];
        }
        can->fifo.txHead = (can->fifo.txHead + 1) % fifoLength;
        return 1;
    }
}

#if 0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
            CAN_RxHeaderTypeDef header;
            HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, can1.dataRX_int);
            can1.rx_id_int   = header.StdId;
            can1.rx_size_int = header.DLC;
            fifoRxDataCAN_push(&can1);
        }
    }
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        if (fifoTxDataCAN_pop(&can1)) {
            if (old_CAN_Send_IT(&can1) == 0) {
                // TODO: implementare errore
            }
        }
    }
}
#endif

#ifdef OLD_CAN_SEND_DATA
bool old_CAN_Send_data(uint32_t tick) {
    bool message_sent = false;

    if (tick % 200 == 0) {
        can1.tx_id     = 0x97;
        can1.dataTx[0] = ltc.voltage[0] >> 8;
        can1.dataTx[1] = (uint8_t)ltc.voltage[1];
        can1.tx_size   = 2;
        old_CAN_Send(&can1);
        message_sent = true;
    }

    return message_sent;
}
#endif

bool CAN_Read_Message() {
    if (fifoRxDataCAN_pop(&can1)) {
        switch (can1.rx_id) {
            case INV_LEFT_ASK_ID:
                if (can1.dataRx[0] == 0x4A) {         // Inverter left temperature
                                                      // invLeftTemp = can->dataRx[1] + (can->dataRx[2] << 8);
                                                      // inverterLeftTemp = (invLeftTemp - 15797) / 112.12;
                } else if (can1.dataRx[0] == 0x49) {  // Motor left temperature
                                                      // motLeftTemp = can->dataRx[1] + (can->dataRx[2] << 8);
                                                      // motorLeftTemp = (motLeftTemp - 9394) / 55.10;
                }
                break;
            case INV_RIGHT_ASK_ID:
                if (can1.dataRx[0] == 0x4A) {         // Inverter right temperature
                                                      // invRightTemp = can->dataRx[1] + (can->dataRx[2] << 8);
                                                      // inverterRightTemp = (invLeftTemp - 15797) / 112.12;
                } else if (can1.dataRx[0] == 0x49) {  // Motor right temperature
                                                      // motRightTemp = can->dataRx[1] + (can->dataRx[2] << 8);
                                                      // motorRightTemp = (motLeftTemp - 9394) / 55.10;
                }
                break;
            case STEER_ASK_ID:
                if (can1.dataRx[0] == 0) {
                    // overridePID = RxData[1]; // 1 - override PID
                } else if (can1.dataRx[0] == 1) {
                    // overridePID = RxData[1];
                    // pumpRequest = RxData[2];
                } else if (can1.dataRx[0] == 2) {
                    // overridePID = RxData[1];
                    // fanRequest = RxData[2];
                } else if (can1.dataRx[0] == 3 && can1.dataRx[1] == 1) {
                    // overridePID = 2;
                }
                break;
            case ACC_TEMP_ASK_ID:
                if (can1.dataRx[0] == 6) {  // Little endian
                                            // tmpHvAvgTemp = can->dataRx[5] + (can->dataRx[4] << 8);
                                            // hvAvgTemp = tmpHvAvgTemp / 100.0;
                } else if (can1.dataRx[0] == 6) {
                    // tmpHvMaxTemp = can->dataRx[7] + (can->dataRx[6] <<
                    // 8); hvMaxTemp = tmpHvMaxTemp / 100.0;
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

/* Callbacks -----------------------------------------------------------------*/

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    _can_error_handler(hcan);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    static CAN_RxPQFI_Data_TypeDef rx_pqfi_data;
    rx_pqfi_data.net = CAN_NET_PRIM;

    if (PQFI_is_full(hpqfi_rx_pcan) != true) {
        HAL_CAN_GetRxMessage(hcan, CANP_RX_FIFO, &(rx_pqfi_data.rxheader), rx_pqfi_data.payload);
        PQFI_insert(hpqfi_rx_pcan, rx_pqfi_data.rxheader.StdId, &rx_pqfi_data);
    } else {
        printl("Primary can rx fifo full", CAN_HEADER);
    }
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    printl("FUCKING RX2", CAN_HEADER);
    CAN_RxHeaderTypeDef h;
    uint8_t data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &h, data);
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
    printl("RX FIFO#0 is full", CAN_HEADER);
}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
    printl("RX FIFO#1 is full", CAN_HEADER);
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    printl("Mailbox 0 complete", CAN_HEADER);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    printl("Mailbox 1 complete", CAN_HEADER);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    printl("Mailbox 2 complete", CAN_HEADER);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief     Initialize fitlers for can interface
 *
 * @param     hcan Can peripheral handle where filters are initialized
 * @return    COMM_StatusTypeDef COMM_OK if all went ok, COMM_ERROR in case of
 * errors
 */
static void _can_apply_filter(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef filter;

    filter.FilterMode       = CAN_FILTERMODE_IDMASK;
    filter.FilterBank       = 0;
    filter.FilterScale      = CAN_FILTERSCALE_16BIT;
    filter.FilterActivation = ENABLE;
    if (hcan->Instance == CANP) {  // can on primary network

        filter.FilterFIFOAssignment = CANP_FILTER_FIFO;

        // Filter for topic TODO: place topic
        filter.FilterIdLow      = 0;
        filter.FilterIdHigh     = 0;
        filter.FilterMaskIdHigh = 0;
        filter.FilterMaskIdLow  = 0;
    }
#ifndef NO_SECONDARY_CAN
    else if (hcan->Instance == CANS) {
        filter.FilterFIFOAssignment = CANS_FILTER_FIFO;

        // Filter for topic TODO: place topic
        filter.FilterIdLow      = 0;
        filter.FilterIdHigh     = 0;
        filter.FilterMaskIdHigh = 0;
        filter.FilterMaskIdLow  = 0;
    }
#endif
    else {
        printl("Wrong can handle while applying filter", CAN_ERR_HEADER);
        return;
    }
    HAL_CAN_ConfigFilter(hcan, &filter);
}

/**
 * @brief     Priority compare function for elements inside hpqfi_rx_pcan
 * 
 * @param     a Priority item A
 * @param     b Priority item B
 * @return    true if A < B
 * @return    false if A >= B
 */
static bool _pqfi_rx_pcan_pri_cmpr_function(PQFI_PriorityTypeDef a, PQFI_PriorityTypeDef b) {
    return a < b;
}

/**
 * @brief Print the error message in the serial console and activate
 *        the CAN error LED
 * @param msg The message to send over UART
 * */
static void _can_error_handler(CAN_HandleTypeDef *hcan) {
    uint32_t err_code = HAL_CAN_GetError(hcan);
    if (err_code == 0)
        return;

    char buf[50];
    sprintf(buf, "Code: %08lX", err_code);
    printl(buf, CAN_ERR_HEADER);

    if ((err_code & HAL_CAN_ERROR_EWG) == HAL_CAN_ERROR_EWG)
        printl("Protocol error warning", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_EPV) == HAL_CAN_ERROR_EPV)
        printl("Error passive", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_BOF) == HAL_CAN_ERROR_BOF)
        printl("Bus-off error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_STF) == HAL_CAN_ERROR_STF)
        printl("Stuff error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_FOR) == HAL_CAN_ERROR_FOR)
        printl("Form error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_ACK) == HAL_CAN_ERROR_ACK)
        printl("ACK error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_BR) == HAL_CAN_ERROR_BR)
        printl("Bit Recessive error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_BD) == HAL_CAN_ERROR_BD)
        printl("Bit Dominant error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_CRC) == HAL_CAN_ERROR_CRC)
        printl("CRC error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_RX_FOV0) == HAL_CAN_ERROR_RX_FOV0)
        printl("FIFO 0 overrun error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_RX_FOV1) == HAL_CAN_ERROR_RX_FOV1)
        printl("FIFO 1 overrun error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_TX_ALST0) == HAL_CAN_ERROR_TX_ALST0)
        printl("TX 0 arbitration lost error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_TX_TERR0) == HAL_CAN_ERROR_TX_TERR0)
        printl("TX 0 transmit error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_TX_ALST1) == HAL_CAN_ERROR_TX_ALST1)
        printl("TX 1 arbitration lost error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_TX_TERR1) == HAL_CAN_ERROR_TX_TERR1)
        printl("TX 1 transmit error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_TX_ALST2) == HAL_CAN_ERROR_TX_ALST2)
        printl("TX 2 arbitration lost error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_TX_TERR2) == HAL_CAN_ERROR_TX_TERR2)
        printl("TX 2 transmit error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_TIMEOUT) == HAL_CAN_ERROR_TIMEOUT)
        printl("Timeout error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_NOT_INITIALIZED) == HAL_CAN_ERROR_NOT_INITIALIZED)
        printl("CAN bus not initialized", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_NOT_READY) == HAL_CAN_ERROR_NOT_READY)
        printl("CAN bus not ready", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_NOT_STARTED) == HAL_CAN_ERROR_NOT_STARTED)
        printl("CAN bus not started", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_PARAM) == HAL_CAN_ERROR_PARAM)
        printl("Parameter error", NO_HEADER);
    if ((err_code & HAL_CAN_ERROR_INTERNAL) == HAL_CAN_ERROR_INTERNAL)
        printl("Internal error", NO_HEADER);

    uint16_t rec_val = (uint16_t)((hcan->Instance->ESR && CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
    if (rec_val > 0) {
        sprintf(buf, "REC (Receive Error Counter) %d", rec_val);
        printl(buf, NO_HEADER);
    }

    uint16_t tec_val = (uint16_t)((hcan->Instance->ESR && CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
    if (tec_val > 0) {
        sprintf(buf, "TEC (Transmit Error Counter) %d", tec_val);
        printl(buf, NO_HEADER);
    }
}

/* USER CODE END 1 */
