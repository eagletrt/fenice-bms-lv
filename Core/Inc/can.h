/**
 ******************************************************************************
 * @file    can.h
 * @brief   This file contains all the function prototypes for
 *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include "common.h"
#include "stdbool.h"

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan3;

/* USER CODE BEGIN Private defines */

/* This constants define which CAN peripherals are uesed for which network */
/* The below mappings must be precise if changed */

/* hcan1 -> primary network */
#define hcanP            hcan1            /* Primary can handle */
#define CANP             CAN1             /* Primary can instance (== hcan->Instance) */
#define CANP_FILTER_FIFO CAN_FILTER_FIFO0 /* Primary can hardware fifo filter */
#define CANP_IT_NOTI_SET                                                                                  \
    CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | \
        CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR

/* hcan2 -> secondary network */
#define hcanS            hcan3             // Secondary can handle
#define CANS             CAN3              // Secondary can instance
#define CANS_FILTER_FIFO CAN_FILTER_FIFO1  // Secondary can hardware fifo filter
#define CANS_IT_NOTI_SET                                                                                  \
    CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_OVERRUN | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | \
        CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR

typedef enum { CAN_NET_PRIM = 0U, CAN_NET_SEC, CAN_NO_NET, NUM_CAN_NET } CAN_NetTypeDef;

#define fifoLength       100
#define C_CANRXFIFO_SIZE 20

typedef struct fifoDataType {
    uint32_t id;
    uint32_t size;
    uint8_t data[8];
} fifoDataType;

typedef struct fifoCanDataType {
    uint8_t rxHead;
    uint8_t rxTail;

    uint8_t txHead;
    uint8_t txTail;

    fifoDataType rx[fifoLength];
    fifoDataType tx[fifoLength];

} fifoCanDataType;

typedef struct {
    int tx_size;  // size of data
    int rx_size;
    int rx_size_int;

    uint8_t dataTx[8];
    uint8_t dataRx[8];
    uint8_t dataRX_int[8];
    uint8_t dataTxBck[8];

    uint32_t tx_id;
    uint32_t rx_id;
    uint32_t rx_id_int;
    uint32_t idBck;
    uint32_t sizeBck;

    CAN_HandleTypeDef *hcan;
    CAN_FilterTypeDef canFilter;

    HAL_StatusTypeDef configFilter_status;
    HAL_StatusTypeDef activateNotif_status;
    HAL_StatusTypeDef canStart_status;

    fifoCanDataType fifo;

    IRQn_Type rx0_interrupt;
    IRQn_Type tx_interrupt;

} canStruct;

typedef uint16_t CAN_MsgNameTypeDef;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN3_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief  This function starts and initializes PRIMARY and SECONDARY can
 *         peripherals for the BMS_LV.
 *         After this function bms_lv is able to receive and send on canBUS 
 *         to other nodes.
 * @return True if all can pripherals started normally otherwise false
 */
bool CAN_start_all();

/**
     * @brief     Send a particular message on primary Canbus
     *
     * @param     can_msg_name  The name of the message to send //TODO add group
     *            of this type
     * @return    COMM_StatusTypeDef COMM_OK if message sent COMM_ERROR if
     *            message was not sent
     */
COMM_StatusTypeDef CAN_Send(CAN_MsgNameTypeDef can_msg_name);

uint8_t fifoRxDataCAN_pop(canStruct *);
uint8_t fifoRxDataCAN_push(canStruct *);
uint8_t fifoTxDataCAN_pop(canStruct *);
uint8_t fifoTxDataCAN_push(canStruct *);
bool CAN_Read_Message();

uint8_t old_CAN_Send(canStruct *);
uint8_t old_CAN_Send_IT(canStruct *can);
bool old_CAN_Send_data();
void CAN_id_tostring(CAN_NetTypeDef net, uint32_t msg_id, char buf[static 50]);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
