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

#include "stdbool.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan3;

/* USER CODE BEGIN Private defines */

#define fifoLength 100
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
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN3_Init(void);

/* USER CODE BEGIN Prototypes */

bool can_init();
bool CAN_initialization(canStruct *can);
uint8_t CAN_Send(canStruct *);
uint8_t CAN_Send_IT(canStruct *can);
uint8_t fifoRxDataCAN_pop(canStruct *);
uint8_t fifoRxDataCAN_push(canStruct *);
uint8_t fifoTxDataCAN_pop(canStruct *);
uint8_t fifoTxDataCAN_push(canStruct *);
int CAN_Read_Message();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
