#ifndef __CAN_H__
#define __CAN_H__

#include "stm32f7xx_hal.h"
#include "stdbool.h"

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

bool can_init();
bool CAN_initialization(canStruct *can);
uint8_t CAN_Send(canStruct *);
uint8_t CAN_Send_IT(canStruct *can);
uint8_t fifoRxDataCAN_pop(canStruct *);
uint8_t fifoRxDataCAN_push(canStruct *);
uint8_t fifoTxDataCAN_pop(canStruct *);
uint8_t fifoTxDataCAN_push(canStruct *);
int CAN_Read_Message();
#endif
