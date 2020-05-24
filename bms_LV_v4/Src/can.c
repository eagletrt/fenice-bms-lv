#include "can.h"

canStruct can1, can3;
extern CAN_HandleTypeDef hcan1;

bool can_init()
{
	bool ret = false;
	if (CAN_initialization(&can1))
	{
		ret = true;
		//report_error_can1();
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
	can->canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	can->canFilter.FilterIdLow = 0;
	can->canFilter.FilterIdHigh = 0;
	can->canFilter.FilterMaskIdHigh = 0;
	can->canFilter.FilterMaskIdLow = 0;
	can->canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can->canFilter.FilterBank = 0;
	can->canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	can->canFilter.FilterActivation = ENABLE;

	// CAN filter configuration
	can->configFilter_status = HAL_CAN_ConfigFilter(can->hcan, &can->canFilter);

	can->activateNotif_status = HAL_CAN_ActivateNotification(can->hcan, can->rx0_interrupt);
	can->activateNotif_status = HAL_CAN_ActivateNotification(can->hcan, can->tx_interrupt);

	can->fifo.rxHead = 0;
	can->fifo.rxTail = 0;
	can->fifo.txHead = 0;
	can->fifo.txTail = 0;

	// CAN start
	can->canStart_status = HAL_CAN_Start(can->hcan);

	if (can->configFilter_status == HAL_OK && can->activateNotif_status == HAL_OK && can->canStart_status == HAL_OK)
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
	can->idBck = can->tx_id;
	can->sizeBck = can->tx_size;

	uint8_t flag = 0; // error

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = can->tx_id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = can->tx_size;
	TxHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTx, &mailbox) == HAL_OK)
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
		can->rx_id = can->fifo.rx[can->fifo.rxTail].id;
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
		can->fifo.rx[can->fifo.rxHead].id = can->rx_id_int;
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
		can->tx_id = can->fifo.tx[can->fifo.txTail].id;
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
		can->fifo.tx[can->fifo.txHead].id = can->tx_id;
		can->fifo.tx[can->fifo.txHead].size = can->tx_size;
		for (uint8_t i = 0; i < can->tx_size; i++)
		{
			can->fifo.tx[can->fifo.txHead].data[i] = can->dataTx[i];
		}
		can->fifo.txHead = (can->fifo.txHead + 1) % fifoLength;
		return 1;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan1) {
		if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0) {
			CAN_RxHeaderTypeDef header;
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, can1.dataRX_int);
			can1.rx_id_int = header.StdId;
			can1.rx_size_int = header.DLC;
            fifoRxDataCAN_push(&can1);
		}
	}
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == &hcan1) {
		if (fifoTxDataCAN_pop(&can1)) {
			if (CAN_Send_IT(&can1) == 0) {
				// TODO: implementare errore
			}
		}
	}
}