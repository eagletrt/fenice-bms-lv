/**
 * @file can_comm.h
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-05-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef CAN_COMM_H__
#define CAN_COMM_H__

#include "../can-lib/lib/primary/c/ids.h"
#include "../can-lib/lib/primary/c/network.h"
#include "can.h"

#define CAN_SLAVE_START_FILTER_BANK 14

#define CAN_WAIT(C)                                       \
    {                                                     \
        uint32_t tick = HAL_GetTick();                    \
        while (HAL_CAN_GetTxMailboxesFreeLevel(C) == 0) { \
            if (HAL_GetTick() > tick + 10)                \
                return HAL_TIMEOUT;                       \
        }                                                 \
    }
HAL_StatusTypeDef can_send(CAN_HandleTypeDef *hcan, uint8_t *buffer, CAN_TxHeaderTypeDef *header);
HAL_StatusTypeDef can_primary_send(uint16_t id);
HAL_StatusTypeDef can_secondary_send(uint16_t id);
extern primary_message_SET_RADIATOR_SPEED_conversion rads_speed_msg;
extern primary_message_SET_PUMPS_SPEED_conversion pumps_speed_msg;
void can_primary_init();
void can_secondary_init();
#endif