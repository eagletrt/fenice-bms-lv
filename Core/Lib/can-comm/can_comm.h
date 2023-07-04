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
#define primary_NETWORK_IMPLEMENTATION
#include "../can-lib/lib/primary/primary_network.h"
#include "can.h"
#include "health_signals.h"

#define CAN_SLAVE_START_FILTER_BANK 14

#define CAN_WAIT(C)                                       \
    {                                                     \
        uint32_t tick = HAL_GetTick();                    \
        while (HAL_CAN_GetTxMailboxesFreeLevel(C) == 0) { \
            if (HAL_GetTick() > tick + 10)                \
                return HAL_TIMEOUT;                       \
        }                                                 \
    }
#define OPEN_BLT_TIME_SET_TIMEOUT_MS 3000
typedef struct {
    uint8_t is_flash_requested;
    uint8_t is_flash_available;
    uint8_t is_time_set_pin_on;
    uint8_t is_time_set_pin_timeout_elapsed;
    uint32_t time_set_initial_time_ms;
    uint32_t time_set_timeout_ms;
    uint8_t charging_done;
    uint8_t state;
} open_blt_status_t;

extern open_blt_status_t open_blt_status;
extern primary_lv_status_t lv_status;
HAL_StatusTypeDef can_send(CAN_HandleTypeDef *hcan, uint8_t *buffer, CAN_TxHeaderTypeDef *header);

/**
 * @brief Function to send a message to the primary network
 * 
 * @param id Message ID
 * @param optional_offset Used to send temperatures or cells voltage, necessary if the message is longer than 8 bytes
 * If not used, set to 0.
 * eg: index 0 -> voltage 0-2, index 1 -> voltage 3-5.
 * 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef can_primary_send(uint16_t id, uint8_t optional_offset);

/**
 * @brief Function to send a message to the secondary network 
 * 
 * @param id Mesage ID
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef can_secondary_send(uint16_t id);
// extern primary_message_SET_RADIATOR_SPEED_conversion rads_speed_msg;
// extern primary_message_SET_PUMPS_SPEED_conversion pumps_speed_msg;
void can_primary_init();
void can_secondary_init();
void open_blt_status_update(health_signals_t *hs, open_blt_status_t *obs);
/**
 * @brief Populate the struct with all feedbacks needed for cansend routine 
 * 
 */

void update_can_feedbacks();
#endif