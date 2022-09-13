/**
 * @file inverters.h
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-06-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __INVERTERS_H
#define __INVERTERS_H

#include "can_comm.h"
#include "main.h"

typedef enum { INV_OFF, INV_RFE_ON, INV_ON} INV_STATUS;

/**
 * @brief Inverters struct
 * uint8_t comm_status: current inverters comm_status (on/off)
 */
typedef struct {
    uint8_t comm_status;
    INV_STATUS inv_status;
    uint32_t last_cmd_timestamp;
    uint32_t rfe_on_timestamp;
} Inverters_struct;

extern Inverters_struct car_inverters;

static inline void init_inverter_struct(Inverters_struct *inv) {
    inv->comm_status      = primary_Toggle_OFF;
    inv->inv_status = INV_OFF;
    inv->last_cmd_timestamp = 0;
    inv->rfe_on_timestamp = 0;
}

/**
 * @brief Set the inverter comm_status before the latch
 * 
 * @param inv 
 * @param comm_status ON/OFF
 */
static inline void set_inverter_status(Inverters_struct *inv, uint8_t comm_status) {
    inv->comm_status      = comm_status;
    inv->last_cmd_timestamp = HAL_GetTick();
}

static inline uint8_t get_inverter_status(Inverters_struct *inv) {
    return inv->comm_status;
}

/**
 * @brief Physically close or open the inverters relays and mark them as latched
 * 
 * @param inv 
 */
static inline void inverters_loop(Inverters_struct *inv) {
    if(HAL_GetTick() - inv->last_cmd_timestamp > 450) {
        inv->comm_status = primary_Toggle_OFF;
    }

    if(inv->comm_status == primary_Toggle_ON) {
        if(inv->inv_status == INV_OFF) {
            HAL_GPIO_WritePin(INV_RFE_GPIO_Port, INV_RFE_Pin, GPIO_PIN_SET);
            inv->inv_status = INV_RFE_ON;
            inv->rfe_on_timestamp = HAL_GetTick();
        } else if(inv->inv_status == INV_RFE_ON && (HAL_GetTick() - inv->rfe_on_timestamp > 500)) {
            HAL_GPIO_WritePin(INV_FRG_GPIO_Port, INV_FRG_Pin, GPIO_PIN_SET);
            inv->inv_status = INV_ON;
        }
    } else if (inv->comm_status == primary_Toggle_OFF && inv->inv_status != INV_OFF) {
        HAL_GPIO_WritePin(INV_RFE_GPIO_Port, INV_RFE_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INV_FRG_GPIO_Port, INV_FRG_Pin, GPIO_PIN_RESET);
        inv->inv_status = INV_OFF;
    }
}

static inline void error_state_inverters(Inverters_struct *inv) {
    set_inverter_status(inv, primary_Toggle_OFF);
    inverters_loop(inv);
}

#endif