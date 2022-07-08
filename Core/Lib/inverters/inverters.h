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

/**
 * @brief Inverters struct
 * uint8_t status: current inverters status (on/off)
 * bool are_latched: the inverters are already set in the scheduled status?
 */
typedef struct {
    uint8_t status;
    bool are_latched;
} Inverters_struct;

extern Inverters_struct car_inverters;

static inline void init_inverter_struct(Inverters_struct *inv) {
    inv->status      = primary_Toggle_OFF;
    inv->are_latched = false;
}

/**
 * @brief Set the inverter status before the latch
 * 
 * @param inv 
 * @param status ON/OFF
 */
static inline void set_inverter_status(Inverters_struct *inv, uint8_t status) {
    inv->status      = status;
    inv->are_latched = false;
}

static inline uint8_t get_inverter_status(Inverters_struct *inv) {
    return inv->status;
}

/**
 * @brief Physically close or open the inverters relays and mark them as latched
 * 
 * @param inv 
 */
static inline void latch_inverters(Inverters_struct *inv) {
    if (inv->status == primary_Toggle_ON) {
        HAL_GPIO_WritePin(INV_RFE_GPIO_Port, INV_RFE_Pin, GPIO_PIN_SET);
        HAL_Delay(1500);  // works
        HAL_GPIO_WritePin(INV_FRG_GPIO_Port, INV_FRG_Pin, GPIO_PIN_SET);
        inv->are_latched = true;
    } else if (inv->status == primary_Toggle_OFF) {
        HAL_GPIO_WritePin(INV_RFE_GPIO_Port, INV_RFE_Pin, GPIO_PIN_RESET);
        HAL_Delay(1500);  // works
        HAL_GPIO_WritePin(INV_FRG_GPIO_Port, INV_FRG_Pin, GPIO_PIN_RESET);
        inv->are_latched = true;
    }
}

static inline void error_state_inverters(Inverters_struct *inv) {
    set_inverter_status(inv, primary_Toggle_OFF);
    latch_inverters(inv);
}
#endif