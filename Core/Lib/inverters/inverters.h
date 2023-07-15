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

typedef enum { INV_OFF, INV_RFE_ON, INV_ON } INV_STATUS;

/**
 * @brief Inverters struct
 * uint8_t comm_status: current inverters comm_status (on/off)
 */
typedef struct {
    uint8_t comm_status;
    uint8_t rfe_pin;
    uint8_t frg_pin;
    uint8_t discharge_pin;
    INV_STATUS inv_status;
    uint32_t last_cmd_timestamp;
    uint32_t rfe_on_timestamp;
    float inv_temps[2];    // 0: left, 1: right
    float motor_temps[2];  // 0: left, 1: right

} Inverters_struct;

extern Inverters_struct car_inverters;

static inline void init_inverter_struct(Inverters_struct *inv) {
    inv->comm_status        = PRIMARY_SET_INVERTER_CONNECTION_STATUS_STATUS_OFF_CHOICE;
    inv->inv_status         = INV_OFF;
    inv->last_cmd_timestamp = 0;
    inv->rfe_on_timestamp   = 0;
    inv->discharge_pin      = 0;
    inv->rfe_pin            = 0;
    inv->frg_pin            = 0;
}

/**
 * @brief Set the inverter comm_status before the latch
 * 
 * @param inv 
 * @param comm_status ON/OFF
 */
static inline void set_inverter_status(Inverters_struct *inv, uint8_t comm_status) {
    inv->comm_status        = comm_status;
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
    if (HAL_GetTick() - inv->last_cmd_timestamp > 450) {
        inv->comm_status = primary_set_inverter_connection_status_status_OFF;
    }

    if (inv->comm_status == primary_set_inverter_connection_status_status_ON) {
        if (inv->inv_status == INV_OFF) {
            mcp23017_set_gpio(&hmcp, MCP23017_PORTB, RFE_EN, GPIO_PIN_SET);
            inv->inv_status       = INV_RFE_ON;
            inv->rfe_on_timestamp = HAL_GetTick();
        } else if (inv->inv_status == INV_RFE_ON && (HAL_GetTick() - inv->rfe_on_timestamp > 500)) {
            mcp23017_set_gpio(&hmcp, MCP23017_PORTB, FRG_EN, GPIO_PIN_SET);
            inv->inv_status = INV_ON;
        }

    } else if (inv->comm_status == primary_set_inverter_connection_status_status_OFF && inv->inv_status != INV_OFF) {
        mcp23017_set_gpio(&hmcp, MCP23017_PORTB, RFE_EN, GPIO_PIN_RESET);
        HAL_Delay(500);
        mcp23017_set_gpio(&hmcp, MCP23017_PORTB, FRG_EN, GPIO_PIN_RESET);
        mcp23017_set_gpio(&hmcp, MCP23017_PORTB, DISCHARGE, GPIO_PIN_RESET);
        inv->inv_status = INV_OFF;
    }
    if (inv->discharge_pin) {
        mcp23017_set_gpio(&hmcp, MCP23017_PORTB, DISCHARGE, GPIO_PIN_SET);
    } else {
        mcp23017_set_gpio(&hmcp, MCP23017_PORTB, DISCHARGE, GPIO_PIN_RESET);
    }
}

static inline void error_state_inverters(Inverters_struct *inv) {
    set_inverter_status(inv, primary_set_inverter_connection_status_status_OFF);
    inverters_loop(inv);
}
static inline void set_inv_motor_temps(Inverters_struct *inv, float temp, uint8_t inv_index) {
    inv->motor_temps[inv_index] = temp;
}

static inline void set_inv_igbt_temps(Inverters_struct *inv, float temp, uint8_t inv_index) {
    inv->inv_temps[inv_index] = temp;
}

static inline void inv_motor_temp_conversion(
    Inverters_struct *inv,
    uint16_t motor_temp,
    uint8_t inv_index) {
    motor_temp = (motor_temp - 9393.9f) / 55.1f;
    set_inv_motor_temps(inv, motor_temp, inv_index);
}

static inline void inv_igbt_temp_conversion(Inverters_struct *inv, uint16_t inv_temp, uint8_t inv_index) {
    float igbt_temp = inv_temp * 0.005f - 38.0f;
    set_inv_igbt_temps(inv, igbt_temp, inv_index);
}



#endif