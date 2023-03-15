/**
 * @file health_signals.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "health_signals.h"

#include <stdio.h>
health_signals_t *ptr_hs;
health_signals_t hs;
void health_test() {
    health_signals_t hss;
    health_signals_t *ptr_hss;
    hss.battery_voltage_out = 1;
    uint8_t val             = *((uint8_t *)ptr_hss);
    volatile bool status    = health_is_fault(ptr_hss);
    hss.battery_current     = 1;
    status                  = health_is_fault(ptr_hss);
}
/**
 * @brief Every Scenario is explained in Relay_scenarios.xlsx (G Drive)
 * Structure:
 * Offset [1] | Offset [0] | Sign(I_Bat) | I_Bat | I_Charger | V_Bat | Relay_Out | LVMS_Out
 * NOTE: The order of the scenario is important!
 * e.g. 0b000010 means that only the Relay_Out is 1
 * 
 */
uint8_t not_fault_scenarios[NOT_FAULT_SCENARIOS_LENGTH] = {
    0b000010,
    0b000100,
    0b001011,
    0b001110,
    0b001111,
    0b100010,
    0b100100,
    0b101011,
    0b101110,
    0b101111,
    0b011110,
    0b011111,
    0b110100,
    0b110110,
    0b110111,
    0b111111};

void health_init(health_signals_t *hs) {
    hs->sign_battery_current = 0;
    hs->battery_current      = 0;
    hs->charger_current      = 0;
    hs->battery_voltage_out  = 0;
    hs->relay_out            = 0;
    hs->lvms_out             = 0;
    hs->offset               = 0;
}
/**
 * @brief Check the system health in order to detect faults
 * 
 * @param hs   Health signals struct 
 * @return true If there is a fault 
 * @return false If there is not a fault
 */
bool health_is_fault(health_signals_t *hs) {
    bool is_fault = true;
    for (uint8_t i = 0; i < NOT_FAULT_SCENARIOS_LENGTH; i++) {
        if (*(uint8_t *)hs == not_fault_scenarios[i]) {
            is_fault = false;
            break;
        }
    }
    return is_fault;
}

void health_set_sign_battery_current(health_signals_t *hs, float current) {
    current < 0 ? (hs->sign_battery_current = 0) : (hs->sign_battery_current = 1);
}
void health_set_battery_current(health_signals_t *hs, float current) {
    current < CURRENT_THRESHOLD_A ? (hs->battery_current = 0) : (hs->battery_current = 1);
}
void health_set_charger_current(health_signals_t *hs, float current) {
    current < CURRENT_THRESHOLD_A ? (hs->charger_current = 0) : (hs->charger_current = 1);
}
void health_set_battery_voltage(health_signals_t *hs, float voltage) {
    voltage < MIN_VOLTAGE_V ? (hs->battery_voltage_out = 0) : (hs->battery_voltage_out = 1);
}
void health_set_relay_out(health_signals_t *hs, float voltage) {
    voltage < MIN_RELAY_VOLTAGE_V ? (hs->relay_out = 0) : (hs->relay_out = 1);
}
void health_set_lvms_out(health_signals_t *hs, float voltage) {
    voltage < MIN_LVMS_VOLTAGE_V ? (hs->lvms_out = 0) : (hs->lvms_out = 1);
}
void health_print(health_signals_t *hs) {
    printf("%u", *(uint8_t *)hs);
}
