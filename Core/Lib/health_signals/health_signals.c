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

#include "usart.h"

#include <stdio.h>

health_signals_t hs;
void health_test() {
    health_signals_t hss;
    hss.battery_voltage_out = 1;
    volatile bool status    = health_check_fault(&hss);
    hss.battery_current     = 1;
    status                  = health_check_fault(&hss);
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
    0b110111,
    0b111111};

void health_init(health_signals_t *ptr_hs) {
    ptr_hs->sign_battery_current = 0;
    ptr_hs->battery_current      = 0;
    ptr_hs->charger_current      = 0;
    ptr_hs->battery_voltage_out  = 0;
    ptr_hs->relay_out            = 0;
    ptr_hs->lvms_out             = 0;
    ptr_hs->offset               = 0;
}
/**
 * @brief Check the system health in order to detect faults
 * 
 * @param hs   Health signals struct 
 * @return true If there is a fault 
 * @return false If there is not a fault
 */
bool health_check_fault(health_signals_t *hs) {
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
    current < MIN_BATTERY_CURRENT_THRESHOLD_mA ? (hs->battery_current = 0) : (hs->battery_current = 1);
}
void health_set_charger_current(health_signals_t *hs, float current) {
    current < MIN_CHARGER_CURRENT_THRESHOLD_mA ? (hs->charger_current = 0) : (hs->charger_current = 1);
}
void health_set_battery_voltage(health_signals_t *hs, float voltage) {
    voltage < MIN_BATTERY_VOLTAGE_mV ? (hs->battery_voltage_out = 0) : (hs->battery_voltage_out = 1);
}
void health_set_relay_out(health_signals_t *hs, float voltage) {
    voltage < MIN_RELAY_VOLTAGE_mV ? (hs->relay_out = 0) : (hs->relay_out = 1);
}
void health_set_lvms_out(health_signals_t *hs, float voltage) {
    voltage < MIN_LVMS_VOLTAGE_mV ? (hs->lvms_out = 0) : (hs->lvms_out = 1);
}
void health_print(health_signals_t *hs) {
    printf("%u", *(uint8_t *)hs);
}

void update_health_status(health_signals_t *ptr_hs, ADC_converted *adcs_converted_values) {
    health_set_lvms_out(ptr_hs, adcs_converted_values->lvms_out);
    health_set_relay_out(ptr_hs, adcs_converted_values->relay_out);
    health_set_battery_voltage(ptr_hs, adcs_converted_values->batt_out);
    health_set_charger_current(ptr_hs, adcs_converted_values->mux_hall.S_HALL2);
    health_set_battery_current(ptr_hs, adcs_converted_values->mux_hall.S_HALL1);
    health_set_sign_battery_current(ptr_hs, adcs_converted_values->mux_hall.S_HALL1);
    is_bms_on_fault = health_check_fault(ptr_hs);
#ifdef DEBUG
    if (is_bms_on_fault) {
        printl("BMS is on fault\n", ERR_HEADER);
    }
#endif
}