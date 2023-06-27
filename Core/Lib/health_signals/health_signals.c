/**
 * @file health_signals.c
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
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

/**
 * @brief These relay scenario are the ones that need to be treated 
 * carefully with further some checks.
 * Every Scenario is explained in Relay_scenarios.xlsx (G Drive)
 * Structure:
 * Offset [1] | Offset [0] | Sign(I_Bat) | I_Bat | I_Charger | V_Bat | Relay_Out | LVMS_Out
 * NOTE: The order of the scenario is important!
 * e.g. 0b000010 means that only the Relay_Out is 1
 * 
 */
uint8_t edge_cases[EDGE_CASES_LENGHT] = {0b000110, 0b100110, 0b000111, 0b100111};

// TODO: Remove from the edge cases: 0b000111, 0b100111

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
    bool is_fault           = true;
    bool edge_case_detected = false;
    for (uint8_t i = 0; i < NOT_FAULT_SCENARIOS_LENGTH; i++) {
        if (*(uint8_t *)hs == not_fault_scenarios[i]) {
            is_fault = false;
            break;
        }
    }
    for (uint8_t i = 0; i < EDGE_CASES_LENGHT; i++) {
        if (*(uint8_t *)hs == edge_cases[i]) {
            edge_case_detected = true;
            break;
        }
    }
    if (edge_case_detected) {
        is_fault = false;
        if (*(uint8_t *)hs == edge_cases[0] || *(uint8_t *)hs == edge_cases[1]) {
            // Check relay status and check whether current is flowing from the charger.
            // If relay previously closed and I_chg == 0 -> fault.
            // If relay status != relay closed then chg is connected to battery, relay is burnt -> fault.
            // TODO: Further check needs to be done!
            if (is_relay_closed) {
                // TODO: check this condition
                if (hs->charger_current == 0) {
                    is_fault = true;
                } else if (HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin) != (uint8_t)is_relay_closed) {
                    is_fault = true;
                }
            }
        }
    }
    // Until the LVMS is not closed for the first time
    // The fault check is done in a lighter way
    if (!is_lvms_closed_for_the_first_time) {
        is_fault = false;
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
void health_set_relay_out(health_signals_t *hs, float v_relay_mV, float v_batt_mV) {
    float diff_voltage = v_relay_mV - v_batt_mV;
    diff_voltage < 0 ? (diff_voltage = -diff_voltage) : (diff_voltage = diff_voltage);
    (diff_voltage < MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV && v_relay_mV > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV &&
     v_batt_mV > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV)
        ? (hs->relay_out = 1)
        : (hs->relay_out = 0);
}
void health_set_lvms_out(health_signals_t *hs, float lvms_out_mV, float v_relay_mV) {
    float diff_voltage = v_relay_mV - lvms_out_mV;
    diff_voltage < 0 ? (diff_voltage = -diff_voltage) : (diff_voltage = diff_voltage);
    (diff_voltage < MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV && v_relay_mV > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV &&
     lvms_out_mV > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV)
        ? (hs->lvms_out = 1)
        : (hs->lvms_out = 0);
    if (hs->lvms_out == 1 && !is_lvms_closed_for_the_first_time) {
        is_lvms_closed_for_the_first_time = true;
    }
}
void health_print(health_signals_t *hs) {
    printf("%u", *(uint8_t *)hs);
}

void update_health_status(health_signals_t *ptr_hs, ADC_converted *adcs_converted_values) {
    health_set_lvms_out(ptr_hs, adcs_converted_values->lvms_out, adcs_converted_values->relay_out);
    health_set_relay_out(ptr_hs, adcs_converted_values->relay_out, adcs_converted_values->batt_out);
    health_set_battery_voltage(ptr_hs, adcs_converted_values->batt_out);
    health_set_charger_current(ptr_hs, adcs_converted_values->mux_hall.S_HALL2);
    health_set_battery_current(ptr_hs, adcs_converted_values->mux_hall.S_HALL1);
    health_set_sign_battery_current(ptr_hs, adcs_converted_values->mux_hall.S_HALL1);
    is_bms_on_fault = health_check_fault(ptr_hs);
    if (is_bms_on_fault) {
        printl("Error in Health condition!\r\nBMS is on fault\n", ERR_HEADER);
        char buf[50];
        char out[5000];
        sprintf(buf, "Error code %u\n", *(uint8_t *)ptr_hs);
        printl(buf, ERR_HEADER);
        // sprintf(
        //     out,
        //     "Health signals bitset\r\n"
        //     "LVMS_OUT: %d (1 if V_LVMS - V_RELAY < %.3f)\r\n"
        //     "RELAY_OUT: %d (1 if V_RELAY - V_BATTERY < %.3f)\r\n"
        //     "BATTERY_VOLTAGE_OUT: %d (1 if > %.3f)\r\n"
        //     "CHARGER_CURRENT_OUT: %d (1 if > %.3f)\r\n"
        //     "BATTERY_CURRENT_OUT: %d (1 if > %.3f)\r\n"
        //     "SIGN_BATTERY_CURRENT_OUT: %d (1 if positive)\r\n",
        //     hs.lvms_out,
        //     MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV,
        //     hs.relay_out,
        //     MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV,
        //     hs.battery_voltage_out,
        //     MIN_BATTERY_VOLTAGE_mV,
        //     hs.charger_current,
        //     MIN_BATTERY_CURRENT_THRESHOLD_mA,
        //     hs.battery_current,
        //     MIN_CHARGER_CURRENT_THRESHOLD_mA,
        //     hs.sign_battery_current);
        //printl(out, NO_HEADER);
        volatile uint8_t x = 0;
    }
#ifdef DEBUG
    if (is_bms_on_fault) {
        printl("BMS is on fault\n", ERR_HEADER);
    }
#endif
}