/**
 * @file health_signals.h
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2023-03-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef HEALTH_SIGNALS_H__
#define HEALTH_SIGNALS_H__

#include "adc.h"

#include <inttypes.h>
#include <stdbool.h>

#define PACKED                           __attribute__((packed))
#define NOT_FAULT_SCENARIOS_LENGTH       14
#define MIN_CHARGER_CURRENT_THRESHOLD_mA 4000.0f
#define MIN_BATTERY_CURRENT_THRESHOLD_mA 2500.0f
#define MIN_BATTERY_VOLTAGE_mV           3300.0 * 6.0f
// Min difference threshold between V Relay and V Battery
#define MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV 2000.0f  // diff v relay (that could be the charger one and bat out)
// Min difference threshold between LVMS out V ans V Relay
#define MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV 2000.0f  // diff lvms out and relay out 5%

typedef struct PACKED {
    uint8_t lvms_out : 1;              // 0 no voltage after LVMS, 1 voltage after LVMS
    uint8_t relay_out : 1;             // 0 no voltage after Relay, 1 voltage after Relay
    uint8_t battery_voltage_out : 1;   // 0 is above threshold, 1 is below threshold - threshold: MIN_VOLTAGE
    uint8_t charger_current : 1;       // 0 is not charging, 1 is charging
    uint8_t battery_current : 1;       // 0 is not flowing current, 1 is flowing current
    uint8_t sign_battery_current : 1;  // O is negative, 1 is positive
    uint8_t offset : 2;                // 2 bits offset to make it 1 byte long
} health_signals_t;

extern health_signals_t hs;
extern uint8_t not_fault_scenarios[NOT_FAULT_SCENARIOS_LENGTH];
bool health_check_fault(health_signals_t *hs);
void health_init(health_signals_t *hs);
void health_set_sign_battery_current(health_signals_t *hs, float current);
void health_set_battery_current(health_signals_t *hs, float current);
void health_set_charger_current(health_signals_t *hs, float current);
void health_set_battery_voltage(health_signals_t *hs, float voltage);
void health_set_relay_out(health_signals_t *hs, float v_relay_mV, float v_batt_mV);
void health_set_lvms_out(health_signals_t *hs, float lvms_out_mV, float v_relay_mV);
void health_print(health_signals_t *hs);
/**
 * @brief Update health status of the board based on the taken measurements
 * 
 */
void update_health_status(health_signals_t *ptr_hs, ADC_converted *adcs_converted_values);
void stocazzo(ADC_converted *adcs_converted_values);
#endif