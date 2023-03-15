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

#include <inttypes.h>
#include <stdbool.h>

#define PACKED                     __attribute__((packed))
#define NOT_FAULT_SCENARIOS_LENGTH 16
#define CURRENT_THRESHOLD_A        0.001
#define MIN_VOLTAGE_V              0.1
#define MIN_RELAY_VOLTAGE_V        0.1
#define MIN_LVMS_VOLTAGE_V         0.1

typedef struct PACKED {
    uint8_t lvms_out : 1;              // 0 no voltage after LVMS, 1 voltage after LVMS
    uint8_t relay_out : 1;             // 0 no voltage after Relay, 1 voltage after Relay
    uint8_t battery_voltage_out : 1;   // 0 is above threshold, 1 is below threshold - threshold: MIN_VOLTAGE
    uint8_t charger_current : 1;       // 0 is not charging, 1 is charging
    uint8_t battery_current : 1;       // 0 is not flowing current, 1 is flowing current
    uint8_t sign_battery_current : 1;  // O is negative, 1 is positive
    uint8_t offset : 2;                // 2 bits offset to make it 1 byte long
} health_signals_t;

extern health_signals_t *ptr_hs;
extern uint8_t not_fault_scenarios[NOT_FAULT_SCENARIOS_LENGTH];
bool health_is_fault(health_signals_t *hs);
void health_init(health_signals_t *hs);
void health_set_sign_battery_current(health_signals_t *hs, float current);
void health_set_battery_current(health_signals_t *hs, float current);
void health_set_charger_current(health_signals_t *hs, float current);
void health_set_battery_voltage(health_signals_t *hs, float voltage);
void health_set_relay_out(health_signals_t *hs, float voltage);
void health_set_lvms_out(health_signals_t *hs, float voltage);
void health_print(health_signals_t *hs);