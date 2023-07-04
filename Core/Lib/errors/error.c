/**
 * @file		error.c
 * @brief		This file contains the functions to handle errors.
 *
 * @date		May 1, 2019
 * @author	Matteo Bonora [matteo.bonora@studenti.unitn.it]
 */

#include "error.h"

#include "fenice-config.h"
#include "tim.h"
#include "usart.h"

#include <stdio.h>
#include <stdlib.h>

primary_lv_errors_converted_t primary_lv_errors;
/**
 * Reaction times by the rules:
 * 	- 500ms for voltages and current
 * 	- 1s for temperatures
 */
enum {
    TIMEOUT_CELL_UNDERVOLTAGE      = 400,
    TIMEOUT_CELL_OVERVOLTAGE       = 400,
    TIMEOUT_OPEN_WIRE              = 400,
    TIMEOUT_OVER_CURRENT           = 400,
    TIMEOUT_CELL_UNDER_TEMPERATURE = 1000,
    TIMEOUT_CELL_OVER_TEMPERATURE  = 1000,
    TIMEOUT_RELAY                  = SOFT,
    TIMEOUT_BMS_MONITOR            = 500,
    TIMEOUT_VOLTAGES_NOT_READY     = 500,
    TIMEOUT_MCP23017               = 1000,
    TIMEOUT_CAN                    = 500,
    TIMEOUT_SPI                    = 500,
    TIMEOUT_RADIATOR               = SOFT,
    TIMEOUT_FAN                    = SOFT,
    TIMEOUT_PUMP                   = SOFT,
    TIMEOUT_ADC_INIT               = 1000,
    TIMEOUT_ADC_MUX                = 500
};

const char *error_names[ERROR_NUM_ERRORS] = {
    [ERROR_RELAY]                  = "Open relay",
    [ERROR_BMS_MONITOR]            = "Bms monitor",
    [ERROR_VOLTAGES_NOT_READY]     = "Voltages not ready to be read (ltc6811_placd error)",
    [ERROR_OPEN_WIRE]              = "Open wire",
    [ERROR_CELL_UNDERVOLTAGE]      = "Cell under voltage",
    [ERROR_CELL_OVERVOLTAGE]       = "Cell over voltage",
    [ERROR_MCP23017]               = "Feedback chip",
    [ERROR_CAN]                    = "Can comm error",
    [ERROR_SPI]                    = "Spi comm error",
    [ERROR_RADIATOR]               = "Radiator",
    [ERROR_FAN]                    = "Fan",
    [ERROR_PUMP]                   = "Pump",
    [ERROR_ADC_INIT]               = "Adc init",
    [ERROR_CELL_UNDER_TEMPERATURE] = "Cell under temp",
    [ERROR_CELL_OVER_TEMPERATURE]  = "Cell over temp",
    [ERROR_OVER_CURRENT]           = "Over current",
    [ERROR_ADC_MUX]                = "Adc multiplexer"};

ERROR_UTILS_InstanceTypeDef cell_undervoltage_error[LV_CELLS_COUNT];
ERROR_UTILS_InstanceTypeDef cell_overvoltage_error[LV_CELLS_COUNT];
ERROR_UTILS_InstanceTypeDef open_wire_error[OPEN_WIRE_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef can_error[CAN_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef spi_error[SPI_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef over_current_error[OVER_CURRENT_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef cell_over_temperature_error[NTC_COUNT];
ERROR_UTILS_InstanceTypeDef cell_under_temperature_error[NTC_COUNT];
ERROR_UTILS_InstanceTypeDef relay_error[RELAY_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef bms_monitor_error[BMS_MONITOR_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef voltage_ready_error[VOLTAGE_READY_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef mcp23017_error[MCP23017_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef radiator_error[RADIATOR_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef fan_error[FAN_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef pump_error[PUMP_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef adc_error[ADC_ERROR_INSTANCES];
ERROR_UTILS_InstanceTypeDef adc_mux_error[ADC_ERROR_INSTANCES];
uint8_t fatal_error[ERROR_NUM_ERRORS] = {0};

ERROR_UTILS_ErrorTypeDef lv_errors[] = {
    {.expiry_delay_ms  = TIMEOUT_CELL_UNDERVOLTAGE,
     .instances        = cell_undervoltage_error,
     .instances_length = LV_CELLS_COUNT},  //ERROR_CELL_UNDERVOLTAGE

    {.expiry_delay_ms  = TIMEOUT_CELL_OVERVOLTAGE,
     .instances        = cell_overvoltage_error,
     .instances_length = LV_CELLS_COUNT},  //ERROR_CELL_OVERVOLTAGE

    {.expiry_delay_ms  = TIMEOUT_OPEN_WIRE,
     .instances        = open_wire_error,
     .instances_length = OPEN_WIRE_ERROR_INSTANCES},  //ERROR_OPEN_WIRE

    {.expiry_delay_ms = TIMEOUT_CAN, .instances = can_error, .instances_length = CAN_ERROR_INSTANCES},  //ERROR_CAN

    {.expiry_delay_ms = TIMEOUT_SPI, .instances = spi_error, .instances_length = SPI_ERROR_INSTANCES},  //ERROR_SPI

    {.expiry_delay_ms  = TIMEOUT_OVER_CURRENT,
     .instances        = over_current_error,
     .instances_length = OVER_CURRENT_ERROR_INSTANCES},  //ERROR_OVER_CURRENT

    {.expiry_delay_ms  = TIMEOUT_CELL_UNDER_TEMPERATURE,
     .instances        = cell_under_temperature_error,
     .instances_length = NTC_COUNT},  //ERROR_CELL_UNDER_TEMPERATURE

    {.expiry_delay_ms  = TIMEOUT_CELL_OVER_TEMPERATURE,
     .instances        = cell_over_temperature_error,
     .instances_length = NTC_COUNT},  //ERROR_CELL_OVER_TEMPERATURE

    {.expiry_delay_ms  = TIMEOUT_RELAY,
     .instances        = relay_error,
     .instances_length = RELAY_ERROR_INSTANCES},  //ERROR_RELAY

    {.expiry_delay_ms  = TIMEOUT_BMS_MONITOR,
     .instances        = bms_monitor_error,
     .instances_length = BMS_MONITOR_ERROR_INSTANCES},  //ERROR_BMS_MONITOR

    {.expiry_delay_ms  = TIMEOUT_VOLTAGES_NOT_READY,
     .instances        = voltage_ready_error,
     .instances_length = VOLTAGE_READY_ERROR_INSTANCES},  //ERROR_VOLTAGES_NOT_READY

    {.expiry_delay_ms  = TIMEOUT_MCP23017,
     .instances        = mcp23017_error,
     .instances_length = MCP23017_ERROR_INSTANCES},  //ERROR_MCP23017

    {.expiry_delay_ms  = TIMEOUT_RADIATOR,
     .instances        = radiator_error,
     .instances_length = RADIATOR_ERROR_INSTANCES},  //ERROR_RADIATOR

    {.expiry_delay_ms = TIMEOUT_FAN, .instances = fan_error, .instances_length = FAN_ERROR_INSTANCES},  //ERROR_FAN

    {.expiry_delay_ms = TIMEOUT_PUMP, .instances = pump_error, .instances_length = PUMP_ERROR_INSTANCES},  //ERROR_PUMP

    {.expiry_delay_ms  = TIMEOUT_ADC_INIT,
     .instances        = adc_error,
     .instances_length = ADC_ERROR_INSTANCES},  //ERROR_ADC_INIT

    {.expiry_delay_ms  = TIMEOUT_ADC_MUX,
     .instances        = adc_mux_error,
     .instances_length = ADC_ERROR_INSTANCES}  //ERROR_ADC_INIT
};

ERROR_UTILS_HandleTypeDef error_handler;

void error_init() {
    error_handler.timer                  = &HTIM_ERR;
    error_handler.errors                 = lv_errors;
    error_handler.errors_length          = ERROR_NUM_ERRORS;
    error_handler.global_expiry_callback = &bms_error_callback;
}

void bms_error_callback(size_t error_index, size_t instance_index) {
    // printl("ERROR TIM ELAPSED\r\n", NOTHING);
    //bms_error_state();
    // if (!fatal_error[error_index])
    set_error_bitset(error_index, 1);
    fatal_error[error_index] = instance_index + 1;
    lv_status.status         = PRIMARY_LV_STATUS_STATUS_ERROR_CHOICE;
}

void set_warning_bitset(error_id type, uint8_t val) {
    switch (type) {
        case ERROR_CELL_UNDERVOLTAGE:
            primary_lv_errors.warnings_cell_undervoltage = val;
            break;
        case ERROR_CELL_OVERVOLTAGE:
            primary_lv_errors.warnings_cell_overvoltage = val;
            break;
        case ERROR_OPEN_WIRE:
            primary_lv_errors.warnings_battery_open_wire = val;
            break;
        case ERROR_CAN:
            primary_lv_errors.warnings_can = val;
            break;
        case ERROR_SPI:
            primary_lv_errors.warnings_spi = val;
            break;
        case ERROR_OVER_CURRENT:
            primary_lv_errors.warnings_over_current = val;
            break;
        case ERROR_CELL_UNDER_TEMPERATURE:
            primary_lv_errors.warnings_cell_under_temperature = val;
            break;
        case ERROR_CELL_OVER_TEMPERATURE:
            primary_lv_errors.warnings_cell_over_temperature = val;
            break;
        case ERROR_RELAY:
            primary_lv_errors.warnings_relay = val;
            break;
        case ERROR_BMS_MONITOR:
            primary_lv_errors.warnings_bms_monitor = val;
            break;
        case ERROR_VOLTAGES_NOT_READY:
            primary_lv_errors.warnings_voltages_not_ready = val;
            break;
        case ERROR_MCP23017:
            primary_lv_errors.warnings_mcp23017 = val;
            break;
        case ERROR_RADIATOR:
            primary_lv_errors.warnings_radiator = val;
            break;
        case ERROR_FAN:
            primary_lv_errors.warnings_fan = val;
            break;
        case ERROR_PUMP:
            primary_lv_errors.warnings_pump = val;
            break;
        case ERROR_ADC_INIT:
            primary_lv_errors.warnings_adc_init = val;
            break;
        case ERROR_ADC_MUX:
            primary_lv_errors.warnings_mux = val;
            break;
        default:
            break;
    }
}

void set_error_bitset(error_id type, uint8_t val) {
    switch (type) {
        case ERROR_CELL_UNDERVOLTAGE:
            primary_lv_errors.errors_cell_undervoltage = val;
            break;
        case ERROR_CELL_OVERVOLTAGE:
            primary_lv_errors.errors_cell_overvoltage = val;
            break;
        case ERROR_OPEN_WIRE:
            primary_lv_errors.errors_battery_open_wire = val;
            break;
        case ERROR_CAN:
            primary_lv_errors.errors_can = val;
            break;
        case ERROR_SPI:
            primary_lv_errors.errors_spi = val;
            break;
        case ERROR_OVER_CURRENT:
            primary_lv_errors.errors_over_current = val;
            break;
        case ERROR_CELL_UNDER_TEMPERATURE:
            primary_lv_errors.errors_cell_under_temperature = val;
            break;
        case ERROR_CELL_OVER_TEMPERATURE:
            primary_lv_errors.errors_cell_over_temperature = val;
            break;
        case ERROR_RELAY:
            primary_lv_errors.errors_relay = val;
            break;
        case ERROR_BMS_MONITOR:
            primary_lv_errors.errors_bms_monitor = val;
            break;
        case ERROR_VOLTAGES_NOT_READY:
            primary_lv_errors.errors_voltages_not_ready = val;
            break;
        case ERROR_MCP23017:
            primary_lv_errors.errors_mcp23017 = val;
            break;
        case ERROR_RADIATOR:
            primary_lv_errors.errors_radiator = val;
            break;
        case ERROR_FAN:
            primary_lv_errors.errors_fan = val;
            break;
        case ERROR_PUMP:
            primary_lv_errors.errors_pump = val;
            break;
        case ERROR_ADC_INIT:
            primary_lv_errors.errors_adc_init = val;
            break;
        case ERROR_ADC_MUX:
            primary_lv_errors.errors_mux = val;
            break;
        default:
            break;
    }
}
void error_set(error_id type, uint8_t offset) {
#ifdef NDEBUG
    char error_buff[50] = {};
    sprintf(error_buff, "ADD ERROR ID: %i-%u, (%s)", type, offset, error_names[type]);
    printl(error_buff, NO_HEADER);
#endif
    set_warning_bitset(type, 1);
    error_utils_error_set(&error_handler, type, offset);
}

void error_reset(error_id type, uint8_t offset) {
#ifdef NDEBUG
    char error_buff[50] = {};
    sprintf(error_buff, "REMOVE ERROR ID: %i-%u, (%s)", type, offset, error_names[type]);
    printl(error_buff, NO_HEADER);
#endif
    set_warning_bitset(type, 0);
    error_utils_error_reset(&error_handler, type, offset);
}

void error_dump(error_t errors[], size_t *error_count) {
    size_t dump_index   = 0;
    bool error_inserted = false;
    for (size_t i = 0; error_utils_get_count(&error_handler) > 0 && i < ERROR_NUM_ERRORS; i++) {
        ERROR_UTILS_ErrorTypeDef *error = &error_handler.errors[i];
        for (size_t j = 0; error_utils_get_count(&error_handler) > 0 && j < error->instances_length; j++) {
            ERROR_UTILS_InstanceTypeDef *instance = &error->instances[j];
            if ((instance->is_triggered && !error_inserted) || (fatal_error[i] - 1 == j)) {
                error_t error_d = {
                    .id        = i,
                    .state     = STATE_WARNING,
                    .timestamp = instance->expected_expiry_ms - error->expiry_delay_ms};
                if (fatal_error[i]) {
                    error_d.state = STATE_FATAL;
                }
                errors[dump_index] = error_d;
                error_inserted     = true;
                dump_index++;
            }
        }
        error_inserted = false;
    }

    *error_count = dump_index;
}