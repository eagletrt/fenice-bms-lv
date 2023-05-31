/**
 * @file		Monitor_int.c
 * @brief		bms monitor interface for voltage reading
 *
 * @date 2021-11-24
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @author Dimitri Corraini (dimitri.corraini@studenti.unitn.it)
 */

#include "monitor_int.h"

#include "cli_bms_lv.h"
#include "error.h"
#include "fenice-config.h"
#include "main.h"
#include "usart.h"
#include "volt.h"

#include <stdio.h>
#include <string.h>

voltage_t voltages[LV_CELLS_COUNT] = {0};
float total_voltage_on_board       = 0.0;
char buff[500];
uint8_t voltage_min_index;
uint8_t volt_status;
float total_voltage_on_board;
LTC6811_HandleTypeDef monitor_handler;

void monitor_init() {
    LTC6811_CFGR config;
    config.ADCOPT             = 0;
    config.DCC                = 0;
    config.DCTO               = 0;
    config.REFON              = 1;
    config.GPIO               = 31;
    monitor_handler.spi       = &hspi2;
    monitor_handler.pin       = LTC_CS_Pin;
    monitor_handler.gpio      = LTC_CS_GPIO_Port;
    monitor_handler.config[0] = config;

    ltc6811_wrcfg(&monitor_handler);
}

uint8_t monitor_get_min_cell() {
    uint8_t min = 0;
    for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
        if (voltages[i] < voltages[min]) {
            min = i;
        }
    }
    return min;
}

uint8_t monitor_read_voltage() {
    volt_status            = VOLT_OK;
    total_voltage_on_board = 0.0;
    VOLT_CHANNEL cells     = VOLT_CHANNEL_ALL;
    if (volt_read(&monitor_handler, LTC6811_MD_7KHZ_3KHZ, LTC6811_DCP_DISABLED, &cells, &voltages, 5) == 1) {
        volt_status = VOLT_ERR;
    } else {
        for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
            // Check whether a cell is undervoltage or overvoltage and set/reset its specific error
            if ((float)voltages[i] / 1000 <= VOLT_MIN_ALLOWED_VOLTAGE) {
                volt_status = VOLT_UNDER_VOLTAGE;
                error_set(ERROR_CELL_UNDERVOLTAGE, i, HAL_GetTick());
            } else if ((float)voltages[i] / 1000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                volt_status = VOLT_OVER_VOLTAGE;
                error_set(ERROR_CELL_OVERVOLTAGE, i, HAL_GetTick());
            } else {
                //cli_bms_debug("SAMPLE AND READ, REMOVED UNDERVOLTAGE", 37);
                if (!is_bms_on_fault) {
                    error_reset(ERROR_CELL_UNDERVOLTAGE, i);
                    error_reset(ERROR_CELL_OVERVOLTAGE, i);
                }
            }
            total_voltage_on_board += (float)voltages[i] / 1000;
        }
        if (total_voltage_on_board < MIN_POWER_ON_VOLTAGE) {
            volt_status = VOLT_UNDER_VOLTAGE;
        }
    }
    return volt_status;
}

uint8_t monitor_print_volt() {
    volt_status            = VOLT_OK;
    voltage_min_index      = -1;
    total_voltage_on_board = 0.0;
    VOLT_CHANNEL cells     = VOLT_CHANNEL_ALL;
    if (volt_read(&monitor_handler, LTC6811_MD_7KHZ_3KHZ, LTC6811_DCP_DISABLED, &cells, &voltages, 5) == 1) {
        printl("LTC ERROR!", ERR_HEADER);
        volt_status = VOLT_ERR;
    } else {
        voltage_min_index = monitor_get_min_cell();
    }
    memset(buff, 0, sizeof(buff));
    for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
        total_voltage_on_board += (float)voltages[i] / 1000;
        if (i == voltage_min_index && voltage_min_index != -1 &&
            (float)voltages[i] / 1000 <= VOLT_MAX_ALLOWED_VOLTAGE &&
            (float)voltages[i] / 1000 >= VOLT_MIN_ALLOWED_VOLTAGE) {
            sprintf(buff, "Cell %u: %.3fV M", i, (float)voltages[i] / 1000);
        } else {
            if ((float)voltages[i] / 1000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                sprintf(buff, "Error! Max allowed voltage exceeded (Cell %u: %.3fV)", i, (float)voltages[i] / 1000);
                error_set(ERROR_CELL_OVERVOLTAGE, i, HAL_GetTick());
                volt_status = VOLT_OVER_VOLTAGE;
                voltages[i] = 0xff;
            } else if ((float)voltages[i] / 1000 <= VOLT_MIN_ALLOWED_VOLTAGE) {
                sprintf(buff, "Error! Min allowed voltage exceeded (Cell %u: %.3fV)", i, (float)voltages[i] / 1000);
                error_set(ERROR_CELL_UNDERVOLTAGE, i, HAL_GetTick());
                volt_status = VOLT_UNDER_VOLTAGE;
                voltages[i] = 0x00;
            } else {
                sprintf(buff, "Cell %u: %.3fV", i, (float)voltages[i] / 1000);
                //cli_bms_debug("READ AND PRINT, REMOVED UNDERVOLTAGE", 36);
                if (!is_bms_on_fault) {
                    error_reset(ERROR_CELL_UNDERVOLTAGE, i);
                    error_reset(ERROR_CELL_OVERVOLTAGE, i);
                }
            }
        }
        printl(buff, NO_HEADER);
    }
    if (total_voltage_on_board < MIN_POWER_ON_VOLTAGE) {
        printl("UNDERVOLTAGE!", ERR_HEADER);
        volt_status = VOLT_UNDER_VOLTAGE;
    }
    return volt_status;
}

uint8_t monitor_print_volt_cli(char *buf) {
    volt_status            = VOLT_OK;
    voltage_min_index      = -1;
    total_voltage_on_board = 0.0;

    voltage_min_index = monitor_get_min_cell();
    memset(buff, 0, sizeof(buff));
    for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
        if (i == voltage_min_index && voltage_min_index != -1 &&
            (float)voltages[i] / 1000 <= VOLT_MAX_ALLOWED_VOLTAGE &&
            (float)voltages[i] / 1000 >= VOLT_MIN_ALLOWED_VOLTAGE) {
            sprintf(buff, "Cell %u: %.3fV M \r\n", i, (float)voltages[i] / 1000);
        } else {
            if ((float)voltages[i] / 1000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                sprintf(
                    buff, "Error! Max allowed voltage exceeded (Cell %u: %.3fV) \r\n", i, (float)voltages[i] / 1000);
                volt_status = VOLT_OVER_VOLTAGE;
                //voltages[i] = 0xff;
            } else if ((float)voltages[i] / 1000 <= VOLT_MIN_ALLOWED_VOLTAGE) {
                sprintf(
                    buff, "Error! Min allowed voltage exceeded (Cell %u: %.3fV) \r\n", i, (float)voltages[i] / 1000);
                error_set(ERROR_CELL_UNDERVOLTAGE, i, HAL_GetTick());
                volt_status = VOLT_UNDER_VOLTAGE;
                //voltages[i] = 0x00;
            } else {
                sprintf(buff, "Cell %u: %.3fV \r\n", i, (float)voltages[i] / 1000);
                //cli_bms_debug("READ AND STORE, REMOVED UNDERVOLTAGE", 36);
                if (!is_bms_on_fault) {
                    error_reset(ERROR_CELL_UNDERVOLTAGE, i);
                    error_reset(ERROR_CELL_OVERVOLTAGE, i);
                }
            }
        }
        sprintf(buf + strlen(buf), "%s", buff);
        total_voltage_on_board += (float)voltages[i] / 1000;
    }
    if (total_voltage_on_board < MIN_POWER_ON_VOLTAGE) {
        volt_status = VOLT_UNDER_VOLTAGE;
        voltages[0] = 0x00;
        voltages[1] = 0x00;
        voltages[2] = 0x00;
        voltages[3] = 0x00;
    }
    sprintf(buf + strlen(buf), "Total voltage on board: %.3fV \r\n", total_voltage_on_board);
    return volt_status;
}

// #ifdef CELL_0_IS_ALIVE
//     if (voltages_pup[0] == 0) {
//         error_set(ERROR_OPEN_WIRE, 0, HAL_GetTick());
//         return;
//     }
// #endif

// #ifdef SIX_CELLS_BATTERY
//     if (voltages_pud[6] == 0) {
//         error_set(ERROR_OPEN_WIRE, 0, HAL_GetTick());
//         return;
//     }
// #endif

//     if (!is_bms_on_fault) {
//         error_reset(ERROR_OPEN_WIRE, 0);
//     }
// }
