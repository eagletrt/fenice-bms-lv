/**
 * @file		Monitor_int.c
 * @brief		bms monitor interface for voltage reading
 *
 * @date 2021-11-24
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @author Dimitri Corraini (dimitri.corraini@studenti.unitn.it)
 */

#include "monitor_int.h"

#include "can_comm.h"
#include "error.h"
#include "fenice-config.h"
#include "main.h"
#include "usart.h"
#include "volt.h"

#include <stdio.h>
#include <string.h>

voltage_t voltages[MAX_LV_CELLS_COUNT] = {0};
float cell_temps[NTC_COUNT]            = {0.0};
float total_voltage_on_board           = 0.0;
char buff[500];
uint8_t voltage_min_index;
uint8_t volt_status;
float total_voltage_on_board;
LTC6811_HandleTypeDef monitor_handler;
uint8_t cell_col_index                                    = 255;
uint8_t cell_row_index                                    = 0;
uint16_t cell_temps_raw[CELL_TEMPS_ARRAY_SIZE][NTC_COUNT] = {0};
uint8_t voltage_warning_flag                              = 0;
VOLT_CHANNEL cells                                        = VOLT_CHANNEL_ALL;
uint8_t nsfw_charger                                      = 0;
lv_thresholds_t lv_thresholds_handler                     = {0};

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

#ifndef SKIP_VOLTAGES_CHECK
    if (ltc6811_rdcfg(&monitor_handler) != HAL_OK) {
        error_set(ERROR_BMS_MONITOR, 0);
    }
#endif
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
    if (volt_read(&monitor_handler, LTC6811_MD_7KHZ_3KHZ, LTC6811_DCP_DISABLED, &cells, &voltages, 5) == 1) {
        volt_status = VOLT_ERR;
        error_set(ERROR_BMS_MONITOR, 0);
    } else {
        error_reset(ERROR_BMS_MONITOR, 0);
        for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
            monitor_tresholds_check((float)voltages[i] / 1000);
            // Check whether a cell is undervoltage or overvoltage and set/reset its specific error
            if ((float)voltages[i] / 1000 <= VOLT_MIN_ALLOWED_VOLTAGE) {
                if (nsfw_charger == 0) {
                    volt_status = VOLT_UNDER_VOLTAGE;
                    error_set(ERROR_CELL_UNDERVOLTAGE, i);
                }
            } else if ((float)voltages[i] / 1000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                if (nsfw_charger == 0) {
                    volt_status = VOLT_OVER_VOLTAGE;
                    error_set(ERROR_CELL_OVERVOLTAGE, i);
                }
            } else {
                //cli_bms_debug("SAMPLE AND READ, REMOVED UNDERVOLTAGE", 37);
                if (!voltage_warning_flag) {
                    if (voltages[i] <= VOLT_WARNING_LEVEL) {
                        voltage_warning_flag = true;
                    }
                }

                if (lv_status.status != PRIMARY_LV_STATUS_STATUS_ERROR_CHOICE) {
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
    if (volt_read(&monitor_handler, LTC6811_MD_7KHZ_3KHZ, LTC6811_DCP_DISABLED, &cells, &voltages, 5) == 1) {
        printl("LTC ERROR!", ERR_HEADER);
        volt_status = VOLT_ERR;
        error_set(ERROR_BMS_MONITOR, 0);
    } else {
        error_reset(ERROR_BMS_MONITOR, 0);
        voltage_min_index = monitor_get_min_cell();
    }
    memset(buff, 0, sizeof(buff));
    for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
        total_voltage_on_board += (float)voltages[i] / 1000;
        monitor_tresholds_check((float)voltages[i] / 1000);
        if (i == voltage_min_index && voltage_min_index != -1 &&
            (float)voltages[i] / 1000 <= VOLT_MAX_ALLOWED_VOLTAGE &&
            (float)voltages[i] / 1000 >= VOLT_MIN_ALLOWED_VOLTAGE) {
            sprintf(buff, "Cell %u: %.3fV M", i, (float)voltages[i] / 1000);
        } else {
            if ((float)voltages[i] / 1000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                sprintf(buff, "Error! Max allowed voltage exceeded (Cell %u: %.3fV)", i, (float)voltages[i] / 1000);
                if (nsfw_charger == 0) {
                    error_set(ERROR_CELL_OVERVOLTAGE, i);
                    volt_status = VOLT_OVER_VOLTAGE;
                    voltages[i] = 0xff;
                }
            } else if ((float)voltages[i] / 1000 <= VOLT_MIN_ALLOWED_VOLTAGE) {
                sprintf(buff, "Error! Min allowed voltage exceeded (Cell %u: %.3fV)", i, (float)voltages[i] / 1000);
                if (nsfw_charger == 0) {
                    error_set(ERROR_CELL_UNDERVOLTAGE, i);
                    volt_status = VOLT_UNDER_VOLTAGE;
                    voltages[i] = 0x00;
                }
            } else {
                sprintf(buff, "Cell %u: %.3fV", i, (float)voltages[i] / 1000);
                //cli_bms_debug("READ AND PRINT, REMOVED UNDERVOLTAGE", 36);
                if (lv_status.status != PRIMARY_LV_STATUS_STATUS_ERROR_CHOICE) {
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
        monitor_tresholds_check((float)voltages[i] / 1000);
        if (i == voltage_min_index && voltage_min_index != -1 &&
            (float)voltages[i] / 1000 <= VOLT_MAX_ALLOWED_VOLTAGE &&
            (float)voltages[i] / 1000 >= VOLT_MIN_ALLOWED_VOLTAGE) {
            sprintf(buff, "Cell %u: %.3fV M \r\n", i, (float)voltages[i] / 1000);
        } else {
            if ((float)voltages[i] / 1000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                sprintf(
                    buff, "Error! Max allowed voltage exceeded (Cell %u: %.3fV) \r\n", i, (float)voltages[i] / 1000);
                if (nsfw_charger == 0) {
                    error_set(ERROR_CELL_OVERVOLTAGE, i);
                    volt_status = VOLT_OVER_VOLTAGE;
                }
                //voltages[i] = 0xff;
            } else if ((float)voltages[i] / 1000 <= VOLT_MIN_ALLOWED_VOLTAGE) {
                sprintf(
                    buff, "Error! Min allowed voltage exceeded (Cell %u: %.3fV) \r\n", i, (float)voltages[i] / 1000);
                if (nsfw_charger == 0) {
                    error_set(ERROR_CELL_UNDERVOLTAGE, i);
                    volt_status = VOLT_UNDER_VOLTAGE;
                }

                //voltages[i] = 0x00;
            } else {
                sprintf(buff, "Cell %u: %.3fV \r\n", i, (float)voltages[i] / 1000);
                //cli_bms_debug("READ AND STORE, REMOVED UNDERVOLTAGE", 36);
                if (lv_status.status != PRIMARY_LV_STATUS_STATUS_ERROR_CHOICE) {
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

void monitor_read_temp() {
    volatile uint16_t raw_temp[3] = {0};
    uint32_t timeout              = 2;
    HAL_StatusTypeDef status;
    if (cell_col_index != 255) {
        ltc6811_adax(&monitor_handler, LTC6811_MD_27KHZ_14KHZ, LTC6811_CHG_GPIO_1);
        // Poll for conversion status with a timeout
        uint32_t t0 = HAL_GetTick();
        while ((status = ltc6811_pladc(&monitor_handler)) != HAL_OK && HAL_GetTick() - t0 < timeout)
            ;
        //volatile uint32_t delta = HAL_GetTick() - t0;
        if (status != HAL_OK) {
            // throw error!
        }
        ltc6811_rdaux(&monitor_handler, LTC6811_AVAR, &raw_temp);

        cell_temps_raw[cell_row_index][cell_col_index] = raw_temp[0];

        // Update col and row indexes according to NTC COUNT and CELL_TEMPS_ARRAY_SIZE
        cell_col_index = (cell_col_index + 1) % NTC_COUNT;
        cell_row_index = (cell_row_index + 1) % CELL_TEMPS_ARRAY_SIZE;

#ifdef NDEBUG
        // Measure how much time a conversion will take
        uint8_t spin = (spin + 1) % (NTC_COUNT * CELL_TEMPS_ARRAY_SIZE);
        if (spin == 0) {
            HAL_GPIO_TogglePin(NC_MCU0_GPIO_Port, NC_MCU0_Pin);
        }
#endif
    } else {
        cell_col_index = 0;
    }
#define MONITOR_MUX_OFFSET 2
    uint8_t monitor_mux_index    = cell_col_index + MONITOR_MUX_OFFSET;
    monitor_handler.config->GPIO = (monitor_mux_index & 1) << 4 | (monitor_mux_index & 2) << 2 |
                                   (monitor_mux_index & 4) | (monitor_mux_index & 8) >> 2 | 1;
    ltc6811_wrcfg(&monitor_handler);
    ltc6811_adax(&monitor_handler, LTC6811_MD_7KHZ_3KHZ, LTC6811_CHG_GPIO_1);
}

void monitor_temp_conversion() {
    for (int i = 0; i < NTC_COUNT; i++) {
        float val = 0;
        for (int j = 0; j < CELL_TEMPS_ARRAY_SIZE; j++) {
            val += cell_temps_raw[j][i];
        }
        val /= CELL_TEMPS_ARRAY_SIZE;
        val        = (val / 10.0) * 1.0;
        float val2 = val * val;
        float val3 = val2 * val;
        float val4 = val3 * val;
#ifndef NON_CRITICAL_SAFETY_CHECKS_BYPASS
        cell_temps[i] = (float)(TEMP_CONST_a + TEMP_CONST_b * val + TEMP_CONST_c * val2 + TEMP_CONST_d * val3 +
                                TEMP_CONST_e * val4);
#endif
        //cell_temps[i] = -25;
#ifndef SKIP_TEMP_READ
        uint8_t over_temps_counter  = 0;
        uint8_t under_temps_counter = 0;
        if (cell_temps[i] > MAX_CELLS_ALLOWED_TEMP && cell_temps[i] < 120.0f) {
            over_temps_counter++;
            error_set(ERROR_CELL_OVER_TEMPERATURE, 0);
        } else if (cell_temps[i] < MIN_CELLS_ALLOWED_TEMP) {
            under_temps_counter++;
            error_set(ERROR_CELL_UNDER_TEMPERATURE, 0);
            //printl("Set error", NO_HEADER);
        } else {
            error_reset(ERROR_CELL_OVER_TEMPERATURE, 0);
            error_reset(ERROR_CELL_UNDER_TEMPERATURE, 0);
        }
        // else {
        //     // char dbg[50];
        //     // sprintf(dbg, "Unset error %d", i);
        //     //printl(dbg, NO_HEADER);
        //     error_reset(ERROR_CELL_OVER_TEMPERATURE, i);
        //     error_reset(ERROR_CELL_UNDER_TEMPERATURE, i);
        // }

#endif
    }
}

void monitor_print_temps(char *buf) {
    memset(buff, 0, sizeof(buff));
    uint8_t under_temp_counter = 0;
    uint8_t over_temp_counter  = 0;
    for (uint8_t i = 0; i < NTC_COUNT; i++) {
        if (cell_temps[i] < MIN_CELLS_ALLOWED_TEMP) {
            under_temp_counter++;
        } else if (cell_temps[i] > MAX_CELLS_ALLOWED_TEMP) {
            over_temp_counter++;
        }
        sprintf(buff, "Cell %u: %.4f[°C]\r\n", i, cell_temps[i]);
        sprintf(buf + strlen(buf), "%s", buff);
    }
    sprintf(
        buf + strlen(buf),
        "Under voltages detected: %u\r\nOver voltages detected: %u\r\nMax number of sensor allowed to be broken: "
        "%u\r\n",
        under_temp_counter,
        over_temp_counter,
        NTC_COUNT - COUNT_MINIMUM_WORKING_NTCS);
}

void monitor_tresholds_check(float volts) {
    if (!lv_thresholds_handler.first_threshold_reached && volts < LV_TRESHOLD_WARNGING_1_V) {
        lv_thresholds_handler.first_threshold_reached  = 1;
        lv_thresholds_handler.first_ths_timestamp      = HAL_GetTick();
        lv_thresholds_handler.first_ths_buzzer_toggles = 0;
    } else if (
        lv_thresholds_handler.first_threshold_reached && !lv_thresholds_handler.second_threshold_reached &&
        volts < LV_TRESHOLD_WARNGING_2_V) {
        lv_thresholds_handler.second_threshold_reached  = 1;
        lv_thresholds_handler.second_ths_timestamp      = HAL_GetTick();
        lv_thresholds_handler.second_ths_buzzer_toggles = 0;
    }
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
