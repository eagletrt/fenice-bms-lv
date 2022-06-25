/**
 * @file		Volt.c
 * @brief		Voltage measurement functions
 *
 * @date 2021-11-24
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 */

#include "volt.h"

#include "cli_bms_lv.h"
#include "error.h"
#include "fenice-config.h"
#include "main.h"
#include "usart.h"

voltage_t voltages[LV_CELLS_COUNT]     = {0};
voltage_t voltages_pup[LV_CELLS_COUNT] = {0};
voltage_t voltages_pud[LV_CELLS_COUNT] = {0};
float total_voltage_on_board           = 0.0;
bms_balancing_cells cells              = 0b0;
char buff[500];
uint8_t voltage_min_index;
uint8_t volt_status;
float total_voltage_on_board;

//bms_balancing_cells cells_to_discharge;
/**
 * @brief Start ADCV with standard parameters
 *        ADC Mode: 7kHz, DCP disabled, CELL all
 * 
 */
void volt_start_basic_measure() {
    ltc6810_adcv(&SPI, MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
}

/**
 * @brief Start ADCV with custom parameters
 * 
 * @param MD  ADC Mode
 * @param DCP Discharge Permitted
 * @param CH  Cell selection
 */
void volt_start_measure(uint8_t MD, uint8_t DCP, uint8_t CH) {
    ltc6810_adcv(&SPI, MD, DCP, CH);
}

/**
 * @brief Return the min voltage cell index
 * 
 * @return uint8_t 
 */
uint8_t volt_get_min() {
    uint8_t min = 0;
    for (uint8_t i = DEAD_CELLS_OFFSET; i < LV_CELLS_COUNT; i++) {
        if (voltages[i] < voltages[min]) {
            min = i;
        }
    }
    return min;
}

/**
 * @brief Invokes ltc6810_read_voltages function
 * 
 * @return uint8_t 
 */
uint8_t volt_read(voltage_t *volts) {
    return ltc6810_read_voltages(&SPI, volts);
}

/**
 * @brief Fatest reading function without any kind of print
 * 
 * @return uint8_t return a volt_status. Output could be
 * VOLT_OK, VOLT_UNDER_VOLTAGE, VOLT_OVER_VOLTAGE, VOLT_ERR,
 */
uint8_t volt_sample_and_read() {
    volt_status            = VOLT_OK;
    total_voltage_on_board = 0.0;
    volt_start_basic_measure();
    HAL_Delay(1);
    if (volt_read(voltages) == 1) {
        volt_status = VOLT_ERR;
    } else {
        for (uint8_t i = DEAD_CELLS_OFFSET; i < LV_CELLS_COUNT; i++) {
            // Check whether a cell is undervoltage or overvoltage and set/reset its specific error
            if ((float)voltages[i] / 10000 <= VOLT_MIN_ALLOWED_VOLTAGE) {
                volt_status = VOLT_UNDER_VOLTAGE;
                error_set(ERROR_CELL_UNDERVOLTAGE, i, HAL_GetTick());
            } else if ((float)voltages[i] / 10000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                volt_status = VOLT_OVER_VOLTAGE;
                error_set(ERROR_CELL_OVERVOLTAGE, i, HAL_GetTick());
            } else {
                //cli_bms_debug("SAMPLE AND READ, REMOVED UNDERVOLTAGE", 37);
                error_reset(ERROR_CELL_UNDERVOLTAGE, i);
                error_reset(ERROR_CELL_OVERVOLTAGE, i);
            }
            total_voltage_on_board += (float)voltages[i] / 10000;
        }
        if (total_voltage_on_board < MIN_POWER_ON_VOLTAGE) {
            volt_status = VOLT_UNDER_VOLTAGE;
        }
    }
    return volt_status;
}

/**
 * @brief Read and print voltages
 * 
 * @return Return VOLT_OK if total voltage on board is above MIN_POWER_ON_VOLTAGE, VOLT_UNDER_VOLTAGE if it's under MIN_POWER_ON_VOLTAGE, 
 * VOLT_OVER_VOLTAGE if there's even just one cell over MAX_VOLTAGE_ALLOWED
 */
uint8_t volt_read_and_print() {
    volt_status            = VOLT_OK;
    voltage_min_index      = -1;
    total_voltage_on_board = 0.0;
    //HAL_Delay(200);
    volt_start_basic_measure();
    HAL_Delay(1);
    if (volt_read(voltages) == 1) {
        printl("LTC ERROR!", ERR_HEADER);
        volt_status = VOLT_ERR;
    } else {
        voltage_min_index = volt_get_min();
    }
    memset(buff, 0, sizeof(buff));
    for (uint8_t i = DEAD_CELLS_OFFSET; i < LV_CELLS_COUNT; i++) {
        total_voltage_on_board += (float)voltages[i] / 10000;
        if (i == voltage_min_index && voltage_min_index != -1 &&
            (float)voltages[i] / 10000 <= VOLT_MAX_ALLOWED_VOLTAGE &&
            (float)voltages[i] / 10000 >= VOLT_MIN_ALLOWED_VOLTAGE) {
            sprintf(buff, "Cell %u: %.3fV M", i, (float)voltages[i] / 10000);
        } else {
            if ((float)voltages[i] / 10000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                sprintf(buff, "Error! Max allowed voltage exceeded (Cell %u: %.3fV)", i, (float)voltages[i] / 10000);
                error_set(ERROR_CELL_OVERVOLTAGE, i, HAL_GetTick());
                volt_status = VOLT_OVER_VOLTAGE;
                voltages[i] = 0xff;
            } else if ((float)voltages[i] / 10000 <= VOLT_MIN_ALLOWED_VOLTAGE) {
                sprintf(buff, "Error! Min allowed voltage exceeded (Cell %u: %.3fV)", i, (float)voltages[i] / 10000);
                error_set(ERROR_CELL_UNDERVOLTAGE, i, HAL_GetTick());
                volt_status = VOLT_UNDER_VOLTAGE;
                voltages[i] = 0x00;
            } else {
                sprintf(buff, "Cell %u: %.3fV", i, (float)voltages[i] / 10000);
                //cli_bms_debug("READ AND PRINT, REMOVED UNDERVOLTAGE", 36);
                error_reset(ERROR_CELL_UNDERVOLTAGE, i);
                error_reset(ERROR_CELL_OVERVOLTAGE, i);
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

/**
 * @brief Reads and store voltages into buf. ! Used in the cli !
 * 
 * @param buf Buffer where voltage message are stored
 * @return Return VOLT_OK if total voltage on board is above MIN_POWER_ON_VOLTAGE, VOLT_UNDER_VOLTAGE if it's under MIN_POWER_ON_VOLTAGE, 
 * VOLT_OVER_VOLTAGE if there's even just one cell over MAX_VOLTAGE_ALLOWED
 */
uint8_t volt_read_and_store(char *buf) {
    volt_status            = VOLT_OK;
    voltage_min_index      = -1;
    total_voltage_on_board = 0.0;
    //HAL_Delay(200);
    volt_start_basic_measure();
    HAL_Delay(1);
    if (volt_read(voltages) == 1) {
        sprintf(buf, "LTC ERROR! \r\n");
        volt_status = VOLT_ERR;
    } else {
        voltage_min_index = volt_get_min();
    }
    memset(buff, 0, sizeof(buff));
    for (uint8_t i = DEAD_CELLS_OFFSET; i < LV_CELLS_COUNT; i++) {
        if (i == voltage_min_index && voltage_min_index != -1 &&
            (float)voltages[i] / 10000 <= VOLT_MAX_ALLOWED_VOLTAGE &&
            (float)voltages[i] / 10000 >= VOLT_MIN_ALLOWED_VOLTAGE) {
            sprintf(buff, "Cell %u: %.3fV M \r\n", i, (float)voltages[i] / 10000);
        } else {
            if ((float)voltages[i] / 10000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                sprintf(
                    buff, "Error! Max allowed voltage exceeded (Cell %u: %.3fV) \r\n", i, (float)voltages[i] / 10000);
                volt_status = VOLT_OVER_VOLTAGE;
                //voltages[i] = 0xff;
            } else if ((float)voltages[i] / 10000 <= VOLT_MIN_ALLOWED_VOLTAGE) {
                sprintf(
                    buff, "Error! Min allowed voltage exceeded (Cell %u: %.3fV) \r\n", i, (float)voltages[i] / 10000);
                error_set(ERROR_CELL_UNDERVOLTAGE, i, HAL_GetTick());
                volt_status = VOLT_UNDER_VOLTAGE;
                //voltages[i] = 0x00;
            } else {
                sprintf(buff, "Cell %u: %.3fV \r\n", i, (float)voltages[i] / 10000);
                //cli_bms_debug("READ AND STORE, REMOVED UNDERVOLTAGE", 36);
                error_reset(ERROR_CELL_UNDERVOLTAGE, i);
                error_reset(ERROR_CELL_OVERVOLTAGE, i);
            }
        }
        sprintf(buf + strlen(buf), "%s", buff);
        total_voltage_on_board += (float)voltages[i] / 10000;
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

void volt_start_open_wire_check(uint8_t status) {
    ltc6810_adow(&SPI, status > 2 ? LTC6810_ADOW_PUP_INACTIVE : LTC6810_ADOW_PUP_ACTIVE);
}

void volt_read_open_wire(uint8_t status) {
    ltc6810_read_voltages(&SPI, status > 2 ? voltages_pud : voltages_pup);
}

void volt_open_wire_check() {
    for (uint8_t i = 1; i < LV_CELLS_COUNT; ++i) {
        int32_t diff = (int32_t)voltages_pup[i] - voltages_pud[i];

        if (diff < -4000) {
            error_set(ERROR_OPEN_WIRE, 0, HAL_GetTick());
            return;
        }
    }

#ifdef CELL_0_IS_ALIVE
    if (voltages_pup[0] == 0) {
        error_set(ERROR_OPEN_WIRE, 0, HAL_GetTick());
        return;
    }
#endif

#ifdef SIX_CELLS_BATTERY
    if (voltages_pud[6] == 0) {
        error_set(ERROR_OPEN_WIRE, 0, HAL_GetTick());
        return;
    }
#endif

    error_reset(ERROR_OPEN_WIRE, 0);
}
