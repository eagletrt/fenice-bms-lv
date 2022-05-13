/**
 * @file		Volt.c
 * @brief		Voltage measurement functions
 *
 * @date 2021-11-24
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 */

#include "volt.h"

#include "main.h"
#include "usart.h"

voltage_t voltages[LV_CELLS_COUNT] = {0};
float total_voltage_on_board       = 0.0;
bms_balancing_cells cells          = 0b0;
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
 * @brief Invokes ltc6810_read_voltages function
 * 
 * @return uint8_t 
 */
uint8_t volt_read() {
    return ltc6810_read_voltages(&SPI, voltages);
}

/**
 * @brief Return the min voltage cell index
 * 
 * @return uint8_t 
 */
uint8_t volt_get_min() {
    uint8_t min = 0;
    for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
        if (voltages[i] < voltages[min]) {
            min = i;
        }
    }
    return min;
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
    if (volt_read() == 1) {
        printl("LTC ERROR!", ERR_HEADER);
        volt_status = VOLT_ERR;
    } else {
        voltage_min_index = volt_get_min();
    }
    memset(buff, 0, sizeof(buff));
    for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
        if (i == voltage_min_index && voltage_min_index != -1) {
            sprintf(buff, "Cell %u: %.3fV M", i, (float)voltages[i] / 10000);
        } else {
            if ((float)voltages[i] / 10000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                sprintf(buff, "Error! Max allowed voltage exceeded (Cell %u: %.3fV)", i, (float)voltages[i] / 10000);
                volt_status = VOLT_OVER_VOLTAGE;
            } else {
                sprintf(buff, "Cell %u: %.3fV", i, (float)voltages[i] / 10000);
            }
        }
        total_voltage_on_board += (float)voltages[i] / 10000;
        printl(buff, NO_HEADER);
    }
    if (total_voltage_on_board < MIN_POWER_ON_VOLTAGE) {
        printl("UNDERVOLTAGE!", ERR_HEADER);
        volt_status = VOLT_UNDER_VOLTAGE;
    }
    return volt_status;
}

/**
 * @brief Reads and store voltages into buf
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
    if (volt_read() == 1) {
        sprintf(buf, "LTC ERROR! \r\n");
        volt_status = VOLT_ERR;
    } else {
        voltage_min_index = volt_get_min();
    }
    memset(buff, 0, sizeof(buff));
    for (uint8_t i = 0; i < LV_CELLS_COUNT; i++) {
        if (i == voltage_min_index && voltage_min_index != -1) {
            sprintf(buff, "Cell %u: %.3fV M \r\n", i, (float)voltages[i] / 10000);
        } else {
            if ((float)voltages[i] / 10000 >= VOLT_MAX_ALLOWED_VOLTAGE) {
                sprintf(
                    buff, "Error! Max allowed voltage exceeded (Cell %u: %.3fV) \r\n", i, (float)voltages[i] / 10000);
                volt_status = VOLT_OVER_VOLTAGE;
            } else {
                sprintf(buff, "Cell %u: %.3fV \r\n", i, (float)voltages[i] / 10000);
            }
        }
        sprintf(buf + strlen(buf), "%s", buff);
        total_voltage_on_board += (float)voltages[i] / 10000;
    }
    if (total_voltage_on_board < MIN_POWER_ON_VOLTAGE) {
        sprintf(buf, "UNDERVOLTAGE! \r\n");
        volt_status = VOLT_UNDER_VOLTAGE;
    }
    return volt_status;
}