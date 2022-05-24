/**
 * @file		Volt.h
 * @brief		Voltage measurement functions
 *
 * @date 2021-11-24
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 */

#ifndef VOLT_H
#define VOLT_H

#include "fenice-config.h"
#include "ltc6810-driver.h"
#include "ltc_config.h"
#include "spi.h"

extern voltage_t voltages[LV_CELLS_COUNT];
extern float total_voltage_on_board;
extern bms_balancing_cells cells;
extern uint8_t volt_status;

typedef enum { VOLT_OK = 0U, VOLT_UNDER_VOLTAGE, VOLT_OVER_VOLTAGE, VOLT_ERR, VOLT_ENUM_SIZE } voltage_meas_status;

void volt_initialization();
void volt_start_basic_measure();
void volt_start_measure(uint8_t MD, uint8_t DCP, uint8_t CH);
uint8_t volt_read();
uint8_t volt_sample_and_read();
uint8_t volt_read_and_print();
uint8_t volt_read_and_store(char *buf);

/**
 * @brief Returns the lower-voltage cell
 * 
 * @return uint16_t The index of the lower-voltage cell
 */
uint8_t volt_get_min();

#endif