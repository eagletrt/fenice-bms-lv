/**
 * @file		Volt.h
 * @brief		Voltage measurement functions
 *
 * @date 2021-11-24
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 */

#ifndef VOLT_H
#define VOLT_H

#include "ltc_config.h"
#include "ltc6810-driver.h"
#include "spi.h"

#define VOLT_MAX_ALLOWED_VOLTAGE 4.2

extern voltage_t voltages[LV_CELLS_COUNT];
extern bms_balancing_cells cells;

void volt_initialization();
void volt_start_basic_measure();
void volt_start_measure(uint8_t MD, uint8_t DCP, uint8_t CH);
uint8_t volt_read();
uint8_t volt_read_and_print();

/**
 * @brief Returns the lower-voltage cell
 * 
 * @return uint16_t The index of the lower-voltage cell
 */
uint8_t volt_get_min();

#endif