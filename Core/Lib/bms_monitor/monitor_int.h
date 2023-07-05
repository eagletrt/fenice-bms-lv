/**
 * @file		Monitor_int.h
 * @brief		bms monitor interface for voltage reading
 *
 * @date 2021-11-24
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @author Dimitri Corraini (dimitri.corraini@studenti.unitn.it)
 */

#ifndef MONITOR_INT_H
#define MONITOR_INT_H

#include "fenice-config.h"
#include "ltc6811.h"
#include "monitor_config.h"
#include "spi.h"

//coefficient for conversion formula from voltage input to temperature
#define TEMP_CONST_a 127.02004615145405
#define TEMP_CONST_b -0.06979687590434158
#define TEMP_CONST_c 2.1026948971763155e-05
#define TEMP_CONST_d -3.3042552498294913e-09
#define TEMP_CONST_e 1.3552262617901958e-13

#define CELL_TEMPS_ARRAY_SIZE 5

extern voltage_t voltages[MAX_LV_CELLS_COUNT];
extern uint16_t cell_temps_raw[CELL_TEMPS_ARRAY_SIZE][NTC_COUNT];
extern float total_voltage_on_board;
extern uint8_t volt_status;
extern LTC6811_HandleTypeDef monitor_handler;
extern uint8_t cell_row_index;
extern uint8_t cell_col_index;
extern uint8_t voltage_warning_flag;
extern float cell_temps[NTC_COUNT];
extern uint8_t nsfw_charger;
typedef enum { VOLT_OK = 0U, VOLT_UNDER_VOLTAGE, VOLT_OVER_VOLTAGE, VOLT_ERR, VOLT_ENUM_SIZE } voltage_meas_status;

/**
 * @brief  Initialize bms monitor configuration
*/
void monitor_init();

/**
 * @brief Fatest reading function without any kind of print
 * 
 * @return uint8_t return a volt_status. Output could be
 * VOLT_OK, VOLT_UNDER_VOLTAGE, VOLT_OVER_VOLTAGE, VOLT_ERR,
 */
uint8_t monitor_read_voltage();

/**
 * @brief Read and print voltages
 * 
 * @return Return VOLT_OK if total voltage on board is above MIN_POWER_ON_VOLTAGE, VOLT_UNDER_VOLTAGE if it's under MIN_POWER_ON_VOLTAGE, 
 * VOLT_OVER_VOLTAGE if there's even just one cell over MAX_VOLTAGE_ALLOWED
 */
uint8_t monitor_print_volt();

/**
 * @brief Reads and store voltages message into buf. ! Used in the cli !
 * 
 * @param buf Buffer where voltage message are stored
 * @return Return VOLT_OK if total voltage on board is above MIN_POWER_ON_VOLTAGE, VOLT_UNDER_VOLTAGE if it's under MIN_POWER_ON_VOLTAGE, 
 * VOLT_OVER_VOLTAGE if there's even just one cell over MAX_VOLTAGE_ALLOWED
 */
uint8_t monitor_print_volt_cli(char *buf);

/**
 * @brief Returns the lower-voltage cell
 * 
 * @return uint8_t The index of the lower-voltage cell
 */
uint8_t monitor_get_min_cell();

/**
 * @brief Convert all the temperature from the temperature matrix and store
 * the new value in an array
*/
void monitor_temp_conversion();

void monitor_read_temp();

void monitor_print_temps(char *buf);

#endif  // MONITOR_INT_H