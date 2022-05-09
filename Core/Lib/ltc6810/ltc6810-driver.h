/**
 ********************************************************************************
 * @file    LTC6810 DRIVER
 * @author  Tommaso Canova
 * @date    24-02-2022
 * @brief   
 ********************************************************************************
 */

#ifndef __LTC6810_DRIVER_H
#define __LTC6810_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ltc_config.h"
#include "spi.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#define VOLTAGE_CONVERSION  //needed to convert raw rx data into voltages

extern const uint16_t crcTable[256];

void init_PEC15_Table(void);

uint16_t ltc6810_pec15(uint8_t data[], uint8_t len);
void ltc6810_enable_cs(SPI_HandleTypeDef *spi);
void ltc6810_disable_cs(SPI_HandleTypeDef *spi);
void ltc6810_wakeup_idle(SPI_HandleTypeDef *spi);
void ltc6810_basic_adcv(SPI_HandleTypeDef *spi);
void ltc6810_adcv(SPI_HandleTypeDef *spi, uint8_t MD, uint8_t DCP, uint8_t CH);
void ltc6810_wrcfg(SPI_HandleTypeDef *spi, uint8_t cfgr[8]);
void ltc6810_read_both_status_register(SPI_HandleTypeDef *spi);
uint8_t ltc6810_read_voltages(SPI_HandleTypeDef *spi, voltage_t *volts);
uint16_t ltc6810_convert_voltages(uint8_t v_data[]);
void ltc6810_read_serial_id(SPI_HandleTypeDef *spi);
char *ltc6810_return_serial_id();
HAL_StatusTypeDef ltc6810_pladc(SPI_HandleTypeDef *spi, uint32_t timeout);
void ltc6810_set_balancing(SPI_HandleTypeDef *spi, bms_balancing_cells cells, int dcto);
void ltc6810_build_dcc(bms_balancing_cells cells, uint8_t cfgr[8]);

#ifdef __cplusplus
}
#endif
#endif