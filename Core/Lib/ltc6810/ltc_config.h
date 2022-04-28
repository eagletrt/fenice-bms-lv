/**
 * @file bms_lv_config.h
 * @brief This file contains configuration settings for the low voltage bms
 * 
 * @date 2021-11-24
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * 
 */

#ifndef BMS_LV_CONFIG
#define BMS_LV_CONFIG

#include <stdint.h>
#include <spi.h>
#include "main.h"
//===========================================================================
//=================================== BOARD =================================
//===========================================================================

#define SPI (hspi2)
#define UART (huart1)

#define SPI_CS_GPIO_Port (LTC_CS_GPIO_Port)
#define SPI_CS_Pin (LTC_CS_Pin)
//===========================================================================
//=================================== COSTRAINTS ============================
//===========================================================================

// Number of porchetto cells
#define LV_CELLS_COUNT 4
// Number of LTC6810 on board
#define TOTAL_ICS 1

#define MD_422HZ_1KHZ  (uint8_t) 0
#define MD_27KHZ_14KHZ (uint8_t) 1
#define MD_7KHZ_3KHZ   (uint8_t) 2
#define MD_26HZ_2KHZ   (uint8_t) 3

#define DCP_DISABLED (uint8_t) 0
#define DCP_ENABLED (uint8_t) 1

#define CELL_CH_ALL (uint8_t) 0
#define CELL_CH_1   (uint8_t) 1
#define CELL_CH_2   (uint8_t) 2
#define CELL_CH_3   (uint8_t) 3
#define CELL_CH_4   (uint8_t) 4
#define CELL_CH_5   (uint8_t) 5
#define CELL_CH_6   (uint8_t) 6

#define PULL_UP_CURRENT   (uint8_t)  1
#define PULL_DOWN_CURRENT (uint8_t)  0

#define SELFTEST_1 (uint8_t)  1
#define SELFTEST_2 (uint8_t)  2

#define AUX_CH_ALL   (uint8_t) 0
#define AUX_CH_GPIO1 (uint8_t) 1
#define AUX_CH_GPIO2 (uint8_t) 2
#define AUX_CH_GPIO3 (uint8_t) 3
#define AUX_CH_GPIO4 (uint8_t) 4
#define AUX_CH_GPIO5 (uint8_t) 5
#define AUX_CH_VREF2 (uint8_t) 6

#define STAT_CH_ALL 0
#define STAT_CH_SOC 1
#define STAT_CH_ITEMP 2
#define STAT_CH_VREGA 3
#define STAT_CH_VREGD 4

#define ADC_OPT_ENABLED 1
#define ADC_OPT_DISABLED 0

#define REG_ALL 0
#define REG_1 1
#define REG_2 2
#define REG_3 3
#define REG_4 4
#define REG_5 5
#define REG_6 6

#define NUM_RX_BYT 8
#define CELL 1
#define AUX 2
#define STAT 3
#define CFGR 0
#define CFGRB 4
#define CS_PIN 10

#define CMD_RDCV_REG_A 4
#define CMD_RDCV_REG_B 6

#define LTC6810_EMU 0
/**
 * Number of registers for each LTC
 */
#define LTC6810_REG_COUNT 2 

/**
 * Number of cells handled by a register
 */
#define LTC6810_REG_CELL_COUNT 3

static const uint8_t rdcv_cmds [LTC6810_REG_COUNT] = {CMD_RDCV_REG_A, CMD_RDCV_REG_B};

#define CMD_WRCFG   0b00000000001
#define CMD_RDCFG   0b00000000010
#define CMD_RDSCTRL 0b00000010100
#define CMD_RDSTATA 0b00000010000
#define CMD_RDSTATB 0b00000010010
#define CMD_RDSID   0b00000101100

static const uint8_t rdstat_cmds [LTC6810_REG_COUNT] = {CMD_RDSTATA, CMD_RDSTATB};

/**
 * @brief Discharge timeut value
 * 
 */
enum {
    DCTO_DISABLED = 0,
    DCTO_30S,
    DCTO_1M,
    DCTO_2M,
    DCTO_3M,
    DCTO_4M,
    DCTO_5M,
    DCTO_10M,
    DCTO_15M,
    DCTO_20M,
    DCTO_30M,
    DCTO_40M,
    DCTO_60M,
    DCTO_75M,
    DCTO_90M,
    DCTO_120M
};

#ifdef MULTICALIBRATIO_ON
    #define MACL 1
#else
    #define MCAL 0
#endif 
static const uint8_t dcc[LV_CELLS_COUNT] = {
    0b10000000 | (MCAL << 6), // DCC0
    0b00000001 | (MCAL << 6), // DCC1
    0b00000010 | (MCAL << 6), // DCC2
    0b00000100 | (MCAL << 6),  // DCC3
    #ifdef LV_CELLS_COUNT_6
        0b00001000 | (MCAL << 6), // DCC4
        0b00010000 | (MCAL << 6), // DCC5
        0b00100000 | (MCAL << 6) // DCC6
    #endif
};

typedef uint16_t voltage_t;

typedef uint8_t bms_balancing_cells;



#endif