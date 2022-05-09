/**
 * @file mcp23017.c
 * @author Tommaso Canova [tommaso.canova@studenti.unitn.it]
 * @brief 
 * @version 0.1
 * @date 2022-05-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MCP23017_H
#define MCP23017_H

#include "stm32f4xx_hal.h"

// Ports
#define MCP23017_PORTA 0x00
#define MCP23017_PORTB 0x01

// Address (A0-A2)
#define MCP23017_ADDRESS 0x20

// I/O Direction
// Default state: MCP23017_IODIR_ALL_INPUT
#define MCP23017_IODIR_ALL_OUTPUT 0x00
#define MCP23017_IODIR_ALL_INPUT  0xFF
#define MCP23017_IODIR_IO0_INPUT  0x01
#define MCP23017_IODIR_IO1_INPUT  0x02
#define MCP23017_IODIR_IO2_INPUT  0x04
#define MCP23017_IODIR_IO3_INPUT  0x08
#define MCP23017_IODIR_IO4_INPUT  0x10
#define MCP23017_IODIR_IO5_INPUT  0x20
#define MCP23017_IODIR_IO6_INPUT  0x40
#define MCP23017_IODIR_IO7_INPUT  0x80

// Input Polarity
// Default state: MCP23017_IPOL_ALL_NORMAL
#define MCP23017_IPOL_ALL_NORMAL   0x00
#define MCP23017_IPOL_ALL_INVERTED 0xFF
#define MCP23017_IPOL_IO0_INVERTED 0x01
#define MCP23017_IPOL_IO1_INVERTED 0x02
#define MCP23017_IPOL_IO2_INVERTED 0x04
#define MCP23017_IPOL_IO3_INVERTED 0x08
#define MCP23017_IPOL_IO4_INVERTED 0x10
#define MCP23017_IPOL_IO5_INVERTED 0x20
#define MCP23017_IPOL_IO6_INVERTED 0x40
#define MCP23017_IPOL_IO7_INVERTED 0x80

// Pull-Up Resistor
// Default state: MCP23017_GPPU_ALL_DISABLED
#define MCP23017_GPPU_ALL_DISABLED 0x00
#define MCP23017_GPPU_ALL_ENABLED  0xFF
#define MCP23017_GPPU_IO0_ENABLED  0x01
#define MCP23017_GPPU_IO1_ENABLED  0x02
#define MCP23017_GPPU_IO2_ENABLED  0x04
#define MCP23017_GPPU_IO3_ENABLED  0x08
#define MCP23017_GPPU_IO4_ENABLED  0x10
#define MCP23017_GPPU_IO5_ENABLED  0x20
#define MCP23017_GPPU_IO6_ENABLED  0x40
#define MCP23017_GPPU_IO7_ENABLED  0x80

enum GPIOA_PINS { FB_RELAY = 0U, FB_INVERTERS, NC, FB_24, FB_PUMPS, FB_SHUTDOWN, FB_RADIATORS, FB_FAN };

enum GPIOB_PINS { FB_MAIN = 0U, FB_PCBS, FB_12 };

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t addr;
    uint8_t gpio[2];
} MCP23017_HandleTypeDef;

extern MCP23017_HandleTypeDef hmcp;

void mcp23017_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr);
HAL_StatusTypeDef mcp23017_read(MCP23017_HandleTypeDef *hdev, uint16_t reg, uint8_t *data);
HAL_StatusTypeDef mcp23017_write(MCP23017_HandleTypeDef *hdev, uint16_t reg, uint8_t *data);
HAL_StatusTypeDef mcp23017_iodir(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t iodir);
HAL_StatusTypeDef mcp23017_ipol(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t ipol);
HAL_StatusTypeDef mcp23017_ggpu(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t pu);
HAL_StatusTypeDef mcp23017_read_gpio(MCP23017_HandleTypeDef *hdev, uint8_t port);
HAL_StatusTypeDef mcp23017_write_gpio(MCP23017_HandleTypeDef *hdev, uint8_t port);
void mcp23017_print_gpioA(MCP23017_HandleTypeDef *hdev);
void mcp23017_print_gpioB(MCP23017_HandleTypeDef *hdev);
void mcp23017_basic_config_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c);
void mcp23017_read_both(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c);
void mcp23017_read_and_print_both(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c);
uint8_t mcp23017_get_state(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port, uint8_t gpio_pin);
#endif