/**
 * @file mcp23017_base.c
 * @author Dimitri Corraini [dimitri.corraini@studenti.unitn.it]
 * @brief 
 * @version 0.1
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "mcp23017_base.h"

#include <stdbool.h>

MCP23017_HandleTypeDef hmcp;

void mcp23017_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr) {
    hdev->hi2c = hi2c;
    hdev->addr = addr << 1;
}

HAL_StatusTypeDef mcp23017_read(MCP23017_HandleTypeDef *hdev, uint16_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, reg, 1, data, 1, I2C_TIMEOUT);
}

HAL_StatusTypeDef mcp23017_write(MCP23017_HandleTypeDef *hdev, uint16_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, reg, 1, data, 1, I2C_TIMEOUT);
}

HAL_StatusTypeDef mcp23017_iodir(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t iodir) {
    uint8_t data[1] = {iodir};
    return mcp23017_write(hdev, REGISTER_IODIRA | port, data);
}

HAL_StatusTypeDef mcp23017_ipol(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t ipol) {
    uint8_t data[1] = {ipol};
    return mcp23017_write(hdev, REGISTER_IPOLA | port, data);
}

HAL_StatusTypeDef mcp23017_ggpu(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t pu) {
    uint8_t data[1] = {pu};
    return mcp23017_write(hdev, REGISTER_GPPUA | port, data);
}

HAL_StatusTypeDef mcp23017_read_gpio(MCP23017_HandleTypeDef *hdev, uint8_t port) {
    uint8_t data[1];
    HAL_StatusTypeDef status;
    status = mcp23017_read(hdev, REGISTER_GPIOA | port, data);
    if (status == HAL_OK)
        hdev->gpio[port] = data[0];
    return status;
}

HAL_StatusTypeDef mcp23017_write_gpio(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t *data) {
    return mcp23017_write(hdev, REGISTER_GPIOA | port, data);
}

/**
 * @brief Set the pin state of a gpio output
 * 
 * @param hdev MCP23017_HandleTypeDef handle for the device
 * @param port GPIO port to be written to this can be either MCP23017_PORTA or MCP23017_PORTB
 * @param pin Pin of the GPIO bank that will be set
 * @param pinState Value to be written to the pin, 0 for low 1 for high
 * @return HAL_OK if all has gone well
*/
HAL_StatusTypeDef mcp23017_set_gpio(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t pin, uint8_t pinState) {
    uint8_t data;

    mcp23017_read_gpio(hdev, port);
    data = hdev->gpio[port];

    if (!pinState) {
        data &= ~(1 << (pin % 8));
    } else {
        data |= (1 << (pin % 8));
    }

    return mcp23017_write_gpio(hdev, port, &data);
}

/**
 * @brief Necessary and tested configuration to initialize correctly the MCP23017
 * 		  Using default address 0x20
 * 
 * @param hdev MCP23017_HandleTypeDef handle for the device
 * @param hi2c I2C Handle
 */
void mcp23017_basic_config_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c) {
    mcp23017_init(hdev, hi2c, MCP23017_ADDRESS);
    mcp23017_iodir(hdev, MCP23017_PORTA, IODIR_GPIOA);
    mcp23017_iodir(hdev, MCP23017_PORTB, IODIR_GPIOB);
    mcp23017_ggpu(hdev, MCP23017_PORTA, MCP23017_GPPU_ALL_DISABLED);
    mcp23017_ggpu(hdev, MCP23017_PORTB, MCP23017_GPPU_ALL_DISABLED);
    mcp23017_ipol(hdev, MCP23017_PORTA, MCP23017_IPOL_ALL_NORMAL);
    mcp23017_ipol(hdev, MCP23017_PORTB, MCP23017_IPOL_ALL_NORMAL);
    mcp23017_write_gpio(hdev, MCP23017_PORTA, &hdev->gpio[0]);
    mcp23017_write_gpio(hdev, MCP23017_PORTB, &hdev->gpio[0]);
}