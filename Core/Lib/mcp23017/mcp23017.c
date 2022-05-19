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

#include "mcp23017.h"

#include "error.h"
#include "main.h"
#include "stdio.h"
#include "usart.h"

#include <string.h>

// Registers
#define REGISTER_IODIRA   0x00
#define REGISTER_IODIRB   0x01
#define REGISTER_IPOLA    0x02
#define REGISTER_IPOLB    0x03
#define REGISTER_GPINTENA 0x04
#define REGISTER_GPINTENB 0x05
#define REGISTER_DEFVALA  0x06
#define REGISTER_DEFVALB  0x07
#define REGISTER_INTCONA  0x08
#define REGISTER_INTCONB  0x09
//	IOCON			0x0A
//	IOCON			0x0B
#define REGISTER_GPPUA   0x0C
#define REGISTER_GPPUB   0x0D
#define REGISTER_INTFA   0x0E
#define REGISTER_INTFB   0x0F
#define REGISTER_INTCAPA 0x10
#define REGISTER_INTCAPB 0x11
#define REGISTER_GPIOA   0x12
#define REGISTER_GPIOB   0x13
#define REGISTER_OLATA   0x14
#define REGISTER_OLATB   0x15

#define I2C_TIMEOUT    10
#define GPIOA_TOTAL_FB 8
#define GPIOB_TOTAL_FB 3

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

HAL_StatusTypeDef mcp23017_write_gpio(MCP23017_HandleTypeDef *hdev, uint8_t port) {
    uint8_t data[1] = {hdev->gpio[port]};
    return mcp23017_write(hdev, REGISTER_GPIOA | port, data);
}

void mcp23017_print_gpioA(MCP23017_HandleTypeDef *hdev, char *out) {
    char mcp_buff[30];
    for (uint8_t i = 0; i < GPIOA_TOTAL_FB; i++) {
        switch (i) {
            case FB_RELAY:
                sprintf(mcp_buff, "FB_RELAY: %d [GPA%d]", ((hdev->gpio[0] & (1 << i)) >> i), i);
                break;
            case FB_INVERTERS:
                sprintf(mcp_buff, "FB_INVERTERS: %d [GPA%d]", ((hdev->gpio[0] & (1 << i)) >> i), i);
                break;
            case FB_24:
                sprintf(mcp_buff, "FB_24: %d [GPA%d]", ((hdev->gpio[0] & (1 << i)) >> i), i);
                break;
            case FB_PUMPS:
                sprintf(mcp_buff, "FB_PUMPS: %d [GPA%d]", ((hdev->gpio[0] & (1 << i)) >> i), i);
                break;
            case FB_SHUTDOWN:
                sprintf(mcp_buff, "FB_SHUTDOWN: %d [GPA%d]", ((hdev->gpio[0] & (1 << i)) >> i), i);
                break;
            case FB_RADIATORS:
                sprintf(mcp_buff, "FB_RADIATORS: %d [GPA%d]", ((hdev->gpio[0] & (1 << i)) >> i), i);
                break;
            case FB_FAN:
                sprintf(mcp_buff, "FB_FAN: %d [GPA%d]", ((hdev->gpio[0] & (1 << i)) >> i), i);
                break;
            default:
                sprintf(mcp_buff, "GPIOA VAL: %d [GPA%d]", ((hdev->gpio[0] & (1 << i)) >> i), i);
                break;
        }
        if (out == NULL) {
            printl(mcp_buff, NO_HEADER);
        } else {
            sprintf(out + strlen(out), "%s \r\n", mcp_buff);
        }
    }
}

void mcp23017_print_gpioB(MCP23017_HandleTypeDef *hdev, char *out) {
    char mcp_buff[30];
    for (uint8_t i = 0; i < GPIOB_TOTAL_FB; i++) {
        switch (i) {
            case FB_MAIN:
                sprintf(mcp_buff, "FB_MAIN: %d [GPB%d]", ((hdev->gpio[1] & (1 << i)) >> i), i);
                break;
            case FB_PCBS:
                sprintf(mcp_buff, "FB_PCBS: %d [GPB%d]", ((hdev->gpio[1] & (1 << i)) >> i), i);
                break;
            case FB_12:
                sprintf(mcp_buff, "FB_12: %d [GPB%d]", ((hdev->gpio[1] & (1 << i)) >> i), i);
                break;
            default:
                sprintf(mcp_buff, "GPIOB VAL: %d [GPB%d]", ((hdev->gpio[1] & (1 << i)) >> i), i);
                break;
        }

        if (out == NULL) {
            printl(mcp_buff, NO_HEADER);
        } else {
            sprintf(out + strlen(out), "%s \r\n", mcp_buff);
        }
    }
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
    error_reset(ERROR_MCP23017, 0);
    if (mcp23017_iodir(hdev, MCP23017_PORTA, MCP23017_IODIR_ALL_INPUT) != HAL_OK) {
        printl("INIT ERROR GPIOA", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    }
    if (mcp23017_iodir(hdev, MCP23017_PORTB, MCP23017_IODIR_ALL_INPUT) != HAL_OK) {
        printl("INIT ERROR GPIOB", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    }
    if (mcp23017_ggpu(hdev, MCP23017_PORTA, MCP23017_GPPU_ALL_DISABLED) != HAL_OK) {
        printl("GGPU ERROR GPIOA", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    }
    if (mcp23017_ggpu(hdev, MCP23017_PORTB, MCP23017_GPPU_ALL_DISABLED) != HAL_OK) {
        printl("GGPU ERROR GPIOB", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    }
    if (mcp23017_ipol(hdev, MCP23017_PORTA, MCP23017_IPOL_ALL_NORMAL) != HAL_OK) {
        printl("IPOL ERROR GPIOA", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    }
    if (mcp23017_ipol(hdev, MCP23017_PORTB, MCP23017_IPOL_ALL_NORMAL) != HAL_OK) {
        printl("IPOL ERROR GPIOB", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    }
}
/**
 * @brief Read and print registers GPIOA and GPIOB
 * 
 * @param hdev MCP23017_HandleTypeDef handle for the device
 * @param hi2c I2C Handle
 */
void mcp23017_read_and_print_both(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(hi2c, hdev->addr, REGISTER_GPIOA, 1, hdev->gpio, 1, 100);
    if (res != HAL_OK) {
        printl("GPIOA READ ERROR", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    } else {
        error_reset(ERROR_MCP23017, 0);
        mcp23017_print_gpioA(hdev, NULL);
    }
    res = HAL_I2C_Mem_Read(hi2c, hdev->addr, REGISTER_GPIOB, 1, hdev->gpio + 1, 1, 100);
    if (res != HAL_OK) {
        printl("GPIOB READ ERROR", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    } else {
        error_reset(ERROR_MCP23017, 0);
        mcp23017_print_gpioB(hdev, NULL);
    }
}

/**
 * @brief      Read registers GPIOA and GPIOB
 * 
 * @param hdev MCP23017_HandleTypeDef handle for the device
 * @param hi2c I2C Handle
 */
void mcp23017_read_both(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(hi2c, hdev->addr, REGISTER_GPIOA, 1, hdev->gpio, 1, 100);
    error_reset(ERROR_MCP23017, 0);
    if (res != HAL_OK) {
        printl("GPIOA READ ERROR", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    }
    res = HAL_I2C_Mem_Read(hi2c, hdev->addr, REGISTER_GPIOB, 1, hdev->gpio + 1, 1, 100);
    if (res != HAL_OK) {
        printl("GPIOB READ ERROR", ERR_HEADER);
        error_set(ERROR_MCP23017, 0, HAL_GetTick());
    }
}

uint8_t mcp23017_get_state(MCP23017_HandleTypeDef *hdev, uint8_t gpio_port, uint8_t gpio_pin) {
    return (hdev->gpio[gpio_port] & (1 << gpio_pin)) >> gpio_pin;
}