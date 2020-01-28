
#include "ltc.h"
/*
* default sample frequency -> 7kHz
* 670 us needed to synchronize
* 
*/

void set_pins(GPIO_TypeDef *pinx, uint8_t pinn)
{
    CS_LTCx = pinx;
    CS_LTCn = pinn;
}

void init()
{
}

void read_voltages()
{
}

void read_temperatures()
{
}

void ltc6813_wakeup_idle(SPI_HandleTypeDef *hspi)
{
    uint8_t data = 0xFF;
    HAL_GPIO_WritePin(CS_LTCx, CS_LTCn, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &data, 1, 1);
    HAL_GPIO_WritePin(CS_LTCx, CS_LTCn, GPIO_PIN_SET);
}
