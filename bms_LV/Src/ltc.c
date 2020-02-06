
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

void ltc6810_wakeup_idle(SPI_HandleTypeDef *hspi)
{
    uint8_t data = 0xFF;
    spi_enable_cs(hspi);
    HAL_SPI_Transmit(hspi, &data, 1, 1);
    spi_disable_cs(hspi);
}
void spi_enable_cs(SPI_HandleTypeDef *spi) {
	HAL_GPIO_WritePin(CS_LTCx, CS_LTCn, GPIO_PIN_RESET);
	while (spi->State != HAL_SPI_STATE_READY);
}

void spi_disable_cs(SPI_HandleTypeDef *spi) {
	while (spi->State != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(CS_LTCx, CS_LTCn, GPIO_PIN_SET);
}

uint16_t _pec15(uint8_t len, uint8_t data[]) {
	uint16_t remainder, address;
	remainder = 16;  // PEC seed
	for (int i = 0; i < len; i++) {
		// calculate PEC table address
		address = ((remainder >> 7) ^ data[i]) & 0xff;
		remainder = (remainder << 8) ^ crcTable[address];
	}
	// The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
	return (remainder * 2);
}