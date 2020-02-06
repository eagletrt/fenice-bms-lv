
#include "ltc.h"
/*
* default sample frequency -> 7kHz
* 670 us needed to synchronize
* 
*/
ltc_struct ltc;

uint16_t get_ADCV_CC(uint8_t Ncell){
	if(Ncell > 6 || Ncell < 1) Ncell = 0;
	return(0x260 | (MD7K << 7) | Ncell);
}
void set_pins(GPIO_TypeDef *pinx, uint8_t pinn)
{
    CS_LTCx = pinx;
    CS_LTCn = pinn;
}

void LTC_init(SPI_HandleTypeDef *hspi, uint8_t _address)
{
	ltc.address = _address;
	ltc.spi = hspi;
}

void read_voltages()
{
	uint8_t cmd[4];
	uint16_t CC;
	uint16_t cmd_pec;
	uint8_t data[8];

	CC = get_ADCV_CC(0);
	cmd[0] = (uint8_t)0x80 | // Address Command mode
		(ltc.address << 3) | // LTC address 
		(CC >> 8);			 // Send 3 most significant bit of CC
	cmd[1] = (uint8_t)(CC);  // Send the other 8 bit of CC

	cmd_pec = _pec15(2, cmd); // Calculate the PEC
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

	ltc6810_wakeup_idle(ltc.spi);

	spi_enable_cs(ltc.spi);
	HAL_SPI_Transmit(ltc.spi, cmd, 4, 100);
	HAL_SPI_Receive(ltc.spi, data, 8, 100);
	spi_disable_cs(ltc.spi);
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
	while ((spi->State != HAL_SPI_STATE_READY) && (spi->State != HAL_SPI_STATE_ERROR));
}

void spi_disable_cs(SPI_HandleTypeDef *spi) {
	while ((spi->State != HAL_SPI_STATE_READY) && (spi->State != HAL_SPI_STATE_ERROR));
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