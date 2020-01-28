
#ifndef LTC_H_
#define LTC_H_

#include "inttypes.h"
#include <stm32f7xx_hal.h>

void init_ltc();
void read_voltages();
void read_temperatures();
void set_pins(GPIO_TypeDef *pinx, uint8_t pinn);
void ltc6813_wakeup_idle(SPI_HandleTypeDef *hspi);

GPIO_TypeDef *CS_LTCx;
uint8_t CS_LTCn;

#endif /* LTC_H_ */