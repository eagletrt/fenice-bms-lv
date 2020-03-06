/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */

  /* USER CODE END ET */

  /* Exported constants --------------------------------------------------------*/
  /* USER CODE BEGIN EC */

  /* USER CODE END EC */

  /* Exported macro ------------------------------------------------------------*/
  /* USER CODE BEGIN EM */

  /* USER CODE END EM */

  void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

  /* USER CODE BEGIN EFP */
  void user_pwm_setvalue(uint32_t, TIM_HandleTypeDef *, uint32_t);
  int CAN_Read_Message();
  int send_CAN_data(uint32_t);

  void check_under_voltage();
  void check_over_temperature();

  int BMS_ON_OFF();

  void write_error_led();

  typedef struct temperatures_struct
  {
    uint32_t value;

    uint32_t prev_value;
    uint32_t desired;
    uint32_t max_temp;
  } temperatures_struct;

  typedef enum
  {
    ON,
    OFF,
    BLINK_error,
    BLINK_underV,
    BLINK_overT,
    BLINK_both
  } LED_STATE;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CURRENT_SENSOR_Pin GPIO_PIN_2
#define CURRENT_SENSOR_GPIO_Port GPIOA
#define Fan2_Pin GPIO_PIN_5
#define Fan2_GPIO_Port GPIOA
#define Fan5_Pin GPIO_PIN_6
#define Fan5_GPIO_Port GPIOA
#define Pump2_Pin GPIO_PIN_12
#define Pump2_GPIO_Port GPIOD
#define Pump1_Pin GPIO_PIN_13
#define Pump1_GPIO_Port GPIOD
#define RELAY_Pin GPIO_PIN_6
#define RELAY_GPIO_Port GPIOC
#define LED_ERR_Pin GPIO_PIN_7
#define LED_ERR_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_4
#define SPI2_CS_GPIO_Port GPIOD
  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
