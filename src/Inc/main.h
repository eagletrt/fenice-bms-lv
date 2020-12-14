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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
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

  void print_uart(char *);

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
#define FAN1_HV_Pin GPIO_PIN_1
#define FAN1_HV_GPIO_Port GPIOA
#define CURRENT_SENSOR_Pin GPIO_PIN_2
#define CURRENT_SENSOR_GPIO_Port GPIOA
#define T_BATT1_Pin GPIO_PIN_3
#define T_BATT1_GPIO_Port GPIOA
#define FAN2_HV_Pin GPIO_PIN_5
#define FAN2_HV_GPIO_Port GPIOA
#define FAN3_HV_Pin GPIO_PIN_6
#define FAN3_HV_GPIO_Port GPIOA
#define T_BATT2_Pin GPIO_PIN_7
#define T_BATT2_GPIO_Port GPIOA
#define RELAY_Pin GPIO_PIN_5
#define RELAY_GPIO_Port GPIOC
#define T_DCDC12_Pin GPIO_PIN_0
#define T_DCDC12_GPIO_Port GPIOB
#define T_DCDC24_Pin GPIO_PIN_1
#define T_DCDC24_GPIO_Port GPIOB
#define FAN6_LV_Pin GPIO_PIN_10
#define FAN6_LV_GPIO_Port GPIOB
#define FAN5_LV_Pin GPIO_PIN_11
#define FAN5_LV_GPIO_Port GPIOB
#define PUMP_2_Pin GPIO_PIN_12
#define PUMP_2_GPIO_Port GPIOD
#define PUMP_1_Pin GPIO_PIN_13
#define PUMP_1_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOC
#define LED_ERR_Pin GPIO_PIN_7
#define LED_ERR_GPIO_Port GPIOC
#define SDMMC2_CS_Pin GPIO_PIN_2
#define SDMMC2_CS_GPIO_Port GPIOD
#define SPI2_CS_Pin GPIO_PIN_4
#define SPI2_CS_GPIO_Port GPIOD
#define SD_DETECT_Pin GPIO_PIN_5
#define SD_DETECT_GPIO_Port GPIOD
#define RADIATOR_1_Pin GPIO_PIN_8
#define RADIATOR_1_GPIO_Port GPIOB
#define RADIATOR_2_Pin GPIO_PIN_9
#define RADIATOR_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
