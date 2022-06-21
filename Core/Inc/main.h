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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mcp23017.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern bool is_bms_on_fault;
extern float bms_fan_duty_cycle;
typedef struct temperatures_struct {
    uint32_t value;

    uint32_t prev_value;
    uint32_t desired;
    uint32_t max_temp;
} temperatures_struct;

typedef enum { ON, OFF, BLINK_error, BLINK_underV, BLINK_overT, BLINK_both } LED_STATE;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void set_sensor_update_time();
void bms_error_state();
void check_on_feedbacks();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_CS_Pin GPIO_PIN_13
#define SD_CS_GPIO_Port GPIOC
#define SD_DETECT_Pin GPIO_PIN_15
#define SD_DETECT_GPIO_Port GPIOC
#define HALL_Pin GPIO_PIN_0
#define HALL_GPIO_Port GPIOC
#define HALL_OCD_Pin GPIO_PIN_1
#define HALL_OCD_GPIO_Port GPIOC
#define HALL_OCD_EXTI_IRQn EXTI1_IRQn
#define TMP_DCDC12_Pin GPIO_PIN_2
#define TMP_DCDC12_GPIO_Port GPIOC
#define TMP_DCDC24_Pin GPIO_PIN_3
#define TMP_DCDC24_GPIO_Port GPIOC
#define TMP_BATT1_Pin GPIO_PIN_0
#define TMP_BATT1_GPIO_Port GPIOA
#define TMP_BATT2_Pin GPIO_PIN_1
#define TMP_BATT2_GPIO_Port GPIOA
#define PUMP_L_Pin GPIO_PIN_4
#define PUMP_L_GPIO_Port GPIOA
#define PUMP_R_Pin GPIO_PIN_5
#define PUMP_R_GPIO_Port GPIOA
#define RAD_L_Pin GPIO_PIN_6
#define RAD_L_GPIO_Port GPIOA
#define RAD_R_Pin GPIO_PIN_7
#define RAD_R_GPIO_Port GPIOA
#define INV_FRG_Pin GPIO_PIN_4
#define INV_FRG_GPIO_Port GPIOC
#define INV_RFE_Pin GPIO_PIN_5
#define INV_RFE_GPIO_Port GPIOC
#define FAN_Pin GPIO_PIN_0
#define FAN_GPIO_Port GPIOB
#define RELAY_Pin GPIO_PIN_1
#define RELAY_GPIO_Port GPIOB
#define LTC_CS_Pin GPIO_PIN_10
#define LTC_CS_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOC
#define L_ERR_Pin GPIO_PIN_7
#define L_ERR_GPIO_Port GPIOC
#define L_OTHER_Pin GPIO_PIN_8
#define L_OTHER_GPIO_Port GPIOC
#define EEPROM_WP_Pin GPIO_PIN_15
#define EEPROM_WP_GPIO_Port GPIOA
#define EEPROM_CS_Pin GPIO_PIN_4
#define EEPROM_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/**
 * @brief     Get the currente state of the low voltage master relay
 * 
 * @return GPIO_PIN_RESET: if LVMR is open
 * @return GPIO_PIN_SET: if LVMR is closed
 */
static inline GPIO_PinState LV_MASTER_RELAY_get_state() {
    //return true;
    return mcp23017_get_state(&hmcp, MCP23017_PORTB, FB_RELAY) == 0x1 ? true : false;
}

/**
 * @brief  Set the low voltage master relay to either on of off.
 *         Switching LVMR takes a little bit of time 50ms  
 * 
 * @param  state gpio pin state
 *         @arg GPIO_PIN_RESET: to open LVMR
  *        @arg GPIO_PIN_SET: to close LVMR
 */
static inline void LV_MASTER_RELAY_set_state(GPIO_PinState state) {
    //HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, state);
    HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, state);
    HAL_Delay(50);  // Let the relay close
}

/**
 * @brief     Get the feedback from the 12V DCDC output.
 *            Used to know if the bms lv is powering the machine.
 *            NOTE: if LV_MASTER_REALY is on (closed) but this feedback is off,
 *            either the LVMS is opened or a fuse is blown.
 * 
 * @return    true The 12V DCDC is powering the machine
 * @return    false The 12V DCDC is off or something else in the main line is opened
 */
static inline bool FDBK_DCDC_12V_get_state() {
    //return true;  //
    return mcp23017_get_state(&hmcp, MCP23017_PORTB, FB_12) == 0x1 ? true : false;
}
/**
 * @brief Get the feedback from the Relay 
 * 
 * @return true  Relay is closed and there are at least 12V on board
 * @return false Relay is open
 */
static inline bool FDBK_RELAY_get_state() {
    //return true;  //
    return mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_RELAY) == 0x1 ? true : false;
}

/**
 * @brief Get the feedback from the 24V DCDC output
 * 
 * @return true  The 24V DCDC it's working properly
 * @return false The 24V DCDC it's not working properly
 */
static inline bool FDBK_DCDC_24V_get_state() {
    //return true;  //
    return mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_24) == 0x1 ? true : false;
}

/**
 * @brief Get the feedback from Pumps voltage supply
 * 
 * @return true Pumps can work because of the presence of 24V
 * @return false Pumps can't work
 */
static inline bool FDBK_24V_PUMPS_get_state() {
    return mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_PUMPS) == 0x1 ? true : false;
}

/**
 * @brief Get the feedback from Radiators voltage supply
 * 
 * @return true Radiators can work because of the presence of 12V
 * @return false Radiators can't work
 */
static inline bool FDBK_12V_RADIATORS_get_state() {
    return mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_RADIATORS) == 0x1 ? true : false;
}

/**
 * @brief Get the feedback from Shutdown fuse
 * 
 * @return true Fuse it's working properly and there are 12V over it
 * @return false Fuse it's not working properly, it could be open
 */
static inline bool FDBK_12V_SHUTDOWN_get_state() {
    return mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_SHUTDOWN) == 0x1 ? true : false;
}

/**
 * @brief Get the feedback from the FANS voltage supply
 * 
 * @return true Fans can work because of the presence of 12V
 * @return false Fans can't work because of the presence of 12V
 */
static inline bool FDBK_12V_FANS_get_state() {
    return mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_FAN) == 0x1 ? true : false;
}

/**
 * @brief Check if there are 12V on board, needed to supply other boards
 * 
 * @return true There are 12V on board
 * @return false There aren't 12V on board
 */
static inline bool FDBK_12V_PCBS_get_state() {
    return mcp23017_get_state(&hmcp, MCP23017_PORTB, FB_FAN) == 0x1 ? true : false;
}

/**
 * @brief Check if LVMS it's closed
 * 
 * @return true LVMS closed
 * @return false LVMS open
 */
static inline bool FDBK_LVMS_get_state() {
    return mcp23017_get_state(&hmcp, MCP23017_PORTB, FB_MAIN) == 0x1 ? true : false;
}

/**
 * @brief Check if Inverters are powered with 24V
 * 
 * @return true The inverter is powered with 24V
 * @return false The inverter is not powered with 24V
 */
static inline bool FDBK_24V_INVERTERS_get_state() {
    return mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_INVERTERS) == 0x1 ? true : false;
}

#define MAX_FAN_DUTY_CYCLE 1.0
#define MIN_FAN_DUTY_CYCLE 0.8
#define MIN_MOTOR_TEMP     20.0  // Maybe change it to 40?
#define MAX_MOTOR_TEMP     60.0

#define CANP hcan1
#define CANS hcan2

#define LOG_HUART huart1

/* RAD_L      -> TIM3 CH1 */
#define RAD_L_HTIM         htim3
#define RAD_L_PWM_TIM_CHNL TIM_CHANNEL_1
/* RAD_R      -> TIM3 CH2 */
#define RAD_R_HTIM         htim3
#define RAD_R_PWM_TIM_CHNL TIM_CHANNEL_2
/* FAN5      -> TIM2 CH4 */
#define FAN5_HTIM         htim2
#define FAN5_PWM_TIM_CHNL TIM_CHANNEL_4
/* FAN6      -> TIM2 CH3 */
#define FAN6_HTIM         htim3
#define FAN6_PWM_TIM_CHNL TIM_CHANNEL_3

/* BUZZER    -> TIM8 CH1 */
#define BZZR_HTIM         htim8
#define BZZR_PWM_TIM_CHNL TIM_CHANNEL_1
#define BUZZER_ALARM_TIME 1000

/* PUMPS*/
#define PUMP_DAC    hdac
#define PUMP_L_CHNL DAC_CHANNEL_2
#define PUMP_R_CHNL DAC_CHANNEL_1

/* MEASUREMENTS */
#define MEASUREMENTS_TIMER                          htim2
#define COOLING_AND_LV_VERSION_TIMER_CHANNEL        TIM_CHANNEL_1
#define CURRENT_TIMER_CHANNEL                       TIM_CHANNEL_2
#define VOLTAGE_AND_TEMPS_TIMER_CHANNEL             TIM_CHANNEL_3
#define COOLING_AND_LV_VERSION_TIMER_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_1
#define CURRENT_TIMER_ACTIVE_CHANNEL                HAL_TIM_ACTIVE_CHANNEL_2
#define VOLTAGE_AND_TEMPS_TIMER_ACTIVE_CHANNEL      HAL_TIM_ACTIVE_CHANNEL_3
#define OPEN_WIRE_MEASUREMENT_TIMER                 htim4
#define OPEN_WIRE_TIMER_CHANNEL                     TIM_CHANNEL_2
#define OPEN_WIRE_TIMER_ACTIVE_CHANNEL              HAL_TIM_ACTIVE_CHANNEL_2

/* ERRORS TIMER*/
#define HTIM_ERR               htim2
#define ERR_TIM_CHANNEL        TIM_CHANNEL_4
#define ERR_TIM_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_4

/* ADC STUFF */

/* CURRENT TRANSDUCER */

#define ADC_ELECTRICAL_CURRENT_READINGS_TIMER_HTIM    htim5
#define ADC_ELECTRICAL_CURRENT_READINGS_TIMER_CHANNEL TIM_CHANNEL_1

/* - HALL (Current transducer) -> ADC 1 CHANNEL10*/
#define CURRENT_TRANSDUCER_HADC     hadc1
#define CURRENT_TRANSDUCER_ADC_CHNL ADC_CHANNEL_10

#define CURRENT_TRANSDUCER_OCD_GPIO_PIN  HALL_OCD_Pin
#define CURRENT_TRANSDUCER_OCD_GPIO_PORT HALL_OCD_GPIO_Port

/* THERMOCOUPLES */
#define ADC_TEMPERATURE_READINGS_TIMER_HTIM htim7

/* - TMP_BATT1 (Battery Temperature sensor #1) -> ADC 1 CHANNEL0*/
#define T_SENS_BATT1_HADC     hadc2
#define T_SENS_BATT1_ADC_CHNL ADC_CHANNEL_0

/* - TMP_BATT2 (Battery Temperature sensor #2) -> ADC 1 CHANNEL1*/
#define T_SENS_BATT2_HADC     hadc2
#define T_SENS_BATT2_ADC_CHNL ADC_CHANNEL_1

/* - TMP_DCDC12 (12 volts DCDC converter Temperature sensor) -> ADC 1 CHANNEL12*/
#define T_SENS_DCDC12V_HADC     hadc2
#define T_SENS_DCDC12V_ADC_CHNL ADC_CHANNEL_12

/* - TMP_DCDC24 (24 volts DCDC converter Temperature sensor) -> ADC 1 CHANNEL13*/
#define T_SENS_DCDC24V_HADC     hadc2
#define T_SENS_DCDC24V_ADC_CHNL ADC_CHANNEL_13

// Commented: enable debugging, Uncommented: disable debugging
//#define NDEBUG
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
