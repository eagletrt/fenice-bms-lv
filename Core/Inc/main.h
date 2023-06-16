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
#define PC13_NC_Pin                          GPIO_PIN_13
#define PC13_NC_GPIO_Port                    GPIOC
#define PC14_NC_Pin                          GPIO_PIN_14
#define PC14_NC_GPIO_Port                    GPIOC
#define RELAY_Pin                            GPIO_PIN_15
#define RELAY_GPIO_Port                      GPIOC
#define CENTER_FB_Pin                        GPIO_PIN_0
#define CENTER_FB_GPIO_Port                  GPIOC
#define PC1_NC_Pin                           GPIO_PIN_1
#define PC1_NC_GPIO_Port                     GPIOC
#define AS_COMPUTER_FB_Pin                   GPIO_PIN_3
#define AS_COMPUTER_FB_GPIO_Port             GPIOC
#define MUX_FB_OUT_Pin                       GPIO_PIN_0
#define MUX_FB_OUT_GPIO_Port                 GPIOA
#define MUX_HALL_Pin                         GPIO_PIN_1
#define MUX_HALL_GPIO_Port                   GPIOA
#define VREF_ADC_Pin                         GPIO_PIN_2
#define VREF_ADC_GPIO_Port                   GPIOA
#define BATT_OUT_ANALOG_FB_Pin               GPIO_PIN_3
#define BATT_OUT_ANALOG_FB_GPIO_Port         GPIOA
#define PUMP_L_Pin                           GPIO_PIN_4
#define PUMP_L_GPIO_Port                     GPIOA
#define PUMP_R_Pin                           GPIO_PIN_5
#define PUMP_R_GPIO_Port                     GPIOA
#define RAD_L_Pin                            GPIO_PIN_6
#define RAD_L_GPIO_Port                      GPIOA
#define RAD_R_Pin                            GPIO_PIN_7
#define RAD_R_GPIO_Port                      GPIOA
#define REAY_OUT_ANALOG_FB_Pin               GPIO_PIN_4
#define REAY_OUT_ANALOG_FB_GPIO_Port         GPIOC
#define LVMS_OUT_ANALOG_FB_Pin               GPIO_PIN_5
#define LVMS_OUT_ANALOG_FB_GPIO_Port         GPIOC
#define FAN_Pin                              GPIO_PIN_0
#define FAN_GPIO_Port                        GPIOB
#define STP_DIRECTION_Pin                    GPIO_PIN_1
#define STP_DIRECTION_GPIO_Port              GPIOB
#define NC_MCU0_Pin                          GPIO_PIN_2
#define NC_MCU0_GPIO_Port                    GPIOB
#define LTC_CS_Pin                           GPIO_PIN_10
#define LTC_CS_GPIO_Port                     GPIOB
#define TIME_SET_Pin                         GPIO_PIN_12
#define TIME_SET_GPIO_Port                   GPIOB
#define BUZZER_Pin                           GPIO_PIN_6
#define BUZZER_GPIO_Port                     GPIOC
#define MUX_A0_Pin                           GPIO_PIN_7
#define MUX_A0_GPIO_Port                     GPIOC
#define MUX_A1_Pin                           GPIO_PIN_8
#define MUX_A1_GPIO_Port                     GPIOC
#define MUX_A2_Pin                           GPIO_PIN_11
#define MUX_A2_GPIO_Port                     GPIOA
#define MUX_A3_Pin                           GPIO_PIN_12
#define MUX_A3_GPIO_Port                     GPIOA
#define STP_PDN_Pin                          GPIO_PIN_12
#define STP_PDN_GPIO_Port                    GPIOC
#define GPIO_EXTENSION_INTERRUPT_A_Pin       GPIO_PIN_2
#define GPIO_EXTENSION_INTERRUPT_A_GPIO_Port GPIOD
#define NC_NC_Pin                            GPIO_PIN_4
#define NC_NC_GPIO_Port                      GPIOB
#define STP_STEP_Pin                         GPIO_PIN_7
#define STP_STEP_GPIO_Port                   GPIOB
/* USER CODE BEGIN Private defines */

/**
 * @brief     Get the currente state of the low voltage master relay
 * 
 * @return GPIO_PIN_RESET: if LVMR is open
 * @return GPIO_PIN_SET: if LVMR is closed
 */
static inline GPIO_PinState LV_MASTER_RELAY_get_state() {
    //return mcp23017_get_state(&hmcp, MCP23017_PORTB, FB_FAN) == 0x1 ? true : false;
    return HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin);
}

/**
 * @brief  Set the low voltage master relay to either on of off.
 *         Switching LVMR takes a little bit of time 50ms  
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
    return mcp23017_get_state(&hmcp, MCP23017_PORTB, FB_FAN) == 0x1 ? true : false;
}
/**
 * @brief Get the feedback from the Relay 
 * 
 * @return true  Relay is closed and there are at least 12V on board
 * @return false Relay is open
 */
static inline bool FDBK_RELAY_get_state() {
    //return true;  //
    return mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_FAN) == 0x1 ? true : false;
}

/**
 * @brief Get the feedback from the 24V DCDC output
 * 
 * @return true  The 24V DCDC it's working properly
 * @return false The 24V DCDC it's not working properly
 */
static inline bool FDBK_DCDC_24V_get_state() {
    //return true;  //
    return mcp23017_get_state(&hmcp, MCP23017_PORTA, FB_FAN) == 0x1 ? true : false;
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
    return mcp23017_get_state(&hmcp, MCP23017_PORTB, FB_FAN) == 0x1 ? true : false;
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

/**
 * @brief Check feedback from the gpio extender
 * 
 * @param feedback gpio feedback
 * @return value of the feedback
 */
static inline uint8_t get_feedback_state(uint8_t feedback) {
    uint8_t port = (feedback < 8) ? MCP23017_PORTA : MCP23017_PORTB;
    return mcp23017_get_state(&hmcp, port, feedback);
}

#define MAX_RADIATOR_DUTY_CYCLE          0.90  // 1.0
#define MIN_RADIATOR_DUTY_CYCLE          0.5
#define MAX_INTERNAL_FAN_DUTY_CYCLE      1.0
#define MIN_INTERNAL_FAN_DUTY_CYCLE      0.1
#define INTERNAL_FAN_M_FACTOR            (MAX_INTERNAL_FAN_DUTY_CYCLE - MIN_INTERNAL_FAN_DUTY_CYCLE) / (60 - 20)
#define INTERNAL_FAN_Q_FACTOR            (MIN_INTERNAL_FAN_DUTY_CYCLE) - (20 * INTERNAL_FAN_M_FACTOR)
#define MIN_INVERTER_TEMP_RADIATORS_ONLY 35.0
#define MAX_INVERTER_TEMP_RADIATORS_ONLY 60.0
#define MIN_INVERTER_TEMP_PUMPS_ONLY     40.0
#define MAX_INVERTER_TEMP_PUMPS_ONLY     60.0

#define CANP hcan1
#define CANS hcan2

#define LOG_HUART huart1

/* RAD_L      -> TIM3 CH1 */
#define RAD_L_HTIM         htim3
#define RAD_L_PWM_TIM_CHNL TIM_CHANNEL_1
/* RAD_R      -> TIM3 CH2 */
#define RAD_R_HTIM         htim3
#define RAD_R_PWM_TIM_CHNL TIM_CHANNEL_2
/* INTERNAL_FAN      -> TIM2 CH3 */
#define INTERNAL_FAN_HTIM         htim3
#define INTERNAL_FAN_PWM_TIM_CHNL TIM_CHANNEL_3

/* BUZZER    -> TIM8 CH1 */
#define BZZR_HTIM         htim8
#define BZZR_PWM_TIM_CHNL TIM_CHANNEL_1
#define BUZZER_ALARM_TIME 200

/* PUMPS*/
#define PUMP_DAC    hdac
#define PUMP_L_CHNL DAC_CHANNEL_2
#define PUMP_R_CHNL DAC_CHANNEL_1

/* MEASUREMENTS */
#define MEASUREMENTS_TIMER                               htim2
#define MEAS_ALL_ANALOG_SIGNALS_TIMER_CHANNEL            TIM_CHANNEL_1
#define MEAS_VOLTS_AND_TEMPS_TIMER_CHANNEL               TIM_CHANNEL_2
#define MEAS_LV_VERSION_AND_COOLING_TIMER_CHANNEL        TIM_CHANNEL_3
#define MEAS_CELL_TEMPS_TIMER_CHANNEL                    TIM_CHANNEL_4
#define MEAS_ALL_ANALOG_SIGNALS_TIMER_ACTIVE_CHANNEL     HAL_TIM_ACTIVE_CHANNEL_1
#define MEAS_VOLTS_AND_TEMPS_TIMER_ACTIVE_CHANNEL        HAL_TIM_ACTIVE_CHANNEL_2
#define MEAS_LV_VERSION_AND_COOLING_TIMER_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_3
#define MEAS_CELL_TEMPS_TIMER_ACTIVE_CHANNEL             HAL_TIM_ACTIVE_CHANNEL_4
#define OPEN_WIRE_MEASUREMENT_TIMER                      htim4
#define OPEN_WIRE_TIMER_CHANNEL                          TIM_CHANNEL_2
#define OPEN_WIRE_TIMER_ACTIVE_CHANNEL                   HAL_TIM_ACTIVE_CHANNEL_2
/* ERRORS TIMER*/
#define HTIM_ERR htim5

/* ADC STUFF */

#define TIMER_ADC_MEAS                htim10
#define TIMER_ADC_CALIBRATION         htim1
#define TIMER_ADC_CALIBRATION_CHANNEL TIM_CHANNEL_1
#define ADC_HALL_AND_FB               hadc2
#define CALIBRATION_ADC               hadc1

// Commented: enable debugging, Uncommented: disable debugging
//#define NDEBUG
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
