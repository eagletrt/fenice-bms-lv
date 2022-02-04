/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "adc.h"
#include "can.h"
#include "dac.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "../Lib/micro-libs/pid/pid.h"
#include "buzzer.h"
#include "common.h"
#include "current_sensor.h"
#include "fenice-config.h"
#include "ltc.h"
#include "pwm.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*----------------------------------------------------------------------------*/
/* Radiator1 -> TIM4 CH3 */
/* Radiator2 -> TIM4 CH4 */

/* FAN1      -> TIM2 CH2 */
/* FAN2      -> TIM2 CH1 */
/* FAN3      -> TIM3 CH1 */
/* FAN5      -> TIM2 CH4 */
/* FAN6      -> TIM2 CH3 */

/* PUMP1     -> TIM4 CH2 */
/* PUMP2     -> TIM4 CH1 */

/*----------------------------------- ERRORS ---------------------------------*/

static bool is_bms_on_requested = true;
static bool is_bms_on           = false;

int UNDER_VOLTAGE    = 0;
int OVER_TEMPERATURE = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

const uint32_t ERR_LED_BLINK_error_MSEC      = 100;
const uint32_t ERR_LED_BLINK_overT_ON_MSEC   = 800;
const uint32_t ERR_LED_BLINK_overT_OFF_MSEC  = 200;
const uint32_t ERR_LED_BLINK_underV_ON_MSEC  = 100;
const uint32_t ERR_LED_BLINK_underV_OFF_MSEC = 900;

int prev_toggle_msec = 0;

// cooling system PWM
pwm_struct lv_pwm;
pwm_struct hv_pwm;
pwm_struct pump_pwm;

// PID
//PID_HandleTypeDef lv_pid;
//PID_HandleTypeDef hv_pid;
//PID_HandleTypeDef pump_pid;

// Temperatures
temperatures_struct lv_temp;
temperatures_struct hv_temp;
temperatures_struct pump_temp;

// LTC
extern canStruct can1, can3;

// ERROR LED
LED_STATE led_state;

uint32_t over_temp_start_msec, under_volt_start_msec;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void _signal_mx_init_succes();
static void _test_buzzer();
static void write_error_led();
static inline void check_over_temperature();
static inline void check_under_voltage();
static inline bool bms_on_off();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool is_sensors_update_time = 0;
int debug_flag                     = 0;
int m_sec_timer                    = 0;

uint32_t led_last_tick;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_DAC_Init();
    MX_I2C3_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_SPI3_Init();
    MX_TIM8_Init();
    /* USER CODE BEGIN 2 */

    // Blink to signal correct MX_XXX_init processes (usuefull for CAN transciever)
    _signal_mx_init_succes();

    BZZR_play_pulses(300, 4);
    LV_MASTER_RELAY_set_state(GPIO_PIN_SET);
    BZZR_play_pulses(300, 4);

    CAN_start_all();  //TODO manage false can start
#if 0
    LTC_init(&ltc, &hspi2, 0, GPIOD, GPIO_PIN_4);  // init function of LTC_6810


    UserAdcConfig.Channel      = ADC_CHANNEL_2;
    UserAdcConfig.Rank         = ADC_REGULAR_RANK_1;
    UserAdcConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &UserAdcConfig) != HAL_OK) {
        Error_Handler();
    }
    HAL_ADC_Start_IT(&hadc1);

    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_Base_Start(&htim8);

    // Initializing PWM
    Init_pwm(&hv_pwm, &htim2, TIM_CHANNEL_1);
    Init_pwm(&lv_pwm, &htim3, TIM_CHANNEL_1);
    Init_pwm(&pump_pwm, &htim4, TIM_CHANNEL_1);

    // Initializing temperatures
    lv_temp.desired   = 45;
    hv_temp.desired   = 45;
    pump_temp.desired = 45;

    lv_temp.max_temp   = 70;
    hv_temp.max_temp   = 70;
    pump_temp.max_temp = 70;

    // Initializing PID
    double sample_time = (htim8.Init.Prescaler * htim8.Init.Period) / 108000000;
    PID_Init(&lv_pid, 10, 0.01, 0.01, (float)sample_time, 100, 1000, PID_MODE_AUTOMATIC, PID_CONTROL_ACTION_REVERSE);
    PID_Init(&hv_pid, 10, 0.01, 0.01, 0.5, 100, 1000, PID_MODE_AUTOMATIC, PID_CONTROL_ACTION_REVERSE);
    PID_Init(&pump_pid, 10, 0.01, 0.01, 0.5, 100, 1000, PID_MODE_AUTOMATIC, PID_CONTROL_ACTION_REVERSE);

    lv_pid.setpoint   = lv_temp.desired;
    hv_pid.setpoint   = hv_temp.desired;
    pump_pid.setpoint = pump_temp.desired;

    lv_temp.value   = 65;
    hv_temp.value   = 50;
    pump_temp.value = 50;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    m_sec_timer         = HAL_GetTick();
    int currentTick     = m_sec_timer;
    int previous_millis = HAL_GetTick();
    led_last_tick       = HAL_GetTick();
#endif
    CAN_RxPQFI_Data_TypeDef rx_data = {};
    while (1) {
        printl("test", NORM_HEADER);
        HAL_Delay(1000);
        //if (CAN_get(&hcanP, &rx_data) != COMM_ERROR) {
        //    CAN_print_rxdata(&rx_data);
        //}
#if 0
        currentTick = HAL_GetTick();

        if (is_sensors_update_time) {
            is_sensors_update_time = false;

            // LTC
            read_voltages(&ltc);

            // Temperatures
            // TODO: read temperatures from adc and read from CANBUS

            lv_pid.input   = lv_temp.value;
            hv_pid.input   = hv_temp.value;
            pump_pid.input = pump_temp.value;

            // PID
            PIDCompute(&lv_pid);
            PIDCompute(&hv_pid);
            PIDCompute(&pump_pid);

            // PWM
            lv_pwm.value   = lv_pid.output;
            hv_pwm.value   = hv_pid.output;
            pump_pwm.value = pump_pid.output;

            check_over_temperature();
            check_under_voltage();
        }

        if (currentTick % 200 == 0) {
            write_pwm_value(&lv_pwm);
            write_pwm_value(&hv_pwm);
            write_pwm_value(&pump_pwm);
        }

#ifndef NDEBUG_VALUES
        M_LOG_STRING("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

        {  // this graphs are used to keep buf[] on the stack and in this
            // local scope
            char buf[100];
            sprintf(buf, "is_bms_on: %d, overT: %d, underV: %d\r\n", is_bms_on, OVER_TEMPERATURE, UNDER_VOLTAGE);
            HAL_UART_Transmit(&huart4, (uint8_t *)buf, strlen(buf), 10);
            sprintf(
                buf,
                "PID outputs lv: %d hv: %d pump: %d\r\n",
                (int)PIDOutputGet(&lv_pid),
                (int)PIDOutputGet(&hv_pid),
                (int)PIDOutputGet(&pump_pid));
            HAL_UART_Transmit(&huart4, (uint8_t *)buf, strlen(buf), 10);
            sprintf(
                buf,
                "Voltages %d %d %d %d %d %d\r\n",
                ltc.voltage[0],
                ltc.voltage[1],
                ltc.voltage[2],
                ltc.voltage[3],
                ltc.voltage[4],
                ltc.voltage[5]);
            HAL_UART_Transmit(&huart4, (uint8_t *)buf, strlen(buf), 10);
            sprintf(buf, "Current from sensor: %lu\r\n", get_current());
            HAL_UART_Transmit(&huart4, (uint8_t *)buf, strlen(buf), 10);
        }
#endif

        if (previous_millis != currentTick) {
            bool ret        = old_CAN_Send_data(currentTick);
            previous_millis = currentTick;
#ifndef NDEBUG_CAN_SEND
            if (ret != 0) {
                char buf[50];
                sprintf(buf, "Sent message: %d\r\n", sent);
                HAL_UART_Transmit(&huart4, (uint8_t *)buf, strlen(buf), 10);
            }
#endif
        }

        if (is_bms_on) {
            HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
            led_state = ON;
        } else {
            HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
            if (UNDER_VOLTAGE) {
                led_state = BLINK_underV;
                if (OVER_TEMPERATURE)
                    led_state = BLINK_error;
            } else {
                if (OVER_TEMPERATURE)
                    led_state = BLINK_overT;
                else
                    led_state = OFF;
            }
        }
        write_error_led(led_state);

        bms_on_off();

        if (HAL_GetTick() - led_last_tick > 1000) {
            led_last_tick = HAL_GetTick();
            HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#endif
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
  */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  RCC Clock Security System interrupt callback
  * @retval None
  */
void HAL_RCC_CSSCallback(void) {
    //TODO change into something meaningfull
    for (int i = 0; i < 3; i++) {
        //HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
        HAL_Delay(100);
    }
}

/* Private functions =--------------------------------------------------------*/
static void write_error_led() {
    // if (led_state == ON) {
    //     HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
    // } else if (led_state == BLINK_error) {
    //     if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_error_MSEC) {
    //         HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
    //         prev_toggle_msec = HAL_GetTick();
    //     }
    // } else if (led_state == BLINK_overT) {
    //     if (HAL_GPIO_ReadPin(LED_ERR_GPIO_Port, LED_ERR_Pin)) {
    //         if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_overT_ON_MSEC) {
    //             HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
    //             prev_toggle_msec = HAL_GetTick();
    //         }
    //     } else {
    //         if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_overT_OFF_MSEC) {
    //             HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
    //             prev_toggle_msec = HAL_GetTick();
    //         }
    //     }
    // } else if (led_state == BLINK_underV) {
    //     if (HAL_GPIO_ReadPin(LED_ERR_GPIO_Port, LED_ERR_Pin)) {
    //         if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_underV_ON_MSEC) {
    //             HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
    //             prev_toggle_msec = HAL_GetTick();
    //         }
    //     } else {
    //         if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_underV_OFF_MSEC) {
    //             HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
    //             prev_toggle_msec = HAL_GetTick();
    //         }
    //     }
    // } else if (led_state == OFF) {
    //     HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
    // } else {
    //     HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
    // }
}

static inline void check_under_voltage() {
    if (get_min_voltage(&ltc) < 10) {
        UNDER_VOLTAGE = 1;
    } else {
        UNDER_VOLTAGE = 0;
    }
}

static inline void check_over_temperature() {
    if (lv_temp.value > lv_temp.max_temp) {
        OVER_TEMPERATURE = 1;
    } else {
        OVER_TEMPERATURE = 0;
    }
}

static inline bool bms_on_off() {
    if (is_bms_on_requested) {
        if (!OVER_TEMPERATURE && !UNDER_VOLTAGE) {
            is_bms_on = true;
        } else {
            is_bms_on = false;
        }
    } else {
        is_bms_on = false;
    }
    return is_bms_on;
}
void is_sensor_update_time() {
    is_sensors_update_time = true;
}
static void _signal_mx_init_succes() {
    printl("MX_INIT Success!", NORM_HEADER);

    //HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
    //for (int i = 0; i < 2; i++) {
    //    HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    //    HAL_Delay(50);
    //    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    //    HAL_Delay(50);
    //    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    //    HAL_Delay(50);
    //    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
    //    HAL_Delay(50);
    //    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_SET);
    //    HAL_Delay(100);
    //    HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
    //    HAL_Delay(50);
    //    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    //    HAL_Delay(50);
    //    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    //    HAL_Delay(50);
    //    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    //    HAL_Delay(50);
    //}
    //HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
}
#if 0
static void _test_buzzer() {
#ifndef NDEBUG
    M_LOG_STRING("+[BMS-LV BUZZER]\r\n +-> Start test sonata\r\n");
#endif
    buzzer_sonata(500, 2);
#ifndef NDEBUG
    M_LOG_STRING("+[BMS-LV BUZZER]\r\n +-> End test sonata\r\n");
#endif
};
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n",
         file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
