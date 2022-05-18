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
#include "adc.h"
#include "can_comm.h"
#include "cli_bms_lv.h"
#include "common.h"
#include "current_sensor.h"
#include "dac_pump.h"
#include "dma.h"
#include "fenice-config.h"
#include "mcp23017.h"
#include "measurements.h"
#include "notes_buzzer.h"
#include "pwm.h"
#include "radiator.h"
#include "stdio.h"
#include "string.h"
#include "timer_utils.h"
#include "volt.h"
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
// pwm_struct lv_pwm;
// pwm_struct hv_pwm;
// pwm_struct pump_pwm;

// PID
//PID_HandleTypeDef lv_pid;
//PID_HandleTypeDef hv_pid;
//PID_HandleTypeDef pump_pid;

// Temperatures
temperatures_struct lv_temp;
temperatures_struct hv_temp;
temperatures_struct pump_temp;

// ERROR LED
LED_STATE led_state;

uint32_t over_temp_start_msec, under_volt_start_msec;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void _signal_mx_init_succes();
//static void _test_buzzer();
//static void write_error_led();
static inline void check_over_temperature();
static inline void check_under_voltage();
static inline bool bms_on_off();
static inline void check_initial_voltage();
static inline void fans_and_radiators_init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool is_sensors_update_time = 0;
int debug_flag                     = 0;
int m_sec_timer                    = 0;

HAL_StatusTypeDef status;
char main_buff[500];
float fan_duty_cycle = 0.8;
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
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_DAC_Init();
    MX_I2C3_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_TIM8_Init();
    MX_SPI3_Init();
    /* USER CODE BEGIN 2 */
    /* USER CODE BEGIN 2 */

    //  MX_DMA_Init() must be executed before MX_ADC_Init() otherwise the ADC doesnt' work in DMA mode correctly
    //  but since __CUBEMIX e' stronzo__(cit.), this isn't enforced by the code generator
    //  threfore manual intervention is necessary
    //  https://community.st.com/s/question/0D50X0000ALudz8SQB/how-to-enable-adc-continuous-mode-with-dma

    MX_DMA_Init();
    MX_ADC1_Init();

#ifdef DEBUG_TIMER_MCU
    DBGMCU->APB1FZ = DBGMCU_APB1_FZ_DBG_TIM2_STOP;
#endif
    // Start DMA handled readings for the current sensor, battery and DCDC(12/24v) temperature sensors
    ADC_start_dma_readings();

    // Blink to signal correct MX_XXX_init processes (usuefull for CAN transciever)
    _signal_mx_init_succes();
    cli_bms_lv_init();

    ltc6810_disable_cs(&SPI);
    // sprintf(main_buff, "LTC ID %s", ltc6810_return_serial_id());
    // printl(main_buff, NO_HEADER);
    printl("Relay out disabled, waiting 1 seconds before reading voltages\r\n", NO_HEADER);
    HAL_Delay(1000);

    //Init feedbacks chip
    mcp23017_basic_config_init(&hmcp, &hi2c3);

    // Buzzer congiguration
    pwm_set_period(&BZZR_HTIM, 1);
    pwm_set_duty_cicle(&BZZR_HTIM, BZZR_PWM_TIM_CHNL, 0.5);

    // Other fan, set at 25 KHz
    pwm_set_period(&FAN6_HTIM, 0.04);
    pwm_set_duty_cicle(&FAN6_HTIM, FAN6_PWM_TIM_CHNL, 0.20);

    radiator_init();
    measurements_init(&MEASUREMENTS_TIMER);

    check_initial_voltage();

    mcp23017_read_both(&hmcp, &hi2c3);

    fans_and_radiators_init();

    //init for both can
    can_primary_init();
    can_secondary_init();

    // can_primary_send(0x1);
    // can_secondary_send(0x1);
    while (1) {
        cli_loop(&cli_bms_lv);
        measurements_flags_check();
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t tim_1s_next    = HAL_GetTick();
    uint32_t tim_10ms_next  = HAL_GetTick();
    uint32_t tim_100ms_next = HAL_GetTick();
    while (1) {
        if (HAL_GetTick() >= tim_1s_next) {
            if (volt_read_and_print() == 1) {
                printl("Correct voltage", NO_HEADER);
            }
            printl("\r\nReading Feedbacks", NO_HEADER);
            mcp23017_read_both(&hmcp, &hi2c3);
            tim_1s_next = HAL_GetTick() + 1000;
        }

        if (HAL_GetTick() > tim_10ms_next) {
            sprintf(
                main_buff,
                "main ADC: %lu %i %i %i %i %i",
                HAL_GetTick(),
                ADC_get_i_sensor_val(),
                ADC_get_t_batt1_val(),
                ADC_get_t_batt2_val(),
                ADC_get_t_dcdc12_val(),
                ADC_get_t_dcdc24_val());
            printl(main_buff, NORM_HEADER);
            ADC_start_dma_readings();

            // Update Timer
            tim_10ms_next = HAL_GetTick() + 10;
        }

        if (HAL_GetTick() > tim_100ms_next) {
            // Update Timer
            tim_100ms_next = HAL_GetTick() + 100;
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
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
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 180;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 2;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Activate the Over-Drive mode
  */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
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
//static void write_error_led() {
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
//}

static inline void check_under_voltage() {
    // if (get_min_voltage(&ltc) < 10) {
    //     UNDER_VOLTAGE = 1;
    // } else {
    //     UNDER_VOLTAGE = 0;
    // }
}

/**
 * @brief Checks if total voltage is above MIN_POWER_ON_VOLTAGE within VOLT_MAX_ATTEMPTS times
 * If total voltage is above the minimum level required the buzzer starts up for BUZZER_ALARM_TIME
 * 
 */
static inline void check_initial_voltage() {
    sprintf(main_buff, "Checking if total voltage on board is above %.2fV", MIN_POWER_ON_VOLTAGE);
    for (uint8_t i = 0; i < VOLT_MAX_ATTEMPTS; i++) {
        sprintf(main_buff, "Closing relay phase: attempt %d of %d", i + 1, VOLT_MAX_ATTEMPTS);
        printl(main_buff, NO_HEADER);
        if (volt_read_and_print() == VOLT_OK) {
            HAL_GPIO_WritePin(L_ERR_GPIO_Port, L_ERR_Pin, GPIO_PIN_RESET);
            printl("Relay on", NORM_HEADER);
            HAL_GPIO_WritePin(L_OTHER_GPIO_Port, L_OTHER_Pin, GPIO_PIN_SET);
            pwm_start_channel(&BZZR_HTIM, BZZR_PWM_TIM_CHNL);
            LV_MASTER_RELAY_set_state(GPIO_PIN_SET);
            HAL_Delay(BUZZER_ALARM_TIME);
            pwm_stop_channel(&BZZR_HTIM, BZZR_PWM_TIM_CHNL);
            i = VOLT_MAX_ATTEMPTS;
            printl("DONE!\r\n", NORM_HEADER);
        }
        HAL_Delay(200);
    }

    if (volt_status != VOLT_OK) {
        HAL_GPIO_WritePin(L_ERR_GPIO_Port, L_ERR_Pin, GPIO_PIN_SET);
    }
}

static inline void fans_and_radiators_init() {
    if (FDBK_12V_FANS_get_state()) {
        pwm_start_channel(&FAN6_HTIM, FAN6_PWM_TIM_CHNL);
    }

    if (FDBK_12V_RADIATORS_get_state()) {
        start_both_radiator(&RAD_R_HTIM, RAD_L_PWM_TIM_CHNL, RAD_R_PWM_TIM_CHNL);
    }
#ifdef PUMP_SAMPLE_TEST
    if (FDBK_24V_PUMPS_get_state()) {
        dac_pump_sample_test(&hdac_pump);
    }
#endif
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
