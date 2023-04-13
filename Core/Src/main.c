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
#include "adc.h"
#include "can_comm.h"
#include "cli_bms_lv.h"
#include "common.h"
#include "current_sensor.h"
#include "current_transducer.h"
#include "dac_pump.h"
#include "dma.h"
#include "error.h"
#include "fenice-config.h"
#include "inverters.h"
#include "mcp23017.h"
#include "measurements.h"
#include "notes_buzzer.h"
#include "pwm.h"
#include "radiator.h"
#include "stdio.h"
#include "string.h"
#include "thermocouple.h"
#include "timer_utils.h"
#include "volt.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*----------------------------------- ERRORS ---------------------------------*/

bool is_bms_on_fault = false;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Inverters_struct car_inverters;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void bms_error_state();
void check_on_feedbacks();
void cooling_routine(uint8_t);
static inline void check_initial_voltage();
static inline void check_DCDCs_voltages();
static inline void fans_radiators_pumps_init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int debug_flag  = 0;
int m_sec_timer = 0;

HAL_StatusTypeDef status;
char main_buff[500];
float bms_fan_duty_cycle = 0.8;
uint32_t errors_timer;
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
    MX_TIM5_Init();
    MX_TIM7_Init();
    MX_TIM4_Init();
    MX_ADC3_Init();
    MX_TIM9_Init();
    MX_UART5_Init();
    MX_TIM10_Init();
    /* USER CODE BEGIN 2 */
    /* USER CODE BEGIN 2 */

    //  MX_DMA_Init() must be executed before MX_ADC_Init() otherwise the ADC doesnt' work in DMA mode correctly
    //  but since __CUBEMIX e' stronzo__(cit.), this isn't enforced by the code generator
    //  threfore manual intervention is necessary
    //  https://community.st.com/s/question/0D50X0000ALudz8SQB/how-to-enable-adc-continuous-mode-with-dma

    MX_DMA_Init();
    // MX_ADC1_Init();
    MX_ADC2_Init();

    // Usefull if it's necessary to stop the timer counter when debugging
#ifdef DEBUG_TIMER_MCU
    DBGMCU->APB1FZ = DBGMCU_APB1_FZ_DBG_TIM2_STOP;
#endif

    // START OF WARM UP STAGE

    // Turn on startup button
    // TODO: change button pin
    //HAL_GPIO_WritePin(L_ERR_GPIO_Port, L_ERR_Pin, GPIO_PIN_SET);

    // Start DMA handled readings for the current transducer, battery and DCDC(12/24v) temperature sensors
    ADC_start_DMA_readings();

    // Blink to signal correct MX_XXX_init processes (usuefull for CAN transciever)
    cli_bms_lv_init();
    error_init();

    init_inverter_struct(&car_inverters);
    //Init feedbacks chip
    mcp23017_basic_config_init(&hmcp, &hi2c3);
    radiator_init();
    dac_pump_handle_init(&hdac_pump, 0.0, 0.0);

    // Buzzer congiguration
    pwm_set_period(&BZZR_HTIM, 1);
    pwm_set_duty_cicle(&BZZR_HTIM, BZZR_PWM_TIM_CHNL, 0.5);

    // Other fan, set at 25 KHz
    pwm_set_period(&INTERNAL_FAN_HTIM, 0.04);
    pwm_set_duty_cicle(&INTERNAL_FAN_HTIM, INTERNAL_FAN_PWM_TIM_CHNL, 0.9);
    pwm_start_channel(&INTERNAL_FAN_HTIM, INTERNAL_FAN_PWM_TIM_CHNL);

    // Keeps SPI CS high
    ltc6810_disable_cs(&SPI);
#ifdef DEBUG_LTC_ID
    sprintf(main_buff, "LTC ID %s", ltc6810_return_serial_id());
    printl(main_buff, NO_HEADER);
#endif
    printl("Relay out disabled, waiting 1 seconds before reading voltages\r\n", NO_HEADER);
    HAL_Delay(1000);

    check_initial_voltage();

    mcp23017_read_both(&hmcp, &hi2c3);
    check_DCDCs_voltages();

    fans_radiators_pumps_init();

    //init for both can
    can_primary_init();
    can_secondary_init();

    measurements_init(&MEASUREMENTS_TIMER);

    // END OF WARM UP STAGE

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    errors_timer = HAL_GetTick();
    while (1) {
        // Running stage
        if (is_bms_on_fault) {
            bms_error_state();
        } else {
            cli_loop(&cli_bms_lv);
            // TODO: Implement
            /* if(flag_mux){
                send_address_to_mux();
                send_adc_dma_request();
            }
            if(can_tim_elapsed){
                send_can_msg();
            }
            */
            //TODO: REMOVE measurements_flags_check();  // measure and sends via can
            check_on_feedbacks();  //check dcdcs and relay fb
            if (error_count() > 0 && errors_timer + 10 > errors_timer) {
                can_primary_send(primary_ID_LV_ERRORS);
                errors_timer = HAL_GetTick();
            }

            inverters_loop(&car_inverters);

            cooling_routine(10);
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

/**
 * @brief Checks if total voltage is above MIN_POWER_ON_VOLTAGE within VOLT_MAX_ATTEMPTS times
 * If total voltage is above the minimum level required the buzzer starts up for BUZZER_ALARM_TIME
 * 
 */
static inline void check_initial_voltage() {
    sprintf(main_buff, "Checking if total voltage on board is above %.2fV", MIN_POWER_ON_VOLTAGE);
    for (uint8_t i = DEAD_CELLS_OFFSET; i < VOLT_MAX_ATTEMPTS; i++) {
        sprintf(main_buff, "Closing relay phase: attempt %d of %d", i + 1, VOLT_MAX_ATTEMPTS);
        printl(main_buff, NO_HEADER);
        if (volt_read_and_print() == VOLT_OK) {
            printl("Relay on", NORM_HEADER);
            // TODO: CHANGE PIN L_OTHER
            //HAL_GPIO_WritePin(L_OTHER_GPIO_Port, L_OTHER_Pin, GPIO_PIN_SET);
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
        error_set(ERROR_RELAY, 0, HAL_GetTick());
        //is_bms_on_fault = true;
    } else {
        error_reset(ERROR_RELAY, 0);
    }
}

/**
 * @brief Initialize internal fan, pumps and radiator with standard value
 * Also set/reset the relative errors
 * 
 */
static inline void fans_radiators_pumps_init() {
    if (FDBK_12V_FANS_get_state()) {
        pwm_start_channel(&INTERNAL_FAN_HTIM, INTERNAL_FAN_PWM_TIM_CHNL);
        error_reset(ERROR_FAN, 0);
    } else {
        error_set(ERROR_FAN, 0, HAL_GetTick());
    }

    if (FDBK_12V_RADIATORS_get_state()) {
        start_both_radiator(&RAD_R_HTIM, RAD_L_PWM_TIM_CHNL, RAD_R_PWM_TIM_CHNL);
        error_reset(ERROR_RADIATOR, 1);
    } else {
        error_set(ERROR_RADIATOR, 1, HAL_GetTick());
    }
    if (FDBK_24V_PUMPS_get_state()) {
        dac_pump_handle_init(&hdac_pump, 0.0, 0.0);
        error_reset(ERROR_PUMP, 0);
    } else {
        error_set(ERROR_PUMP, 0, HAL_GetTick());
    }
#ifdef PUMP_SAMPLE_TEST
    if (FDBK_24V_PUMPS_get_state()) {
        dac_pump_sample_test(&hdac_pump);
    }
#endif
}

/**
 * @brief Set/unset errors for the DCDC converters on board
 * 
 */
static inline void check_DCDCs_voltages() {
    !FDBK_DCDC_12V_get_state() ? error_set(ERROR_DCDC12, 0, HAL_GetTick()) : error_reset(ERROR_DCDC12, 0);
    !FDBK_DCDC_24V_get_state() ? error_set(ERROR_DCDC24, 0, HAL_GetTick()) : error_reset(ERROR_DCDC24, 0);
}

/**
 * @brief Check feedbacks given by the MCP23017 and set the relative errors
 * 
 */
void check_on_feedbacks() {
    mcp23017_read_both(&hmcp, &hi2c3);
    check_DCDCs_voltages();
    !FDBK_RELAY_get_state() ? error_set(ERROR_RELAY, 0, HAL_GetTick()) : error_reset(ERROR_RELAY, 0);
}

/**
 * @brief Error routine executed when the bms is on fault
 * 
 */
void bms_error_state() {
    // ERROR stage
    // TODO: CHANGE PIN L_OTHER
    //HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(L_ERR_GPIO_Port, L_ERR_Pin, GPIO_PIN_SET);
    error_state_inverters(&car_inverters);
    printl("ERROR STATE", ERR_HEADER);
    //HAL_GPIO_WritePin(L_OTHER_GPIO_Port, L_OTHER_Pin, GPIO_PIN_RESET);
    while (1) {
        cli_loop(&cli_bms_lv);
        measurements_flags_check();  // measure and sends via can
        //HAL_GPIO_TogglePin(L_ERR_GPIO_Port, L_ERR_Pin);
        HAL_Delay(100);
    }
}

/**
 * @brief Cooling routine of the car and internal BMS
 * Set values for: internal fan, radiators, pumps
 * 
 * @param temp Average temperature of the two inverters
 */
void cooling_routine(uint8_t temp) {
    float local_rad_speed, local_pump_speed;
    THC_get_temperature_C(&hTHC_BATT1);
    THC_get_temperature_C(&hTHC_BATT2);
    float max_dcdc_temp = (THC_get_temperature_C(&hTHC_DCDC12V) >= THC_get_temperature_C(&hTHC_DCDC24V))
                              ? THC_get_temperature_C(&hTHC_DCDC12V)
                              : THC_get_temperature_C(&hTHC_DCDC24V);
#ifndef INTERNAL_FAN_DEBUG
    bms_fan_duty_cycle = (INTERNAL_FAN_Q_FACTOR + (max_dcdc_temp * INTERNAL_FAN_M_FACTOR));
#endif
    pwm_set_duty_cicle(&INTERNAL_FAN_HTIM, INTERNAL_FAN_PWM_TIM_CHNL, 1 - bms_fan_duty_cycle);
    pwm_start_channel(&INTERNAL_FAN_HTIM, INTERNAL_FAN_PWM_TIM_CHNL);

    /* 
        When automatic mode == true the values are controlled by the BMS
        When automatic mode == false the values are controlled by the Steering Wheel
    */
    if (radiator_handle.automatic_mode) {
        set_radiator_dt(&RAD_L_HTIM, RAD_L_PWM_TIM_CHNL, get_radiator_dt(temp));
        set_radiator_dt(&RAD_R_HTIM, RAD_R_PWM_TIM_CHNL, get_radiator_dt(temp));
    } else {
        local_rad_speed = rads_speed_msg.radiators_speed * (MAX_RADIATOR_DUTY_CYCLE - MIN_RADIATOR_DUTY_CYCLE) +
                          MIN_RADIATOR_DUTY_CYCLE;
        // Clipping to max duty cycle allowed to avoid overcurrent (when in combo with pumps)
        if (local_rad_speed > MAX_RADIATOR_DUTY_CYCLE) {
            local_rad_speed = MAX_RADIATOR_DUTY_CYCLE;
        }
        // Clipping to minimum duty cycle allowed to spin the radiator
        else if (local_rad_speed < MIN_RADIATOR_DUTY_CYCLE) {
            local_rad_speed = MIN_RADIATOR_DUTY_CYCLE;
        }

        set_radiator_dt(&RAD_L_HTIM, RAD_L_PWM_TIM_CHNL, local_rad_speed);
        set_radiator_dt(&RAD_R_HTIM, RAD_R_PWM_TIM_CHNL, local_rad_speed);
    }
    if (hdac_pump.automatic_mode) {
        dac_pump_store_and_set_value_on_both_channels(
            &hdac_pump, dac_pump_get_voltage(temp), dac_pump_get_voltage(temp));
    } else {
        local_pump_speed = pumps_speed_msg.pumps_speed * (MAX_OPAMP_OUT - MIN_OPAMP_OUT) + MIN_OPAMP_OUT;

        // Clipping to max duty cycle allowed to avoid overcurrent (when in combo with pumps)
        if (local_pump_speed > MAX_OPAMP_OUT) {
            local_pump_speed = MAX_OPAMP_OUT;
        }
        // Clipping to minimum duty cycle allowed to spin the radiator
        else if (local_pump_speed < MIN_OPAMP_OUT) {
            local_pump_speed = MIN_OPAMP_OUT;
        }

        dac_pump_store_and_set_value_on_both_channels(&hdac_pump, local_pump_speed, local_pump_speed);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
        */
    if (HAL_ADC_Init(&CURRENT_TRANSDUCER_HADC) != HAL_OK || HAL_ADC_Init(&T_SENS_BATT1_HADC) != HAL_OK) {
        error_set(ERROR_ADC_INIT, 0, HAL_GetTick());
    }
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
