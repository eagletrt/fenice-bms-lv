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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid_controller.h"
#include "ltc.h"
#include "stdio.h"
#include "string.h"
#include "can.h"
#include "current_sensor.h"
#include "pwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ----------------------------------- ERRORS ---------------------------------*/

int BMS_ON_REQUEST = 1;
int BMS_IS_ON = 0;

int UNDER_VOLTAGE = 0;
int OVER_TEMPERATURE = 0;

//Pump
//PD13  -> PWM  -> Pump1
//PD12  -> PWM  -> Pump2

//Radiator
//PC0   -> ADC  -> Radiator1
//PC1   -> ADC  -> Radiator2

/* ---------------- ID ------------------- */
#define BMS_LV_ASK_ID 0xFF     // Foo Fighters
#define STEER_ASK_ID 0xAF      // Steering wheel
#define INV_LEFT_ASK_ID 0x181  // Inverter left
#define INV_RIGHT_ASK_ID 0x182 // Inverter right
#define ACC_TEMP_ASK_ID 0xAA   // Accumulator temperatures
#define ECU_ASK_ID 0xF8        // ECU initial check ID

/* ---------------- PIN ------------------- */
#define RELAY_PIN GPIOC
#define RELAY_PIN_N GPIO_PIN_6

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
const bool DEBUG_CAN_SEND = false;
const bool DEBUG_VALUES = true;
const bool DEBUG_SENSOR_CURRENT = false;
const bool DEBUG_LTC = false;

const uint32_t ERR_LED_BLINK_error_MSEC = 100;
const uint32_t ERR_LED_BLINK_overT_ON_MSEC = 800;
const uint32_t ERR_LED_BLINK_overT_OFF_MSEC = 200;
const uint32_t ERR_LED_BLINK_underV_ON_MSEC = 100;
const uint32_t ERR_LED_BLINK_underV_OFF_MSEC = 900;

int prev_toggle_msec = 0;

// cooling system PWM
pwm_struct lv_pwm;
pwm_struct hv_pwm;
pwm_struct pump_pwm;

// PID
PIDControl lv_pid;
PIDControl hv_pid;
PIDControl pump_pid;

// Temperatures
temperatures_struct lv_temp;
temperatures_struct hv_temp;
temperatures_struct pump_temp;

// LTC
ltc_struct ltc;
extern canStruct can1, can3;

// ERROR LED
LED_STATE led_state;

char txt[100];
ADC_ChannelConfTypeDef UserAdcConfig = {0};
uint8_t cont_ms, cont_dec, cont_sec, cont_min, cont_hours;

uint32_t over_temp_start_msec, under_volt_start_msec;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM7_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int sensor_update_flag = 0;
int debug_flag = 0;
int m_sec_timer = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  sprintf(txt, "\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);

  //ERROR_LED
  for (int i = 0; i < 9; i++)
  {
    HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
    HAL_Delay(100);
  }

  //BUZZER
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  can1.rx0_interrupt = CAN1_RX0_IRQn;
  can1.tx_interrupt = CAN1_TX_IRQn;
  can1.hcan = &hcan1;

  sprintf(txt, "CAN initialization... ");
  HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
  if (can_init())
    sprintf(txt, "DONE\r\n");
  else
    sprintf(txt, "FAILED\r\n");
  HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);

  sprintf(txt, "Initializing LTC\r\n");
  LTC_init(&ltc, &hspi2, 0, GPIOD, GPIO_PIN_4); //init function of LTC_6810

  UserAdcConfig.Channel = ADC_CHANNEL_2;
  UserAdcConfig.Rank = ADC_REGULAR_RANK_1;
  UserAdcConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &UserAdcConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_Start_IT(&hadc1);

  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim7);

  // Initializing PWM
  Init_pwm(&hv_pwm, &htim2, TIM_CHANNEL_1);
  Init_pwm(&lv_pwm, &htim3, TIM_CHANNEL_1);
  Init_pwm(&pump_pwm, &htim4, TIM_CHANNEL_1);

  // Initializing temperatures
  lv_temp.desired = 45;
  hv_temp.desired = 45;
  pump_temp.desired = 45;

  lv_temp.max_temp = 70;
  hv_temp.max_temp = 70;
  pump_temp.max_temp = 70;

  // Initializing PID
  double sample_time = (htim7.Init.Prescaler * htim7.Init.Period) / 108000000;
  PIDInit(&lv_pid, 10, 0.01, 0.01, (float)sample_time, 100, 1000, AUTOMATIC, REVERSE);
  PIDInit(&hv_pid, 10, 0.01, 0.01, 0.5, 100, 1000, AUTOMATIC, REVERSE);
  PIDInit(&pump_pid, 10, 0.01, 0.01, 0.5, 100, 1000, AUTOMATIC, REVERSE);

  lv_pid.setpoint = lv_temp.desired;
  hv_pid.setpoint = hv_temp.desired;
  pump_pid.setpoint = pump_temp.desired;

  lv_temp.value = 65;
  hv_temp.value = 50;
  pump_temp.value = 50;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  m_sec_timer = HAL_GetTick();
  int currentTick = m_sec_timer;
  int previous_millis = HAL_GetTick();
  while (1)
  {

    currentTick = HAL_GetTick();

    if (sensor_update_flag == 1)
    {
      sensor_update_flag = 0;

      // LTC
      read_voltages(&ltc);

      // Temperatures
      // TODO: read temperatures from adc and read from CANBUS

      lv_pid.input = lv_temp.value;
      hv_pid.input = hv_temp.value;
      pump_pid.input = pump_temp.value;

      // PID
      PIDCompute(&lv_pid);
      PIDCompute(&hv_pid);
      PIDCompute(&pump_pid);

      // PWM
      lv_pwm.value = lv_pid.output;
      hv_pwm.value = hv_pid.output;
      pump_pwm.value = pump_pid.output;

      check_over_temperature();
      check_under_voltage();
    }

    if (currentTick % 200 == 0)
    {
      write_pwm_value(&lv_pwm);
      write_pwm_value(&hv_pwm);
      write_pwm_value(&pump_pwm);
    }

    if (debug_flag)
    {
      if (DEBUG_VALUES)
      {
        sprintf(txt, "\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
        sprintf(txt, "BMS_IS_ON: %d, overT: %d, underV: %d\r\n", BMS_IS_ON, OVER_TEMPERATURE, UNDER_VOLTAGE);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
        sprintf(txt, "PID outputs lv: %d hv: %d pump: %d\r\n", (int)PIDOutputGet(&lv_pid), (int)PIDOutputGet(&hv_pid), (int)PIDOutputGet(&pump_pid));
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
        sprintf(txt, "Voltages %d %d %d %d %d %d\r\n", ltc.voltage[0], ltc.voltage[1], ltc.voltage[2], ltc.voltage[3], ltc.voltage[4], ltc.voltage[5]);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
        sprintf(txt, "Current from sensor: %lu\r\n", get_current());
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
      }
      debug_flag = 0;
    }

    if (previous_millis != currentTick)
    {
      int sent = send_CAN_data(currentTick);
      previous_millis = currentTick;
      if (sent != 0 && DEBUG_CAN_SEND)
      {
        sprintf(txt, "Sent message: %d\r\n", sent);
        HAL_UART_Transmit(&huart4, (uint8_t *)txt, strlen(txt), 10);
      }
    }

    if (BMS_IS_ON)
    {
      HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
      led_state = ON;
    }
    else
    {
      HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
      if (UNDER_VOLTAGE)
      {
        led_state = BLINK_underV;
        if (OVER_TEMPERATURE)
          led_state = BLINK_error;
      }
      else
      {
        if (OVER_TEMPERATURE)
          led_state = BLINK_overT;
        else
          led_state = OFF;
      }
    }

    BMS_ON_OFF();
    write_error_led(led_state);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CAN1_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* CAN1_SCE_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN1_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 208;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 108;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 54;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 108;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 5400;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RELAY_Pin | LED_ERR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RELAY_Pin LED_ERR_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin | LED_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    if (UserAdcConfig.Channel == ADC_CHANNEL_2)
    {
      uint32_t current;
      uint32_t adc_val;
      adc_val = HAL_ADC_GetValue(hadc);
      current = calc_current(adc_val);
      push_into_array(current);
      mean_current();
    }
    HAL_ADC_Start_IT(&hadc1);
  }
}

void write_error_led()
{
  switch (led_state)
  {
  case ON:
    HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
    break;

  case BLINK_error:
    if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_error_MSEC)
    {
      HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
      prev_toggle_msec = HAL_GetTick();
    }
    break;

  case BLINK_overT:
    if (HAL_GPIO_ReadPin(LED_ERR_GPIO_Port, LED_ERR_Pin))
    {
      if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_overT_ON_MSEC)
      {
        HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
        prev_toggle_msec = HAL_GetTick();
      }
    }
    else
    {
      if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_overT_OFF_MSEC)
      {
        HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
        prev_toggle_msec = HAL_GetTick();
      }
    }
    break;

  case BLINK_underV:
    if (HAL_GPIO_ReadPin(LED_ERR_GPIO_Port, LED_ERR_Pin))
    {
      if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_underV_ON_MSEC)
      {
        HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
        prev_toggle_msec = HAL_GetTick();
      }
    }
    else
    {
      if (HAL_GetTick() - prev_toggle_msec > ERR_LED_BLINK_underV_OFF_MSEC)
      {
        HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
        prev_toggle_msec = HAL_GetTick();
      }
    }
    break;

  case OFF:
    HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
    break;

  default:
    HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
    break;
  }
}

void check_under_voltage()
{
  if (get_min_voltage(&ltc) < 10)
  {
    UNDER_VOLTAGE = 1;
  }
  else
  {
    UNDER_VOLTAGE = 0;
  }
}

void check_over_temperature()
{
  if (lv_temp.value > lv_temp.max_temp)
  {
    OVER_TEMPERATURE = 1;
  }
  else
  {
    OVER_TEMPERATURE = 0;
  }
}

int BMS_ON_OFF()
{
  if (BMS_ON_REQUEST == 1)
  {
    if (!OVER_TEMPERATURE && !UNDER_VOLTAGE)
    {
      BMS_IS_ON = 1;
    }
    else
    {
      BMS_IS_ON = 0;
    }
  }
  else
  {
    BMS_IS_ON = 0;
  }
  return BMS_IS_ON;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim5)
  {
    cont_ms++;
    if (cont_ms == 100)
    {
      cont_ms = 0;
      cont_dec++;
      if (cont_dec == 10)
      {
        cont_dec = 0;
        cont_sec++;

        debug_flag = 1;

        if (cont_sec == 60)
        {
          cont_sec = 0;
          cont_min++;
          if (cont_min == 60)
          {
            cont_min = 0;
            cont_hours++;
          }
        }
      }
    }
  }
  if (htim == &htim7)
  {
    sensor_update_flag = 1;
  }
}

int send_CAN_data(uint32_t tick)
{
  int message_sent = 0;

  if (tick % 200 == 0)
  {
    can1.tx_id = 0x97;
    can1.dataTx[0] = ltc.voltage[0] >> 8;
    can1.dataTx[1] = (uint8_t)ltc.voltage[1];
    can1.tx_size = 2;
    CAN_Send(&can1);
    message_sent = 1;
  }

  return message_sent;
}

void loading()
{
  int delay = 100;
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay(delay * 3);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_Delay(delay);
}

int CAN_Read_Message()
{
  if (fifoRxDataCAN_pop(&can1))
  {
    switch (can1.rx_id)
    {
    case INV_LEFT_ASK_ID:
      if (can1.dataRx[0] == 0x4A)
      { // Inverter left temperature
        // invLeftTemp = can->dataRx[1] + (can->dataRx[2] << 8);
        // inverterLeftTemp = (invLeftTemp - 15797) / 112.12;
      }
      else if (can1.dataRx[0] == 0x49)
      { // Motor left temperature
        // motLeftTemp = can->dataRx[1] + (can->dataRx[2] << 8);
        // motorLeftTemp = (motLeftTemp - 9394) / 55.10;
      }
      break;
    case INV_RIGHT_ASK_ID:
      if (can1.dataRx[0] == 0x4A)
      { // Inverter right temperature
        // invRightTemp = can->dataRx[1] + (can->dataRx[2] << 8);
        // inverterRightTemp = (invLeftTemp - 15797) / 112.12;
      }
      else if (can1.dataRx[0] == 0x49)
      { // Motor right temperature
        // motRightTemp = can->dataRx[1] + (can->dataRx[2] << 8);
        // motorRightTemp = (motLeftTemp - 9394) / 55.10;
      }
      break;
    case STEER_ASK_ID:
      if (can1.dataRx[0] == 0)
      {
        // overridePID = RxData[1]; // 1 - override PID
      }
      else if (can1.dataRx[0] == 1)
      {
        // overridePID = RxData[1];
        // pumpRequest = RxData[2];
      }
      else if (can1.dataRx[0] == 2)
      {
        // overridePID = RxData[1];
        // fanRequest = RxData[2];
      }
      else if (can1.dataRx[0] == 3 && can1.dataRx[1] == 1)
      {
        // overridePID = 2;
      }
      break;
    case ACC_TEMP_ASK_ID:
      if (can1.dataRx[0] == 6)
      { // Little endian
        // tmpHvAvgTemp = can->dataRx[5] + (can->dataRx[4] << 8);
        // hvAvgTemp = tmpHvAvgTemp / 100.0;
      }
      else if (can1.dataRx[0] == 6)
      {
        // tmpHvMaxTemp = can->dataRx[7] + (can->dataRx[6] << 8);
        // hvMaxTemp = tmpHvMaxTemp / 100.0;
      }
      break;
    case ECU_ASK_ID:
      break;
    case BMS_LV_ASK_ID:
      break;

    default:
      break;
    }
    return 0;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
