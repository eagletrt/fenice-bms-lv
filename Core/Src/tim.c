/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "main.h"

/* Radiator1 -> TIM4 CH3 */
/* Radiator2 -> TIM4 CH4 */

/* FAN1      -> TIM2 CH2 */
/* FAN2      -> TIM2 CH1 */
/* FAN3      -> TIM3 CH1 */
/* FAN5      -> TIM2 CH4 */
/* FAN6      -> TIM2 CH3 */

/* PUMP1     -> TIM4 CH2 */
/* PUMP2     -> TIM4 CH1 */
/* BUZZER    -> TIM8 CH1 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

/* TIM2 init function */
void MX_TIM2_Init(void) {
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 4294967295;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}
/* TIM3 init function */
void MX_TIM3_Init(void) {
    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};
    TIM_OC_InitTypeDef sConfigOC              = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);
}
/* TIM8 init function */
void MX_TIM8_Init(void) {
    /* USER CODE BEGIN TIM8_Init 0 */

    /* USER CODE END TIM8_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig           = {0};
    TIM_MasterConfigTypeDef sMasterConfig               = {0};
    TIM_OC_InitTypeDef sConfigOC                        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM8_Init 1 */

    /* USER CODE END TIM8_Init 1 */
    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 83;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 99;
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM8_Init 2 */

    /* USER CODE END TIM8_Init 2 */
    HAL_TIM_MspPostInit(&htim8);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle) {
    if (tim_baseHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspInit 0 */

        /* USER CODE END TIM3_MspInit 0 */
        /* TIM3 clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();
        /* USER CODE BEGIN TIM3_MspInit 1 */

        /* USER CODE END TIM3_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspInit 0 */

        /* USER CODE END TIM8_MspInit 0 */
        /* TIM8 clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE();
        /* USER CODE BEGIN TIM8_MspInit 1 */

        /* USER CODE END TIM8_MspInit 1 */
    }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (timHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspPostInit 0 */

        /* USER CODE END TIM3_MspPostInit 0 */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3
    */
        GPIO_InitStruct.Pin       = RAD_L_Pin | RAD_R_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = FAN_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(FAN_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM3_MspPostInit 1 */

        /* USER CODE END TIM3_MspPostInit 1 */
    } else if (timHandle->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspPostInit 0 */

        /* USER CODE END TIM8_MspPostInit 0 */

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**TIM8 GPIO Configuration
    PC6     ------> TIM8_CH1
    */
        GPIO_InitStruct.Pin       = BUZZER_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM8_MspPostInit 1 */

        /* USER CODE END TIM8_MspPostInit 1 */
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle) {
    if (tim_baseHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();
        /* USER CODE BEGIN TIM2_MspDeInit 1 */

        /* USER CODE END TIM2_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspDeInit 0 */

        /* USER CODE END TIM3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();
        /* USER CODE BEGIN TIM3_MspDeInit 1 */

        /* USER CODE END TIM3_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspDeInit 0 */

        /* USER CODE END TIM8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM8_CLK_DISABLE();
        /* USER CODE BEGIN TIM8_MspDeInit 1 */

        /* USER CODE END TIM8_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim8) {
        set_sensor_update_time();
    }
}

/**
 * @brief     Check whether the timer is placed in either APB1 or APB2 bus
 * 
 * @param     __HANDLE__ TIM Handle
 * @return    True if the timer is on APB1 bus false if it is on APB2 
 */
#define _M_GET_TIM_APB_PLACEMENT(__HANDLE__) (((__HANDLE__)->Instance < (TIM_TypeDef *)APB2PERIPH_BASE) ? 1U : 0U)

#ifdef TIM_STANDARD_FUNC
uint32_t TIM_GetInternalClkFreq(TIM_HandleTypeDef *htim) {
    RCC_ClkInitTypeDef clkconfig;
    uint32_t pFLatency;
    uint32_t uwTimclock, uwAPBxPrescaler = 0U;

    /* Get clock configuration */
    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    /* Get APBx prescaler and base clock*/
    if (_M_GET_TIM_APB_PLACEMENT(htim)) {
        uwAPBxPrescaler = clkconfig.APB1CLKDivider;
        uwTimclock      = HAL_RCC_GetPCLK1Freq();
    } else {
        uwAPBxPrescaler = clkconfig.APB2CLKDivider;
        uwTimclock      = HAL_RCC_GetPCLK2Freq();
    }

    /* Compute final clock */
    if (uwAPBxPrescaler != RCC_HCLK_DIV1) {
        /* When the prescaler is not one the frequncy is multipled by 2 for timers */
        uwTimclock *= 2;
    }

    return uwTimclock;
}
#endif

/* USER CODE END 1 */
