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
#include "adc.h"
#include "error.h"
#include "main.h"
#include "measurements.h"
#include "monitor_int.h"

/* Radiator1 -> TIM4 CH3 */
/* Radiator2 -> TIM4 CH4 */

/* FAN1      -> TIM2 CH2 */
/* FAN2      -> TIM2 CH1 */
/* FAN3      -> TIM3 CH1 */
/* FAN5      -> TIM2 CH4 */
/* INTERNAL_FAN      -> TIM2 CH3 */

/* PUMP1     -> TIM4 CH2 */
/* PUMP2     -> TIM4 CH1 */
/* BUZZER    -> TIM8 CH1 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;

/* TIM1 init function */
void MX_TIM1_Init(void) {
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig               = {0};
    TIM_OC_InitTypeDef sConfigOC                        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 90 - 1;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 100 - 1;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 50 - 1;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
}
/* TIM2 init function */
void MX_TIM2_Init(void) {
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};
    TIM_OC_InitTypeDef sConfigOC              = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 8999;
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
    if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_TIMING;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
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
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);
}
/* TIM4 init function */
void MX_TIM4_Init(void) {
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC          = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_OC_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_TIMING;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}
/* TIM5 init function */
void MX_TIM5_Init(void) {
    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 0;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 4294967295;
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */
}
/* TIM7 init function */
void MX_TIM7_Init(void) {
    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance               = TIM7;
    htim7.Init.Prescaler         = 900 - 1;
    htim7.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim7.Init.Period            = 100 - 1;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */
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
/* TIM10 init function */
void MX_TIM10_Init(void) {
    /* USER CODE BEGIN TIM10_Init 0 */

    /* USER CODE END TIM10_Init 0 */

    /* USER CODE BEGIN TIM10_Init 1 */

    /* USER CODE END TIM10_Init 1 */
    htim10.Instance               = TIM10;
    htim10.Init.Prescaler         = 180 - 1;
    htim10.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim10.Init.Period            = 500 - 1;
    htim10.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM10_Init 2 */

    /* USER CODE END TIM10_Init 2 */
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle) {
    if (tim_pwmHandle->Instance == TIM1) {
        /* USER CODE BEGIN TIM1_MspInit 0 */

        /* USER CODE END TIM1_MspInit 0 */
        /* TIM1 clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();

        /* TIM1 interrupt Init */
        HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        /* USER CODE BEGIN TIM1_MspInit 1 */

        /* USER CODE END TIM1_MspInit 1 */
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle) {
    if (tim_baseHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspInit 0 */

        /* USER CODE END TIM3_MspInit 0 */
        /* TIM3 clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();
        /* USER CODE BEGIN TIM3_MspInit 1 */

        /* USER CODE END TIM3_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM5) {
        /* USER CODE BEGIN TIM5_MspInit 0 */

        /* USER CODE END TIM5_MspInit 0 */
        /* TIM5 clock enable */
        __HAL_RCC_TIM5_CLK_ENABLE();

        /* TIM5 interrupt Init */
        HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
        /* USER CODE BEGIN TIM5_MspInit 1 */

        /* USER CODE END TIM5_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM7) {
        /* USER CODE BEGIN TIM7_MspInit 0 */

        /* USER CODE END TIM7_MspInit 0 */
        /* TIM7 clock enable */
        __HAL_RCC_TIM7_CLK_ENABLE();

        /* TIM7 interrupt Init */
        HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
        /* USER CODE BEGIN TIM7_MspInit 1 */

        /* USER CODE END TIM7_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspInit 0 */

        /* USER CODE END TIM8_MspInit 0 */
        /* TIM8 clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE();

        /* TIM8 interrupt Init */
        HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
        /* USER CODE BEGIN TIM8_MspInit 1 */

        /* USER CODE END TIM8_MspInit 1 */
    } else if (tim_baseHandle->Instance == TIM10) {
        /* USER CODE BEGIN TIM10_MspInit 0 */

        /* USER CODE END TIM10_MspInit 0 */
        /* TIM10 clock enable */
        __HAL_RCC_TIM10_CLK_ENABLE();

        /* TIM10 interrupt Init */
        HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        /* USER CODE BEGIN TIM10_MspInit 1 */

        /* USER CODE END TIM10_MspInit 1 */
    }
}

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *tim_ocHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (tim_ocHandle->Instance == TIM4) {
        /* USER CODE BEGIN TIM4_MspInit 0 */

        /* USER CODE END TIM4_MspInit 0 */
        /* TIM4 clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM4 GPIO Configuration
    PB7     ------> TIM4_CH2
    */
        GPIO_InitStruct.Pin       = STP_STEP_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(STP_STEP_GPIO_Port, &GPIO_InitStruct);

        /* TIM4 interrupt Init */
        HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        /* USER CODE BEGIN TIM4_MspInit 1 */

        /* USER CODE END TIM4_MspInit 1 */
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
    PB1     ------> TIM3_CH4
    */
        GPIO_InitStruct.Pin       = RAD_L_Pin | RAD_R_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = FAN_Pin | STP_DIRECTION_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle) {
    if (tim_pwmHandle->Instance == TIM1) {
        /* USER CODE BEGIN TIM1_MspDeInit 0 */

        /* USER CODE END TIM1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM1_CLK_DISABLE();

        /* TIM1 interrupt Deinit */
        /* USER CODE BEGIN TIM1:TIM1_UP_TIM10_IRQn disable */
        /**
    * Uncomment the line below to disable the "TIM1_UP_TIM10_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
        /* HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn); */
        /* USER CODE END TIM1:TIM1_UP_TIM10_IRQn disable */

        /* USER CODE BEGIN TIM1_MspDeInit 1 */

        /* USER CODE END TIM1_MspDeInit 1 */
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle) {
    if (tim_baseHandle->Instance == TIM2) {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();

        /* TIM2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
        /* USER CODE BEGIN TIM2_MspDeInit 1 */

        /* USER CODE END TIM2_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM3) {
        /* USER CODE BEGIN TIM3_MspDeInit 0 */

        /* USER CODE END TIM3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();
        /* USER CODE BEGIN TIM3_MspDeInit 1 */

        /* USER CODE END TIM3_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM5) {
        /* USER CODE BEGIN TIM5_MspDeInit 0 */

        /* USER CODE END TIM5_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM5_CLK_DISABLE();

        /* TIM5 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM5_IRQn);
        /* USER CODE BEGIN TIM5_MspDeInit 1 */

        /* USER CODE END TIM5_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM7) {
        /* USER CODE BEGIN TIM7_MspDeInit 0 */

        /* USER CODE END TIM7_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM7_CLK_DISABLE();

        /* TIM7 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM7_IRQn);
        /* USER CODE BEGIN TIM7_MspDeInit 1 */

        /* USER CODE END TIM7_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspDeInit 0 */

        /* USER CODE END TIM8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM8_CLK_DISABLE();

        /* TIM8 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
        /* USER CODE BEGIN TIM8_MspDeInit 1 */

        /* USER CODE END TIM8_MspDeInit 1 */
    } else if (tim_baseHandle->Instance == TIM10) {
        /* USER CODE BEGIN TIM10_MspDeInit 0 */

        /* USER CODE END TIM10_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM10_CLK_DISABLE();

        /* TIM10 interrupt Deinit */
        /* USER CODE BEGIN TIM10:TIM1_UP_TIM10_IRQn disable */
        /**
    * Uncomment the line below to disable the "TIM1_UP_TIM10_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
        /* HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn); */
        /* USER CODE END TIM10:TIM1_UP_TIM10_IRQn disable */

        /* USER CODE BEGIN TIM10_MspDeInit 1 */

        /* USER CODE END TIM10_MspDeInit 1 */
    }
}

void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *tim_ocHandle) {
    if (tim_ocHandle->Instance == TIM4) {
        /* USER CODE BEGIN TIM4_MspDeInit 0 */

        /* USER CODE END TIM4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /**TIM4 GPIO Configuration
    PB7     ------> TIM4_CH2
    */
        HAL_GPIO_DeInit(STP_STEP_GPIO_Port, STP_STEP_Pin);

        /* TIM4 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
        /* USER CODE BEGIN TIM4_MspDeInit 1 */

        /* USER CODE END TIM4_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == MEASUREMENTS_TIMER.Instance || htim->Instance == OPEN_WIRE_MEASUREMENT_TIMER.Instance) {
        measurements_oc_handler(htim);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static int samples = 0;
    //static int cell    = 0;
    if (htim->Instance == TIMER_ADC_MEAS.Instance) {
        /*** NOTE: Using mux_fb_index_internal only because the address of the mux is shared between the
                      muxes *and because fb_mux has 16 channels and hall_mux only 8,
            the second mux has to "wait" until *all the channels of the first mux have been read
        ***/
        if (adc_status_flags.is_value_stored) {
            if (samples % N_ADC_SAMPLES == 0 && samples != 0) {
                if (!adc_status_flags.hall_calibration) {
                    adc_status_flags.mux_address_index       = (adc_status_flags.mux_address_index + 1) % MAX_MUX_LEN;
                    adc_status_flags.mux_hall_index_internal = (adc_status_flags.mux_hall_index_internal + 1) %
                                                               MUX_HALL_LEN;
                    adc_status_flags.mux_fb_index_internal = (adc_status_flags.mux_fb_index_internal + 1) % MUX_FB_LEN;
                } else {
                    adc_status_flags.mux_address_index       = (adc_status_flags.mux_address_index + 2) % 6;
                    adc_status_flags.mux_hall_index_internal = (adc_status_flags.mux_hall_index_internal + 2) % 6;
                }
                samples = 0;
            }
            volatile uint8_t add = ADC_get_mux_address_by_port_name(adc_status_flags.mux_address_index);
            if (add == 255) {
                error_set(ERROR_ADC_MUX, 0);
            } else {
                error_reset(ERROR_ADC_MUX, 0);
                ADC_set_mux_address(add);
            }

            HAL_ADC_Start_DMA(&ADC_HALL_AND_FB, (uint32_t *)&adc2_channels, ADC2_CHANNELS_LEN);
            samples++;
            adc_status_flags.is_value_stored = false;
        }
    } else if (htim->Instance == HTIM_ERR.Instance) {
        ERROR_UTILS_TimerElapsedCallback(&error_handler, htim);
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
