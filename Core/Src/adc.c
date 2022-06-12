/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "stdbool.h"
#include "tim.h"
/*
* NOTES:
* How the ADC should be configured:
* - hadcx.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
*   this value selects the prescaler applied to the ADC peripheral clock line
*   (APB2 clock peripheral line (PCLK2), to check this look into
*   fenice-bms-lv/Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f446xx.h)
*   A CUBEMX bugs prevents to use division lower then DIV4 but in reality it
*   works even with DIV2
*   Set this value to define the base ADC CLK sample time wich will influence
*   the conversion time.
* - hadcx.Init.Resolution            = ADC_RESOLUTION_12B;
*   This value sets the ADC resolution, smaller resolutions let you run faster:
*    12 bits: 12 ADCCLK cycles
*    10 bits: 10 ADCCLK cycles
*    8 bits: 8 ADCCLK cycles
*    6 bits: 6 ADCCLK cycles
* - hadcx.Init.ScanConvMode          = ENABLE;
*   this makes the ADC scan multiple channels instead of only one
*   (channel == input pin/peripheral)
* - hadcx.Init.ContinuousConvMode    = DISABLE;
*   enabling this option would mean that once all channels are converted the
*   ADC restarts instanlty conversion from the first channel without calling a
*   HAL_ADC_Start_XXX(hadcX).
*   Since the ADC will be run in DMA mode we don't want to always keep the DMA
*   busy by useless DMA transfers if we need, say, a value each 2 ms
* - hadcx.Init.DiscontinuousConvMode = DISABLE;
*   this will be disabled because in theory is used to use the ADC in Injected
*   Conversion Mode
* - hadcx.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
*   This option would let us use a external timer for starting the conversion,
*   we unfortunately dont' have one available for ADC1
* - hadcx.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
*   Sets who would be the trigger for the readings: software , timer capture
*   compare or timer trigger output
* - hadcx.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
*   How the sampled data is aligned in the halword (16byte) ADC Data register:
*                   ADC DATA register
*   DATA RIGHT:    XXXX DDDD DDDD DDDD  (use this)
*   DATA LEFT:     DDDD DDDD DDDD XXXX
* - hadcx.Init.NbrOfConversion       = 5;
*   If ContinuousConvMode = ENABLE this defines how many channels (a.k.a ranks)
*   will be converted by the ADC in a single run
* - hadcx.Init.DMAContinuousRequests = ENABLE; (https://electronics.stackexchange.com/questions/500827/dma-continuous-request-functionality)
*  Two types of conversions:
*  DMA one shot when DMAContinuousRequests=DISABLE
*  - The ADC sends a DMA transfer request each time a new conversion data word
*    is available.
*  - The ADC stops DMA transfer requests once ALL dma transfers are completed
*  (DMA_EOT interrupt occures)
*  Meaning: the DMA must be restarted when all transfers are completed
*  DMA one shot when DMAContinuousRequests=ENABLE (DMA must be configured in circular mode)
*  - The ADC sends a DMA transfer request each time a new conversion data word
*    is available.
*  - The ADC will continuously send DMA transfers even if the DMA has reached
*  the last DMA transfer (DMA_EOT interrupt occures). This allows the DMA to be
*  configured in circular mode to handle a continuous analog input data stream.
*  Meaning: the DMA must not be restarted when all transfers are completed
*  because its always ready to rewrite the data buffer
*  We want ENABLE => [channel1,....,channel5] => [dbuf1,...,dbuf5] once ADC
*  reaches channel5 the DMA will not stop but repostion to dbuf1 and wait for
*  when a new conversion will be ready (this will happen if we restart the ADC
*  becasuse continuous conversion mode is disabled)
* - hadcx.Init.EOCSelection          = ADC_EOC_SEQ_CONV;  (not important)
*   Specifies what EOC (End Of Conversion) flag is used for conversion by
*   polling and interruption: end of conversion of each rank or complete
*   sequence.
*   Meaning: this will tell in polling mode when the conversion is complete or
*   in iterrupt mode when to trigger the interrupt. Once per conversion or when
*   all conversion are complete.
*
*  NOTES FOR RANKS/CHANNEL CONVERSIONS:
*  - sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
*  This values can be either:
*   ADC_SAMPLETIME_3CYCLES
*   ADC_SAMPLETIME_15CYCLES
*   ADC_SAMPLETIME_28CYCLES
*   ADC_SAMPLETIME_56CYCLES
*   ADC_SAMPLETIME_84CYCLES
*   ADC_SAMPLETIME_112CYCLES
*   ADC_SAMPLETIME_144CYCLES
*   ADC_SAMPLETIME_480CYCLES
* Meaning: once the resolution of the ADC is set it defines the base number of
* ADCCLK cycles necessary to perform that conversion. Then you can define how
* much extra time the ADC will hold the value before sampling it (notice that
* the minimum value is 3 CYCLES)
* **** To compute the total conversion time of channel/rank use: ****
* **** Tconv = (ChannelSamplingTime + ADCResolutionSamplingTime)/ADCCLK   ******
    - APB2 CLOCK at 50 MHz, ADC prescaler 4 DIV => ADC CLK = 12.5 MHz
    12 bit Resolution => ADCResolutionSamplingTime = 12 cycles
    ChannelSamplingTime => 112 CYCLES.
    conversion Time = (112 + 12) / 12.5 MHz = 9.9 us
*/

#define N_ADC1_CONVERSIONS (1U)
#define N_ADC2_CONVERSIONS (4U)

#define ADC_FSVR_mV (3300U)  // ADC Full-Scale Voltage Range or span in milliVolts (VrefHi-VrefLow)

/* WARNING: change the order of this enums if the rank order in the ADC changes */
typedef enum { adc1_values_idx_HO_50S_SP33, adc1_values_idx_NUM_ADC_VALUES = N_ADC1_CONVERSIONS } __adc1_values_idx_TD;

typedef enum {
    adc2_values_idx_term_couple_batt1,
    adc2_values_idx_term_couple_batt2,
    adc2_values_idx_term_couple_dcdc12v,
    adc2_values_idx_term_couple_dcdc24v,
    adc2_values_idx_NUM_ADC_VALUES = N_ADC2_CONVERSIONS
} __adc2_values_idx_TD;

/* These arrays will hold ADC values retrived via DMA */
uint16_t adc1_values[N_ADC1_CONVERSIONS] = {};
uint16_t adc2_values[N_ADC2_CONVERSIONS] = {};

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void) {
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T5_CC1;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
    sConfig.Channel      = ADC_CHANNEL_10;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}
/* ADC2 init function */
void MX_ADC2_Init(void) {
    /* USER CODE BEGIN ADC2_Init 0 */

    /* USER CODE END ADC2_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC2_Init 1 */

    /* USER CODE END ADC2_Init 1 */
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
    hadc2.Instance                   = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode          = ENABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion       = 4;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank    = 2;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank    = 3;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
    sConfig.Channel = ADC_CHANNEL_13;
    sConfig.Rank    = 4;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC2_Init 2 */

    /* USER CODE END ADC2_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (adcHandle->Instance == ADC1) {
        /* USER CODE BEGIN ADC1_MspInit 0 */

        /* USER CODE END ADC1_MspInit 0 */
        /* ADC1 clock enable */
        __HAL_RCC_ADC1_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    */
        GPIO_InitStruct.Pin  = HALL_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(HALL_GPIO_Port, &GPIO_InitStruct);

        /* ADC1 DMA Init */
        /* ADC1 Init */
        hdma_adc1.Instance                 = DMA2_Stream0;
        hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode                = DMA_CIRCULAR;
        hdma_adc1.Init.Priority            = DMA_PRIORITY_MEDIUM;
        hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);

        /* ADC1 interrupt Init */
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
        /* USER CODE BEGIN ADC1_MspInit 1 */

        /* USER CODE END ADC1_MspInit 1 */
    } else if (adcHandle->Instance == ADC2) {
        /* USER CODE BEGIN ADC2_MspInit 0 */

        /* USER CODE END ADC2_MspInit 0 */
        /* ADC2 clock enable */
        __HAL_RCC_ADC2_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**ADC2 GPIO Configuration
    PC2     ------> ADC2_IN12
    PC3     ------> ADC2_IN13
    PA0-WKUP     ------> ADC2_IN0
    PA1     ------> ADC2_IN1
    */
        GPIO_InitStruct.Pin  = TMP_DCDC12_Pin | TMP_DCDC24_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = TMP_BATT1_Pin | TMP_BATT2_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ADC2 DMA Init */
        /* ADC2 Init */
        hdma_adc2.Instance                 = DMA2_Stream2;
        hdma_adc2.Init.Channel             = DMA_CHANNEL_1;
        hdma_adc2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc2.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc2.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc2.Init.Mode                = DMA_CIRCULAR;
        hdma_adc2.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_adc2.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc2);

        /* ADC2 interrupt Init */
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
        /* USER CODE BEGIN ADC2_MspInit 1 */

        /* USER CODE END ADC2_MspInit 1 */
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle) {
    if (adcHandle->Instance == ADC1) {
        /* USER CODE BEGIN ADC1_MspDeInit 0 */

        /* USER CODE END ADC1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC1_CLK_DISABLE();

        /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    */
        HAL_GPIO_DeInit(HALL_GPIO_Port, HALL_Pin);

        /* ADC1 DMA DeInit */
        HAL_DMA_DeInit(adcHandle->DMA_Handle);

        /* ADC1 interrupt Deinit */
        /* USER CODE BEGIN ADC1:ADC_IRQn disable */
        /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
        /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
        /* USER CODE END ADC1:ADC_IRQn disable */

        /* USER CODE BEGIN ADC1_MspDeInit 1 */

        /* USER CODE END ADC1_MspDeInit 1 */
    } else if (adcHandle->Instance == ADC2) {
        /* USER CODE BEGIN ADC2_MspDeInit 0 */

        /* USER CODE END ADC2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC2_CLK_DISABLE();

        /**ADC2 GPIO Configuration
    PC2     ------> ADC2_IN12
    PC3     ------> ADC2_IN13
    PA0-WKUP     ------> ADC2_IN0
    PA1     ------> ADC2_IN1
    */
        HAL_GPIO_DeInit(GPIOC, TMP_DCDC12_Pin | TMP_DCDC24_Pin);

        HAL_GPIO_DeInit(GPIOA, TMP_BATT1_Pin | TMP_BATT2_Pin);

        /* ADC2 DMA DeInit */
        HAL_DMA_DeInit(adcHandle->DMA_Handle);

        /* ADC2 interrupt Deinit */
        /* USER CODE BEGIN ADC2:ADC_IRQn disable */
        /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
        /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
        /* USER CODE END ADC2:ADC_IRQn disable */

        /* USER CODE BEGIN ADC2_MspDeInit 1 */

        /* USER CODE END ADC2_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

uint8_t ADC_get_resolution_bits(ADC_HandleTypeDef *adcHandle) {
    // To get this value look at the reference manual RM0390
    // page 385 section 13.13.2 register ADC_CR1 RES[1:0] bits
    uint8_t ret = 12;
    switch (ADC_GET_RESOLUTION(adcHandle)) {
        case 0U:
            ret = 12U;
            break;
        case 1U:
            ret = 10U;
            break;
        case 2U:
            ret = 8U;
            break;
        case 4U:
            ret = 6U;
            break;
    }
    return ret;
}

uint32_t ADC_get_tot_voltage_levels(ADC_HandleTypeDef *adcHandle) {
    uint8_t value = ADC_get_resolution_bits(adcHandle);
    return ((uint32_t)(1U << value));  // 2^value
}

float ADC_get_value_mV(ADC_HandleTypeDef *adcHandle, uint32_t value_from_adc) {
    return value_from_adc * ((float)ADC_FSVR_mV / ADC_get_tot_voltage_levels(adcHandle));
}

void ADC_start_DMA_readings() {
    static bool oneTime = true;
    if (oneTime) {
        oneTime = false;
        // Timer for the electrical current readings (trigger on Capture Compare so enable in PWM mode)
        HAL_TIM_PWM_Start(&ADC_ELECTRICAL_CURRENT_READINGS_TIMER_HTIM, ADC_ELECTRICAL_CURRENT_READINGS_TIMER_CHANNEL);
        // Timer for the temperature readings
        HAL_TIM_Base_Start_IT(&ADC_TEMPERATURE_READINGS_TIMER_HTIM);

        // This ADC must be enabled only once because it gets triggered via HW
        HAL_ADC_Start_DMA(&CURRENT_TRANSDUCER_HADC, (uint32_t *)adc1_values, N_ADC1_CONVERSIONS);
    }

    // This ADC is configured in SOFTWARE TRIGGERED conversion
    // So this routine MUST be called by software each time a new conversion is necessary
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_values, N_ADC2_CONVERSIONS);
};
/**
 * NOTE: prepare to read shitty getters
 * Since __CUBEMIX e' stronzo (cit.)__ we can't map in an automaitc way adc ranks to a constant in the code
 * (we would need to manually write the MX_ADC_Init ourselves loosing CUBE MX functionality :( )
 * Therefore if ADC ranks change this values need to change accorgingly to their position and meaning
 **/

uint16_t ADC_get_HO_50S_SP33_1106_sensor_val() {
    return adc1_values[adc1_values_idx_HO_50S_SP33];
};

uint16_t ADC_get_t_batt1_val() {
    return adc2_values[adc2_values_idx_term_couple_batt1];
};

uint16_t ADC_get_t_batt2_val() {
    return adc2_values[adc2_values_idx_term_couple_batt2];
};

uint16_t ADC_get_t_dcdc12_val() {
    return adc2_values[adc2_values_idx_term_couple_dcdc12v];
};

uint16_t ADC_get_t_dcdc24_val() {
    return adc2_values[adc2_values_idx_term_couple_dcdc24v];
};

// TODO: delete old code
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//    if (hadc == &hadc1) {
//        if (UserAdcConfig.Channel == ADC_CHANNEL_2) {
//            uint32_t current;
//            uint32_t adc_val;
//            adc_val = HAL_ADC_GetValue(hadc);
//            current = calc_current(adc_val);
//            push_into_array(current);
//            mean_current();
//        }
//        HAL_ADC_Start_IT(&hadc1);
//        new = true;
//    }
//}

/* USER CODE END 1 */
