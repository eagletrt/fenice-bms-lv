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

#define N_ADC1_CONVERSIONS              (HO_50S_SP33_SAMPLES_FOR_AVERAGE)
#define N_ADC2_CONVERSIONS              (4U)
#define N_ADC2_SAMPLES_FOR_EACH_CHANNEL (10U)
#define N_ADC2_VALUES_SIZE              (N_ADC2_CONVERSIONS * N_ADC2_SAMPLES_FOR_EACH_CHANNEL)

#define ADC_FSVR_mV (3300U)  // ADC Full-Scale Voltage Range or span in milliVolts (VrefHi-VrefLow)

/* WARNING: change the order of this enums if the rank order in the ADC changes */
//typedef enum { adc1_values_idx_HO_50S_SP33, adc1_values_idx_NUM_ADC_VALUES = N_ADC1_CONVERSIONS } __adc1_values_idx_TD;
typedef enum {
    adc2_values_idx_term_couple_batt1 = 0U,
    adc2_values_idx_term_couple_batt2,
    adc2_values_idx_term_couple_dcdc12v,
    adc2_values_idx_term_couple_dcdc24v,
    adc2_values_idx_NUM_ADC_VALUES = N_ADC2_CONVERSIONS
} __adc2_values_idx_TD;

/* These arrays will hold ADC values retrived via DMA */
uint16_t adc1_values[N_ADC1_CONVERSIONS] = {};
uint16_t adc2_values[N_ADC2_VALUES_SIZE] = {};

typedef enum {
    MUX_I0  = 0b0000,
    MUX_I1  = 0b1000,
    MUX_I2  = 0b0100,
    MUX_I3  = 0b1100,
    MUX_I4  = 0b0010,
    MUX_I5  = 0b1010,
    MUX_I6  = 0b0110,
    MUX_I7  = 0b1110,
    MUX_I8  = 0b0001,
    MUX_I9  = 0b1001,
    MUX_I10 = 0b0101,
    MUX_I11 = 0b1101,
    MUX_I12 = 0b0011,
    MUX_I13 = 0b1011,
    MUX_I14 = 0b0111,
    MUX_I15 = 0b1111,
} MUX_ADDRESSES;

typedef enum {
    HALL_OCD   = MUX_I0,
    S_HALL0    = MUX_I1,
    HALL_OCD1  = MUX_I2,
    S_HALL1    = MUX_I3,
    HALL_OCD2  = MUX_I4,
    S_HALL2    = MUX_I5,
    LVAC_TEMP0 = MUX_I6,
    LVAC_TEMP1 = MUX_I7,
} MUX_HALL_INPUTS;

typedef enum {
    SD_END           = MUX_I0,
    BSPD_FB          = MUX_I1,
    IMD_FB           = MUX_I2,
    LVMS_FB          = MUX_I3,
    RES_FB           = MUX_I4,
    TSMS_FB          = MUX_I5,
    LV_ENCL_1_FB     = MUX_I6,
    LV_ENCL_2_FB     = MUX_I7,
    HV_ENCL_1_FB     = MUX_I8,
    HV_ENCL_2_FB     = MUX_I9,
    BACK_PLATE_FB    = MUX_I10,
    HVD_FB           = MUX_I11,
    AMS_FB           = MUX_I12,
    ASMS_FB          = MUX_I13,
    INTERLOCK_IMD_FB = MUX_I14,
    SD_START         = MUX_I15,
} MUX_FB_INPUT;

typedef struct {
    uint8_t MUX_FB_OUT;
    uint8_t MUX_FB_OUT;
    uint8_t MUX_HALL;
    uint8_t MUX_HALL;
} ADC2_Channels;

typedef struct {
    uint16_t HALL_OCD0;
    uint16_t S_HALL0;
    uint16_t HALL_OCD1;
    uint16_t S_HALL1;
    uint16_t HALL_OCD2;
    uint16_t S_HALL2;
    uint16_t LVAC_TEMP0;
    uint16_t LVAC_TEMP1;
} MUX_HALL_RAW_VALUES;

typedef struct {
    uint16_t SD_END;
    uint16_t BSPD_FB;
    uint16_t IMD_FB;
    uint16_t LVMS_FB;
    uint16_t RES_FB;
    uint16_t TSMS_FB;
    uint16_t LV_ENCL_1_FB;
    uint16_t LV_ENCL_2_FB;
    uint16_t HV_ENCL_1_FB;
    uint16_t HV_ENCL_2_FB;
    uint16_t BACK_PLATE_FB;
    uint16_t HVD_FB;
    uint16_t AMS_FB;
    uint16_t ASMS_FB;
    uint16_t INTERLOCK_IMD_FB;
    uint16_t SD_START;
} MUX_FB_RAW_VALUES;

ADC2_Channels adc_channels;
MUX_HALL_RAW_VALUES adcs_raw_hall[10];  // Multiplexed values
MUX_FB_RAW_VALUES adcs_raw_fb[10];      // Multiplexed values
uint16_t adcs_raw_as_computer_fb[10];   // Direct ADC
uint16_t adcs_raw_relay_out[10];        // Direct ADC
uint16_t adcs_raw_lvms_out[10];         // Direct ADC
uint16_t adcs_raw_batt_out[10];         // Direct ADC

/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle){
   static bool first_sample = true;
    static uint16_t avg;
    if (first_sample) {
        avg          = ch.MUX_FB_OUT;
        first_sample = false;
    }
    avg = 0.99 * avg + 0.01 * ch.MUX_FB_OUT;
    adcs[1].HALL_OCD0 = ch.MUX_HALL;
}
*/

/* USER CODE END 0 */

ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

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
    hadc2.Init.NbrOfConversion       = 5;
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
    sConfig.Channel = ADC_CHANNEL_13;
    sConfig.Rank    = 3;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank    = 4;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank    = 5;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC2_Init 2 */

    /* USER CODE END ADC2_Init 2 */
}
/* ADC3 init function */
void MX_ADC3_Init(void) {
    /* USER CODE BEGIN ADC3_Init 0 */

    /* USER CODE END ADC3_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC3_Init 1 */

    /* USER CODE END ADC3_Init 1 */
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
    hadc3.Instance                   = ADC3;
    hadc3.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode          = ENABLE;
    hadc3.Init.ContinuousConvMode    = ENABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion       = 1;
    hadc3.Init.DMAContinuousRequests = ENABLE;
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
    sConfig.Channel      = ADC_CHANNEL_3;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC3_Init 2 */

    /* USER CODE END ADC3_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (adcHandle->Instance == ADC2) {
        /* USER CODE BEGIN ADC2_MspInit 0 */

        /* USER CODE END ADC2_MspInit 0 */
        /* ADC2 clock enable */
        __HAL_RCC_ADC2_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**ADC2 GPIO Configuration
    PC3     ------> ADC2_IN13
    PA0-WKUP     ------> ADC2_IN0
    PA1     ------> ADC2_IN1
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15
    */
        GPIO_InitStruct.Pin  = AS_COMPUTER_FB_Pin | REAY_OUT_ANALOG_FB_Pin | LVMS_OUT_ANALOG_FB_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = MUX_FB_OUT_Pin | MUX_HALL_Pin;
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
    } else if (adcHandle->Instance == ADC3) {
        /* USER CODE BEGIN ADC3_MspInit 0 */

        /* USER CODE END ADC3_MspInit 0 */
        /* ADC3 clock enable */
        __HAL_RCC_ADC3_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**ADC3 GPIO Configuration
    PA3     ------> ADC3_IN3
    */
        GPIO_InitStruct.Pin  = BATT_OUT_ANALOG_FB_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(BATT_OUT_ANALOG_FB_GPIO_Port, &GPIO_InitStruct);

        /* ADC3 DMA Init */
        /* ADC3 Init */
        hdma_adc3.Instance                 = DMA2_Stream0;
        hdma_adc3.Init.Channel             = DMA_CHANNEL_2;
        hdma_adc3.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc3.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc3.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc3.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc3.Init.Mode                = DMA_CIRCULAR;
        hdma_adc3.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_adc3.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc3) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc3);

        /* ADC3 interrupt Init */
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
        /* USER CODE BEGIN ADC3_MspInit 1 */

        /* USER CODE END ADC3_MspInit 1 */
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle) {
    if (adcHandle->Instance == ADC2) {
        /* USER CODE BEGIN ADC2_MspDeInit 0 */

        /* USER CODE END ADC2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC2_CLK_DISABLE();

        /**ADC2 GPIO Configuration
    PC3     ------> ADC2_IN13
    PA0-WKUP     ------> ADC2_IN0
    PA1     ------> ADC2_IN1
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15
    */
        HAL_GPIO_DeInit(GPIOC, AS_COMPUTER_FB_Pin | REAY_OUT_ANALOG_FB_Pin | LVMS_OUT_ANALOG_FB_Pin);

        HAL_GPIO_DeInit(GPIOA, MUX_FB_OUT_Pin | MUX_HALL_Pin);

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
    } else if (adcHandle->Instance == ADC3) {
        /* USER CODE BEGIN ADC3_MspDeInit 0 */

        /* USER CODE END ADC3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC3_CLK_DISABLE();

        /**ADC3 GPIO Configuration
    PA3     ------> ADC3_IN3
    */
        HAL_GPIO_DeInit(BATT_OUT_ANALOG_FB_GPIO_Port, BATT_OUT_ANALOG_FB_Pin);

        /* ADC3 DMA DeInit */
        HAL_DMA_DeInit(adcHandle->DMA_Handle);

        /* ADC3 interrupt Deinit */
        /* USER CODE BEGIN ADC3:ADC_IRQn disable */
        /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
        /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
        /* USER CODE END ADC3:ADC_IRQn disable */

        /* USER CODE BEGIN ADC3_MspDeInit 1 */

        /* USER CODE END ADC3_MspDeInit 1 */
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
    return ((uint32_t)(1U << value) - 1);  // 2^value - 1
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

        // This ADC must be enabled only once because it gets triggered via HW by a timer
        HAL_ADC_Start_DMA(&CURRENT_TRANSDUCER_HADC, (uint32_t *)adc1_values, N_ADC1_CONVERSIONS);
    }

    // This ADC is configured in SOFTWARE TRIGGERED conversion
    // So this routine MUST be called by software each time a new conversion is necessary
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_values, N_ADC2_VALUES_SIZE);
};
/**
 * NOTE: prepare to read shitty getters
 * Since __CUBEMIX e' stronzo (cit.)__ we can't map in an automaitc way adc ranks to a constant in the code
 * (we would need to manually write the MX_ADC_Init ourselves loosing CUBE MX functionality :( )
 * Therefore if ADC ranks change this values need to change accorgingly to their position and meaning
 **/

uint16_t ADC_get_HO_50S_SP33_1106_sensor_val() {
    uint32_t result = 0;
    for (uint32_t i = 0; i < HO_50S_SP33_SAMPLES_FOR_AVERAGE; i++) {
        result += adc1_values[i];
    }
    return (uint16_t)(result / HO_50S_SP33_SAMPLES_FOR_AVERAGE);
};

uint16_t ADC_get_t_batt1_val() {
    uint32_t result = 0;
    for (uint32_t i = 0; i < N_ADC2_SAMPLES_FOR_EACH_CHANNEL; i++) {
        result += adc2_values[(N_ADC2_CONVERSIONS * i) + adc2_values_idx_term_couple_batt1];
    }
    return (uint16_t)(result / N_ADC2_SAMPLES_FOR_EACH_CHANNEL);
}

uint16_t ADC_get_t_batt2_val() {
    uint32_t result = 0;
    for (uint32_t i = 0; i < N_ADC2_SAMPLES_FOR_EACH_CHANNEL; i++) {
        result += adc2_values[(N_ADC2_CONVERSIONS * i) + adc2_values_idx_term_couple_batt2];
    }
    return (uint16_t)(result / N_ADC2_SAMPLES_FOR_EACH_CHANNEL);
};

uint16_t ADC_get_t_dcdc12_val() {
    uint32_t result = 0;
    for (uint32_t i = 0; i < N_ADC2_SAMPLES_FOR_EACH_CHANNEL; i++) {
        result += adc2_values[(N_ADC2_CONVERSIONS * i) + adc2_values_idx_term_couple_dcdc12v];
    }
    return (uint16_t)(result / N_ADC2_SAMPLES_FOR_EACH_CHANNEL);
};

uint16_t ADC_get_t_dcdc24_val() {
    uint32_t result = 0;
    for (uint32_t i = 0; i < N_ADC2_SAMPLES_FOR_EACH_CHANNEL; i++) {
        result += adc2_values[(N_ADC2_CONVERSIONS * i) + adc2_values_idx_term_couple_dcdc24v];
    }
    return (uint16_t)(result / N_ADC2_SAMPLES_FOR_EACH_CHANNEL);
};

/* USER CODE END 1 */
