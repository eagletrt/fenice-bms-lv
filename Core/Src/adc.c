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
#include "current_transducer.h"
#include "error.h"
#include "stdbool.h"
#include "tim.h"

#ifdef DEBUG
#include "cli_bms_lv.h"
#include "stdio.h"
#include "string.h"

char buffer[200] = "";
#endif

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

#define ADC_FSVR_mV (3300U)  // ADC Full-Scale Voltage Range or span in milliVolts (VrefHi-VrefLow)
ADC_status_flags_t adc_status_flags;

ADC2_Channels_t adc2_channels;
ADC2_Sampled_Signals_t adc2_sampled_signals;
ADC_converted adcs_converted_values;

bool vref_calibration = true;
uint16_t vref;  // Voltage used by the ADC as a reference
primary_lv_feedbacks_converted_t primary_lv_feedbacks_converted;

void ADC_start_ADC2_readings() {
    HAL_TIM_Base_Start_IT(&TIMER_ADC_MEAS);
}

/**
 * This is done to know the voltage used by the ADC as a reference
 * to see how this formula has been found check the paper at link: http://www.efton.sk/STM32/STM32_VREF.pdf
 * or go in Doc/STM32_VREF.pdf
*/
void ADC_Vref_Calibration() {
    uint16_t buffer[N_ADC_CALIBRATION_CHANNELS * N_ADC_CALIBRATION_SAMPLES] = {};
    uint32_t vdda                                                           = 0;
    uint32_t vref_int                                                       = 0;
    uint16_t *factory_calibration                                           = (uint16_t *)0x1FFF7A2A;
    HAL_TIM_PWM_Start(&TIMER_ADC_CALIBRATION, TIMER_ADC_CALIBRATION_CHANNEL);
    HAL_ADC_Start_DMA(&CALIBRATION_ADC, (uint32_t *)&buffer, N_ADC_CALIBRATION_CHANNELS * N_ADC_CALIBRATION_SAMPLES);

    //Wait until 500 samples has been reached
    while (vref_calibration) {
    }

    HAL_TIM_PWM_Stop(&TIMER_ADC_CALIBRATION, TIMER_ADC_CALIBRATION_CHANNEL);
    for (uint16_t i = 0; i < N_ADC_CALIBRATION_SAMPLES; i++) {
        vref_int += buffer[N_ADC_CALIBRATION_CHANNELS * i];
        vdda += buffer[N_ADC_CALIBRATION_CHANNELS * i + 1];
    }

    vdda /= 500;
    vref_int /= 500;

    vref = ADC_get_value_mV(&hadc1, vdda) * ((float)*factory_calibration / vref_int);
    if (vref < 3100 || vref > 3300) {
        error_set(ERROR_ADC_INIT, 0);
    }
}

void ADC_init_mux() {
    ADC_set_mux_address(0x0);
}

void ADC_init_status_flags() {
    adc_status_flags.is_adc2_conv_complete   = false;
    adc_status_flags.is_value_stored         = true;  // in order to start the first conversion
    adc_status_flags.mux_address_index       = 0;
    adc_status_flags.mux_hall_index_internal = 0;
    adc_status_flags.mux_fb_index_internal   = 0;
    adc_status_flags.mux_hall_index_external = 0;
    adc_status_flags.mux_fb_index_external   = 0;
}

uint8_t ADC_get_mux_address_by_port_name(uint8_t portname) {
    /*
    NOTE: MUX_FB_INPUT_ADDRESSES has been used to define the mux addresses
    anyway because the muxes share the same address pins the MUX_HALL_INPUTS_ADDRESSES
    are also (implicitly) included in this switch case
    */

    switch (portname) {
        case SD_END:
            return MUX_I0;
        case BSPD_FB:
            return MUX_I1;
        case IMD_FB:
            return MUX_I2;
        case LVMS_FB:
            return MUX_I3;
        case RES_FB:
            return MUX_I4;
        case S_HALL2:
            return MUX_I5;
        case LV_ENCL_FB:
            return MUX_I6;
        case LVAC_TEMP1:
            return MUX_I7;
        case HV_ENCL_1_FB:
            return MUX_I8;
        case HV_ENCL_2_FB:
            return MUX_I9;
        case BACK_PLATE_FB:
            return MUX_I10;
        case HVD_FB:
            return MUX_I11;
        case AMS_FB:
            return MUX_I12;
        case ASMS_FB:
            return MUX_I13;
        case INTERLOCK_IMD_FB:
            return MUX_I14;
        case SD_START:
            return MUX_I15;
        default:
            return 255;
    }
}

void ADC_set_mux_address(uint8_t address) {
    // A0 A1 A2 A3
    HAL_GPIO_WritePin(MUX_A3_GPIO_Port, MUX_A3_Pin, (address & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, (address & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_A1_GPIO_Port, MUX_A1_Pin, (address & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, (address & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ADC_Routine() {
    if (adc_status_flags.is_adc2_conv_complete) {
        // First channel
        if (adc_status_flags.mux_address_index < MUX_HALL_LEN) {
            uint16_t *mux_hall_val =
                (uint16_t *)&adc2_sampled_signals.adcs_raw_hall[adc_status_flags.mux_hall_index_external];
            *(mux_hall_val + adc_status_flags.mux_hall_index_internal) = adc2_channels.mux_hall;
            adc_status_flags.mux_hall_index_external = (adc_status_flags.mux_hall_index_external + 1) %
                                                       N_ADC_SAMPLES_MUX_HALL;
        }

        // Second channel
        uint16_t *mux_fb_val = (uint16_t *)&adc2_sampled_signals.adcs_raw_fb[adc_status_flags.mux_fb_index_external];
        *(mux_fb_val + adc_status_flags.mux_fb_index_internal) = adc2_channels.mux_fb;
        adc_status_flags.mux_fb_index_external = (adc_status_flags.mux_fb_index_external + 1) % N_ADC_SAMPLES_MUX_FB;

        // Third channel
        uint16_t *as_computer_fb_val =
            &adc2_sampled_signals.adcs_raw_as_computer_fb[adc_status_flags.as_computer_fb_index_external];
        *(as_computer_fb_val)                          = adc2_channels.adcs_as_computer_fb;
        adc_status_flags.as_computer_fb_index_external = (adc_status_flags.as_computer_fb_index_external + 1) %
                                                         N_ADC_SAMPLES_AS_COMPUTER_FB;

        // Fourth channel
        uint16_t *relay_out_val = &adc2_sampled_signals.adcs_raw_relay_out[adc_status_flags.relay_out_index_external];
        *(relay_out_val)        = adc2_channels.adcs_relay_out;
        adc_status_flags.relay_out_index_external = (adc_status_flags.relay_out_index_external + 1) %
                                                    N_ADC_SAMPLES_RELAY_OUT;

        // Fifth channel
        uint16_t *lvms_out_val = &adc2_sampled_signals.adcs_raw_lvms_out[adc_status_flags.lvms_out_index_external];
        *(lvms_out_val)        = adc2_channels.adcs_lvms_out;
        adc_status_flags.lvms_out_index_external = (adc_status_flags.lvms_out_index_external + 1) %
                                                   N_ADC_SAMPLES_LVMS_OUT;

        // Sixth channel
        uint16_t *batt_out_val = &adc2_sampled_signals.adcs_raw_batt_out[adc_status_flags.batt_out_index_external];
        *(batt_out_val)        = adc2_channels.adcs_batt_out;
        adc_status_flags.batt_out_index_external = (adc_status_flags.batt_out_index_external + 1) %
                                                   N_ADC_SAMPLES_BATT_OUT;

        adc_status_flags.is_adc2_conv_complete = false;
        adc_status_flags.is_value_stored       = true;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle) {
    if (AdcHandle->Instance == ADC_HALL_AND_FB.Instance) {
        adc_status_flags.is_adc2_conv_complete   = true;
        adc_status_flags.mux_hall_index_external = (adc_status_flags.mux_hall_index_external + 1) % N_ADC_SAMPLES;
        adc_status_flags.is_value_stored         = true;
    } else if (AdcHandle->Instance == CALIBRATION_ADC.Instance) {
        vref_calibration = false;
    }
}

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

float ADC_get_calibrated_mV(ADC_HandleTypeDef *adcHandle, uint32_t value_from_adc) {
    return value_from_adc * ((float)vref / ADC_get_tot_voltage_levels(adcHandle));
}

void relay_out_conversion() {
    uint32_t result = 0;
    for (uint8_t i = 0; i < N_ADC_SAMPLES_RELAY_OUT; i++) {
        result += adc2_sampled_signals.adcs_raw_relay_out[i];
    }
    adcs_converted_values.relay_out = ADC_get_calibrated_mV(&ADC_HALL_AND_FB, result / N_ADC_SAMPLES_RELAY_OUT) *
                                      ADC2_VOLTAGE_DIVIDER_MULTIPLIER;

#ifdef DEBUG
    sprintf(buffer, "Relay out time: %lu\r\n", (HAL_GetTick() / 10));
    cli_bms_debug(buffer, strlen(buffer));
#endif
}

void lvms_out_conversion() {
    uint32_t result = 0;
    for (uint8_t i = 0; i < N_ADC_SAMPLES_LVMS_OUT; i++) {
        result += adc2_sampled_signals.adcs_raw_lvms_out[i];
    }
    adcs_converted_values.lvms_out = ADC_get_calibrated_mV(&ADC_HALL_AND_FB, result / N_ADC_SAMPLES_LVMS_OUT) *
                                     ADC2_VOLTAGE_DIVIDER_MULTIPLIER;
#ifdef DEBUG
    sprintf(buffer, "Lvms out time: %lu\r\n", HAL_GetTick() / 10);
    cli_bms_debug(buffer, strlen(buffer));
#endif
}

void as_computer_fb_conversion() {
    uint32_t result = 0;
    for (uint8_t i = 0; i < N_ADC_SAMPLES_AS_COMPUTER_FB; i++) {
        result += adc2_sampled_signals.adcs_raw_as_computer_fb[i];
    }
    adcs_converted_values.as_computer_fb =
        ADC_get_calibrated_mV(&ADC_HALL_AND_FB, result / N_ADC_SAMPLES_AS_COMPUTER_FB) *
        ADC2_VOLTAGE_DIVIDER_MULTIPLIER;
#ifdef DEBUG
    sprintf(buffer, "as computer fb time: %lu\r\n", HAL_GetTick() / 10);
    cli_bms_debug(buffer, strlen(buffer));
#endif
}

void mux_hall_conversion() {
    uint32_t result = 0;
    uint16_t *mux_hall_raw;
    float *mux_hall_converted = (float *)&adcs_converted_values.mux_hall;
    for (uint8_t i = 0; i < MUX_HALL_LEN; i++) {
        for (uint8_t j = 0; j < N_ADC_SAMPLES_MUX_HALL; j++) {
            mux_hall_raw = (uint16_t *)(&adc2_sampled_signals.adcs_raw_hall[j]) + i;
            result += *mux_hall_raw;
        }
        if (i != S_HALL0 && i != S_HALL1 && i != S_HALL2) {
            *(mux_hall_converted + i) = ADC_get_calibrated_mV(&ADC_HALL_AND_FB, result / N_ADC_SAMPLES_MUX_HALL);
        } else if (i == S_HALL1) {
            *(mux_hall_converted + i) = CT_get_electric_current_mA(result / N_ADC_SAMPLES_MUX_HALL) -
                                        S_HALL_1_OFFSET_mA;
        } else if (i == S_HALL2) {
            *(mux_hall_converted + i) = CT_get_electric_current_mA(result / N_ADC_SAMPLES_MUX_HALL) -
                                        S_HALL_2_OFFSET_mA;
        } else {
            *(mux_hall_converted + i) = CT_get_electric_current_mA(result / N_ADC_SAMPLES_MUX_HALL);
        }
        result = 0;
    }

#ifdef DEBUG
    sprintf(buffer, "Mux hall time: %lu\r\n", HAL_GetTick() / 10);
    cli_bms_debug(buffer, strlen(buffer));
#endif
}

void mux_fb_conversion() {
    uint32_t result = 0;
    uint16_t *mux_fb_raw;
    float *mux_fb_converted = (float *)&adcs_converted_values.mux_fb;

    for (uint8_t i = 0; i < MUX_FB_LEN; i++) {
        if (i != N_CONNECTED_1 || i != N_CONNECTED_2) {
            for (uint8_t j = 0; j < N_ADC_SAMPLES_MUX_FB; j++) {
                mux_fb_raw = (uint16_t *)(&adc2_sampled_signals.adcs_raw_fb[j]) + i;
                result += *mux_fb_raw;
            }

            *(mux_fb_converted + i) = ADC_get_calibrated_mV(&ADC_HALL_AND_FB, result / N_ADC_SAMPLES_MUX_FB) *
                                      ADC2_VOLTAGE_DIVIDER_MULTIPLIER;
            result = 0;
        }
    }

#ifdef DEBUG
    sprintf(buffer, "Mux fb time: %lu\r\n", HAL_GetTick() / 10);
    cli_bms_debug(buffer, strlen(buffer));
#endif
}

void batt_out_conversion() {
    uint32_t result = 0;

    for (uint8_t i = 0; i < N_ADC_SAMPLES_BATT_OUT; i++) {
        result += adc2_sampled_signals.adcs_raw_batt_out[i];
    }
    adcs_converted_values.batt_out = ADC_get_calibrated_mV(&ADC_HALL_AND_FB, result / N_ADC_SAMPLES_BATT_OUT) *
                                     ADC2_VOLTAGE_DIVIDER_MULTIPLIER;

#ifdef DEBUG
    sprintf(buffer, "Batt out time: %lu\r\n", HAL_GetTick() / 10);
    cli_bms_debug(buffer, strlen(buffer));
#endif
}

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 6;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    */
    GPIO_InitStruct.Pin = VREF_ADC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VREF_ADC_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream4;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
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
    PA3     ------> ADC2_IN3
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15
    */
    GPIO_InitStruct.Pin = AS_COMPUTER_FB_Pin|REAY_OUT_ANALOG_FB_Pin|LVMS_OUT_ANALOG_FB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MUX_FB_OUT_Pin|MUX_HALL_Pin|BATT_OUT_ANALOG_FB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA2_Stream2;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    */
    HAL_GPIO_DeInit(VREF_ADC_GPIO_Port, VREF_ADC_Pin);

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
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PC3     ------> ADC2_IN13
    PA0-WKUP     ------> ADC2_IN0
    PA1     ------> ADC2_IN1
    PA3     ------> ADC2_IN3
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15
    */
    HAL_GPIO_DeInit(GPIOC, AS_COMPUTER_FB_Pin|REAY_OUT_ANALOG_FB_Pin|LVMS_OUT_ANALOG_FB_Pin);

    HAL_GPIO_DeInit(GPIOA, MUX_FB_OUT_Pin|MUX_HALL_Pin|BATT_OUT_ANALOG_FB_Pin);

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

/* USER CODE END 1 */
