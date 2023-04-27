/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */
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
    HALL_OCD = 0,
    S_HALL0,
    HALL_OCD1,
    S_HALL1,
    HALL_OCD2,
    S_HALL2,
    LVAC_TEMP0,
    LVAC_TEMP1,
    MUX_HALL_LEN,
} MUX_HALL_INPUTS_ADDRESSES;

typedef enum {
    SD_END = 0,
    BSPD_FB,
    IMD_FB,
    LVMS_FB,
    RES_FB,
    TSMS_FB,
    LV_ENCL_1_FB,
    LV_ENCL_2_FB,
    HV_ENCL_1_FB,
    HV_ENCL_2_FB,
    BACK_PLATE_FB,
    HVD_FB,
    AMS_FB,
    ASMS_FB,
    INTERLOCK_IMD_FB,
    SD_START,
    MUX_FB_LEN,
} MUX_FB_INPUT_ADDRESSES;

typedef struct __attribute__((packed)) {
    uint16_t HALL_OCD0;
    uint16_t S_HALL0;
    uint16_t HALL_OCD1;
    uint16_t S_HALL1;
    uint16_t HALL_OCD2;
    uint16_t S_HALL2;
    uint16_t LVAC_TEMP0;
    uint16_t LVAC_TEMP1;
} MUX_HALL_VALUES;

typedef struct __attribute__((packed)) {
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
} MUX_FB_VALUES;

typedef struct {
    bool is_adc2_conv_complete;
    bool is_adc3_conv_complete;
    uint8_t mux_hall_index_external;        // grande quanto N_SAMPLES
    uint8_t mux_hall_index_internal;        // grande quanto MUX
    uint8_t mux_fb_index_external;          // grande quanto N_SAMPLES
    uint8_t mux_fb_index_internal;          // grande quanto MUX
    uint8_t as_computer_fb_index_external;  // grande quanto N_SAMPLES
    uint8_t relay_out_index_external;       // grande quanto N_SAMPLES
    uint8_t lvms_out_index_external;        // grande quanto N_SAMPLES
    uint8_t batt_out_index_external;        // grande quanto N_SAMPLES
} ADC_status_flags_t;

typedef struct __attribute__((packed)) {
    uint16_t mux_hall;
    uint16_t mux_fb;
    uint16_t adcs_as_computer_fb;
    uint16_t adcs_relay_out;
    uint16_t adcs_lvms_out;
} ADC2_Channels_t;

#define N_ADC_SAMPLES_MUX_HALL       10
#define N_ADC_SAMPLES_MUX_FB         10
#define N_ADC_SAMPLES_AS_COMPUTER_FB 10
#define N_ADC_SAMPLES_RELAY_OUT      10
#define N_ADC_SAMPLES_LVMS_OUT       10
#define N_ADC_SAMPLES_BATT_OUT       10

typedef struct {
    MUX_HALL_VALUES adcs_raw_hall[N_ADC_SAMPLES_MUX_HALL];      // Multiplexed values
    MUX_FB_VALUES adcs_raw_fb[N_ADC_SAMPLES_MUX_FB];            // Multiplexed values
    uint16_t adcs_raw_as_computer_fb[N_ADC_SAMPLES_RELAY_OUT];  // Direct ADC
    uint16_t adcs_raw_relay_out[N_ADC_SAMPLES_RELAY_OUT];       // Direct ADC
    uint16_t adcs_raw_lvms_out[N_ADC_SAMPLES_LVMS_OUT];         // Direct ADC
} ADC2_Sampled_Signals_t;

extern uint16_t adcs_raw_batt_out[N_ADC_SAMPLES_BATT_OUT];  // Direct ADC (used with DMA)

#define ADC2_CHANNELS_LEN (sizeof(ADC2_Channels_t) / sizeof(uint16_t))
extern ADC2_Channels_t adc2_channels;
extern ADC2_Sampled_Signals_t adc2_sampled_signals;
extern ADC_status_flags_t adc_status_flags;

/**
 *  @brief Number of samples used to compute the value of HO_50S_SP33 current transducer
 *        HO_50S_SP33 value is calculated as the average of this samples
 */
#define HO_50S_SP33_SAMPLES_FOR_AVERAGE (10U)

extern ADC_ChannelConfTypeDef UserAdcConfig;
/* USER CODE END Private defines */

void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief Start the dedicated timer for ADC measurements
 * 
 */
void ADC_start_MUX_readings();

/**
 * @brief Init mux address pins to 0x0
 * 
 */
void ADC_init_mux();

/**
 * @brief Set the mux address object
 * @example ADC_set_mux_address(0x0A)
 * @param address A valid address between 0 and 15
 */
void ADC_set_mux_address(uint8_t address);

/**
 * @brief Get the mux address by port name object
 * @example get_mux_address_by_port_name(HALL_OCD)
 * @param portname Use MUX_HALL_INPUTS_ADDRESSES or MUX_FB_INPUTS_ADDRESSES
 * @return uint8_t A valid address between 0 and 15, or -1 if error
 */
uint8_t ADC_get_mux_address_by_port_name(uint8_t portname);

/**
 * @brief ADC routine which is called in order to 
 * transfer the data from the DMA buffer to a custom struct
 * 
 */
void ADC_Routine();

/**
 * @brief  Get resoultion bits of the ADC
 * @param  adcHandle @ref ADC_HandleTypeDef
 * @return The adc bit-width resolution (12bit,10bit...) 
 */
uint8_t ADC_get_resolution_bits(ADC_HandleTypeDef *adcHandle);

/**
 * @brief  Get the ADC total number of voltage levels (range of resolution)
 * @param  adcHandle @ref ADC_HandleTypeDef
 * @return ADC voltage levels (2^resolution_bits) 
 */
uint32_t ADC_get_tot_voltage_levels(ADC_HandleTypeDef *adcHandle);

/**
 * @brief  Get the value in mV expressed by the output of value from the ADC
 * @param  adcHandle @ref ADC_HandleTypeDef
 * @param  value_from_adc output value from the adc
 * @return Value in mV expressed by the adc
 */
float ADC_get_value_mV(ADC_HandleTypeDef *adcHandle, uint32_t value_from_adc);

/**
 * @brief Start a reading of all ADC values in DMA mode.
 *        This functions is a wrapper HAL_ADC_Start_DMA(...)it, starts the ADC
 *        of all sensors in DMA mode 
 * @note  The necessity of this function was due to confine the array of ADC
 *        values into the ADC module. Proper getters will be used to acces the
 *        values retirived from the adc
 *         Since we did not configure the ADC in circular mode we need to restart
 *        the conversions each time Thus this function must be recalled in the
 *        code if we want new values (by SFW timer or inside the interrupt of a
 *        HW timer)
 */
void ADC_start_DMA_readings();

/**
 * @brief Battery voltage feedback value getter (mean value over )
 * @return Battery voltage feedback value
 */
uint16_t ADC_get_batt_fb_raw();

/**
 * @brief Set both muxes (feedback_mux and hall_mux) address in order to read 
 * their values with the ADC
 * 
 * @param address address of the mux to be set  
 */
void ADC_set_mux_address(uint8_t address);

/**
 * @brief current transducer value getter (the value is the average over @ref HO_50S_SP33_SAMPLES_FOR_AVERAGE samples)
 * @return current transducer value
 */
uint16_t ADC_get_HO_50S_SP33_1106_sensor_val();

/**
 * @brief Battery Temperature sensor #1 value getter
 * @return Battery Temperature sensor #1 value
 */
uint16_t ADC_get_t_batt1_val();

/**
 * @brief Battery Temperature sensor #2 value getter
 * @return Battery Temperature sensor #2 value
 */
uint16_t ADC_get_t_batt2_val();

/**
 * @brief DCDC 12v Temperature sensor value getter
 * @return DCDC 12v Temperature sensor value
 */
uint16_t ADC_get_t_dcdc12_val();

/**
 * @brief DCDC 24v Temperature sensor value getter
 * @return DCDC 24v Temperature sensor value
 */
uint16_t ADC_get_t_dcdc24_val();

/**
 * @brief Battery voltage feedback value getter (mean value over )
 * @return Battery voltage feedback value
 */
uint16_t ADC_get_batt_fb_raw();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
