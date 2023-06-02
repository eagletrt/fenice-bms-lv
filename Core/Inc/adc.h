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

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

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
    HALL_OCD0 = 0,
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
    bool hall_calibration;                  //flag for hall sensor calibration, true if it needs to be done
    bool is_value_stored;                   //check if value is stored in the buffer
    uint8_t mux_hall_index_external;        // long as N_SAMPLES
    uint8_t mux_hall_index_internal;        //  long as MUX
    uint8_t mux_fb_index_external;          //  long as N_SAMPLES
    uint8_t mux_fb_index_internal;          //  long as MUX
    uint8_t as_computer_fb_index_external;  //  long as N_SAMPLES
    uint8_t relay_out_index_external;       //  long as N_SAMPLES
    uint8_t lvms_out_index_external;        //  long as N_SAMPLES
    uint8_t batt_out_index_external;        //  long as N_SAMPLES
    uint8_t mux_address_index;              //  long as MAX_MUX_LEN
} ADC_status_flags_t;

typedef struct {
    uint16_t mux_hall;
    uint16_t mux_fb;
    uint16_t adcs_as_computer_fb;
    uint16_t adcs_relay_out;
    uint16_t adcs_lvms_out;
    uint16_t adcs_batt_out;
} ADC2_Channels_t;

typedef struct {
    float HALL_OCD0;
    float S_HALL0;
    float HALL_OCD1;
    float S_HALL1;
    float HALL_OCD2;
    float S_HALL2;
    float LVAC_TEMP0;
    float LVAC_TEMP1;
} MUX_HALL_CONVERTED;

typedef struct {
    float SD_END;
    float BSPD_FB;
    float IMD_FB;
    float LVMS_FB;
    float RES_FB;
    float TSMS_FB;
    float LV_ENCL_1_FB;
    float LV_ENCL_2_FB;
    float HV_ENCL_1_FB;
    float HV_ENCL_2_FB;
    float BACK_PLATE_FB;
    float HVD_FB;
    float AMS_FB;
    float ASMS_FB;
    float INTERLOCK_IMD_FB;
    float SD_START;
} MUX_FB_CONVERTED;

typedef struct {
    MUX_FB_CONVERTED mux_fb;
    MUX_HALL_CONVERTED mux_hall;
    float as_computer_fb;
    float relay_out;
    float lvms_out;
    float batt_out;
} ADC_converted;

#define MAX_MUX_LEN                  16
#define N_ADC_SAMPLES                200
#define N_ADC_SAMPLES_MUX_HALL       N_ADC_SAMPLES
#define N_ADC_SAMPLES_MUX_FB         N_ADC_SAMPLES
#define N_ADC_SAMPLES_AS_COMPUTER_FB N_ADC_SAMPLES
#define N_ADC_SAMPLES_RELAY_OUT      N_ADC_SAMPLES
#define N_ADC_SAMPLES_LVMS_OUT       N_ADC_SAMPLES
#define N_ADC_SAMPLES_BATT_OUT       N_ADC_SAMPLES

#define N_ADC_CALIBRATION_SAMPLES  500
#define N_ADC_CALIBRATION_CHANNELS 2

typedef struct {
    MUX_HALL_VALUES adcs_raw_hall[N_ADC_SAMPLES_MUX_HALL];      // Multiplexed values
    MUX_FB_VALUES adcs_raw_fb[N_ADC_SAMPLES_MUX_FB];            // Multiplexed values
    uint16_t adcs_raw_as_computer_fb[N_ADC_SAMPLES_RELAY_OUT];  // Direct ADC
    uint16_t adcs_raw_relay_out[N_ADC_SAMPLES_RELAY_OUT];       // Direct ADC
    uint16_t adcs_raw_lvms_out[N_ADC_SAMPLES_LVMS_OUT];         // Direct ADC
    uint16_t adcs_raw_batt_out[N_ADC_SAMPLES_BATT_OUT];         //Direct ADC
} ADC2_Sampled_Signals_t;

#define ADC2_CHANNELS_LEN (sizeof(ADC2_Channels_t) / sizeof(uint16_t))
extern ADC2_Channels_t adc2_channels;
extern ADC2_Sampled_Signals_t adc2_sampled_signals;
extern ADC_status_flags_t adc_status_flags;
extern ADC_converted adcs_converted_values;

extern ADC_ChannelConfTypeDef UserAdcConfig;
/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief Start the dedicated timer for ADC2 measurements
 * 
 */
void ADC_start_ADC2_readings();

/**
* @brief Calibrate the vref used to convert raw adc values to mV
*/
void ADC_Vref_Calibration();

/**
* @brief Get sensor voltage at current 0 to have a vref for current measurement
*/
void ADC_hall_sensor_calibration();

/**
 * @brief Init mux address pins to 0x0
 * 
 */
void ADC_init_mux();

/**
 * @brief Init adcs status flags
*/
void ADC_init_status_flags();

/**
 * @brief Get the mux address by port name object
 * @example get_mux_address_by_port_name(HALL_OCD)
 * @param portname Use MUX_HALL_INPUTS_ADDRESSES or MUX_FB_INPUTS_ADDRESSES
 * @return uint8_t A valid address between 0 and 15, or 255 if error
 */
uint8_t ADC_get_mux_address_by_port_name(uint8_t portname);

/**
 * @brief Set the mux address object
 * @example ADC_set_mux_address(0x0A)
 * @param address A valid address between 0 and 15
 */
void ADC_set_mux_address(uint8_t address);

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
 * @brief  Get the value in mV expressed by the output of value from the ADC based on the vref(VDDA)
 * @param  adcHandle @ref ADC_HandleTypeDef
 * @param  value_from_adc output value from the adc
 * @return Value in mV expressed by the adc
 */
float ADC_get_calibrated_mV(ADC_HandleTypeDef *adcHandle, uint32_t value_from_adc);

/**
 * @brief Converts relay out raw adc value in mV value
*/
void relay_out_conversion();

/**
 * @brief Converts lvms out raw adc value in mV value
*/
void lvms_out_conversion();

/**
 * @brief Converts as computer fb out raw adc value in mV value
*/
void as_computer_fb_conversion();

/**
 * @brief Converts mux hall raw adc values in mV values
 * @note All the output of mux hall that are called S_HALL are converted in mA values
*/
void mux_hall_conversion();

/**
 * @brief Converts mux fb raw adc values in mV values
*/
void mux_fb_conversion();

/**
 * @brief Converts batt out raw adc value in mV value
*/
void batt_out_conversion();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

