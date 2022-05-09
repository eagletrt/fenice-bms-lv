/**
 * @file dac_pump.c
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-05-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "dac_pump.h"

DAC_Pump_Handle hdac_pump;
/**
 * @brief             Initialize the handle with given values
 * 
 * @param hdp         DAC_Pump_Handle
 * @param pump_l_volt future left voltage
 * @param pump_r_volt future right voltage
 */
void dac_pump_handle_init(DAC_Pump_Handle *hdp, float pump_l_volt, float pump_r_volt) {
    hdp->last_analog_value_L = pump_l_volt;
    hdp->last_analog_value_R = pump_r_volt;
    hdp->is_L_on             = 0;
    hdp->is_R_on             = 0;
}

/**
 * @brief          Set last stored analog value for both pumps using DAC pheripheral based on latest analagos vals
 * 
 * @param hdp      DAC_Pump_Handle
 * @return uint8_t error: if -1 comething went wrong
 */
uint8_t dac_pump_set_value_on_both_channels(DAC_Pump_Handle *hdp) {
    uint8_t error = 0;

    HAL_DAC_Start(&PUMP_DAC, PUMP_L_CHNL);
    HAL_DAC_Start(&PUMP_DAC, PUMP_R_CHNL);

    if (HAL_DAC_SetValue(&PUMP_DAC, PUMP_L_CHNL, DAC_ALIGN_12B_R, hdp->last_analog_value_L) != HAL_OK) {
        error = -1;
    } else {
        hdp->is_L_on = 1;
    }
    if (HAL_DAC_SetValue(&PUMP_DAC, PUMP_R_CHNL, DAC_ALIGN_12B_R, hdp->last_analog_value_R) != HAL_OK) {
        error = -1;
    } else {
        hdp->is_R_on = 1;
    }

    return error;
}

/**
 * @brief 
 * 
 * @param hdp         DAC_Pump_Handle
 * @param pump_l_volt Digital output voltage for left pump
 * @param pump_r_volt Digital output voltage for right pump
 * @return uint8_t    error: if -1 comething went wrong
 */
uint8_t dac_pump_store_and_set_value_on_both_channels(DAC_Pump_Handle *hdp, float pump_l_volt, float pump_r_volt) {
    uint8_t error = 0;
    uint32_t analog_val_l, analog_val_r;
    analog_val_l = dac_pump_digital_volt_to_analog(pump_l_volt);
    analog_val_r = dac_pump_digital_volt_to_analog(pump_r_volt);

    HAL_DAC_Start(&PUMP_DAC, PUMP_L_CHNL);
    HAL_DAC_Start(&PUMP_DAC, PUMP_R_CHNL);

    if (HAL_DAC_SetValue(&PUMP_DAC, PUMP_L_CHNL, DAC_ALIGN_12B_R, analog_val_l) != HAL_OK) {
        error = -1;
    } else {
        hdp->is_L_on             = 1;
        hdp->last_analog_value_L = analog_val_l;
    }
    if (HAL_DAC_SetValue(&PUMP_DAC, PUMP_R_CHNL, DAC_ALIGN_12B_R, analog_val_r) != HAL_OK) {
        error = -1;
    } else {
        hdp->is_R_on             = 1;
        hdp->last_analog_value_R = analog_val_r;
    }

    return error;
}
/**
 * @brief Stores and write a given voltage value to specific channel using DAC
 * 
 * @param hdp DAC_Pump_Handle
 * @param channel DAC Channel such as PUMP_L_CHNL or PUMP_R_CHNL
 * @param digital_value float digital value such as 2.5 V
 * @return uint8_t error: if -1 comething went wrong
 */
uint8_t dac_pump_store_and_set_value_on_single_channel(DAC_Pump_Handle *hdp, uint32_t channel, float digital_value) {
    uint8_t error         = 0;
    uint32_t analog_value = dac_pump_digital_volt_to_analog(digital_value);
    HAL_DAC_Start(&PUMP_DAC, channel);
    if (HAL_DAC_SetValue(&PUMP_DAC, channel, DAC_ALIGN_12B_R, analog_value) != HAL_OK) {
        error = -1;
    } else {
        if (channel == PUMP_L_CHNL) {
            hdp->last_analog_value_L = analog_value;
            hdp->is_L_on             = 1;
        } else if (channel == PUMP_R_CHNL) {
            hdp->last_analog_value_R = analog_value;
            hdp->is_R_on             = 1;
        }
    }
    return error;
}

/**
 * @brief Break single pump based on given channel
 * 
 * @param hdp DAC_Pump_Handle
 * @param channel DAC Channel such as PUMP_L_CHNL or PUMP_R_CHNL
 * @return uint8_t error: if -1 comething went wrong
 */
uint8_t dac_pump_break_single(DAC_Pump_Handle *hdp, uint32_t channel) {
    uint8_t error         = 0;
    uint32_t analog_value = dac_pump_digital_volt_to_analog(0.0);
    HAL_DAC_Start(&PUMP_DAC, channel);
    if (HAL_DAC_SetValue(&PUMP_DAC, channel, DAC_ALIGN_12B_R, analog_value) != HAL_OK) {
        error = -1;
    } else {
        if (channel == PUMP_L_CHNL) {
            hdp->last_analog_value_L = analog_value;
            hdp->is_L_on             = 0;
        } else if (channel == PUMP_R_CHNL) {
            hdp->last_analog_value_R = analog_value;
            hdp->is_R_on             = 0;
        }
    }
    HAL_DAC_Stop(&PUMP_DAC, channel);
    return error;
}
/**
 * @brief Break both pumps
 * 
 * @param hdp DAC_Pump_Handle
 * @return uint8_t error: if -1 comething went wrong
 */
uint8_t dac_pump_break_both(DAC_Pump_Handle *hdp) {
    uint8_t error         = 0;
    uint32_t analog_value = dac_pump_digital_volt_to_analog(0.0);
    HAL_DAC_Start(&PUMP_DAC, PUMP_R_CHNL);
    if (HAL_DAC_SetValue(&PUMP_DAC, PUMP_R_CHNL, DAC_ALIGN_12B_R, analog_value) != HAL_OK) {
        error = -1;
    }
    if (HAL_DAC_SetValue(&PUMP_DAC, PUMP_L_CHNL, DAC_ALIGN_12B_R, analog_value) != HAL_OK) {
        error = -1;
    } else {
        hdp->last_analog_value_L = 0;
        hdp->is_L_on             = 0;
        hdp->last_analog_value_R = 0;
        hdp->is_R_on             = 0;
    }
    HAL_DAC_Stop(&PUMP_DAC, PUMP_R_CHNL);
    HAL_DAC_Stop(&PUMP_DAC, PUMP_L_CHNL);
    return error;
}

/**
 * @brief Stores and write a given excursion of MAX_OPAMP_OUT value to specific channel
 * 
 * @param hdp     DAC_Pump_Handle
 * @param channel DAC Channel such as PUMP_L_CHNL or PUMP_R_CHNL
 * @param proportional Percentage of MAX_OPAMP_OUT eg: 0.5 will output 0.5*MAX_OPAMP_OUT
 * @return uint8_t error: if -1 comething went wrong
 */
uint8_t dac_pump_store_and_set_proportional_on_single_channel(
    DAC_Pump_Handle *hdp,
    uint32_t channel,
    float proportional) {
    uint8_t error         = 0;
    uint32_t analog_value = dac_pump_proportional_to_analog(proportional);
    HAL_DAC_Start(&PUMP_DAC, channel);
    if (HAL_DAC_SetValue(&PUMP_DAC, channel, DAC_ALIGN_12B_R, analog_value) != HAL_OK) {
        error = -1;
    } else {
        if (channel == PUMP_L_CHNL) {
            hdp->last_analog_value_L = analog_value;
            hdp->is_L_on             = 1;
        } else if (channel == PUMP_R_CHNL) {
            hdp->last_analog_value_R = analog_value;
            hdp->is_R_on             = 1;
        }
    }
    return error;
}

uint8_t dac_pump_store_and_set_proportional_on_both_channels(
    DAC_Pump_Handle *hdp,
    float proportional_l,
    float proportional_r) {
    uint8_t error = 0;
    uint32_t analog_val_l, analog_val_r;
    analog_val_l = dac_pump_proportional_to_analog(proportional_l);
    analog_val_r = dac_pump_proportional_to_analog(proportional_r);

    HAL_DAC_Start(&PUMP_DAC, PUMP_L_CHNL);
    HAL_DAC_Start(&PUMP_DAC, PUMP_R_CHNL);

    if (HAL_DAC_SetValue(&PUMP_DAC, PUMP_L_CHNL, DAC_ALIGN_12B_R, analog_val_l) != HAL_OK) {
        error = -1;
    } else {
        hdp->is_L_on             = 1;
        hdp->last_analog_value_L = analog_val_l;
    }
    if (HAL_DAC_SetValue(&PUMP_DAC, PUMP_R_CHNL, DAC_ALIGN_12B_R, analog_val_r) != HAL_OK) {
        error = -1;
    } else {
        hdp->is_R_on             = 1;
        hdp->last_analog_value_R = analog_val_r;
    }

    return error;
}

/**
 * @brief Sample test to check whether the DAC it's working properly
 * 
 * @param hdp 
 */
void dac_pump_sample_test(DAC_Pump_Handle *hdp) {
    dac_pump_handle_init(hdp, 0.0, 0.0);

    if (dac_pump_store_and_set_value_on_single_channel(&hdac_pump, PUMP_L_CHNL, 1.5) == -1) {
        printl("PUMP ERROR", NO_HEADER);
    }  // 2.25V
    if (dac_pump_store_and_set_value_on_single_channel(&hdac_pump, PUMP_L_CHNL, 2) == -1) {
        printl("PUMP ERROR", NO_HEADER);
    }  // 3
    if (dac_pump_store_and_set_value_on_single_channel(&hdac_pump, PUMP_L_CHNL, 3.3) == -1) {
        printl("PUMP ERROR", NO_HEADER);
    }

    if (dac_pump_store_and_set_value_on_single_channel(&hdac_pump, PUMP_R_CHNL, 1.5) == -1) {
        printl("PUMP ERROR", NO_HEADER);
    };  // 2.25V
    if (dac_pump_store_and_set_value_on_single_channel(&hdac_pump, PUMP_R_CHNL, 2) == -1) {
        printl("PUMP ERROR", NO_HEADER);
    }  // 3
    if (dac_pump_store_and_set_value_on_single_channel(&hdac_pump, PUMP_R_CHNL, 3.3) == -1) {
        printl("PUMP ERROR", NO_HEADER);
    }

    dac_pump_break_single(&hdac_pump, PUMP_L_CHNL);
    dac_pump_break_single(&hdac_pump, PUMP_R_CHNL);
    dac_pump_store_and_set_value_on_both_channels(&hdac_pump, 1, 1);
    dac_pump_store_and_set_proportional_on_both_channels(&hdac_pump, 0.5, 0.5);
    dac_pump_break_both(&hdac_pump);
}

/**
 * @brief Get DAC voltage level to pilot the pumps, based on a given motor temperature
 * 
 * @param temp Motor temperature
 * @return float Voltage level
 */
float dac_pump_get_voltage(float temp) {
    return (MAX_OPAMP_OUT - MIN_OPAMP_OUT) / (MAX_MOTOR_TEMP - MIN_MOTOR_TEMP) * (temp - MIN_MOTOR_TEMP);
}