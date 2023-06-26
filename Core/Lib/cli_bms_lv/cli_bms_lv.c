/**
 * @file cli_bms_lv.c
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-05-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "cli_bms_lv.h"

#include "../current_transducer/current_transducer.h"
#include "../thermocouple/thermocouple.h"
#include "adc.h"  // TODO: remove this eventually
#include "can.h"
#include "can_comm.h"
#include "dac.h"
#include "dac_pump.h"
#include "error.h"
#include "health_signals.h"
#include "i2c.h"
#include "mcp23017.h"
#include "measurements.h"
#include "monitor_int.h"
#include "notes_buzzer.h"
#include "radiator.h"
#include "tim.h"
#include "usart.h"

/* CAN LIB */
#include "../can-lib/lib/primary/primary_network.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define N_COMMANDS 16

cli_command_func_t _cli_volts;
cli_command_func_t _cli_radiator;
cli_command_func_t _cli_pumps;
cli_command_func_t _cli_temps;
cli_command_func_t _cli_adc;
cli_command_func_t _cli_feedbacks;
cli_command_func_t _cli_dmesg;
cli_command_func_t _cli_reset;
cli_command_func_t _cli_wizard;
cli_command_func_t _cli_help;
cli_command_func_t _cli_can_send;
cli_command_func_t _cli_inv;
cli_command_func_t _cli_cooling;
cli_command_func_t _cli_errors;
cli_command_func_t _cli_health_signals;
cli_command_func_t _cli_signal_injection;

cli_command_func_t *commands[N_COMMANDS] = {
    &_cli_volts,
    &_cli_radiator,
    &_cli_pumps,
    &_cli_temps,
    &_cli_adc,
    &_cli_feedbacks,
    &_cli_dmesg,
    &_cli_reset,
    &_cli_wizard,
    &_cli_can_send,
    &_cli_inv,
    &_cli_cooling,
    &_cli_errors,
    &_cli_health_signals,
    &_cli_signal_injection,
    &_cli_help};

char *command_names[N_COMMANDS] = {
    "volts",
    "radiator",
    "pumps",
    "temps",
    "adc",
    "feedbacks",
    "dmesg",
    "reset",
    "wizard",
    "can",
    "inv",
    "cooling",
    "errors",
    "health",
    "inject",
    "?"};

char *volt_status_name[VOLT_ENUM_SIZE] = {
    [VOLT_OK]            = "VOLT OK",
    [VOLT_UNDER_VOLTAGE] = "VOLT UNDER VOLTAGE",
    [VOLT_OVER_VOLTAGE]  = "VOLT OVER VOLTAGE",
    [VOLT_ERR]           = "VOLT ERROR",
};

cli_t cli_bms_lv;
bool dmesg_ena = true;  // debug message enabled

void cli_bms_lv_init() {
    cli_bms_lv.uart           = &LOG_HUART;
    cli_bms_lv.cmds.functions = commands;
    cli_bms_lv.cmds.names     = command_names;
    cli_bms_lv.cmds.count     = N_COMMANDS;

    char init[94];
    sprintf(init, "\r\n\n***** Fenice BMS LV *****\r\n***** Less is more! *****\r\n");
    strcat(init, cli_ps);

    cli_init(&cli_bms_lv);
    cli_print(&cli_bms_lv, init, strlen(init));
}

void cli_bms_debug(char *msg, size_t length) {
    if (dmesg_ena) {
        char out[300] = {'\0'};
        float tick    = (float)HAL_GetTick() / 1000;
        // add prefix
        sprintf(out, "[%.2f]", tick);
        strcat(out, msg);
        length += strlen(out);
        //add suffix
        sprintf(out + length, "\r\n");
        length += 2;
        cli_print(&cli_bms_lv, out, length);
    }
}

void _cli_health_signals(uint16_t argc, char **argv, char *out) {
    out[0] = '\0';
    sprintf(
        out,
        "Health signals bitset\r\n"
        "Health code: %u\r\n"
        "LVMS_OUT: %d (1 if V_LVMS - V_RELAY < %.3f)\r\n"
        "RELAY_OUT: %d (1 if V_RELAY - V_BATTERY < %.3f)\r\n"
        "BATTERY_VOLTAGE_OUT: %d (1 if > %.3f)\r\n"
        "CHARGER_CURRENT_OUT: %d (1 if > %.3f)\r\n"
        "BATTERY_CURRENT_OUT: %d (1 if > %.3f)\r\n"
        "SIGN_BATTERY_CURRENT_OUT: %d (1 if positive)\r\n",
        *(uint8_t *)&hs,
        hs.lvms_out,
        MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV,
        hs.relay_out,
        MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV,
        hs.battery_voltage_out,
        MIN_BATTERY_VOLTAGE_mV,
        hs.charger_current,
        MIN_BATTERY_CURRENT_THRESHOLD_mA,
        hs.battery_current,
        MIN_CHARGER_CURRENT_THRESHOLD_mA,
        hs.sign_battery_current);
}
void _cli_volts(uint16_t argc, char **argv, char *out) {
    out[0] = '\0';
    monitor_print_volt_cli(out);
    if (strcmp((argv[1]), "status") == 0) {
        sprintf(out + strlen(out), "Voltage status: %s \r\n", volt_status_name[volt_status]);
    }
}

void _cli_signal_injection(uint16_t argc, char **argv, char *out) {
    if (argc < 2) {
        sprintf(
            out, "Invalid arguments\r\nUsage: inject {temp|current} {index} {value}\r\ne.g. inject temp 0 25.0\r\n");
    } else {
        if (strcmp(argv[1], "temp") == 0) {
            if (argc < 4) {
                sprintf(out, "Invalid arguments\r\nUsage: inject temp {index} {value}\r\ne.g. inject temp 0 25.0\r\n");
            } else {
                uint8_t index = atoi(argv[2]);
                float value   = atof(argv[3]);
                if (index < NTC_COUNT) {
                    cell_temps[index] = value;
                    sprintf(out, "Temperature %d set to %0.2f\r\n", index, value);
                    injection_done = 1;
                } else {
                    sprintf(out, "Invalid index\r\n");
                }
            }
        } else if (strcmp(argv[1], "current") == 0) {
            if (argc < 4) {
                sprintf(
                    out,
                    "Invalid arguments\r\nUsage: inject current {index} {value}\r\ne.g. inject current 0 25.0\r\n");
            } else {
                uint8_t index = atoi(argv[2]);
                float value   = atof(argv[3]);
                if (index < 3) {
                    if (index == 0) {
                        adcs_converted_values.mux_hall.S_HALL0 = value;
                    } else if (index == 1) {
                        adcs_converted_values.mux_hall.S_HALL1 = value;
                    } else if (index == 2) {
                        adcs_converted_values.mux_hall.S_HALL2 = value;
                    }
                    sprintf(out, "Current S_HALL_%d set to %0.2f\r\n", index, value);
                    injection_done = 1;
                } else {
                    sprintf(out, "Invalid index\r\n");
                }
            }
        } else {
            sprintf(
                out,
                "Invalid arguments\r\nUsage: inject {temp|current} {index} {value}\r\ne.g. inject temp 0 25.0\r\n");
        }
    }
}

void _cli_radiator(uint16_t argc, char **argv, char *out) {
    if (argc < 2) {
        sprintf(out, "Invalid arguments \r\nUsage: radiator {L/R/B/info} {duty_cycle_value/off}\r\n");
    } else {
        if (atof(argv[2]) <= 1.0) {
            // atof("off") == 0
            if (strcmp(argv[1], "L") == 0) {
                set_radiator_dt(&RAD_L_HTIM, RAD_L_PWM_TIM_CHNL, atof(argv[2]));
                sprintf(out, "Left radiator set at %0.2f%%\r\n", atof(argv[2]) * 100);
            } else if (strcmp(argv[1], "R") == 0) {
                set_radiator_dt(&RAD_R_HTIM, RAD_R_PWM_TIM_CHNL, atof(argv[2]));
                sprintf(out, "Right radiator set at %0.2f%%\r\n", atof(argv[2]) * 100);
            } else if (strcmp(argv[1], "internal") == 0) {
                bms_fan_duty_cycle = atof(argv[2]);
                sprintf(out, "Set bms fan dt: %0.2f%%\r\n", atof(argv[2]) * 100);
            } else if (strcmp(argv[1], "info") == 0) {
                sprintf(
                    out,
                    "Radiators status:\r\n\t Left Duty Cycle: %.2f\r\n\t Right Duty Cycle: %.2f\
            \r\n\t Is right on: %d\r\n\t Is left on: %d\r\n Automatic mode:%s\r\nInternal fan dt: %0.2f%%\r\n",
                    radiator_handle.duty_cycle_l,
                    radiator_handle.duty_cycle_r,
                    radiator_handle.right_is_on,
                    radiator_handle.left_is_on,
                    radiator_handle.automatic_mode ? "true" : "false",
                    bms_fan_duty_cycle * 100);
            } else {
                set_radiator_dt(&RAD_R_HTIM, RAD_R_PWM_TIM_CHNL, atof(argv[2]));
                set_radiator_dt(&RAD_L_HTIM, RAD_L_PWM_TIM_CHNL, atof(argv[2]));
                sprintf(out, "Both radiator set at %0.2f%%\r\n", atof(argv[2]) * 100);
            }
        } else {
            sprintf(out, "Invalid duty cycle \r\n");
        }
    }
}
void _cli_pumps(uint16_t argc, char **argv, char *out) {
    // pumps P 0.5 -> MAX_OPAMP_OUT*0.5
    // pumps V 3.3 -> 3.3
    // p: proportional to MAX_OPAMP_OUT
    // v: volt
    if (argc < 2) {
        sprintf(out, "Invalid arguments \r\nUsage: pumps {info/p/v} {max_out_percentage/voltage_level}\r\n");
    } else {
        if (strcmp(argv[1], "info") == 0) {
            sprintf(
                out,
                "Pumps status\r\n\tPumps L:%.2f [%.2fV] R:%.2f [%.2fV]\r\n Automatic mode:%s\r\n",
                (hdac_pump.last_analog_value_L > 0) ? (float)(hdac_pump.last_analog_value_L) / MAX_DAC_OUT : 0.0,
                dac_pump_analog_to_digital(hdac_pump.last_analog_value_L),
                (hdac_pump.last_analog_value_R > 0) ? (float)(hdac_pump.last_analog_value_R) / MAX_DAC_OUT : 0.0,
                dac_pump_analog_to_digital(hdac_pump.last_analog_value_R),
                hdac_pump.automatic_mode ? "true" : "false");
        } else if (strcmp(argv[1], "p") == 0) {
            if (atof(argv[2]) <= 1.0) {
                dac_pump_store_and_set_proportional_on_both_channels(&hdac_pump, atof(argv[2]), atof(argv[2]));
                sprintf(out, "Pumps set at %.2fV\r\n", MAX_OPAMP_OUT * atof(argv[2]));
            } else {
                sprintf(out, "Error: Proportional value must be <= 1.0 \r\n");
            }
        } else if (strcmp(argv[1], "v") == 0) {
            if (atof(argv[2]) <= MAX_OPAMP_OUT) {
                dac_pump_store_and_set_value_on_both_channels(&hdac_pump, atof(argv[2]), atof(argv[2]));
                sprintf(out, "Pumps set at %.2fV\r\n", atof(argv[2]));
            } else {
                sprintf(out, "Error: Max voltage level exceeded\r\n");
            }
        } else {
            sprintf(out, "Second arguement it's invalid \r\n");
        }
    }
}
void _cli_temps(uint16_t argc, char **argv, char *out) {
    out[0] = '\0';
    monitor_print_temps(out);
}

void _cli_adc(uint16_t argc, char **argv, char *out) {
    if (argc < 2) {
        sprintf(out, "Invalid arguments \r\nUsage: adc {hall/fb/ind}\r\n");
    } else {
        if (strcmp(argv[1], "hall") == 0) {
            sprintf(
                out,
                "Current values\r\nOCD (Over Current Detection) and S (shunt current value)\r\n"
                "Hall ocd 0 (AS_Battery): %.2f mV\r\n"
                "Hall ocd 1 (LV_Battery): %.2f mV\r\n"
                "Hall ocd 2 (Charger): %.2f mV\r\n"
                "S hall 0 (AS_Battery): %.2f mA\r\n"
                "S hall 1 (LV_Battery): %.2f mA\r\n"
                "S hall 2 (Charger): %.2f mA\r\n",
                adcs_converted_values.mux_hall.HALL_OCD0,
                adcs_converted_values.mux_hall.HALL_OCD1,
                adcs_converted_values.mux_hall.HALL_OCD2,
                adcs_converted_values.mux_hall.S_HALL0,
                adcs_converted_values.mux_hall.S_HALL1,
                adcs_converted_values.mux_hall.S_HALL2);
        } else if (strcmp(argv[1], "fb") == 0) {
            sprintf(
                out,
                "SD_END: %.2f mV\r\n"
                "BSPD_FB: %.2f mV\r\n"
                "IMD_FB: %.2f mV\r\n"
                "LVMS_FB: %.2f mV\r\n"
                "RES_FB: %.2f mV\r\n"
                "TSMS_FB: %.2f mV\r\n"
                "LV_ENCL_1_FB: %.2f mV\r\n"
                "LV_ENCL_2_FB: %.2f mV\r\n"
                "HV_ENCL_1_FB: %.2f mV\r\n"
                "HV_ENCL_2_FB: %.2f mV\r\n"
                "BACK_PLATE_FB: %.2f mV\r\n"
                "HVD_FB: %.2f mV\r\n"
                "AMS_FB: %.2f mV\r\n"
                "ASMS_FB: %.2f mV\r\n"
                "INTERLOCK_IMD_FB: %.2f mV\r\n"
                "SD_START: %.2f mV\r\n",
                adcs_converted_values.mux_fb.SD_END,
                adcs_converted_values.mux_fb.BSPD_FB,
                adcs_converted_values.mux_fb.IMD_FB,
                adcs_converted_values.mux_fb.LVMS_FB,
                adcs_converted_values.mux_fb.RES_FB,
                adcs_converted_values.mux_fb.TSMS_FB,
                adcs_converted_values.mux_fb.LV_ENCL_1_FB,
                adcs_converted_values.mux_fb.LV_ENCL_2_FB,
                adcs_converted_values.mux_fb.HV_ENCL_1_FB,
                adcs_converted_values.mux_fb.HV_ENCL_2_FB,
                adcs_converted_values.mux_fb.BACK_PLATE_FB,
                adcs_converted_values.mux_fb.HVD_FB,
                adcs_converted_values.mux_fb.AMS_FB,
                adcs_converted_values.mux_fb.ASMS_FB,
                adcs_converted_values.mux_fb.INTERLOCK_IMD_FB,
                adcs_converted_values.mux_fb.SD_START);
        } else if (strcmp(argv[1], "ind") == 0) {
            sprintf(
                out,
                "As computer fb: %.2f mV\r\n"
                "Relay out: %.2f mV\r\n"
                "Lvms out: %.2f mV\r\n"
                "Batt out: %.2f mV\r\n",
                adcs_converted_values.as_computer_fb,
                adcs_converted_values.relay_out,
                adcs_converted_values.lvms_out,
                adcs_converted_values.batt_out);
        }
    }
}

void _cli_feedbacks(uint16_t argc, char **argv, char *out) {
    out[0] = '\0';
    mcp23017_read_both(&hmcp);
    mcp23017_print_gpioA(&hmcp, out);
    mcp23017_print_gpioB(&hmcp, out);
}
void _cli_dmesg(uint16_t argc, char **argv, char *out) {
    sprintf(out, "This is a debug message \r\n");
}

void _cli_reset(uint16_t argc, char **argv, char *out) {
    HAL_NVIC_SystemReset();
}

void _cli_help(uint16_t argc, char **argv, char *out) {
    sprintf(out, "command list:\r\n");
    for (uint8_t i = 0; i < N_COMMANDS; i++) {
        sprintf(out + strlen(out), "- %s\r\n", cli_bms_lv.cmds.names[i]);
    }
}

void _cli_wizard(uint16_t argc, char **argv, char *out) {
    sprintf(out, "Dj Khaled: another one!\r\n");
    BUZ_sborati(&BZZR_HTIM);
}

void _cli_can_send(uint16_t argc, char **argv, char *out) {
    if (argc < 2) {
        sprintf(out, "Invalid arguments \r\nUsage: can {volts/cooling/current/temp}\r\n");
    } else {
        if (strcmp(argv[1], "volts") == 0) {
            can_primary_send(PRIMARY_LV_CELLS_VOLTAGE_INDEX, 0);
            can_primary_send(PRIMARY_LV_CELLS_VOLTAGE_INDEX, 1);
        } else if (strcmp(argv[1], "cooling") == 0) {
            can_primary_send(PRIMARY_COOLING_STATUS_INDEX, 0);
        } else if (strcmp(argv[1], "total") == 0) {
            can_primary_send(PRIMARY_LV_TOTAL_VOLTAGE_INDEX, 0);
        } else if (strcmp(argv[1], "current") == 0) {
            can_primary_send(PRIMARY_LV_CURRENTS_INDEX, 0);
        } else if (strcmp(argv[1], "temp") == 0) {
            can_primary_send(PRIMARY_LV_CELLS_TEMP_INDEX, 0);
        }
    }
}

void _cli_errors(uint16_t argc, char **argv, char *out) {
    size_t count = 0;
    error_t errors[ERROR_NUM_ERRORS];
    error_dump(errors, &count);

    uint32_t now = HAL_GetTick();
    sprintf(out, "total %u\r\n", count);
    for (uint16_t i = 0; i < count; i++) {
        sprintf(
            out + strlen(out),
            "\r\nid..........%i (%s)\r\n"
            "timestamp...T+%lu (%lums ago)\r\n"
            "offset......%u\r\n"
            "state.......%s\r\n",
            errors[i].id,
            error_names[errors[i].id],
            errors[i].timestamp,
            now - errors[i].timestamp,
            errors[i].offset,
            errors[i].state == STATE_WARNING ? "warning" : "fatal");
    }
}

void _cli_inv(uint16_t argc, char **argv, char *out) {
    if (strcmp(argv[1], "on") == 0) {
        // HAL_GPIO_WritePin(INV_RFE_GPIO_Port, INV_RFE_Pin, GPIO_PIN_SET);
        // HAL_Delay(1500);  // works
        // HAL_GPIO_WritePin(INV_FRG_GPIO_Port, INV_FRG_Pin, GPIO_PIN_SET);
    } else if (strcmp(argv[1], "off") == 0) {
        // HAL_GPIO_WritePin(INV_RFE_GPIO_Port, INV_RFE_Pin, GPIO_PIN_RESET);
        // HAL_Delay(1500);  // works
        // HAL_GPIO_WritePin(INV_FRG_GPIO_Port, INV_FRG_Pin, GPIO_PIN_RESET);
    }
    // sprintf(
    //     out,
    //     "[RFE]: %u\r\n[FRG]: %u\r\n",
    //     HAL_GPIO_ReadPin(INV_RFE_GPIO_Port, INV_RFE_Pin),
    //     HAL_GPIO_ReadPin(INV_FRG_GPIO_Port, INV_FRG_Pin));
}

void _cli_cooling(uint16_t argc, char **argv, char *out) {
    sprintf(
        out,
        "Radiators status:\r\n\t Left Duty Cycle: %.2f\r\n\t Right Duty Cycle: %.2f\
            \r\n\t Is right on: %d\r\n\t Is left on: %d\r\n\tRadiator automatic mode:%s\r\nInternal fan dt: %0.2f%%\r\n \
        Pumps status\r\n\tPumps L:%.2f [%.2fV] R:%.2f [%.2fV]\r\n\tPumps automatic mode:%s\r\n",
        radiator_handle.duty_cycle_l,
        radiator_handle.duty_cycle_r,
        radiator_handle.right_is_on,
        radiator_handle.left_is_on,
        radiator_handle.automatic_mode ? "true" : "false",
        bms_fan_duty_cycle * 100,
        (hdac_pump.last_analog_value_L > 0) ? (float)(hdac_pump.last_analog_value_L) / MAX_DAC_OUT : 0.0,
        dac_pump_analog_to_digital(hdac_pump.last_analog_value_L),
        (hdac_pump.last_analog_value_R > 0) ? (float)(hdac_pump.last_analog_value_R) / MAX_DAC_OUT : 0.0,
        dac_pump_analog_to_digital(hdac_pump.last_analog_value_R),
        hdac_pump.automatic_mode ? "true" : "false");
}