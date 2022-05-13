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
#include "usart.h"
#include "volt.h"
#include "mcp23017.h"
#include "i2c.h"
#include "radiator.h"
#include "dac_pump.h"
#include "tim.h"
#include "dac.h"
#include "adc.h"
#include "notes_buzzer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define N_COMMANDS 9

cli_command_func_t _cli_volts;
cli_command_func_t _cli_radiator;
cli_command_func_t _cli_pumps;
cli_command_func_t _cli_temps;
cli_command_func_t _cli_feedbacks;
cli_command_func_t _cli_dmesg;
cli_command_func_t _cli_reset;
cli_command_func_t _cli_help;
cli_command_func_t _cli_wizard;

cli_command_func_t *commands[N_COMMANDS] = {
    & _cli_volts,
    & _cli_radiator,
    & _cli_pumps,
    & _cli_temps,
    & _cli_feedbacks,
    & _cli_dmesg,
    & _cli_reset,
    & _cli_wizard,
    & _cli_help
};

char *command_names[N_COMMANDS] = 
{"volts", "radiator", "pumps", "temps", "feedbacks", "dmesg", "reset", "wizard","?"};

char *volt_status_name[VOLT_ENUM_SIZE] = {
    [VOLT_OK] = "VOLT OK",
    [VOLT_UNDER_VOLTAGE] = "VOLT UNDER VOLTAGE", 
    [VOLT_OVER_VOLTAGE] = "VOLT OVER VOLTAGE", 
    [VOLT_ERR] = "VOLT ERROR", 
};

cli_t cli_bms_lv;
bool dmesg_ena = true; // debug message enabled

void cli_bms_lv_init(){
    cli_bms_lv.uart = &LOG_HUART;
    cli_bms_lv.cmds.functions = commands;
    cli_bms_lv.cmds.names = command_names;
    cli_bms_lv.cmds.count = N_COMMANDS;

    char init[94];
    sprintf(init, "\r\n\n***** Fenice BMS LV *****\r\n***** Less is more! *****\r\n");
    strcat(init, cli_ps);

    cli_init(&cli_bms_lv);
    cli_print(&cli_bms_lv, init, strlen(init));
}

void cli_bms_debug(char *msg, size_t length){
    if(dmesg_ena){
        char out[300] = {'\0'};
        float tick = (float) HAL_GetTick() / 1000;
        // add prefix
        sprintf(out, "[%.2f]", tick);
        strcat(out, msg);
        length += strlen(out);
        //add suffix
        sprintf(out+ length, "\r\n");
        length+=2;
        cli_print(&cli_bms_lv, out, length);
    }
}


void _cli_volts(uint16_t argc, char **argv, char *out){
    out[0] = '\0';
    volt_read_and_store(out);
     if(strcmp((argv[1]), "status") == 0){
        sprintf(out+strlen(out), "Voltage status: %s \r\n", volt_status_name[volt_status]);
    }
}
void _cli_radiator(uint16_t argc, char **argv, char *out){
    if(argc < 2){
        sprintf(out, "Invalid arguments \r\nUsage: radiator {L/R/B/info} {duty_cycle_value/off}\r\n");
    }else{
        if(atof(argv[2]) <= 1.0){
            // atof("off") == 0 
            if(strcmp(argv[1], "L") == 0){
                set_radiator_dt(&RAD_L_HTIM, RAD_L_PWM_TIM_CHNL, atof(argv[2]));
                sprintf(out, "Left radiator set at %0.2f%%\r\n", atof(argv[2])*100);
            }else if(strcmp(argv[1], "R") == 0){
                set_radiator_dt(&RAD_R_HTIM, RAD_R_PWM_TIM_CHNL, atof(argv[2]));
                sprintf(out, "Right radiator set at %0.2f%%\r\n", atof(argv[2])*100);
            }else if(strcmp(argv[1], "info") == 0){
            sprintf(out, "Radiators status:\r\n\t Left Duty Cycle: %.2f\r\n\t Right Duty Cycle: %.2f\
            \r\n\t Is right on: %d\r\n\t Is left on: %d\r\n",
            radiator_handle.duty_cycle_l, radiator_handle.duty_cycle_r, radiator_handle.right_is_on, radiator_handle.left_is_on);
            }else{
                set_radiator_dt(&RAD_R_HTIM, RAD_R_PWM_TIM_CHNL, atof(argv[2]));
                set_radiator_dt(&RAD_L_HTIM, RAD_L_PWM_TIM_CHNL, atof(argv[2]));
                sprintf(out, "Both radiator set at %0.2f%%\r\n", atof(argv[2])*100);
            }
        }else{
            sprintf(out, "Invalid duty cycle \r\n");
        }
        
    }
}
void _cli_pumps(uint16_t argc, char **argv, char *out){
    // pumps P 0.5 -> MAX_OPAMP_OUT*0.5
    // pumps V 3.3 -> 3.3
    // p: proportional to MAX_OPAMP_OUT
    // v: volt
    if(argc < 2){
        sprintf(out, "Invalid arguments \r\nUsage: pumps {p/V} {max_out_percentage/voltage_level}\r\n");
    }else{
        if(strcmp(argv[1], "p") == 0){
            if(atof(argv[2]) <= 1.0){
                dac_pump_store_and_set_proportional_on_both_channels(&hdac_pump, atof(argv[2]), atof(argv[2]));
                sprintf(out, "Pumps set at %.2fV\r\n", MAX_OPAMP_OUT*atof(argv[2]));
            }else{
                sprintf(out, "Error: Proportional value must be <= 1.0 \r\n");
            }
        }else if(strcmp(argv[1], "v") == 0){
            if(atof(argv[2]) <= MAX_OPAMP_OUT){
                dac_pump_store_and_set_value_on_both_channels(&hdac_pump, atof(argv[2]),atof(argv[2]));
                sprintf(out, "Pumps set at %.2fV\r\n", atof(argv[2]));
            }else{
                sprintf(out, "Error: Max voltage level exceeded\r\n");
            }
        }else{
            sprintf(out, "Second arguement it's invalid \r\n");
        }   
    }
}
void _cli_temps(uint16_t argc, char **argv, char *out){
    sprintf(
                out,
                "ADC sensors:\r\n\tCurrent: %i [A]\r\n\tBatt1: %i [°C]\r\n\tBatt2: %i [°C]\r\n\tDCDC 12V: %i[°C]\r\n\tDCDC 24V: %i[°C]\r\n",
                ADC_get_i_sensor_val(),
                ADC_get_t_batt1_val(),
                ADC_get_t_batt2_val(),
                ADC_get_t_dcdc12_val(),
                ADC_get_t_dcdc24_val());
            ADC_start_dma_readings();
}
void _cli_feedbacks(uint16_t argc, char **argv, char *out){
    out[0] = '\0';
    mcp23017_read_both(&hmcp, &hi2c3);
    mcp23017_print_gpioA(&hmcp, out);
    mcp23017_print_gpioB(&hmcp, out);
}
void _cli_dmesg(uint16_t argc, char **argv, char *out){
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

void _cli_wizard(uint16_t argc, char **argv, char *out){
    sprintf(out, "Dj Khaled: another one!\r\n");
    BUZ_sborati(&BZZR_HTIM);
}