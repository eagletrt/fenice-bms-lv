/**
 * @file cli_bms_lv.h
 * @author Tommaso Canova (tommaso.canova@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-05-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef CLI_BMS_LV_H__
#define CLI_BMS_LV_H__

#include "../micro-libs/cli/cli.h"
#include "fenice-config.h"

extern cli_t cli_bms_lv;

void cli_bms_lv_init();
/**
 * @brief Print messages in the cli if dmesg is enabled
 * 
 * @param msg 
 * @param length 
 */
void cli_bms_debug(char *msg, size_t length);

#endif