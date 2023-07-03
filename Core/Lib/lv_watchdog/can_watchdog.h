/**
 * @file can_watchdog.h
 * @author Corraini Dimitri (dimitri.corraini@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-07-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef CAN_WATCHDOG_H
#define CAN_WATCHDOG_H

void wdg_init();
void wdg_update_and_check_timestamps();
void wdg_update_message_timestamp(uint32_t id);

#endif  //CAN_WATCHDOG_H