/**
 * @file can_watchdog.c
 * @author Corraini Dimitri (dimitri.corraini@studenti.unitn.it)
 * @brief 
 * @version 0.1
 * @date 2022-07-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "can_watchdog.h"

#include "../can-lib/lib/primary/primary_network.h"
#include "../can-lib/lib/primary/primary_watchdog.h"
#include "inverters.h"

primary_watchdog watchdog = {0};
uint32_t ids_to_watch[]   = {
    PRIMARY_TS_STATUS_FRAME_ID,
};

#define WATCHLIST_SIZE (sizeof(ids_to_watch) / sizeof(ids_to_watch[0]))

uint32_t timestamps[WATCHLIST_SIZE];

void wdg_init() {
    for (size_t i = 0; i < WATCHLIST_SIZE; i++) {
        CANLIB_BITSET_ARRAY(watchdog.activated, primary_watchdog_index_from_id(ids_to_watch[i]));
    }
}

void wdg_update_and_check_timestamps() {
    /* Update timestamps */
    for (size_t i = 0; i < WATCHLIST_SIZE; i++) {
        primary_watchdog_reset(&watchdog, ids_to_watch[i], timestamps[i]);
    }

    primary_watchdog_timeout(&watchdog, HAL_GetTick());

    /* Check timings */
    for (size_t i = 0; i < WATCHLIST_SIZE; i++) {
        bool timed_out = CANLIB_BITTEST_ARRAY(watchdog.timeout, primary_watchdog_index_from_id(ids_to_watch[i]));

        if (!timed_out)
            continue;

        switch (ids_to_watch[i]) {
            case PRIMARY_TS_STATUS_FRAME_ID:
                car_inverters.discharge_pin = 0;
                break;
            default:
                break;
        }
    }
}

void wdg_update_message_timestamp(uint32_t id) {
    for (size_t i = 0; i < WATCHLIST_SIZE; i++) {
        if (primary_watchdog_index_from_id(ids_to_watch[i]) == id) {
            timestamps[i] = HAL_GetTick();
            return;
        }
    }
}