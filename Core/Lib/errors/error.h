/**
 * @file		error.h
 * @brief		This file contains the functions to handle errors.
 *
 * @date		May 1, 2019
 * @author		Matteo Bonora [matteo.bonora@studenti.unitn.it]
 * @author		Simone Ruffini[simone.ruffini@tutanota.com]
 */

#ifndef ERROR_H
#define ERROR_H

#include "error-utils.h"

#include <inttypes.h>
#include <stdbool.h>

#define error_toggle_check(condition, error_type, index) \
    if ((condition)) {                                   \
        error_set((error_type), (index));                \
    } else {                                             \
        error_reset((error_type), (index));              \
    }

#define OPEN_WIRE_ERROR_INSTANCES     1
#define CAN_ERROR_INSTANCES           2  //One instance for can primary and one for can secondary
#define SPI_ERROR_INSTANCES           1
#define OVER_CURRENT_ERROR_INSTANCES  3  // 3 is the number of current sensor present on the bms lv
#define RELAY_ERROR_INSTANCES         1
#define BMS_MONITOR_ERROR_INSTANCES   1
#define VOLTAGE_READY_ERROR_INSTANCES 1
#define MCP23017_ERROR_INSTANCES      1
#define RADIATOR_ERROR_INSTANCES      2
#define FAN_ERROR_INSTANCES           1
#define PUMP_ERROR_INSTANCES          2  // First one is referred to left pump while the other one is the right one
#define ADC_ERROR_INSTANCES           1
/**
 * @brief	Error type definitions
 *
 * @details	To add an error type you need to add it to this enum, then you
 * need to create the error reference variable in error_data.h and you need to
 * link the error to the reference in the error_reference array.
 */
typedef enum {
    ERROR_CELL_UNDERVOLTAGE,
    ERROR_CELL_OVERVOLTAGE,
    ERROR_OPEN_WIRE,
    ERROR_CAN,
    ERROR_SPI,
    ERROR_OVER_CURRENT,
    ERROR_CELL_UNDER_TEMPERATURE,
    ERROR_CELL_OVER_TEMPERATURE,
    ERROR_RELAY,
    ERROR_BMS_MONITOR,
    ERROR_VOLTAGES_NOT_READY,
    ERROR_MCP23017,
    ERROR_RADIATOR,
    ERROR_FAN,
    ERROR_PUMP,
    ERROR_ADC_INIT,
    ERROR_ADC_MUX,
    ERROR_NUM_ERRORS

} __attribute__((__packed__)) error_id;

extern const char *error_names[ERROR_NUM_ERRORS];

// Do not use UIN32_MAX because this value is used internally for arithmetic calculations
// having UINT32_MAX causes overflow errors
typedef enum {
    SOFT    = (uint32_t)(UINT32_MAX >> 1),  //@10KHz 2 d + 11 h + 39 min + 8.3648 s
    SHORT   = 500,
    REGULAR = 1000,
    INSTANT = 0
} __attribute__((__packed__)) error_timeout;

/**
 * @brief an error is active when it enters the error list (ERROR_ACTIVE)
 * an error is fatal when it stays active till the error timeout (ERROR_FATAL)
 * an error becomes inactive
 **/
typedef enum { STATE_WARNING, STATE_FATAL } __attribute__((__packed__)) error_state;

/** @brief	Defines an error instance */
typedef struct {
    error_id id;        /* Defines the type of error */
    uint8_t offset;     /* Identifies different instances of a type */
    error_state state;
    uint32_t timestamp; /* Last time the error activated */
} error_t;

extern ERROR_UTILS_HandleTypeDef error_handler;

/**
 * @brief Initializes the error library
*/
void error_init();

/**
 * @brief Activates an error
 * @param type The error type
 * @param offset The offset of the instance in error
*/
void error_set(error_id type, uint8_t offset);

/**
 * @brief Deactivates an error
 * @param type The error type
 * @param offset The offset of the instance in error
*/
void error_reset(error_id type, uint8_t offset);

/**
 * @brief Global callback for the error library
 * @param type The error type
 * @param offset The offset of the instance in error
*/
void bms_error_callback(size_t error_index, size_t instance_index);

/**
 * @brief Insert in an array all the current errors presents on the bms lv
 * @param errors Array in which the errors are copied
 * @param error_count Number of errors that are currently presents
 * @return An array of running errors
*/
void error_dump(error_t errors[], size_t *error_count);

#endif /* ERROR_H_ */
