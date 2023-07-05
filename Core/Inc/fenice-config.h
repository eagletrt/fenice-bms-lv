#ifndef FENICE_CONFIG_H
#define FENICE_CONFIG_H

#define NDEBUG_FULL

// if main debug is active than every dbug must be activeated independetly
#ifndef NDEBUG_FULL
#define NDEBUG_LTC
#define NDEBUG_CAN_SEND
#define NDEBUG_VALUES
#define NDEBUG_SENSOR_CURRENT
#endif

#define NDEBUG_CAN_SEND
#define NDEBUG_VALUES
#define NDEBUG_SENSOR_CURRENT
//#define NDEBUG_LTC

/**
 * Maximum can payload. for CAN 2.0A is 8 bytes
 */
#define CAN_MAX_PAYLOAD_LENGTH 8

#define MAX_LV_CELLS_COUNT 12
#define LV_CELLS_COUNT     6
#define NTC_COUNT          12
/**
 * BMS LV COSTRAINTS
 */

/**
 * @brief LTC6811 needs a different hardware configuration if it used with fewer than 12 cells
 * In a six cells configuration, without using this flag, the first six cells are seen in the position [0-2] and [6-8]
 * This flag allows to see the first six cells in the position [0-5]
 * 
 */
#define LTC_SIX_CELLS_HW_FIX

//#define DEAD_CELLS_OFFSET \ 1  //If one or more cells are not working anymore this offset could help to exclude those cells
/** BASED ON LIPO STANDARD */
#define VOLT_MAX_ALLOWED_VOLTAGE 4.2f
#define VOLT_MIN_ALLOWED_VOLTAGE 3.3f
#define VOLT_WARNING_LEVEL       3.5f

/** Total number of attempts to read initial voltage needed to decide whether to close the relay */
#define VOLT_MAX_ATTEMPTS 5

#define MIN_POWER_ON_VOLTAGE  VOLT_MIN_ALLOWED_VOLTAGE *(float)(LV_CELLS_COUNT)   // 19.8 V
#define MAX_POWER_ON_VOLTAGE  VOLT_MAX_ALLOWED_VOLTAGE *(float)(LV_CELLS_COUNT);  // 25,2 V
#define TOTAL_CAN_PHERIPERALS 2

#define MAX_CELLS_ALLOWED_TEMP 60.0f
#define MIN_CELLS_ALLOWED_TEMP 0.0f

#define MAX_DCDC12_ALLOWED_TEMP 70.0f
#define MIN_DCDC12_ALLOWED_TEMP 0.0f

#define MAX_DCDC24_ALLOWED_TEMP 70.0f
#define MIN_DCDC24_ALLOWED_TEMP 0.0f

#define MAX_CURRENT_ALLOWED_mA 30000.0f
// OCD (Over Current Detection) is pulled up at 3V3,
// When OCD is detected the pin goes to ground
#define MIN_OCD_VALUE_TO_DETECT_OVERCURRENT_mV 500.0f
//#define NON_CRITICAL_SAFETY_CHECKS_BYPASS
//#define SKIP_TEMP_READ
// Minimum number of temperature sensors with correct value for which the battery system can work properly
#define COUNT_MINIMUM_WORKING_NTCS 5
#endif