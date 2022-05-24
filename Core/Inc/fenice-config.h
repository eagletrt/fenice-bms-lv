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
#define NDEBUG_LTC

/**
 * Maximum can payload. for CAN 2.0A is 8 bytes
 */
#define CAN_MAX_PAYLOAD_LENGTH 8

/**
 * BMS LV COSTRAINTS
 */

/** BASED ON LIPO STANDARD */
#define VOLT_MAX_ALLOWED_VOLTAGE 4.2
#define VOLT_MIN_ALLOWED_VOLTAGE 3.3

/** Total number of attempts to read initial voltage needed to decide whether to close the relay */
#define VOLT_MAX_ATTEMPTS 5

#define MIN_POWER_ON_VOLTAGE  12.0
#define MAX_POWER_ON_VOLTAGE  VOLT_MAX_ALLOWED_VOLTAGE * 4;
#define TOTAL_CAN_PHERIPERALS 2

#endif