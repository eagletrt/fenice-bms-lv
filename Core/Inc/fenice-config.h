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

// TODO: remove this

/* ---------------- ID ------------------- */
#define BMS_LV_ASK_ID    0xFF   // Foo Fighters
#define STEER_ASK_ID     0xAF   // Steering wheel
#define INV_LEFT_ASK_ID  0x181  // Inverter left
#define INV_RIGHT_ASK_ID 0x182  // Inverter right
#define ACC_TEMP_ASK_ID  0xAA   // Accumulator temperatures
#define ECU_ASK_ID       0xF8   // ECU initial check ID

/**
 * Maximum can payload. for CAN 2.0A is 8 bytes
 */
#define CAN_MAX_PAYLOAD_LENGTH 8

/**
 * BMS LV COSTRAINTS
 */

#define MIN_POWER_ON_VOLTAGE 10.5

#endif