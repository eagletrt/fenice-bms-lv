/**
 * @file      fan.h
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      2021-05-11
 * @updated
 * @ingroup
 * @prefix    FAN
 * 
 * @brief     Header file for all Fan peripherals
 * 
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _TEMPLATE_H_
#define _TEMPLATE_H_

/* Includes ------------------------------------------------------------------*/
#include "../Lib/micro-libs/pid/pid.h"
#include "common.h"
#include "tim.h"
/* Exported defines ----------------------------------------------------------*/
/** @defgroup FAN_Instance 
 *  @{
 */
#define FAN_INSTANCE_BMS_HV  (0x00000000U)
#define FAN_INSTANCE_BMS_LV  (0x00000001U)
#define FAN_INSTANCE_COOLING (0x00000002U)
/**
 *  @}
 */

/* Exported types ------------------------------------------------------------*/
typedef struct {
    const uint32_t instance; /*!< FAN instance. Can by a value of @ref FAN_Instance. */
    //TIM_HandleTypeDef *const htim;  /*!< Timer handle pointer, generates PWM. */
    //PID_HandleTypeDef *const hpid;  /*!< Pid handle pointer, controls PWM duty cycle. */
    const uint32_t tim_pwm_channel; /*!< PWM Timer channel. 
                                        Can be a member of @ref TIM_Channel*/

} FAN_HandleTypeDef;

extern FAN_HandleTypeDef fan_bms_hv;
extern FAN_HandleTypeDef fan_bms_lv;
extern FAN_HandleTypeDef fan_cooling;
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void FAN_Init();
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros -----------------------------------------------------------*/
#endif