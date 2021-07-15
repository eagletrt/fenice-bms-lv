/**
 * @file      fan.c
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      2021-05-11
 * @updated
 * @ingroup   
 * @prefix    FAN
 * 
 * @brief     FAN peripheral driver.
 * 
 */
/* Includes ------------------------------------------------------------------*/
#include "peripherals/fan.h"

//#include "../Lib/micro-libs/pid/pid.h"
#include "common.h"
#include "tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BMS_HV_TIM_PWM_CHNL  TIM_CHANNEL_1
#define BMS_LV_TIM_PWM_CHNL  TIM_CHANNEL_1
#define COOLING_TIM_PWM_CHNL TIM_CHANNEL_1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void _fan_init_sequence(FAN_HandleTypeDef *hfan);
/* Exported variables --------------------------------------------------------*/
#if 0
static PID_HandleTypeDef hpid1;
static PID_HandleTypeDef hpid2;
static PID_HandleTypeDef hpid3;

FAN_HandleTypeDef hfan_bms_hv =
    {.instance = FAN_INSTANCE_BMS_HV, .htim = &htim2, .hpid = &hpid1, .tim_pwm_channel = BMS_HV_TIM_PWM_CHNL};
FAN_HandleTypeDef hfan_bms_lv =
    {.instance = FAN_INSTANCE_BMS_LV, .htim = &htim3, .hpid = &hpid2, .tim_pwm_channel = BMS_LV_TIM_PWM_CHNL};
FAN_HandleTypeDef hfan_cooling =
    {.instance = FAN_INSTANCE_COOLING, .htim = &htim4, .hpid = &hpid3, .tim_pwm_channel = COOLING_TIM_PWM_CHNL};
/* Exported functions --------------------------------------------------------*/
void FAN_Init() {
    // Make fan_bms_hv init sequence (ramp to 30% to 0)
    // This is done in polling so expect to wait some time
    _fan_init_sequence(&hfan_bms_hv);
    _fan_init_sequence(&hfan_bms_lv);
    _fan_init_sequence(&hfan_cooling);
}

static void _fan_init_sequence(FAN_HandleTypeDef *hfan) {
#define INIT_SEQUENCE_STEPS       (10U)
#define INIT_SEQUENCE_DURATION_MS (2000U)

#ifndef NDEBUG
    switch (hfan->instance) {
        case FAN_INSTANCE_BMS_HV:
            M_LOG_STRING("+[BMS-LV FAN_BMS_HV]\t Test Start");
            break;
        case FAN_INSTANCE_BMS_LV:
            M_LOG_STRING("+[BMS-LV FAN_BMS_LV]\t Test Start");
            break;
        case FAN_INSTANCE_COOLING:
            M_LOG_STRING("+[BMS-LV FAN_COOLING]\t Test Start");
            break;
        default:
            M_LOG_STRING("+[BMS-LV ERR]\t FAN-ERR: WRONG FAN INSTANCE");
            Error_Handler();
            break;
    }
#endif
    // The sequence ramps to 30% of the fan speed from 0.
    // It does it with INIT_SEQUENCE_STEPS in INIT_SEQUENCE_DURATION_MS time
    uint32_t tim_pulse_step = (uint32_t)(0.3 * __HAL_TIM_GET_AUTORELOAD(hfan->htim) / INIT_SEQUENCE_STEPS);
    for (int i = 0; i < INIT_SEQUENCE_STEPS; i++) {
        // Set pulse of PWM for this step,
        __HAL_TIM_SET_COMPARE(hfan->htim, hfan->tim_pwm_channel, i * tim_pulse_step);
        HAL_TIM_PWM_Start(hfan->htim, hfan->tim_pwm_channel);
        HAL_Delay((uint32_t)(1.0 * INIT_SEQUENCE_DURATION_MS / INIT_SEQUENCE_STEPS));
    }

    __HAL_TIM_SET_COMPARE(hfan->htim, hfan->tim_pwm_channel, 0);

#ifndef NDEBUG
    switch (hfan->instance) {
        case FAN_INSTANCE_BMS_HV:
            M_LOG_STRING("+[BMS-LV FAN_BMS_HV]\t end test");
            break;
        case FAN_INSTANCE_BMS_LV:
            M_LOG_STRING("+[BMS-LV FAN_BMS_LV]\t end test");
            break;
        case FAN_INSTANCE_COOLING:
            M_LOG_STRING("+[BMS-LV FAN_COOLING]\t end test");
            break;
        default:
            Error_Handler();
            break;
    }
#endif
#undef INIT_SEQUENCE_DURATION_MS
#undef INIT_SEQUENCE_STEPS
}
#endif