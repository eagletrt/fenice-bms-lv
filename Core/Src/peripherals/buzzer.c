/**
 * @file      buzzer.c
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      2021-05-28
 * @updated
 * @ingroup
 * @prefix    BZZR
 * 
 * @brief     Driver for the buzzer 
 * 
 */
/* Includes ------------------------------------------------------------------*/
#include "buzzer.h"

#include "inttypes.h"
#include "usart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static COMM_StatusTypeDef prepare_peak_db_resp_freq();
static inline COMM_StatusTypeDef chck_powered();

/* Exported functions --------------------------------------------------------*/
COMM_StatusTypeDef BZZR_play_pulses(uint32_t pulse_period, uint8_t numb_of_pulses) {
    // volatile bool val = FDBK_DCDC_12V_get_state();
    // if (chck_powered() != COMM_OK) {
    //     printl("Trying to ring an unpowered buzzer", ERR_HEADER);
    //     return COMM_ERROR;
    // }
    // if (prepare_peak_db_resp_freq() != COMM_OK) {
    //     printl("Something wrong with timer", ERR_HEADER);
    //     return COMM_ERROR;
    // }

    printl("Buzzer Start pulses", NORM_HEADER);
    //BZZR_HTIM.Instance->CCR1 = 500;
    for (int i = 0; i < numb_of_pulses; i++) {
        HAL_TIM_PWM_Start(&BZZR_HTIM, BZZR_PWM_TIM_CHNL);
        HAL_Delay(pulse_period);
        HAL_TIM_PWM_Stop(&BZZR_HTIM, BZZR_PWM_TIM_CHNL);
        HAL_Delay(pulse_period);
    }
    printl("Buzzer end pulses", NORM_HEADER);
    return COMM_OK;
}

/* Private functions ---------------------------------------------------------*/
/**
 * @brief    Checks if the buzzer is powered. 
 * 
 * @return   COMM status:
 *           -SUCCESS: COMM_OK, the buzzer is powered
 *           -ERROR: COMM_ERROR, the buzzer is powered
 */
static inline COMM_StatusTypeDef chck_powered() {
    return LV_MASTER_RELAY_get_state() && FDBK_DCDC_12V_get_state() ? COMM_OK : COMM_ERROR;
}

/**
 * @brief     Prepares the buzzer to play a frequency with peak decibel response
 *            NOTE: for this pariticular buzzer the frequency is C_PEAK_FREQ_HZ
 *            NOTE: the output frequency will not be exact due to rounding errors on ARR
 * 
 * @return    COMM Status:
 *            -SUCCESS: COMM_OK, the frequency was set correctly
 *            -ERROR: COMM_ERROR, something went wrong, i.e. timer is not ready
 */
static COMM_StatusTypeDef prepare_peak_db_resp_freq() {
    // Check if the timer is already in use
    if (BZZR_HTIM.State != HAL_TIM_STATE_READY) {
        return COMM_ERROR;
    }
    // Assume that the timer is cloked from apb bus
    uint32_t input_clk = TIM_GetInternalClkFreq(&BZZR_HTIM);

    // Compute a usefull prescaler value
    // If the clk is the internal couter it will be alway over 1MHz
    uint32_t prescaler = (uint32_t)(input_clk / 1000000U);
    __HAL_TIM_SET_PRESCALER(&BZZR_HTIM, prescaler - 1U);

    // Set Auto Reload Register to C_PEK_REQ_HZ.
    uint32_t arr = (uint32_t)((input_clk / prescaler) / C_PEAK_FREQ_HZ);
    __HAL_TIM_SET_AUTORELOAD(&BZZR_HTIM, arr - 1U);

    // Set the duty cycle of the SPWM
    __HAL_TIM_SET_COMPARE(&BZZR_HTIM, BZZR_PWM_TIM_CHNL, (uint32_t)(arr / 2) - 1U);
    return COMM_OK;
}
