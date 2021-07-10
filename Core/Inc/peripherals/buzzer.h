/**
 * @file      buzzer.h
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      2021-05-28
 * @updated
 * @ingroup
 * @prefix    BZZR
 * 
 * @brief     Buzzer driver header files
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BUZZER_H_
#define _BUZZER_H_

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "main.h"
#include "tim.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define C_MAX_REQ_HZ   3000
#define C_PEAK_FREQ_HZ 2000
#define C_MIN_REQ_HZ   1000
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief     Ring the buzzer with regular pulses at the best frequency response.
 * @details   The reproduced melody will be: 
 *            numb_of_pulses * (pulse_period[sound_on]+pulse_period[sound_off]).
 *            NOTE: this is a blocking function
 *            WARNING: long combination of periods+pulses will stall the main process
 * 
 * @param     pulse_period Period of a pulse in milliseconds
 * @param     numb_of_pulses Number of pulses
 * @return    COMM Status:
 *           -SUCCESS: COMM_OK, the pulses were reprodueced
 *           -ERROR: COMM_ERROR, otherwise
 */
COMM_StatusTypeDef BZZR_play_pulses(uint32_t pulse_period, uint8_t numb_of_pulses);

COMM_StatusTypeDef BZZR_epic_wizard_music();
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros -----------------------------------------------------------*/
#endif