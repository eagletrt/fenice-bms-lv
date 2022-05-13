/* GOD BLESS ALEX SARTORI*/
#include "main.h"
#include "pwm.h"
#include "math.h"
#include "tim.h"
#include "notes_buzzer.h"

#include <string.h>

void fans_init() {
    pwm_set_period(&BZZR_HTIM, PWM_FANS_STANDARD_PERIOD);
    fans_set_speed(0.15);
    pwm_start_channel(&BZZR_HTIM, BZZR_PWM_TIM_CHNL);
}
void fans_set_speed(float power) {
    if (power > 1 || power < 0)
        return;
    pwm_set_duty_cicle(&BZZR_HTIM, BZZR_PWM_TIM_CHNL, 1 - power);
}

typedef enum {
    A0 = 0U,
    AS0,
    B0,
    C0,
    CS0,
    D0,
    DS0,
    E0,
    F0,
    FS0,
    G0,
    GS0,
    A1,
    AS1,
    B1,
    C1,
    CS1,
    D1,
    DS1,
    E1,
    F1,
    FS1,
    G1,
    GS1,
    A2,
    AS2,
    B2,
    C2,
    CS2,
    D2,
    DS2,
    E2,
    F2,
    FS2,
    G2,
    GS2,
    Pause,
    End
} NoteNameEnum;

uint16_t notes_freqs[] = {440,  466,  494,  523,  554,  587,  622,  659,  698,  740,  784,  831,
                          880,  932,  988,  1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661,
                          1760, 1865, 1976, 2093, 2217, 2349, 2439, 2637, 2794, 2960, 3136, 3322};

typedef struct {
    NoteNameEnum note;
    uint8_t beats;
} NoteTypeDef;

NoteTypeDef gandalf[] = {{FS1, 4},   {Pause, 4}, {FS1, 2},   {FS1, 1},   {FS1, 1},   {E1, 1},    {FS1, 1},   {Pause, 2},
    {FS1, 4},   {Pause, 4}, {FS1, 2},   {FS1, 1},   {FS1, 1},   {E1, 1},    {FS1, 1},   {FS1, 2},
    {FS1, 2},   {Pause, 2}, {A2, 4},    {FS1, 2},   {Pause, 2}, {E1, 4},
    {D1, 2},    {Pause, 2}, {B1, 2},    {B1, 2},    {CS1, 2},   {D1, 2},    {B1, 2},    {FS1, 2},
    {FS1, 2},   {Pause, 4}, {FS1, 2},   {FS1, 1},   {FS1, 1},   {E1, 1},    {FS1, 1},   {Pause, 2}, {FS1, 2},

    {FS1, 2},   {Pause, 4}, {FS1, 2},   {FS1, 1},   {FS1, 1},   {E1, 1},    {FS1, 1},   {FS1, 4},
    {Pause, 2}, {A2, 4},    {FS1, 2},   {Pause, 2}, {E1, 4},    {D1, 3},
    {Pause, 2}, {B1, 2},    {B1, 2},    {CS1, 2},   {D1, 2},    {B1, 2},    {FS1, 4},
    {Pause, 4}, {FS1, 2},   {FS1, 1},   {FS1, 1},   {E1, 1},    {FS1, 1},   {Pause, 2}, {FS1, 4},
    {Pause, 4}, {FS1, 2},   {FS1, 1},   {FS1, 1},   {E1, 1},    {FS1, 1},   {FS1, 2},   {Pause, 2},
    {A2, 4},    {FS1, 2},   {Pause, 2}, {E1, 4},    {D1, 2},    {Pause, 2},

    {B1, 2},    {B1, 2},    {CS1, 2},   {D1, 2},    {B1, 2},    {Pause, 2}, {Pause, 4},

    {End, 0}};

uint16_t BPM = 600;

void _play_note(NoteTypeDef note, TIM_HandleTypeDef *htim) {
    uint16_t beat_duration_ms = 60 * 1000 / BPM;

    if (note.note != Pause) {
        pwm_generate_wave(htim, BZZR_PWM_TIM_CHNL, PWM_SINE_WAVE, &notes_freqs[note.note], 1);
        HAL_Delay(beat_duration_ms * note.beats);
        pwm_stop_channel(htim, BZZR_PWM_TIM_CHNL);
    } else {
        HAL_Delay(beat_duration_ms * note.beats);
    }

    HAL_Delay(20);
}

void BUZ_sborati(TIM_HandleTypeDef *htim) {
    NoteTypeDef *n = gandalf;

    while (n->note != End)
        _play_note(*n++, htim);

    pwm_stop_channel(htim, BZZR_PWM_TIM_CHNL);
}



