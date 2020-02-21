#include "current_sensor.h"

#define LEN_ARRAY 100
uint32_t array_val[LEN_ARRAY];

void push_into_array(uint32_t val){
    for(int i = LEN_ARRAY - 1; i > 0; i--){
        array_val[i] = array_val[i - 1];
    }
    array_val[0] = val;
}

uint32_t getCurrent(uint32_t adcVAL){
    uint32_t ADC_voltages;
    ADC_voltages = (uint32_t)((adcVAL * (Vcc*1024/RESOLUTION))/1024);
    if(ADC_voltages >= REFERENCE_VOLTAGE) ADC_voltages = ADC_voltages - REFERENCE_VOLTAGE;
    else ADC_voltages = REFERENCE_VOLTAGE - ADC_voltages;
    return ADC_voltages =  (uint32_t)(100 * ADC_voltages);
    return (uint32_t)((MAX_mA/OFFSET) * ADC_voltages);
}

uint32_t mean_current(){
    uint64_t val = 0;
    for(int i = 0 ; i < LEN_ARRAY; i++){
        val+=array_val[i];
    }
    return val/LEN_ARRAY;
}