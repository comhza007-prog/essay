#include "BMS_task.h"
float Vrefdata=0.0f;
float Batvalt=-1.0f;
float MIN_VLT=23.0f;
volatile uint32_t g_exp5_bms_errors=0,g_exp5_bms_valid=0;
volatile uint32_t g_exp5_bms_last_ms=0;
void BMS_task(void const *argument) {
    uint32_t ref_count=0,bat_count=0,ok;
    (void)argument;
    for(;;) {
        ok=1;
        if(HAL_ADC_Start(&hadc1)!=HAL_OK) ok=0;
        if(ok && HAL_ADC_PollForConversion(&hadc1,1U)==HAL_OK) ref_count=HAL_ADC_GetValue(&hadc1);
        else ok=0;
        if(HAL_ADC_Stop(&hadc1)!=HAL_OK) ok=0;
        if(HAL_ADC_Start(&hadc3)!=HAL_OK) ok=0;
        if(ok && HAL_ADC_PollForConversion(&hadc3,1U)==HAL_OK) bat_count=HAL_ADC_GetValue(&hadc3);
        else ok=0;
        if(HAL_ADC_Stop(&hadc3)!=HAL_OK) ok=0;
        /* HAL_OK is zero: do NOT loop on !HAL_ADC_PollForConversion(). */
        taskENTER_CRITICAL();
        if(ok && ref_count>0U) {
            Vrefdata=1.21f/(float)ref_count;
            Batvalt=(float)bat_count*Vrefdata*10.0909090909091f;
            g_exp5_bms_valid=1;
            g_exp5_bms_last_ms=HAL_GetTick();
        } else {
            Batvalt=-1.0f;g_exp5_bms_valid=0;g_exp5_bms_errors++;
        }
        taskEXIT_CRITICAL();
        osDelay(10U);
    }
}



