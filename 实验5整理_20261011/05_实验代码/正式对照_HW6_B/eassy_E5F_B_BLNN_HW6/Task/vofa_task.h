#ifndef VOFA_TASK_H
#define VOFA_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

#include "experiment5_formal_config.h"
#define VOFA_CHANNEL_COUNT E5_FORMAL_TELEMETRY_CHANNELS

typedef struct
{
    uint8_t message[(VOFA_CHANNEL_COUNT + 1U) * 4U];
    uint8_t temp[4];
    uint8_t tail[4];
} Vofa_t;

typedef union
{
    float float_data;
    uint32_t long_data;
} FloatLongType;

void Vofa_tail_init(void);
void Vofa_send_justfloat(float *send_message);
void vofa_task(void const *argument);

#endif
