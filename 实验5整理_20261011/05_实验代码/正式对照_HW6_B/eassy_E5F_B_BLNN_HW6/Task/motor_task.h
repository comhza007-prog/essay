#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "experiment5_formal_config.h"
#define E5_TELEMETRY_CHANNELS E5_FORMAL_TELEMETRY_CHANNELS
extern volatile uint8_t g_exp5_abort;
extern volatile uint32_t g_exp5_uart_errors;
void motor_task(void const *argument);
void e5_telemetry_snapshot(float channels[E5_TELEMETRY_CHANNELS]);
#endif
