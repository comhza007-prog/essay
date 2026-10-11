#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "stdint.h"

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;
    int32_t total_ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    float rotor_revolutions;
    float output_revolutions;
    volatile uint32_t rx_count;
    volatile uint32_t last_rx_ms;
    uint32_t diag_last_cycles;
    float diag_encoder_rpm; /* per-accepted-frame encoder average; diagnostics only */
} motor_measure_t;

extern motor_measure_t motor_3508[2];
extern void CAN_cmd_3508(int16_t M1, int16_t M2);

#endif
