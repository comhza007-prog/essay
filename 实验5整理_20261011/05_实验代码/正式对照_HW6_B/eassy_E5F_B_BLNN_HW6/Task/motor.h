#ifndef MOTOR_H
#define MOTOR_H

#include "CAN_receive.h"
#include "pid.h"
#include "experiment3_config.h"
#include <math.h>

#define CHASSIS_LEFT_SPD_KP             5.3f
#define CHASSIS_LEFT_SPD_KI             0.02f
#define CHASSIS_LEFT_SPD_KD             2.5f
#define CHASSIS_LEFT_SPD_MAX_OUT        EXP3_REQUEST_LIMIT
#define CHASSIS_LEFT_SPD_MAX_IOUT       EXP3_CURRENT_I_LIMIT

#define CHASSIS_LEFT_ANG_KP             0.2f
#define CHASSIS_LEFT_ANG_KI             0.0f
#define CHASSIS_LEFT_ANG_KD             0.5f
#define CHASSIS_LEFT_ANG_MAX_OUT        EXP3_SPEED_LIMIT_RPM
#define CHASSIS_LEFT_ANG_MAX_IOUT       0.0f

#define CHASSIS_RIGHT_SPD_KP            5.3f
#define CHASSIS_RIGHT_SPD_KI            0.02f
#define CHASSIS_RIGHT_SPD_KD            2.5f
#define CHASSIS_RIGHT_SPD_MAX_OUT       EXP3_REQUEST_LIMIT
#define CHASSIS_RIGHT_SPD_MAX_IOUT      EXP3_CURRENT_I_LIMIT

#define CHASSIS_RIGHT_ANG_KP            0.2f
#define CHASSIS_RIGHT_ANG_KI            0.0f
#define CHASSIS_RIGHT_ANG_KD            0.5f
#define CHASSIS_RIGHT_ANG_MAX_OUT       EXP3_SPEED_LIMIT_RPM
#define CHASSIS_RIGHT_ANG_MAX_IOUT      0.0f

extern void motor_init(void);
extern void motor_track_reference_m(float reference_m);
extern void motor_stop_and_clear(void);

extern float chassis_total[2];
extern float chassis_ang_init[2];
extern pid_type_def SPEED_PID[2];
extern pid_type_def ANGLE_PID[2];

#endif
