#include "motor.h"

extern motor_measure_t motor_3508[2];

float chassis_ang_init[2];
float chassis_total[2];

pid_type_def SPEED_PID[2];
pid_type_def ANGLE_PID[2];

static float chassis_left_spdpid_param[3] = {
    CHASSIS_LEFT_SPD_KP,
    CHASSIS_LEFT_SPD_KI,
    CHASSIS_LEFT_SPD_KD
};
static float chassis_left_angpid_param[3] = {
    CHASSIS_LEFT_ANG_KP,
    CHASSIS_LEFT_ANG_KI,
    CHASSIS_LEFT_ANG_KD
};
static float chassis_right_spdpid_param[3] = {
    CHASSIS_RIGHT_SPD_KP,
    CHASSIS_RIGHT_SPD_KI,
    CHASSIS_RIGHT_SPD_KD
};
static float chassis_right_angpid_param[3] = {
    CHASSIS_RIGHT_ANG_KP,
    CHASSIS_RIGHT_ANG_KI,
    CHASSIS_RIGHT_ANG_KD
};

static float clampf(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void motor_init(void)
{
    chassis_ang_init[0] = (float)motor_3508[0].total_ecd;
    chassis_ang_init[1] = (float)motor_3508[1].total_ecd;

    PID_init(&SPEED_PID[0], chassis_left_spdpid_param,
             CHASSIS_LEFT_SPD_MAX_OUT, CHASSIS_LEFT_SPD_MAX_IOUT);
    PID_init(&ANGLE_PID[0], chassis_left_angpid_param,
             CHASSIS_LEFT_ANG_MAX_OUT, CHASSIS_LEFT_ANG_MAX_IOUT);

    PID_init(&SPEED_PID[1], chassis_right_spdpid_param,
             CHASSIS_RIGHT_SPD_MAX_OUT, CHASSIS_RIGHT_SPD_MAX_IOUT);
    PID_init(&ANGLE_PID[1], chassis_right_angpid_param,
             CHASSIS_RIGHT_ANG_MAX_OUT, CHASSIS_RIGHT_ANG_MAX_IOUT);
}

void motor_track_reference_m(float reference_m)
{
    float reference_counts = reference_m / EXP3_METERS_PER_MOTOR_COUNT;
    float left_speed_set;
    float right_speed_set;

    /* Positive gantry travel: left encoder increases, right encoder decreases. */
    chassis_total[0] = chassis_ang_init[0] + reference_counts;
    chassis_total[1] = chassis_ang_init[1] - reference_counts;

    PID_calc(&ANGLE_PID[0], (float)motor_3508[0].total_ecd, chassis_total[0]);
    PID_calc(&ANGLE_PID[1], (float)motor_3508[1].total_ecd, chassis_total[1]);

    left_speed_set = clampf(ANGLE_PID[0].out, EXP3_SPEED_LIMIT_RPM);
    right_speed_set = clampf(ANGLE_PID[1].out, EXP3_SPEED_LIMIT_RPM);

    PID_calc(&SPEED_PID[0], (float)motor_3508[0].speed_rpm, left_speed_set);
    PID_calc(&SPEED_PID[1], (float)motor_3508[1].speed_rpm, right_speed_set);
}

void motor_stop_and_clear(void)
{
    PID_clear(&SPEED_PID[0]);
    PID_clear(&SPEED_PID[1]);
    PID_clear(&ANGLE_PID[0]);
    PID_clear(&ANGLE_PID[1]);
}
