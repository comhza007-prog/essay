#ifndef E5_MOTION_H
#define E5_MOTION_H
#include <stdint.h>
#include <float.h>
#include "experiment5_hw_profile.h"
#include "experiment3_config.h"
#include "experiment5_formal_config.h"
#define E5_MOTION_BUILD E5_SELECTED_BUILD
#define E5_MOTION_LIMIT 1200
#define E5_ARM_MS 8000U
#define E5_RUN_MS 20000U
/* Qualification only: unloaded, 2 mm, constraint-aware BLNN enabled.
 * Matched HW6 gains, trajectory, limits and guards; not formal data. */
static int e5_motion_finite(float x) { return x==x && x<=FLT_MAX && x>=-FLT_MAX; }
static uint32_t e5_motion_reference(uint32_t ms,float ref[3]) {
    float u,s,ds,dds,sign;
    ref[0]=0;ref[1]=0;ref[2]=0;
    if(ms<4000U) return 1U;
    if(ms>=20000U) return 6U;
    if(ms>=14000U) return 5U;
    if(ms>=8000U && ms<10000U) {ref[0]=.002f;return 3U;}
    sign=ms<8000U?1.0f:-1.0f;
    u=(float)(ms-(ms<8000U?4000U:10000U))/4000.0f;
    s=u*u*u*(10.0f+u*(-15.0f+6.0f*u));
    ds=30.0f*u*u*(1.0f-u)*(1.0f-u);
    dds=60.0f*u*(1.0f-u)*(1.0f-2.0f*u);
    ref[0]=.002f*(sign>0?s:1.0f-s);
    ref[1]=sign*.002f*ds/4.0f;
    ref[2]=sign*.002f*dds/16.0f;
    return sign>0?2U:4U;
}
static int e5_motion_quantize(float current,int16_t *command) {
    *command=0;
    if(!e5_motion_finite(current) || current>E5_MOTION_LIMIT || current<-E5_MOTION_LIMIT) return 0;
    *command=(int16_t)current;
    return 1;
}
/* Hardware velocity measurement: motor-side rpm, right-axis sign reversal.
 * Position still uses unwrapped encoder counts; no position differencing. */
static void e5_motion_velocity(int16_t left_rpm,int16_t right_rpm,float qv[2]) {
    const float metres_per_rpm=EXP3_METERS_PER_MOTOR_COUNT*EXP3_ENCODER_CPR/60.0f;
    float vl=(float)left_rpm*metres_per_rpm;
    float vr=-(float)right_rpm*metres_per_rpm;
    qv[0]=.5f*(vl+vr);qv[1]=(vr-vl)/.60f;
}
/* Strict comparison, symmetric signed boundaries, no abs(int16) overflow.
 * Hard fault handling in motor_task still overrides any braking to zero. */
static uint32_t e5_motion_rpm_fault(int16_t rpm) {
    return rpm>E5_HW_HARD_RPM || rpm<-E5_HW_HARD_RPM;
}
/* Preserve the existing 10-30 rpm drive taper. Above 30 motor rpm, enforce
 * torque opposing rotation with 10 counts/rpm, capped at 200 counts.
 * Keep any stronger controller braking unchanged; NEVER enlarge limits.
 * Hard >E5_HW_HARD_RPM fault in the task overrides this guard to zero.
 * This is electrical braking, not a mechanical holding brake.
 * Caller must integrate actual integer command into its AW residual. */
static uint32_t e5_motion_speed_guard(int16_t rpm,int16_t *command) {
    int32_t speed=(int32_t)rpm;
    int32_t old=(int32_t)*command;
    int32_t braking,aligned;
    if(speed<0) speed=-speed;
    if(speed<=E5_HW_SOFT_RPM) {
        if(speed>E5_HW_TAPER_RPM && ((rpm>0 && old>0)||(rpm<0 && old<0))) {
            *command=(int16_t)(old*(E5_HW_SOFT_RPM-speed)/(E5_HW_SOFT_RPM-E5_HW_TAPER_RPM));
            return 1U;
        }
        return 0U;
    }
    braking=E5_HW_BRAKE_COUNTS_PER_RPM*(speed-E5_HW_SOFT_RPM);
    if(braking>E5_HW_BRAKE_LIMIT) braking=E5_HW_BRAKE_LIMIT;
    aligned=rpm>0?old:-old;
    if(aligned>-braking) {
        *command=(int16_t)(rpm>0?-braking:braking);
        return 1U;
    }
    return 0U;
}
#endif
