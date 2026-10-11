#include "motor_task.h"
#include "blnn_core.h"
#include "CAN_receive.h"
#include "experiment3_config.h"
#include "experiment5_motion.h"
#include "periodic_release.h"
#include "experiment5_fault_trace.h"
#include "experiment5_formal_actuator.h"
#include <math.h>
#include <string.h>
/* HW6 condition B CANDIDATE, BASE qualification ONLY. No load; 2 mm.
 * Provisional gain floors 12/1, no Coulomb or skew stiffness feedforward.
 * Filtered core feedback; raw-rpm active speed envelope.
 * CA-only final actuator increment projection; common HW6 baseline stays.
 * Conflict first contracts feasible NN, then withdraws if necessary.
 * Both exceptions freeze learning; NEVER bypass speed braking.
 * CA guards inside its core; BASE/BLNN guard outside, ONCE per command.
 * BLNN/CA_BLNN start from four nodes/axis and zero weights each boot.
 * Sector/output/learning/growth gates enabled; max 60 nodes/axis.
 * Same common core parameters, trajectory and external guards in all methods.
 * NOT identical to frozen v3: hardware revalidation required before formal data.
 * Fault bits: 1/2 CAN age,4 temperature,8 skew,16 travel,32 abort,
 * 64 numeric,128 CAN TX,256 timing,512 RPM,1024 tracking/direction,
 * 2048 battery,4096 rejected CAN motor frame/read error.
 * Zero command is not a brake or proof of CAN delivery. */
volatile uint8_t g_exp5_abort=0;
volatile uint32_t g_exp5_uart_errors=0;
volatile uint32_t g_exp5_drive_permit=0;
extern volatile uint32_t g_exp5_can_errors;
extern volatile uint32_t g_exp5_can_rejected;
extern volatile uint32_t g_exp5_dma_completed,g_exp5_dma_skipped,g_exp5_uart_last_error;
extern volatile uint32_t g_exp5_bms_errors,g_exp5_bms_valid;
extern volatile uint32_t g_exp5_bms_last_ms;
extern float Batvalt;
static e5_state controller;
static e5_input input;
static e5_output output;
static float published[E5_TELEMETRY_CHANNELS],channels[E5_TELEMETRY_CHANNELS];
static motor_measure_t feedback[2];
static e5_fault_trace fault_trace;
void e5_telemetry_snapshot(float values[E5_TELEMETRY_CHANNELS]) {
    taskENTER_CRITICAL();memcpy(values,published,sizeof(published));
    e5_trace_next(&fault_trace,&values[96]);taskEXIT_CRITICAL();
}
static void publish(void) {
    taskENTER_CRITICAL();memcpy(published,channels,sizeof(published));taskEXIT_CRITICAL();
}
static void read_feedback(void) {
    taskENTER_CRITICAL();memcpy(feedback,motor_3508,sizeof(feedback));taskEXIT_CRITICAL();
}
static uint32_t health(uint32_t now,const float position[2],uint32_t armed) {
    uint32_t f=0,ch;
    for(ch=0;ch<2;ch++) {
        if(feedback[ch].rx_count==0 || now-feedback[ch].last_rx_ms>50U) f|=1U<<ch;
        if(feedback[ch].temperate>=60U) f|=4U;
        if(e5_motion_rpm_fault(feedback[ch].speed_rpm)) f|=512U;
        if(armed && fabsf(position[ch])>.004f) f|=16U;
    }
    if(armed && fabsf(position[0]-position[1])>.0015f) f|=8U;
    if(!g_exp5_bms_valid || now-g_exp5_bms_last_ms>100U ||
       !e5_motion_finite(Batvalt) || Batvalt<23.0f || Batvalt>30.0f) f|=2048U;
    if(g_exp5_abort) f|=32U;
    if(g_exp5_can_errors) f|=128U;
    if(g_exp5_can_rejected) f|=4096U;
    return f;
}
void motor_task(void const *argument) {
    TickType_t last_wake;
    uint32_t boot,start=0,now,elapsed=0,fault=0,phase=0,armed=0,done=0;
    uint32_t last_cycles=0,previous_valid=0,late=0,overruns=0,sat[2]={0,0};
    uint32_t ch,begin,core_begin,core_end,rebase_events=0,loop_count=0;
    uint32_t stack_checked=0,stack_bytes=4096,tracking_ms=0,stationary_ms=0;
    uint32_t speed_guard_count[2]={0,0},speed_guard_mask=0;
    uint32_t first_fault=0,first_fault_ms=0;
    int16_t before_guard[2]={0,0};
    int32_t guard_delta[2]={0,0};
    int32_t rpm_peak[2]={0,0},rpm_abs;
    int32_t origin[2]={0,0};
    int16_t command[2]={0,0};
    int16_t previous_command[2]={0,0};
    e5_trace_record trace_row;
    float position[2]={0,0},raw_q[2]={0,0},measured_velocity[2]={0,0},velocity[2]={0,0},accel[2]={0,0};
    float v_before,period=0,period_peak=0,core_us=0,core_peak=0,whole_us,whole_peak=0;
    const float cycles_per_us=(float)SystemCoreClock/1000000.0f;
    (void)argument;
    e5_trace_reset(&fault_trace);
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0U;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    boot=HAL_GetTick();last_wake=xTaskGetTickCount();
    if(!e5_reset(&controller,E5_SELECTED_METHOD,4)) fault|=64U;
    input.limit[0]=E5_SELECTED_LIMIT_LEFT;input.limit[1]=E5_SELECTED_LIMIT_RIGHT;
    for(;;) {
        begin=DWT->CYCCNT;read_feedback();now=HAL_GetTick();
        trace_row.ms=now;
        for(ch=0;ch<2;ch++) {
            trace_row.rpm[ch]=feedback[ch].speed_rpm;
            trace_row.encoder_rpm[ch]=feedback[ch].diag_encoder_rpm;
            trace_row.previous_command[ch]=previous_command[ch];
        }
        e5_trace_push(&fault_trace,&trace_row);
        command[0]=0;command[1]=0;g_exp5_drive_permit=0;
        before_guard[0]=0;before_guard[1]=0;
        guard_delta[0]=0;guard_delta[1]=0;
        speed_guard_mask=0;
        for(ch=0;ch<2;ch++) {
            rpm_abs=feedback[ch].speed_rpm;
            if(rpm_abs<0) rpm_abs=-rpm_abs;
            if(rpm_abs>rpm_peak[ch]) rpm_peak[ch]=rpm_abs;
        }
        if(previous_valid) {
            period=(float)(begin-last_cycles)/cycles_per_us;
            if(period>period_peak) period_peak=period;
            if(period>1500.0f) late++;
            if(armed && !done && (period<750.0f || period>1250.0f)) fault|=256U;
        }
        last_cycles=begin;
        if(!armed && !done && !fault) {
            if(health(now,position,0)==0 &&
               feedback[0].speed_rpm<=5 && feedback[0].speed_rpm>=-5 &&
               feedback[1].speed_rpm<=5 && feedback[1].speed_rpm>=-5) stationary_ms++;
            else stationary_ms=0;
            if(now-boot>=E5_ARM_MS) {
                fault|=health(now,position,0);
                if(stationary_ms<500U) fault|=512U;
                if(!fault) {
                    origin[0]=feedback[0].total_ecd;origin[1]=feedback[1].total_ecd;
                    start=now;armed=1;previous_valid=0;
                }
            }
        }
        if(g_exp5_abort) fault|=32U;
        if(g_exp5_can_errors) fault|=128U;
        for(ch=0;ch<2;ch++) {
            position[ch]=armed?(float)((int64_t)feedback[ch].total_ecd-(int64_t)origin[ch])*EXP3_METERS_PER_MOTOR_COUNT:0;
            if(ch) position[ch]=-position[ch];
        }
        raw_q[0]=.5f*(position[0]+position[1]);raw_q[1]=(position[1]-position[0])/.60f;
        if(armed) fault|=health(now,position,1);
        e5_motion_velocity(feedback[0].speed_rpm,feedback[1].speed_rpm,measured_velocity);
        for(ch=0;ch<2;ch++) {
            if(previous_valid && armed) {
                v_before=velocity[ch];
                velocity[ch]+=.26959730895f*(measured_velocity[ch]-velocity[ch]);
                accel[ch]+=.11808862170f*((velocity[ch]-v_before)/.001f-accel[ch]);
            } else {velocity[ch]=0;accel[ch]=0;}
            /* Keep core feedback filtered. ONLY the independent braking
             * envelope uses raw rpm; direct feedback failed mismatch tests. */
            input.q[ch]=raw_q[ch];input.velocity[ch]=velocity[ch];input.acceleration[ch]=accel[ch];
        }
        previous_valid=1;
        if(armed && !done && !fault) {
            elapsed=now-start;
            phase=e5_motion_reference(elapsed,input.reference);
            if(phase==6U) {
                done=1;
                if(fabsf(position[0])>.0005f || fabsf(position[1])>.0005f ||
                   fabsf(position[0]-position[1])>.0005f) fault|=1024U;
            } else {
                if(fabsf(position[0]-input.reference[0])>.001f ||
                   fabsf(position[1]-input.reference[0])>.001f) tracking_ms++;
                else tracking_ms=0;
                if(tracking_ms>=1000U) fault|=1024U;
                if(phase==2U && input.reference[0]>.00025f &&
                   (position[0]<-.00025f || position[1]<-.00025f)) fault|=1024U;
                if(!fault) {
                    core_begin=DWT->CYCCNT;
                    input.raw_rpm[0]=feedback[0].speed_rpm;input.raw_rpm[1]=feedback[1].speed_rpm;
                    if(!e5_step(&controller,&input,&output)) fault|=64U;
                    core_end=DWT->CYCCNT;core_us=(float)(core_end-core_begin)/cycles_per_us;
                    if(core_us>core_peak) core_peak=core_us;
                    if(!e5_formal_finalize(E5_SELECTED_METHOD,&input,&output,
                                          command,before_guard,&speed_guard_mask))fault|=64U;
                    for(ch=0;ch<2;ch++) {
                        if(speed_guard_mask&(1U<<ch))speed_guard_count[ch]++;
                        guard_delta[ch]=(int32_t)command[ch]-(int32_t)before_guard[ch];
                        /* Core AW plus ONE outer residual correction.
                         * CA integer/guard correction = zero; BASE/BLNN
                         * include quantization and their external guard. */
                        controller.aw[ch]+=.001f*16.0f*((float)command[ch]-output.current[ch]);
                        if(!e5_motion_finite(controller.aw[ch])) fault|=64U;
                        /* Common metric: final integer near hard amplitude edge.
                         * NOT all guard interventions or unmet NN demand. */
                        if(fabsf((float)command[ch])>=input.limit[ch]-1.0f)sat[ch]++;
                    }
                }
            }
        }
        /* Check time budget BEFORE enqueueing a nonzero command. */
        if((float)(DWT->CYCCNT-begin)/cycles_per_us>=900.0f) fault|=256U;
        if(fault || done || !armed) {command[0]=0;command[1]=0;g_exp5_drive_permit=0;}
        else g_exp5_drive_permit=1;
        CAN_cmd_3508(command[0],command[1]);
        if(g_exp5_can_errors) {
            fault|=128U;g_exp5_drive_permit=0;command[0]=0;command[1]=0;
            CAN_cmd_3508(0,0);
        }
        if(fault) {
            phase=9;done=1;g_exp5_drive_permit=0;
            if(!first_fault) {first_fault=fault;first_fault_ms=now-start;e5_trace_latch(&fault_trace,now);}
        }
        channels[0]=(float)now*.001f;channels[1]=0;channels[2]=(float)phase;
        channels[3]=input.reference[0];channels[4]=input.reference[1];
        channels[5]=position[0];channels[6]=position[1];
        channels[7]=input.velocity[0]-.30f*input.velocity[1];channels[8]=input.velocity[0]+.30f*input.velocity[1];
        for(ch=0;ch<2;ch++) {
            channels[9+ch]=output.requested[ch];channels[11+ch]=output.current[ch];
            channels[13+ch]=(float)command[ch]; /* attempted, not delivery acknowledgement */
            channels[15+ch]=(float)feedback[ch].given_current;
            channels[17+ch]=fabsf((float)command[ch]-output.requested[ch])>1.0f?1.0f:0.0f;
            channels[19+ch]=input.limit[ch];channels[23+ch]=controller.aw[ch];
            channels[25+ch]=(float)command[ch]-output.requested[ch];
            channels[29+ch]=(float)feedback[ch].temperate;
            channels[33+ch]=output.dhat[ch];channels[35+ch]=(float)controller.net[ch].count;
            channels[37+ch]=output.norm[ch];channels[39+ch]=output.sigma[ch];
            channels[49+ch]=output.sector[ch];channels[51+ch]=controller.k2[ch];
            channels[53+ch]=raw_q[ch];channels[55+ch]=accel[ch];channels[61+ch]=(float)sat[ch];
        }
        channels[21]=E5_SELECTED_METHOD;channels[22]=(float)fault;channels[27]=Batvalt;
        channels[28]=(float)g_exp5_can_rejected; /* new diagnostic count; not load */
        channels[31]=output.gate;
        channels[32]=(armed&&!done&&!fault&&E5_SELECTED_METHOD!=E5_BASE)?
                     (E5_SELECTED_METHOD==E5_BLNN?1.0f:output.learning_gate):0.0f;
        channels[41]=core_us;channels[42]=core_peak;channels[43]=period;channels[44]=period_peak;
        channels[45]=(float)late;channels[46]=(float)controller.step;
        /* Previously unused placeholders: actual per-axis CAN feedback age. */
        channels[47]=feedback[0].rx_count?(float)(now-feedback[0].last_rx_ms):-1.0f;
        channels[48]=feedback[1].rx_count?(float)(now-feedback[1].last_rx_ms):-1.0f;
        channels[57]=(float)overruns;
        if(now-stack_checked>=100U) {
            stack_bytes=(uint32_t)(uxTaskGetStackHighWaterMark(NULL)*sizeof(StackType_t));stack_checked=now;
        }
        channels[58]=(float)stack_bytes;channels[59]=(float)g_exp5_uart_errors;
        channels[60]=whole_peak;channels[63]=E5_MOTION_BUILD;
        /* Common reference/increment diagnostics during active control only.
         * BASE delta=0; BLNN observed delta is NOT capped by CA's 300 counts.
         * 107-channel JustFloat, old fault replay remains I96..I103. */
        channels[64]=(armed&&!done&&!fault)?output.ca_guarded_base[0]:0;
        channels[65]=(armed&&!done&&!fault)?output.ca_guarded_base[1]:0;
        channels[66]=(armed&&!done&&!fault)?output.ca_delta[0]:0;
        channels[67]=(armed&&!done&&!fault)?output.ca_delta[1]:0;
        channels[68]=(armed&&!done&&!fault)?(float)output.ca_mask:0;
        channels[69]=(float)g_exp5_dma_completed;channels[70]=(float)g_exp5_dma_skipped;
        channels[71]=(float)g_exp5_uart_last_error;
        channels[72]=(float)g_exp5_bms_errors;channels[73]=(float)g_exp5_bms_valid;
        channels[74]=(float)controller.growth;channels[75]=(float)controller.net[0].projections;
        channels[76]=(float)controller.net[1].projections;channels[77]=(float)controller.ca_fallbacks;
        channels[78]=(float)loop_count++;channels[79]=(float)elapsed*.001f;
        channels[80]=(float)feedback[0].speed_rpm;channels[81]=(float)feedback[1].speed_rpm;
        channels[82]=(float)rpm_peak[0];channels[83]=(float)rpm_peak[1];
        channels[84]=(float)speed_guard_count[0];channels[85]=(float)speed_guard_count[1];
        channels[86]=(float)speed_guard_mask;channels[87]=E5_HW_PROFILE_ID;
        channels[88]=measured_velocity[0]-.30f*measured_velocity[1];
        channels[89]=measured_velocity[0]+.30f*measured_velocity[1];
        channels[90]=(float)before_guard[0];channels[91]=(float)before_guard[1];
        channels[92]=(float)guard_delta[0];channels[93]=(float)guard_delta[1];
        channels[94]=(float)first_fault;channels[95]=(float)first_fault_ms*.001f;
        channels[104]=(float)E5_SELECTED_LOAD_G; /* configured label, not a sensor */
        channels[105]=(float)controller.ca_contractions;
        channels[106]=(armed&&!done&&!fault)?1.0f:0.0f;
        whole_us=(float)(DWT->CYCCNT-begin)/cycles_per_us;
        if(whole_us>=950.0f) fault|=256U;
        if(fault) {
            if(!first_fault) {first_fault=fault;first_fault_ms=now-start;e5_trace_latch(&fault_trace,now);}
            channels[94]=(float)first_fault;channels[95]=(float)first_fault_ms*.001f;
            phase=9;done=1;g_exp5_drive_permit=0;command[0]=0;command[1]=0;
            CAN_cmd_3508(0,0);channels[2]=9;channels[22]=(float)fault;channels[13]=0;channels[14]=0;
            channels[32]=0;channels[64]=0;channels[65]=0;channels[66]=0;channels[67]=0;channels[68]=0;channels[106]=0;
        }
        publish();
        whole_us=(float)(DWT->CYCCNT-begin)/cycles_per_us;
        if(whole_us>whole_peak) whole_peak=whole_us;
        if(whole_us>=1000.0f) {
            overruns++;fault|=256U;done=1;phase=9;g_exp5_drive_permit=0;
            command[0]=0;command[1]=0;
            CAN_cmd_3508(0,0);channels[2]=9;channels[22]=(float)fault;
            if(!first_fault) {first_fault=fault;first_fault_ms=now-start;e5_trace_latch(&fault_trace,now);}
            channels[94]=(float)first_fault;channels[95]=(float)first_fault_ms*.001f;
            channels[13]=0;channels[14]=0;channels[57]=(float)overruns;
            channels[32]=0;channels[64]=0;channels[65]=0;channels[66]=0;channels[67]=0;channels[68]=0;channels[106]=0;publish();
        }
        previous_command[0]=command[0];previous_command[1]=command[1];
        last_wake=(TickType_t)e5_release_base((uint32_t)xTaskGetTickCount(),(uint32_t)last_wake,1U,&rebase_events);
        vTaskDelayUntil(&last_wake,1U);
    }
}
