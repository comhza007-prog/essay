#ifndef E5_FORMAL_ACTUATOR_H
#define E5_FORMAL_ACTUATOR_H
#include "blnn_core.h"
#include "experiment5_motion.h"
/* Match the existing qualified actuation chains, not a new controller.
 * CA already performed integer allocation + speed guard INSIDE its core.
 * BASE/BLNN use the original float clip -> integer -> ONE speed guard.
 * This helper does NOT integrate AW. motor_task keeps its original one
 * quantization/guard residual correction; CA's correction is exactly zero.
 * Diagnostic BASE reference never changes any method's final command. */
static int e5_formal_finalize(uint32_t method,const e5_input *in,e5_output *out,
                              int16_t command[2],int16_t before_guard[2],
                              uint32_t *speed_mask) {
    uint32_t ch;float base;int16_t guarded_base;
    if(!in||!out||!command||!before_guard||!speed_mask ||
       (method!=E5_BASE&&method!=E5_BLNN&&method!=E5_CA_BLNN))return 0;
    *speed_mask=0U;command[0]=command[1]=0;
    for(ch=0;ch<2;ch++) {
        if(!e5_motion_quantize((float)out->current[ch],&command[ch]))goto invalid;
        if(method==E5_CA_BLNN) {
            /* NO second taper of the CA command. */
            before_guard[ch]=(int16_t)out->ca_before_guard[ch];
            if(out->ca_speed_mask&(1U<<ch))*speed_mask|=1U<<ch;
        } else {
            before_guard[ch]=command[ch];
            if(e5_motion_speed_guard(in->raw_rpm[ch],&command[ch]))*speed_mask|=1U<<ch;
            base=(float)out->ca_base[ch];
            if(base>(float)in->limit[ch])base=(float)in->limit[ch];
            if(base<-(float)in->limit[ch])base=-(float)in->limit[ch];
            if(!e5_motion_quantize(base,&guarded_base))goto invalid;
            e5_motion_speed_guard(in->raw_rpm[ch],&guarded_base);
            out->ca_guarded_base[ch]=guarded_base;
            /* An observed FINAL increment, NOT a constraint on ordinary NN. */
            out->ca_delta[ch]=(e5_real)(command[ch]-guarded_base);
        }
    }
    return 1;
invalid:
    command[0]=command[1]=0;*speed_mask=0U;return 0;
}
#endif
