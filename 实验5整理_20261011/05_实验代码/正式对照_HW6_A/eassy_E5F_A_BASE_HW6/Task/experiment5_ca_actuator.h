#ifndef E5_CA_ACTUATOR_H
#define E5_CA_ACTUATOR_H
#include "blnn_core.h"
#include "experiment5_motion.h"
/* PROVISIONAL HW6 CA-only adapter, not the frozen simulation algorithm.
 * Counts, NOT amperes. BASE dynamics/gains and existing speed guard stay.
 * Bound FINAL increment relative to guarded BASE at the SAME state.
 * Never reapply the non-idempotent taper afterwards.
 * Normal NN slew: 2 counts/ms. If that neighborhood is infeasible,
 * first retain a contraction aligned with the CURRENT target, bounded by
 * the old component magnitudes, only if no speed guard is intervening.
 * Contraction/withdrawal are explicit slew exceptions and freeze learning.
 * Constraint feasibility takes priority over NN slew, NEVER over braking.
 * These properties are NOT a closed-loop/hardware stability proof. */
#define E5_CA_MAX_DELTA 300
#define E5_CA_DELTA_STEP 2
#define E5_CA_AMPLITUDE 1U
#define E5_CA_PROJECTED 2U
#define E5_CA_SECTOR 4U
#define E5_CA_SPEED 8U
#define E5_CA_WITHDRAWN 16U
#define E5_CA_CONTRACTED 32U
typedef struct {
    int16_t base[2], command[2], delta[2], before_guard[2];
    uint32_t mask, speed_mask;
} e5_ca_actuation;
static int32_t e5_ca_abs(int32_t x) { return x<0?-x:x; }
static e5_real e5_ca_clip(e5_real x,e5_real l) { return x<-l?-l:(x>l?l:x); }
static int e5_ca_sector_ok(const e5_real sigma[2],int32_t dl,int32_t dr) {
    const e5_real k=(e5_real).006197916666666667;
    e5_real work,budget;
    work=sigma[0]*k*(e5_real)(dl-dr);
    budget=(e5_real)10*sigma[0]*sigma[0];
    if(work>budget+(e5_real)1e-10*((e5_real)1+budget)) return 0;
    work=-sigma[1]*(e5_real).3*k*(e5_real)(dl+dr);
    budget=(e5_real)6.25*sigma[1]*sigma[1];
    return work<=budget+(e5_real)1e-10*((e5_real)1+budget);
}
static int e5_ca_envelope_ok(int16_t rpm,int32_t command,int32_t limit) {
    int32_t speed=e5_ca_abs((int32_t)rpm),aligned=rpm>0?command:-command,brake;
    if(e5_ca_abs(command)>limit) return 0;
    if(speed>E5_HW_SOFT_RPM) {
        brake=E5_HW_BRAKE_COUNTS_PER_RPM*(speed-E5_HW_SOFT_RPM);
        if(brake>E5_HW_BRAKE_LIMIT) brake=E5_HW_BRAKE_LIMIT;
        return aligned<=-brake;
    }
    if(speed>E5_HW_TAPER_RPM)
        return aligned<=limit*(E5_HW_SOFT_RPM-speed)/(E5_HW_SOFT_RPM-E5_HW_TAPER_RPM);
    return 1;
}
/* Intersect the CURRENT target ray with the CURRENT linear bounds.
 * Baseline is already feasible, so lambda=0 remains feasible. This is NOT
 * a new guard or a guarantee of plant stability. At most 36 integer
 * candidates are then rechecked; no unbounded search in the motor task. */
static int e5_ca_contract(const e5_real sigma[2],const int16_t rpm[2],
    const int32_t lim[2],const int16_t previous[2],const int32_t target[2],
    e5_ca_actuation *a) {
    const e5_real k=(e5_real).006197916666666667;
    e5_real cap=1,work,budget,bound,level;
    int32_t ch,speed,aligned,ceiling,c,center[2],dl,dr,dist,best;
    int32_t round_l,round_r;
    unsigned step;
    /* Never carry additional NN memory through an infeasible speed-guard
     * transition. In that situation retain v1's immediate NN withdrawal.
     * Partial contraction is ONLY for conflicts without guard intervention. */
    if(a->speed_mask) return 0;
    /* An exceptional projection cannot increase any component beyond its
     * previous magnitude or reverse its sign. If the target is incompatible
     * with that history, fall back to v1's withdrawal, not stale retention. */
    for(ch=0;ch<2;ch++) {
        if((int32_t)previous[ch]*target[ch]<0) return 0;
        if(target[ch]) {
            bound=(e5_real)e5_ca_abs(previous[ch])/(e5_real)e5_ca_abs(target[ch]);
            if(bound<cap) cap=bound;
        }
    }
    work=sigma[0]*k*(e5_real)(target[0]-target[1]);
    budget=(e5_real)10*sigma[0]*sigma[0];
    if(work>0 && budget/work<cap) cap=budget/work;
    work=-sigma[1]*(e5_real).3*k*(e5_real)(target[0]+target[1]);
    budget=(e5_real)6.25*sigma[1]*sigma[1];
    if(work>0 && budget/work<cap) cap=budget/work;
    for(ch=0;ch<2;ch++) {
        c=target[ch];
        if(c) {
            bound=(e5_real)((c>0?lim[ch]:-lim[ch])-a->base[ch])/(e5_real)c;
            if(bound<cap) cap=bound;
        }
        speed=e5_ca_abs((int32_t)rpm[ch]);
        if(speed>E5_HW_TAPER_RPM) {
            aligned=rpm[ch]>0?a->base[ch]:-a->base[ch];
            c=rpm[ch]>0?target[ch]:-target[ch];
            if(speed>E5_HW_SOFT_RPM) {
                ceiling=E5_HW_BRAKE_COUNTS_PER_RPM*(speed-E5_HW_SOFT_RPM);
                if(ceiling>E5_HW_BRAKE_LIMIT) ceiling=E5_HW_BRAKE_LIMIT;
                ceiling=-ceiling;
            } else ceiling=lim[ch]*(E5_HW_SOFT_RPM-speed)/(E5_HW_SOFT_RPM-E5_HW_TAPER_RPM);
            if(c>0) {
                bound=(e5_real)(ceiling-aligned)/(e5_real)c;
                if(bound<cap) cap=bound;
            }
        }
    }
    if(cap<0) cap=0;
    /* Zero remains an explicit competitor. Integer candidates are checked
     * independently, so rounding can never bypass a safety constraint. */
    a->delta[0]=0;a->delta[1]=0;
    best=target[0]*target[0]+target[1]*target[1];
    /* Quantization may invalidate the continuous boundary. Try three more
     * contractions, keep the feasible point closest to the CURRENT target.
     * Rounding never reverses a component or increases its magnitude. */
    for(step=0;step<4;step++) {
        level=cap*(e5_real)(4U-step)/(e5_real)4;
        for(ch=0;ch<2;ch++) center[ch]=(int32_t)(level*(e5_real)target[ch]);
        for(round_l=-1;round_l<=1;round_l++)
        for(round_r=-1;round_r<=1;round_r++) {
            dl=center[0]+round_l;dr=center[1]+round_r;
            if(e5_ca_abs(dl)>e5_ca_abs(previous[0]) || e5_ca_abs(dr)>e5_ca_abs(previous[1]) ||
               dl*(int32_t)previous[0]<0 || dr*(int32_t)previous[1]<0 ||
               e5_ca_abs(dl)>E5_CA_MAX_DELTA || e5_ca_abs(dr)>E5_CA_MAX_DELTA) continue;
            if(!e5_ca_envelope_ok(rpm[0],(int32_t)a->base[0]+dl,lim[0]) ||
               !e5_ca_envelope_ok(rpm[1],(int32_t)a->base[1]+dr,lim[1]) ||
               !e5_ca_sector_ok(sigma,dl,dr)) continue;
            dist=(dl-target[0])*(dl-target[0])+(dr-target[1])*(dr-target[1]);
            if(dist<best) {
                best=dist;a->delta[0]=(int16_t)dl;a->delta[1]=(int16_t)dr;
            }
        }
    }
    return a->delta[0] || a->delta[1];
}
static int e5_ca_allocate(const e5_real bi[2],const e5_real correction[2],
    const e5_real sigma[2],const e5_real limits[2],const int16_t rpm[2],
    const int16_t previous[2],e5_ca_actuation *a) {
    e5_real scale=1,peak=0;
    int32_t target[2],lim[2],dl,dr,cl,cr,dist,best=0x7fffffff,tie=0;
    int16_t wanted[2];
    uint32_t ch;int found=0;
    a->mask=0;a->speed_mask=0;
    for(ch=0;ch<2;ch++) {
        if(!e5_motion_finite((float)bi[ch])||!e5_motion_finite((float)correction[ch])||
           !e5_motion_finite((float)sigma[ch])||!e5_motion_finite((float)limits[ch])||
           limits[ch]<E5_HW_BRAKE_LIMIT||limits[ch]>E5_MOTION_LIMIT) return 0;
        lim[ch]=(int32_t)limits[ch];
        if(correction[ch]>peak) peak=correction[ch];
        if(-correction[ch]>peak) peak=-correction[ch];
    }
    if(peak>E5_CA_MAX_DELTA) {scale=(e5_real)E5_CA_MAX_DELTA/peak;a->mask|=E5_CA_AMPLITUDE;}
    for(ch=0;ch<2;ch++) {
        if(!e5_motion_quantize((float)e5_ca_clip(bi[ch],(e5_real)lim[ch]),&a->base[ch])||
           !e5_motion_quantize((float)e5_ca_clip(bi[ch]+scale*correction[ch],(e5_real)lim[ch]),&wanted[ch])) return 0;
        a->before_guard[ch]=wanted[ch];
        if(e5_motion_speed_guard(rpm[ch],&a->base[ch])) a->speed_mask|=1U<<ch;
        if(e5_motion_speed_guard(rpm[ch],&wanted[ch])) a->speed_mask|=1U<<ch;
        if(!e5_ca_envelope_ok(rpm[ch],a->base[ch],lim[ch])) return 0;
        target[ch]=(int32_t)wanted[ch]-(int32_t)a->base[ch];
    }
    if(a->speed_mask) a->mask|=E5_CA_SPEED;
    if(!e5_ca_sector_ok(sigma,target[0],target[1])) a->mask|=E5_CA_SECTOR;
    /* 25 integer candidates: deterministic finite work per 1-ms step.
     * Zero is feasible for sectors/envelopes, but may be outside slew bounds.
 * In that case first try a feasible contraction of the previous NN. */
    for(dl=(int32_t)previous[0]-E5_CA_DELTA_STEP;dl<=(int32_t)previous[0]+E5_CA_DELTA_STEP;dl++)
    for(dr=(int32_t)previous[1]-E5_CA_DELTA_STEP;dr<=(int32_t)previous[1]+E5_CA_DELTA_STEP;dr++) {
        if(e5_ca_abs(dl)>E5_CA_MAX_DELTA||e5_ca_abs(dr)>E5_CA_MAX_DELTA) continue;
        cl=(int32_t)a->base[0]+dl;cr=(int32_t)a->base[1]+dr;
        if(!e5_ca_envelope_ok(rpm[0],cl,lim[0])||!e5_ca_envelope_ok(rpm[1],cr,lim[1])||
           !e5_ca_sector_ok(sigma,dl,dr)) continue;
        dist=(dl-target[0])*(dl-target[0])+(dr-target[1])*(dr-target[1]);
        if(!found||dist<best||(dist==best&&e5_ca_abs(dl)+e5_ca_abs(dr)<tie)) {
            found=1;best=dist;tie=e5_ca_abs(dl)+e5_ca_abs(dr);
            a->delta[0]=(int16_t)dl;a->delta[1]=(int16_t)dr;
        }
    }
    if(!found) {
        if(e5_ca_contract(sigma,rpm,lim,previous,target,a)) a->mask|=E5_CA_CONTRACTED;
        else {a->delta[0]=0;a->delta[1]=0;a->mask|=E5_CA_WITHDRAWN;}
    }
    for(ch=0;ch<2;ch++) {
        if(a->delta[ch]!=target[ch]) a->mask|=E5_CA_PROJECTED;
        a->command[ch]=(int16_t)((int32_t)a->base[ch]+a->delta[ch]);
    }
    return 1;
}
#endif
