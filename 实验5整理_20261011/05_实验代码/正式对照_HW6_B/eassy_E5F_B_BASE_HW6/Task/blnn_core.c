#include "blnn_core.h"
#include "experiment5_hw_profile.h"
#include "experiment5_ca_actuator.h"
#include <math.h>
#include <string.h>
#include <float.h>
#ifdef E5_USE_DOUBLE
#define R(x) ((e5_real)(x))
#define EXP exp
#define SQRT sqrt
#define TANH tanh
#define COS cos
#define SIN sin
#define ABS fabs
#define REAL_MAX DBL_MAX
#else
#define R(x) ((e5_real)(x))
#define EXP expf
#define SQRT sqrtf
#define TANH tanhf
#define COS cosf
#define SIN sinf
#define ABS fabsf
#define REAL_MAX FLT_MAX
#endif
static e5_real clip(e5_real x,e5_real lo,e5_real hi) { return x<lo?lo:(x>hi?hi:x); }
static int finite_value(e5_real x) { return x==x && x<=REAL_MAX && x>=-REAL_MAX; }
static void force_to_current(const e5_real f[2],e5_real i[2]) {
    const e5_real k=R(0.006197916666666667), le=R(.60);
    i[0]=f[0]/(2*k)-f[1]/(le*k);
    i[1]=-f[0]/(2*k)-f[1]/(le*k);
}
static void features(const e5_real c[5],const e5_real x[5],e5_real f[3]) {
    uint32_t j; e5_real d=0,t;
    for(j=0;j<5;j++){t=c[j]-x[j];d+=t*t;}
    f[0]=EXP(-d);f[1]=COS(f[0]);f[2]=SIN(f[0]);
}
static e5_real prediction(const e5_network *n,const e5_real x[5],uint32_t channel,e5_real cached[E5_MAX_NODES][3]) {
    uint32_t j; e5_real *f,sum[3]={0,0,0};
    for(j=0;j<n->count;j++) {
        f=cached[j];
        features(n->center[j],x,f);
        sum[0]+=n->weight[j][0]*f[0];sum[1]+=n->weight[j][1]*f[1];sum[2]+=n->weight[j][2]*f[2];
    }
    return clip(sum[0]+sum[1]+sum[2],channel?R(-10):R(-80),channel?R(10):R(80));
}
static int grow(e5_network *n,const e5_real x[5],e5_real *novelty) {
    uint32_t i,j,k,indices[3]={0,0,0};
    e5_real best[3]={R(REAL_MAX),R(REAL_MAX),R(REAL_MAX)},d,t,average[5]={0,0,0,0,0};
    for(i=0;i<n->count;i++) {
        d=0;for(j=0;j<5;j++){t=n->center[i][j]-x[j];d+=t*t;}
        /* Stable nearest-node order for ties, matching MATLAB sort. */
        for(j=0;j<3;j++) if(d<best[j]) {
            for(k=2;k>j;k--){best[k]=best[k-1];indices[k]=indices[k-1];}
            best[j]=d;indices[j]=i;break;
        }
    }
    d=0;
    for(j=0;j<5;j++) {
        average[j]=(n->center[indices[0]][j]+n->center[indices[1]][j]+n->center[indices[2]][j])/R(3);
        t=x[j]-average[j];d+=t*t;
    }
    *novelty=SQRT(d);
    if(*novelty<=R(.3)) return 0;
    for(j=0;j<5;j++) n->center[n->count][j]=average[j]+R(.7)*(x[j]-average[j]);
    for(j=0;j<3;j++) n->weight[n->count][j]=0;
    n->count++; return 1;
}
int e5_reset(e5_state *s,uint32_t method,uint32_t initial_nodes) {
    uint32_t ch,n,j;const e5_real base[5]={-1,R(-.5),0,R(.5),1};
    if(!s) return 0;
    memset(s,0,sizeof(*s));
    if((method!=E5_BASE && method!=E5_BLNN && method!=E5_CA_BLNN) || initial_nodes<4 || initial_nodes>60) {
        s->fault=1;return 0;
    }
    s->method=method;s->k2[0]=R(E5_HW_K2_Y_MIN);s->k2[1]=R(E5_HW_K2_ALPHA_MIN);s->gate=1;
    s->fixed_nodes=initial_nodes==60;
    for(ch=0;ch<2;ch++) {
        s->net[ch].count=method==E5_BASE?0:initial_nodes;
        for(n=0;n<s->net[ch].count;n++) for(j=0;j<5;j++) s->net[ch].center[n][j]=base[(j+5-n%5)%5];
    }
    return 1;
}
int e5_step(e5_state *s,const e5_input *in,e5_output *out) {
    const e5_real mass[2]={R(11.528),R(.34584)},damp[2]={R(57.4),R(20.5)},stiff[2]={0,R(E5_HW_SYNC_STIFFNESS_FF)};
    const e5_real gamma[2]={25,18},k1[2]={40,25},phi[2]={R(.025),R(.012)};
    const e5_real scales[2][5]={{2,R(.7),1,R(.08),R(.7)},{1,R(.20),R(.020),R(.010),R(.20)}};
    const e5_real k=R(.006197916666666667),dt=R(.001);
    e5_real chi[2][5],base[2],bi[2],correction[2],neg[2],limits[2],e,ed,ref,refv,refa;
    e5_real safe=1,headroom=1,learn,gate,work,budget,d,cap,rising,raw[3],*f,norm,w,wp;
    uint32_t ch,n,j,hit,preview_nodes[2];
    e5_ca_actuation actuation;
    if(!s||!in||!out) return 0;
    memset(out,0,sizeof(*out));
    if(s->fault) return 0;
    for(ch=0;ch<2;ch++) {
        if(!finite_value(in->q[ch])||!finite_value(in->velocity[ch])||!finite_value(in->acceleration[ch])||
           !finite_value(in->limit[ch])||in->limit[ch]<=0) goto invalid;
    }
    for(j=0;j<3;j++) if(!finite_value(in->reference[j])) goto invalid;
    for(ch=0;ch<2;ch++) {
        ref=ch?0:in->reference[0];refv=ch?0:in->reference[1];refa=ch?0:in->reference[2];
        e=in->q[ch]-ref;ed=in->velocity[ch]-refv;out->sigma[ch]=ed+gamma[ch]*e;
        base[ch]=damp[ch]*in->velocity[ch]+stiff[ch]*in->q[ch]+(ch?0:R(E5_HW_COULOMB_FF)*TANH(in->velocity[ch]/R(.01)))+
          mass[ch]*refa-k1[ch]*out->sigma[ch]-s->k2[ch]*TANH(out->sigma[ch]/phi[ch])-mass[ch]*gamma[ch]*ed;
        chi[ch][0]=in->acceleration[ch];chi[ch][1]=in->velocity[ch];chi[ch][2]=in->q[ch];chi[ch][3]=e;chi[ch][4]=ed;
        for(j=0;j<5;j++) chi[ch][j]=clip(chi[ch][j]/scales[ch][j],-2,2);
        limits[ch]=clip(in->limit[ch],0,16000);
        preview_nodes[ch]=s->net[ch].count;
        out->dhat[ch]=prediction(&s->net[ch],chi[ch],ch,s->features[ch]);out->sector[ch]=1;
        if(s->method==E5_CA_BLNN) {
            work=-out->sigma[ch]*out->dhat[ch];budget=R(.25)*k1[ch]*out->sigma[ch]*out->sigma[ch];
            if(work>budget && work>0) out->sector[ch]=budget/work;
        }
        neg[ch]=-out->sector[ch]*out->dhat[ch];
    }
    base[0]+=k*(s->aw[0]-s->aw[1]);base[1]+=R(-.3)*k*(s->aw[0]+s->aw[1]);
    force_to_current(base,bi);force_to_current(neg,correction);
    for(ch=0;ch<2;ch++) {
        /* Diagnostic raw BASE reference for all methods; no control change. */
        out->ca_base[ch]=bi[ch];
        if(!finite_value(bi[ch])||!finite_value(correction[ch])) goto invalid;
        d=(limits[ch]-ABS(bi[ch]))/limits[ch];if(d<headroom) headroom=d;
        if(s->method==E5_CA_BLNN && correction[ch]!=0) {
            if(ABS(bi[ch])>limits[ch] && bi[ch]*correction[ch]>0) safe=0;
            cap=(correction[ch]>0?limits[ch]-bi[ch]:-limits[ch]-bi[ch])/correction[ch];
            if(cap<safe) safe=cap>0?cap:0;
        }
    }
    safe=clip(safe,0,1);rising=s->gate+dt/(R(.030)+dt)*(safe-s->gate);gate=safe<rising?safe:rising;
    learn=clip(headroom/R(.15),0,1);if(learn>gate) learn=gate;
    if(s->method==E5_CA_BLNN) {
        for(ch=0;ch<2;ch++) correction[ch]*=gate;
        if(!e5_ca_allocate(bi,correction,out->sigma,limits,in->raw_rpm,s->ca_delta,&actuation)) goto invalid;
        out->ca_mask=actuation.mask;out->ca_speed_mask=actuation.speed_mask;
        if(actuation.mask) learn=0;
        if(actuation.mask&E5_CA_WITHDRAWN) s->ca_fallbacks++;
        if(actuation.mask&E5_CA_CONTRACTED) s->ca_contractions++;
        for(ch=0;ch<2;ch++) {
            s->ca_delta[ch]=actuation.delta[ch];out->ca_delta[ch]=actuation.delta[ch];
            out->ca_base[ch]=bi[ch];out->ca_guarded_base[ch]=actuation.base[ch];
            out->ca_before_guard[ch]=actuation.before_guard[ch];
            /* Final = guarded BASE + admitted NN increment. Rejected NN must
             * not become a baseline AW bias. Therefore current-requested is
             * guarded BASE - attempted BASE, integrated exactly once. */
            out->requested[ch]=bi[ch]+actuation.delta[ch];
            out->current[ch]=actuation.command[ch];
        }
    }
    s->gate=gate;out->gate=gate;out->learning_gate=learn;s->step++;
    for(ch=0;ch<2;ch++) {
        e5_network *net=&s->net[ch];
        e5_real learning=s->method==E5_CA_BLNN?learn:1;
        if(net->count && !s->fixed_nodes && s->step%20==0 && learning>=R(.5) && net->count<60)
            s->growth+=(uint32_t)grow(net,chi[ch],&out->novelty[ch]);
        norm=0;hit=0;
        for(n=0;n<net->count;n++) {
            f=s->features[ch][n];
            /* Only a newly appended zero-weight node needs new features.
             * Existing features and pre-update output are exactly reusable. */
            if(n>=preview_nodes[ch]) features(net->center[n],chi[ch],f);
            for(j=0;j<3;j++) {
                w=net->weight[n][j]+dt*(ch?18:25)*(learning*out->sigma[ch]*f[j]-R(.01)*net->weight[n][j]);
                wp=clip(w,ch?-15:-30,ch?15:30);if(wp!=w) hit=1;
                net->weight[n][j]=wp;norm+=wp*wp;
            }
        }
        net->projections+=hit;out->norm[ch]=SQRT(norm);
        /* Appending zero-weight nodes cannot change the same-step output. */
        if(s->method!=E5_CA_BLNN) raw[ch]=base[ch]-gate*out->sector[ch]*out->dhat[ch];
    }
    if(s->method!=E5_CA_BLNN) force_to_current(raw,out->requested);
    for(ch=0;ch<2;ch++) {
        if(s->method!=E5_CA_BLNN) out->current[ch]=clip(out->requested[ch],-limits[ch],limits[ch]);
        s->aw[ch]+=dt*(16*(out->current[ch]-out->requested[ch])-20*s->aw[ch]);
        s->k2[ch]=clip(s->k2[ch]+dt*(ch?2:5)*(ABS(out->sigma[ch])-s->k2[ch]),
            ch?R(E5_HW_K2_ALPHA_MIN):R(E5_HW_K2_Y_MIN),ch?10:24);
        if(!finite_value(out->requested[ch])||!finite_value(s->aw[ch])||!finite_value(out->norm[ch])||!finite_value(s->k2[ch])) goto invalid;
    }
    return 1;
invalid:
    s->fault=1;memset(out,0,sizeof(*out));return 0;
}
