#ifndef E5_BLNN_CORE_H
#define E5_BLNN_CORE_H
#include <stdint.h>
#ifdef E5_USE_DOUBLE
typedef double e5_real;
#else
typedef float e5_real;
#endif
#define E5_MAX_NODES 60
#define E5_BASE 10
#define E5_BLNN 11
#define E5_CA_BLNN 12
typedef struct {
    e5_real center[E5_MAX_NODES][5];
    e5_real weight[E5_MAX_NODES][3];
    uint32_t count, projections;
} e5_network;
typedef struct {
    e5_network net[2];
    e5_real aw[2], k2[2], gate;
    uint32_t step, growth, method, fixed_nodes, fault;
    /* Per-step scratch: caller-owned, avoids stack/heap allocation. */
    e5_real features[2][E5_MAX_NODES][3];
    /* CA-only FINAL increment relative to guarded BASE at the same state. */
    int16_t ca_delta[2];
    uint32_t ca_fallbacks;
    uint32_t ca_contractions;
} e5_state;
typedef struct {
    e5_real q[2], velocity[2], acceleration[2];
    e5_real reference[3]; /* translation q, v, a; rotation reference is zero */
    e5_real limit[2]; /* positive motor command counts, not amperes */
    int16_t raw_rpm[2]; /* CA hardware adapter; ignored by BASE/BLNN */
} e5_input;
typedef struct {
    e5_real requested[2], current[2], sigma[2], dhat[2];
    e5_real gate, learning_gate, sector[2], norm[2], novelty[2];
    e5_real ca_base[2], ca_guarded_base[2], ca_delta[2], ca_before_guard[2];
    uint32_t ca_mask, ca_speed_mask;
} e5_output;
/* No heap allocation. State is owned by the caller and reset once per run.
 * Float actuator output is intentional: no physical CAN writes in this module.
 * Return 0 and latch state.fault on invalid inputs or numerical failure. */
int e5_reset(e5_state *s, uint32_t method, uint32_t initial_nodes);
int e5_step(e5_state *s, const e5_input *in, e5_output *out);
#endif
