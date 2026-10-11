#ifndef E5_FAULT_TRACE_H
#define E5_FAULT_TRACE_H
#include <stdint.h>
#include <string.h>
#define E5_TRACE_CAPACITY 64U
/* Diagnostic only. No control outputs, allocation, rejection or filtering.
 * One record per motor-task cycle. Encoder rpm is a per-CAN-frame average,
 * NOT an independent sensor or exactly contemporaneous ESC rpm estimate. */
typedef struct {
    uint32_t ms;
    int16_t rpm[2],previous_command[2];
    float encoder_rpm[2];
} e5_trace_record;
typedef struct {
    e5_trace_record rows[E5_TRACE_CAPACITY];
    uint32_t head,count,frozen,fault_ms,cursor;
} e5_fault_trace;
static void e5_trace_reset(e5_fault_trace *t) {memset(t,0,sizeof(*t));}
static void e5_trace_push(e5_fault_trace *t,const e5_trace_record *r) {
    if(t->frozen) return;
    t->rows[t->head]=*r;t->head=(t->head+1U)%E5_TRACE_CAPACITY;
    if(t->count<E5_TRACE_CAPACITY)t->count++;
}
static void e5_trace_latch(e5_fault_trace *t,uint32_t fault_ms) {
    if(!t->frozen) {t->fault_ms=fault_ms;t->cursor=0;t->frozen=1U;}
}
/* Called only while taking the telemetry snapshot, under the same lock.
 * Replay consumes one row per snapshot, not one per motor cycle.
 * Newest FIRST: the triggering cycle and preceding milliseconds are sent
 * before older history. Index is replay order, NOT chronological order.
 * Index exposes transmission gaps; after the last row hold it unchanged.
 * Collect at least 5 seconds AFTER fault to receive all 64 rows at 40 ms. */
static void e5_trace_next(e5_fault_trace *t,float values[8]) {
    uint32_t slot;
    const e5_trace_record *r;
    memset(values,0,8U*sizeof(float));values[7]=-1.0f;
    if(!t->frozen || !t->count)return;
    slot=(t->head+E5_TRACE_CAPACITY-1U-t->cursor)%E5_TRACE_CAPACITY;
    r=&t->rows[slot];
    values[0]=(float)(int32_t)(r->ms-t->fault_ms); /* relative ms */
    values[1]=(float)r->rpm[0];values[2]=(float)r->rpm[1];
    values[3]=r->encoder_rpm[0];values[4]=r->encoder_rpm[1];
    values[5]=(float)r->previous_command[0];values[6]=(float)r->previous_command[1];
    values[7]=(float)t->cursor;
    if(t->cursor+1U<t->count)t->cursor++;
}
#endif
