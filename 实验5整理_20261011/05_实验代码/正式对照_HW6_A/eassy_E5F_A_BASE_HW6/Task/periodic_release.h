#ifndef E5_PERIODIC_RELEASE_H
#define E5_PERIODIC_RELEASE_H
#include <stdint.h>
/* Preserve exactly-due release. Rebase only MORE THAN a full period late. */
static uint32_t e5_release_base(uint32_t now,uint32_t previous,uint32_t period,uint32_t *events) {
    if(now-previous>period) {(*events)++;return now;}
    return previous;
}
#endif
