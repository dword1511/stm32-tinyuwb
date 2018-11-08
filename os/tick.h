#ifndef __RANGING_TICK_H__
#define __RANGING_TICK_H__

#include <stdint.h>

extern void     tick_setup(void);
extern uint32_t tick_get_uptime(void); /* Gets uptime in milliseconds (warps every 49 days). */
extern void     tick_sleep(uint32_t ms); /* Sleep for milliseconds, w.r.t uptime. */
extern void     tick_delay_us(uint32_t us);

#endif /* __RANGING_TICK_H__ */
