#ifndef __RANGING_TICK_H__
#define __RANGING_TICK_H__

#include <stdint.h>

extern void     tick_setup(void);
extern volatile uint32_t tick_get_uptime(void); /* Get uptime in milliseconds (warps every 49 days). */
extern void     tick_sleep(uint32_t ms); /* Sleep for milliseconds, w.r.t uptime. */
extern void     tick_sleep_until(uint32_t target_ms); /* Sleep till target uptime in milliseconds */
extern void     tick_delay_us(uint32_t us); /* Busy waiting for some approximate microseconds */

#endif /* __RANGING_TICK_H__ */
