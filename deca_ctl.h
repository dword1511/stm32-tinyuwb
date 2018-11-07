#ifndef __DECA_CTL_H__
#define __DECA_CTL_H__

#include <os/tick.h>

#define portGetTickCnt() tick_get_uptime()

extern void deca_ctl_init_entry(void);

#endif /* __DECA_CTL_H__ */
