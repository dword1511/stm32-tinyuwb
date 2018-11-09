#ifndef __RANGING_OS_H__
#define __RANGING_OS_H__

#include <os/tick.h>


/* Goodies from linux kernel */

#define __same_type(a, b) __builtin_types_compatible_p(typeof(a), typeof(b))

#define BUILD_BUG_ON_ZERO(e) (sizeof(struct { int:-!!(e); }))

#define __must_be_array(a)	BUILD_BUG_ON_ZERO(__same_type((a), &(a)[0]))

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]) + __must_be_array(arr))

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})


extern void os_init(void);
extern void os_halt(void);
extern void os_panic(void);

extern void os_dvfs_msi(unsigned f);
extern void os_dvfs_hsi16(void);

#endif /* __RANGING_OS_H__ */
