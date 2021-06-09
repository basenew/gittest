#ifndef __COMMON_H__
#define __COMMON_H__

#include "rtthread.h"

#ifdef __cplusplus
extern "C" {
#endif



#define MS_TO_TICKS(_MS) (rt_tick_from_millisecond(_MS))


int32_t os_gettime_ms(void);





#ifdef __cplusplus
}
#endif

#endif
