#ifndef __LORA_RADIO_TIMER_H__
#define __LORA_RADIO_TIMER_H__

#include "timer.h"
#include <stdint.h>
#include <time.h>

typedef uint32_t TimerTime_t;

#define TimerInit timer_init
#define TimerStart timer_start
#define TimerStop timer_stop
#define TimerSetValue timer_set_value

/*!
 * \brief Read the current time
 *
 * \retval time returns current time
 */
TimerTime_t TimerGetCurrentTime(void);

/*!
 * \brief Return the Time elapsed since a fix moment in Time
 * \remark TimerGetElapsedTime will return 0 for argument 0.
 *
 * \param [IN] past         fix moment in Time
 * \retval time             returns elapsed time
 */
TimerTime_t TimerGetElapsedTime(TimerTime_t past);

#define TimerEvent_t struct Timer // rtick_timer_event_t

// typedef struct timespec SysTime_t;

#define TIMERTIME_T_MAX                             ( ( uint32_t )~0 )

#endif
