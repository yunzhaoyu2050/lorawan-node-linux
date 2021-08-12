#ifndef __LORA_RADIO_TIMER_H__
#define __LORA_RADIO_TIMER_H__

// #include <rtconfig.h>
// #include <rtthread.h>
#include "timer.h"
#include <stdint.h>
#include <time.h>
#ifdef PKG_USING_MULTI_RTIMER

#include "hw_rtc_stm32.h"
#include "multi_rtimer.h"

#else

#define TimerInit timer_init          // rtick_timer_init
#define TimerStart timer_start        // rtick_timer_start
#define TimerStop timer_stop          // rtick_timer_stop
// #define TimerReset           // rtick_timer_reset
#define TimerSetValue timer_set_value // rtick_timer_set_value
#define TimerGetCurrentTime(now_ts)                                            \
  clock_gettime(CLOCK_MONOTONIC, now_ts) // rtick_timer_get_current_time
uint32_t GetElapsedTime(struct timespec *last_ts);
#define TimerGetElapsedTime GetElapsedTime
#define TimerEvent_t struct Timer // rtick_timer_event_t

// /*!
//  * \brief Timer object description
//  */
// typedef struct TimerEvent_s
// {
//     struct rt_timer timer;
// }rtick_timer_event_t;

// /*!
//  * \brief Timer time variable definition
//  */
// #ifndef TimerTime_t
typedef uint32_t TimerTime_t;
typedef struct timespec SysTime_t;
// #define TIMERTIME_T_MAX                             ( ( uint32_t )~0 )

// void rtick_timer_init( rtick_timer_event_t *obj, void ( *callback )( void )
// );

// void rtick_timer_start( rtick_timer_event_t *obj );

// void rtick_timer_stop( rtick_timer_event_t *obj );

// void rtick_timer_reset( rtick_timer_event_t *obj );

// void rtick_timer_set_value( rtick_timer_event_t *obj, uint32_t value );

// TimerTime_t rtick_timer_get_current_time( void );

// TimerTime_t rtick_timer_get_elapsed_time( TimerTime_t past );

// #endif


/*!
 * \brief Number of seconds elapsed between Unix and GPS epoch
 */
#define UNIX_GPS_EPOCH_OFFSET 315964800

#endif

#endif
