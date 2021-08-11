/*
 * Copyright (c) 2016 Zibin Zheng <znbin@qq.com>
 * All rights reserved
 */
 
#ifndef _MULTI_TIMER_H_
#define _MULTI_TIMER_H_

#include <stdint.h>
#include <stddef.h>

/**
  * 0.1 : old github version
  * 0.2 : change api version
  */
// #define __TIMER_VERSION 0x010
#define __TIMER_VERSION 0x020

/*
It means 1 tick for 1ms. 
Your can configurate for your tick time such as 5ms/10ms and so on.
*/
#define CFG_TIMER_1_TICK_N_MS   10

typedef struct Timer {
    uint32_t        cur_ticks;          /* Record current timer start tick */
    uint32_t        cur_expired_time;   /* Record current timer expired time */
    uint32_t        timeout;    /* Delay (xx ms) time to start tiemr */
    uint32_t        repeat;     /* Timer interval expired time (xx ms) */
    void *          arg;        /* Input argument for timeout_cb function */
    void            (*timeout_cb)(void *arg); /* Timer expired callback function */
    struct Timer*   next;       /* Pointer to next timer */
} Timer;

#ifdef __cplusplus  
extern "C" {  
#endif  

/**
  * @brief  Initializes the timer struct handle.
  * @param  handle: the timer handle strcut.
  * @param  timeout_cb: timeout callback.
  * @param  timeout: delay to start the timer.
  * @param  repeat: repeat interval time.
  * @param  arg: the input argument for timeout_cb fucntion.
  * @retval None
  */
#if (__TIMER_VERSION == 0x010)
void timer_init(struct Timer* handle, void(*timeout_cb)(void *arg), \
            uint32_t timeout, uint32_t repeat, void *arg);
#elif (__TIMER_VERSION == 0x020)
void timer_init(struct Timer* handle, void (*timeout_cb)(void *arg), void *arg);
#endif

/**
  * @brief  Start the timer work, add the handle into work list.
  * @param  btn: target handle strcut.
  * @retval 0: succeed. -1: already exist.
  */
#if (__TIMER_VERSION == 0x010)
int  timer_start(struct Timer* handle);
#elif (__TIMER_VERSION == 0x020)
int timer_start(struct Timer* handle, uint32_t timeout, uint32_t repeat)
#endif
/**
  * @brief  Stop the timer work, remove the handle off work list.
  * @param  handle: target handle strcut.
  * @retval 0: succeed. -1: timer not exist.
  */
int timer_stop(struct Timer* handle);

/**
  * @brief  background ticks, timer repeat invoking interval nms.
  * @param  None.
  * @retval None.
  */
void timer_ticks(void);

/**
  * @brief  main loop.
  * @param  None.
  * @retval None
  */
void timer_loop(void);

#ifdef __cplusplus
} 
#endif

#endif
