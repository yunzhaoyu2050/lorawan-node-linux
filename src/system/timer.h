/*!
 * \file      timer.h
 *
 * \brief     Timer objects and scheduling management implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

// /*!
//  * \brief Timer object description
//  */
// typedef struct TimerEvent_s
// {
//     uint32_t Timestamp;                  //! Current timer value
//     uint32_t ReloadValue;                //! Timer delay value
//     bool IsStarted;                      //! Is the timer currently running
//     bool IsNext2Expire;                  //! Is the next timer to expire
//     void ( *Callback )( void* context ); //! Timer IRQ callback function
//     void *Context;                       //! User defined data object pointer to pass back
//     struct TimerEvent_s *Next;           //! Pointer to the next Timer object.
// }TimerEvent_t;

/*!
 * \brief Timer time variable definition
 */
#ifndef TimerTime_t
typedef uint32_t TimerTime_t;
#define TIMERTIME_T_MAX                             ( ( uint32_t )~0 )
#endif

// /*!
//  * \brief Initializes the timer object
//  *
//  * \remark TimerSetValue function must be called before starting the timer.
//  *         this function initializes timestamp and reload value at 0.
//  *
//  * \param [IN] obj          Structure containing the timer object parameters
//  * \param [IN] callback     Function callback called at the end of the timeout
//  */
// void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) );

// /*!
//  * \brief Sets a user defined object pointer
//  *
//  * \param [IN] context User defined data object pointer to pass back
//  *                     on IRQ handler callback
//  */
// void TimerSetContext( TimerEvent_t *obj, void* context );

// /*!
//  * Timer IRQ event handler
//  */
// void TimerIrqHandler( void );

// /*!
//  * \brief Starts and adds the timer object to the list of timer events
//  *
//  * \param [IN] obj Structure containing the timer object parameters
//  */
// void TimerStart( TimerEvent_t *obj );

// /*!
//  * \brief Checks if the provided timer is running
//  *
//  * \param [IN] obj Structure containing the timer object parameters
//  *
//  * \retval status  returns the timer activity status [true: Started,
//  *                                                    false: Stopped]
//  */
// bool TimerIsStarted( TimerEvent_t *obj );

// /*!
//  * \brief Stops and removes the timer object from the list of timer events
//  *
//  * \param [IN] obj Structure containing the timer object parameters
//  */
// void TimerStop( TimerEvent_t *obj );

// /*!
//  * \brief Resets the timer object
//  *
//  * \param [IN] obj Structure containing the timer object parameters
//  */
// void TimerReset( TimerEvent_t *obj );

// /*!
//  * \brief Set timer new timeout value
//  *
//  * \param [IN] obj   Structure containing the timer object parameters
//  * \param [IN] value New timer timeout value
//  */
// void TimerSetValue( TimerEvent_t *obj, uint32_t value );

/*!
 * \brief Read the current time
 *
 * \retval time returns current time
 */
TimerTime_t TimerGetCurrentTime( void );

/*!
 * \brief Return the Time elapsed since a fix moment in Time
 *
 * \remark TimerGetElapsedTime will return 0 for argument 0.
 *
 * \param [IN] past         fix moment in Time
 * \retval time             returns elapsed time
 */
TimerTime_t TimerGetElapsedTime( TimerTime_t past );

/*!
 * \brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * \param [IN] period Time period to compensate
 * \param [IN] temperature Current temperature
 *
 * \retval Compensated time period
 */
TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature );

// /*!
//  * \brief Processes pending timer events
//  */
// void TimerProcess( void );

/*
It means 1 tick for 1ms.
Your can configurate for your tick time such as 5ms/10ms and so on.
*/
#define CFG_TIMER_1_TICK_N_MS 10

typedef struct Timer
{
    uint32_t cur_ticks;            /* Record current timer start tick */
    uint32_t cur_expired_time;     /* Record current timer expired time */
    uint32_t timeout;              /* Delay (xx ms) time to start tiemr */
    uint32_t repeat;               /* Timer interval expired time (xx ms) */
    void *arg;                     /* Input argument for timeout_cb function */
    void (*timeout_cb)(void *arg); /* Timer expired callback function */
    struct Timer *next;            /* Pointer to next timer */
    bool enable;                   /* Whether to enable the timer */
} Timer;

#define TimerEvent_t struct Timer

/**
 * @brief  Initializes the timer struct handle.
 * @param  handle: the timer handle strcut.
 * @param  timeout_cb: timeout callback.
 * @param  timeout: delay to start the timer.
 * @param  repeat: repeat interval time.
 * @param  arg: the input argument for timeout_cb fucntion.
 * @retval None
 */
void timer_init(struct Timer *handle, void (*timeout_cb)(void *arg), void *arg);

/**
 * @brief  Start the timer work, add the handle into work list.
 * @param  btn: target handle strcut.
 * @retval 0: succeed. -1: already exist.
 */
int timer_start(struct Timer *handle);

int timer_set_value(struct Timer *handle, uint32_t timeout, uint32_t repeat);

/**
 * @brief  Stop the timer work, remove the handle off work list.
 * @param  handle: target handle strcut.
 * @retval 0: succeed. -1: timer not exist.
 */
int timer_stop(struct Timer *handle);

int timer_destroy(struct Timer *handle);

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

#define TimerInit timer_init
#define TimerStart timer_start
#define TimerStop timer_stop
#define TimerSetValue timer_set_value

#ifdef __cplusplus
}
#endif

#endif // __TIMER_H__
