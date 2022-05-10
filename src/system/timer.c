/*!
 * \file      timer.c
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
#include "utilities.h"
#include "board.h"
#include "rtc-board.h"
#include "timer.h"
#include <time.h>
// /*!
//  * Safely execute call back
//  */
// #define ExecuteCallBack( _callback_, context ) \
//     do                                         \
//     {                                          \
//         if( _callback_ == NULL )               \
//         {                                      \
//             while( 1 );                        \
//         }                                      \
//         else                                   \
//         {                                      \
//             _callback_( context );             \
//         }                                      \
//     }while( 0 );

// /*!
//  * Timers list head pointer
//  */
// static TimerEvent_t *TimerListHead = NULL;

// /*!
//  * \brief Adds or replace the head timer of the list.
//  *
//  * \remark The list is automatically sorted. The list head always contains the
//  *         next timer to expire.
//  *
//  * \param [IN]  obj Timer object to be become the new head
//  * \param [IN]  remainingTime Remaining time of the previous head to be replaced
//  */
// static void TimerInsertNewHeadTimer( TimerEvent_t *obj );

// /*!
//  * \brief Adds a timer to the list.
//  *
//  * \remark The list is automatically sorted. The list head always contains the
//  *         next timer to expire.
//  *
//  * \param [IN]  obj Timer object to be added to the list
//  * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
//  */
// static void TimerInsertTimer( TimerEvent_t *obj );

// /*!
//  * \brief Sets a timeout with the duration "timestamp"
//  *
//  * \param [IN] timestamp Delay duration
//  */
// static void TimerSetTimeout( TimerEvent_t *obj );

// /*!
//  * \brief Check if the Object to be added is not already in the list
//  *
//  * \param [IN] timestamp Delay duration
//  * \retval true (the object is already in the list) or false
//  */
// static bool TimerExists( TimerEvent_t *obj );

// void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) )
// {
//     obj->Timestamp = 0;
//     obj->ReloadValue = 0;
//     obj->IsStarted = false;
//     obj->IsNext2Expire = false;
//     obj->Callback = callback;
//     obj->Context = NULL;
//     obj->Next = NULL;
// }

// void TimerSetContext( TimerEvent_t *obj, void* context )
// {
//     obj->Context = context;
// }

// void TimerStart( TimerEvent_t *obj )
// {
//     uint32_t elapsedTime = 0;

//     CRITICAL_SECTION_BEGIN( );

//     if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
//     {
//         CRITICAL_SECTION_END( );
//         return;
//     }

//     obj->Timestamp = obj->ReloadValue;
//     obj->IsStarted = true;
//     obj->IsNext2Expire = false;

//     if( TimerListHead == NULL )
//     {
//         RtcSetTimerContext( );
//         // Inserts a timer at time now + obj->Timestamp
//         TimerInsertNewHeadTimer( obj );
//     }
//     else
//     {
//         elapsedTime = RtcGetTimerElapsedTime( );
//         obj->Timestamp += elapsedTime;

//         if( obj->Timestamp < TimerListHead->Timestamp )
//         {
//             TimerInsertNewHeadTimer( obj );
//         }
//         else
//         {
//             TimerInsertTimer( obj );
//         }
//     }
//     CRITICAL_SECTION_END( );
// }

// static void TimerInsertTimer( TimerEvent_t *obj )
// {
//     TimerEvent_t* cur = TimerListHead;
//     TimerEvent_t* next = TimerListHead->Next;

//     while( cur->Next != NULL )
//     {
//         if( obj->Timestamp > next->Timestamp )
//         {
//             cur = next;
//             next = next->Next;
//         }
//         else
//         {
//             cur->Next = obj;
//             obj->Next = next;
//             return;
//         }
//     }
//     cur->Next = obj;
//     obj->Next = NULL;
// }

// static void TimerInsertNewHeadTimer( TimerEvent_t *obj )
// {
//     TimerEvent_t* cur = TimerListHead;

//     if( cur != NULL )
//     {
//         cur->IsNext2Expire = false;
//     }

//     obj->Next = cur;
//     TimerListHead = obj;
//     TimerSetTimeout( TimerListHead );
// }

// bool TimerIsStarted( TimerEvent_t *obj )
// {
//     return obj->IsStarted;
// }

// void TimerIrqHandler( void )
// {
//     TimerEvent_t* cur;
//     TimerEvent_t* next;

//     uint32_t old =  RtcGetTimerContext( );
//     uint32_t now =  RtcSetTimerContext( );
//     uint32_t deltaContext = now - old; // intentional wrap around

//     // Update timeStamp based upon new Time Reference
//     // because delta context should never exceed 2^32
//     if( TimerListHead != NULL )
//     {
//         for( cur = TimerListHead; cur->Next != NULL; cur = cur->Next )
//         {
//             next = cur->Next;
//             if( next->Timestamp > deltaContext )
//             {
//                 next->Timestamp -= deltaContext;
//             }
//             else
//             {
//                 next->Timestamp = 0;
//             }
//         }
//     }

//     // Execute immediately the alarm callback
//     if ( TimerListHead != NULL )
//     {
//         cur = TimerListHead;
//         TimerListHead = TimerListHead->Next;
//         cur->IsStarted = false;
//         ExecuteCallBack( cur->Callback, cur->Context );
//     }

//     // Remove all the expired object from the list
//     while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp < RtcGetTimerElapsedTime( ) ) )
//     {
//         cur = TimerListHead;
//         TimerListHead = TimerListHead->Next;
//         cur->IsStarted = false;
//         ExecuteCallBack( cur->Callback, cur->Context );
//     }

//     // Start the next TimerListHead if it exists AND NOT running
//     if( ( TimerListHead != NULL ) && ( TimerListHead->IsNext2Expire == false ) )
//     {
//         TimerSetTimeout( TimerListHead );
//     }
// }

// void TimerStop( TimerEvent_t *obj )
// {
//     CRITICAL_SECTION_BEGIN( );

//     TimerEvent_t* prev = TimerListHead;
//     TimerEvent_t* cur = TimerListHead;

//     // List is empty or the obj to stop does not exist
//     if( ( TimerListHead == NULL ) || ( obj == NULL ) )
//     {
//         CRITICAL_SECTION_END( );
//         return;
//     }

//     obj->IsStarted = false;

//     if( TimerListHead == obj ) // Stop the Head
//     {
//         if( TimerListHead->IsNext2Expire == true ) // The head is already running
//         {
//             TimerListHead->IsNext2Expire = false;
//             if( TimerListHead->Next != NULL )
//             {
//                 TimerListHead = TimerListHead->Next;
//                 TimerSetTimeout( TimerListHead );
//             }
//             else
//             {
//                 RtcStopAlarm( );
//                 TimerListHead = NULL;
//             }
//         }
//         else // Stop the head before it is started
//         {
//             if( TimerListHead->Next != NULL )
//             {
//                 TimerListHead = TimerListHead->Next;
//             }
//             else
//             {
//                 TimerListHead = NULL;
//             }
//         }
//     }
//     else // Stop an object within the list
//     {
//         while( cur != NULL )
//         {
//             if( cur == obj )
//             {
//                 if( cur->Next != NULL )
//                 {
//                     cur = cur->Next;
//                     prev->Next = cur;
//                 }
//                 else
//                 {
//                     cur = NULL;
//                     prev->Next = cur;
//                 }
//                 break;
//             }
//             else
//             {
//                 prev = cur;
//                 cur = cur->Next;
//             }
//         }
//     }
//     CRITICAL_SECTION_END( );
// }

// static bool TimerExists( TimerEvent_t *obj )
// {
//     TimerEvent_t* cur = TimerListHead;

//     while( cur != NULL )
//     {
//         if( cur == obj )
//         {
//             return true;
//         }
//         cur = cur->Next;
//     }
//     return false;
// }

// void TimerReset( TimerEvent_t *obj )
// {
//     TimerStop( obj );
//     TimerStart( obj );
// }

// void TimerSetValue( TimerEvent_t *obj, uint32_t value )
// {
//     uint32_t minValue = 0;
//     uint32_t ticks = RtcMs2Tick( value );

//     TimerStop( obj );

//     minValue = RtcGetMinimumTimeout( );

//     if( ticks < minValue )
//     {
//         ticks = minValue;
//     }

//     obj->Timestamp = ticks;
//     obj->ReloadValue = ticks;
// }

TimerTime_t TimerGetCurrentTime( void )
{
    // uint32_t now = RtcGetTimerValue( );
    // return  RtcTick2Ms( now );
    struct timespec tn = {0};
    // clock_gettime(CLOCK_REALTIME, &tn);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tn);
    // clock_gettime(CLOCK_MONOTONIC, &tn);
    return ((tn.tv_sec) * 1000 + (tn.tv_nsec) / 1000000);
}

TimerTime_t TimerGetElapsedTime( TimerTime_t past )
{
    // if ( past == 0 )
    // {
    //     return 0;
    // }
    // uint32_t nowInTicks = RtcGetTimerValue( );
    // uint32_t pastInTicks = RtcMs2Tick( past );

    // // Intentional wrap around. Works Ok if tick duration below 1ms
    // return RtcTick2Ms( nowInTicks - pastInTicks );
    return (TimerGetCurrentTime() - past);
}

// static void TimerSetTimeout( TimerEvent_t *obj )
// {
//     int32_t minTicks= RtcGetMinimumTimeout( );
//     obj->IsNext2Expire = true;

//     // In case deadline too soon
//     if( obj->Timestamp  < ( RtcGetTimerElapsedTime( ) + minTicks ) )
//     {
//         obj->Timestamp = RtcGetTimerElapsedTime( ) + minTicks;
//     }
//     RtcSetAlarm( obj->Timestamp );
// }

TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature )
{
    return RtcTempCompensation( period, temperature );
}

// void TimerProcess( void )
// {
//     RtcProcess( );
// }

// timer handle list head.
static struct Timer *head_handle = NULL;

// Timer ticks
// static uint32_t _timer_ticks = (1 << 32)- 1000; // only for test tick clock
// overflow
static uint32_t _timer_ticks = 0;

/**
 * @brief  Initializes the timer struct handle.
 * @param  handle: the timer handle strcut.
 * @param  timeout_cb: timeout callback.
 * @param  timeout: delay to start the timer.
 * @param  repeat: repeat interval time.
 * @param  arg: the input argument for timeout_cb fucntion.
 * @retval None
 */
void timer_init(struct Timer *handle, void (*timeout_cb)(void *arg),
                void *arg)
{
    handle->timeout_cb = timeout_cb;
    handle->cur_ticks = _timer_ticks;
    handle->arg = arg;
    struct Timer *target = head_handle;
    while (target)
    {
        if (target == handle)
        {
            return;
        }
        target = target->next;
    }
    handle->next = head_handle;
    head_handle = handle;

    handle->enable = false;
}

/**
 * @brief  Start the timer work, add the handle into work list.
 * @param  btn: target handle strcut.
 * @retval 0: succeed. -1: already exist.
 */
int timer_start(struct Timer *handle)
{
    handle->enable = true;
    return 0;
}

int timer_set_value(struct Timer *handle, uint32_t timeout, uint32_t repeat)
{
    struct Timer *target = head_handle;
    // handle->enable = true;
    while (target)
    {
        if (target == handle)
        {
            target->timeout = timeout;
            target->repeat = repeat;
            handle->cur_expired_time = target->timeout;
            return 0;
        }
        target = target->next;
    }
    return -1;
}

/**
 * @brief  Stop the timer work, remove the handle off work list.
 * @param  handle: target handle strcut.
 * @retval 0: succeed. -1: timer not exist.
 */
int timer_stop(struct Timer *handle)
{
    handle->enable = false;
    return 0;
}

int timer_destroy(struct Timer *handle)
{
    struct Timer **curr;
    for (curr = &head_handle; *curr;)
    {
        struct Timer *entry = *curr;
        if (entry == handle)
        {
            *curr = entry->next;
            // free(entry);
            return 0; // found specified timer
        }
        else
        {
            curr = &entry->next;
        }
    }
    return 0;
}

/**
 * @brief  main loop.
 * @param  None.
 * @retval None
 */
void timer_loop(void)
{
    struct Timer *target;

    for (target = head_handle; target; target = target->next)
    {
        if (target->enable)
        {
            /*
            More detail on tick-clock overflow, please see
            https://blog.csdn.net/szullc/article/details/115332326
            */
            if (_timer_ticks - target->cur_ticks >= target->cur_expired_time)
            {
                uint32_t exp_time = target->cur_expired_time;
                if (target->repeat == 0)
                {
                    timer_stop(target);
                    continue;
                }
                else
                {
                    target->cur_ticks = _timer_ticks;
                    target->cur_expired_time = target->repeat;
                }
                if (exp_time != 0)
                {
                    target->timeout_cb(target->arg);
                }
            }
        }
    }
}

/**
 * @brief  background ticks, timer repeat invoking interval nms.
 * @param  None.
 * @retval None.
 */
void timer_ticks(void) { _timer_ticks += CFG_TIMER_1_TICK_N_MS; }
