/*!
 * Copyright (c) 2016 Zibin Zheng <znbin@qq.com>
 * All rights reserved
 * \author Zibin Zheng
 * \author Zhao Yu
 */

#include "timer.h"
#include <stdio.h>

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
                void *arg) {
  // memset(handle, sizeof(struct Timer), 0);
  handle->timeout_cb = timeout_cb;
  handle->cur_ticks = _timer_ticks;
  handle->cur_expired_time = handle->timeout;
  handle->arg = arg;
  // printf("cur_ticks: %u, cur_expired_time: %u, _timer_ticks: %u, timeout:
  // %u\r\n",
  //  handle->cur_ticks, handle->cur_expired_time, _timer_ticks, timeout);
  struct Timer *target = head_handle;
  while (target) {
    if (target == handle) {
      return; // already exist.
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
int timer_start(struct Timer *handle) {
  // struct Timer *target = head_handle;
  // while (target) {
  //   if (target == handle) {
  //     return -1; // already exist.
  //   }
  //   target = target->next;
  // }
  // handle->next = head_handle;
  // head_handle = handle;
  handle->enable = true;
  return 0;
}

int timer_set_value(struct Timer *handle, uint32_t timeout, uint32_t repeat) {
  struct Timer *target = head_handle;
  while (target) {
    if (target == handle) {
      target->timeout = timeout;
      target->repeat = repeat;
      return 0; // already exist.
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
int timer_stop(struct Timer *handle) {
  // struct Timer **curr;

  // for (curr = &head_handle; *curr;) {
  //   struct Timer *entry = *curr;
  //   if (entry == handle) {
  //     *curr = entry->next;
  //     // free(entry);
  //     return 0; // found specified timer
  //   } else {
  //     curr = &entry->next;
  //   }
  // }
  handle->enable = false;
  return 0;
}

int timer_destroy(struct Timer *handle) {
  struct Timer **curr;
  for (curr = &head_handle; *curr;) {
    struct Timer *entry = *curr;
    if (entry == handle) {
      *curr = entry->next;
      // free(entry);
      return 0; // found specified timer
    } else {
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
void timer_loop(void) {
  struct Timer *target;

  for (target = head_handle; target; target = target->next) {
    if (target->enable) {
      /*
      More detail on tick-clock overflow, please see
      https://blog.csdn.net/szullc/article/details/115332326
      */
      if (_timer_ticks - target->cur_ticks >= target->cur_expired_time) {
        // printf("cur_ticks: %u, cur_expired_time: %u, _timer_ticks: %u\r\n",
        //        target->cur_ticks, target->cur_expired_time, _timer_ticks);
        if (target->repeat == 0) {
          timer_stop(target);
        } else {
          target->cur_ticks = _timer_ticks;
          target->cur_expired_time = target->repeat;
        }
        target->timeout_cb(target->arg);
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
