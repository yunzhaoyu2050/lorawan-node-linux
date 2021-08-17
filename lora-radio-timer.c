#include "lora-radio-timer.h"
#include "lora-radio-config.h"
#include <stdint.h>

#include "utiles.h"

TimerTime_t TimerGetCurrentTime(void) {
  struct timespec tn = {0};
  clock_gettime(CLOCK_MONOTONIC, &tn);
  return ((tn.tv_sec) * 1000 + (tn.tv_nsec) / 1000000);
}

TimerTime_t TimerGetElapsedTime(TimerTime_t past) {
  return (TimerGetCurrentTime() - past);
}

TimerTime_t TimerTempCompensation(TimerTime_t period, float temperature) {
  return RtcTempCompensation(period, temperature);
}
