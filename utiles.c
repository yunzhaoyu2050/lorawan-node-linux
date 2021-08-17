#include "utiles.h"
#include <unistd.h>
#include <math.h>

void wait_ms(unsigned long a) {
#if _XOPEN_SOURCE >= 600 || _POSIX_C_SOURCE >= 200112L
  struct timespec dly;
  struct timespec rem;
  dly.tv_sec = a / 1000;
  dly.tv_nsec = ((long)a % 1000) * 1000000;
  if ((dly.tv_sec > 0) || ((dly.tv_sec == 0) && (dly.tv_nsec > 100000))) {
    clock_nanosleep(CLOCK_MONOTONIC, 0, &dly, &rem);
  }
  return;
#endif
  usleep(a * 1000);
}

void wait_us(unsigned long a) {
#if _XOPEN_SOURCE >= 600 || _POSIX_C_SOURCE >= 200112L
  struct timespec dly;
  struct timespec rem;
  dly.tv_sec = a / 1000000;
  dly.tv_nsec = ((long)a % 1000000) * 1000;
  if ((dly.tv_sec > 0) || ((dly.tv_sec == 0) && (dly.tv_nsec > 100000))) {
    clock_nanosleep(CLOCK_MONOTONIC, 0, &dly, &rem);
  }
  return;
#endif
  usleep(a);
}

int ts_diff(struct timespec *t1, struct timespec *t2) {
  return (t1->tv_sec - t2->tv_sec) * 1000 +
         (t1->tv_nsec - t2->tv_nsec) / 1000000;
}

TimerTime_t RtcTempCompensation(TimerTime_t period, float temperature) {
/*!
 * \brief Temperature coefficient of the clock source
 */
#define RTC_TEMP_COEFFICIENT                            ( -0.035f )

/*!
 * \brief Temperature coefficient deviation of the clock source
 */
#define RTC_TEMP_DEV_COEFFICIENT                        ( 0.0035f )

/*!
 * \brief Turnover temperature of the clock source
 */
#define RTC_TEMP_TURNOVER                               ( 25.0f )

/*!
 * \brief Turnover temperature deviation of the clock source
 */
#define RTC_TEMP_DEV_TURNOVER                           ( 5.0f )

  float k = RTC_TEMP_COEFFICIENT;
  float kDev = RTC_TEMP_DEV_COEFFICIENT;
  float t = RTC_TEMP_TURNOVER;
  float tDev = RTC_TEMP_DEV_TURNOVER;
  float interim = 0.0f;
  float ppm = 0.0f;

  if (k < 0.0f) {
    ppm = (k - kDev);
  } else {
    ppm = (k + kDev);
  }
  interim = (temperature - (t - tDev));
  ppm *= interim * interim;

  // Calculate the drift in time
  interim = ((float)period * ppm) / 1000000.0f;
  // Calculate the resulting time period
  interim += period;
  interim = floor(interim);

  if (interim < 0.0f) {
    interim = (float)period;
  }

  // Calculate the resulting period
  return (TimerTime_t)interim;
}
