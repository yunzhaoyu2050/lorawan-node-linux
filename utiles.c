#include "utiles.h"
#include <unistd.h>
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
