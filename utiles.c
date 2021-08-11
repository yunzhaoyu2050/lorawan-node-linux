#include <time.h>
#include "utiles.h"

void wait_ms(unsigned long a) {
    struct timespec dly;
    struct timespec rem;
    dly.tv_sec = a / 1000;
    dly.tv_nsec = ((long)a % 1000) * 1000000;
    if ((dly.tv_sec > 0) || ((dly.tv_sec == 0) && (dly.tv_nsec > 100000))) {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &dly, &rem);
    }
    return;
}

void wait_us(unsigned long a) {
    struct timespec dly;
    struct timespec rem;
    dly.tv_sec = a / 1000000;
    dly.tv_nsec = ((long)a % 1000000) * 1000;
    if ((dly.tv_sec > 0) || ((dly.tv_sec == 0) && (dly.tv_nsec > 100000))) {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &dly, &rem);
    }
    return;
}
