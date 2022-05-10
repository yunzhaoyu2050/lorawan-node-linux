#ifndef __UTILES_H__
#define __UTILES_H__
#include <stdio.h>
#include <time.h>

void wait_ms(unsigned long a);
void wait_us(unsigned long a);
// inline int ts_diff(struct timespec *t1, struct timespec *t2) {
//   return (t1->tv_sec - t2->tv_sec) * 1000 +
//          (t1->tv_nsec - t2->tv_nsec) / 1000000;
// }
int ts_diff(struct timespec *t1, struct timespec *t2);
// inline int ts_diff_tv(struct timespec *t1, struct timespec *t2,
//                       struct timespec *ttmp) {
//   ttmp->tv_sec = (t1->tv_sec - t2->tv_sec);
//   ttmp->tv_nsec = (t1->tv_nsec - t2->tv_nsec);
//   return 0;
// }
#endif // __UTILES_H__