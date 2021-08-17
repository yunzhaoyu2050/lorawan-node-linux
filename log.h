#ifndef __LOG_H__
#define __LOG_H__
#include <stdio.h>
/* log level */
#define ERROR 0
#define WARN 1
#define INFO 2
#define DEBUG 3
#define ASSERT(cond)                                                           \
  if (!(cond)) {                                                               \
    printf("%s:%d\n", __FILE__, __LINE__);                                     \
    exit(1);                                                                   \
  }
void LogOut(int eventTraceLevel, char *format, ...);
// public function
#define log(LEVEL, fmt, ...)                                                   \
  LogOut(LEVEL, "[%s][%d][%s]: " fmt, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#endif // __LOG_H__