#include "log.h"
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

extern int g_cfgLogLevel;
extern char log_file_name[];

#define DATESIZE 32
int useSyslog = 0, syslog_opened = 0;
int useFilelog = 0, fd_filelog = 0, proc_fd = 0;

void LogOut(int level, char *format, ...)
{
  va_list va_ap;
  if (level <= g_cfgLogLevel)
  {
    char buf[2048];
    char out_buf[640];
    char theDate[DATESIZE];
    char *extra_msg = "";
    time_t theTime = time(NULL);
    strftime(theDate, DATESIZE, "%d/%b/%Y %H:%M:%S", localtime(&theTime));
    va_start(va_ap, format);
    int n = vsnprintf(buf, sizeof(buf) - 1, format, va_ap);
    va_end(va_ap);
    buf[n] = '\0';

    if (level == 0 /* TRACE_ERROR */)
      extra_msg = "ERROR";
    else if (level == 1 /* TRACE_WARNING */)
      extra_msg = "WARNING";
    else if (level == 2 /* TRACE_INFO */)
      extra_msg = "INFO";
    else if (level == 3 /* TRACE_DEBUG */)
      extra_msg = "DEBUG";

    while (buf[strlen(buf) - 1] == '\n')
      buf[strlen(buf) - 1] = '\0';
    if (useFilelog)
    {
      if (fd_filelog <= 0)
      {
        fd_filelog = open(log_file_name, O_WRONLY);
        if (fd_filelog > 0)
        {
          lseek(fd_filelog, 0, SEEK_END);
        }
        else
        {
          fd_filelog = open(log_file_name, O_WRONLY | O_CREAT, 666);
          if (fd_filelog < 0)
          {
            return;
          }
        }
        proc_fd = open("/proc/uptime", O_RDONLY);
      }
      char uptime[64];
      int uptime_len;
      lseek(proc_fd, 0, SEEK_SET);
      uptime_len = read(proc_fd, uptime, sizeof(uptime));
      if (uptime_len > 0)
      {
        char *p;
        uptime[uptime_len] = '\0';
        p = strchr(uptime, ' ');
        if (p != NULL)
        {
          *p = '\0';
        }
      }
      else
      {
        uptime[0] = '\0';
        close(proc_fd);
        proc_fd = open("/proc/uptime", O_RDONLY);
      }
      int len = snprintf(out_buf, sizeof(out_buf), "[%s][%s][%s]:%s\n", uptime,
                         theDate, extra_msg, buf);
      write(fd_filelog, out_buf, len);
      return;
    }
    if (useSyslog)
    {
      if (!syslog_opened)
      {
        openlog("gateway", LOG_PID, LOG_DAEMON);
        syslog_opened = 1;
      }

      snprintf(out_buf, sizeof(out_buf), "[%s]:%s", extra_msg, buf);
      syslog(LOG_INFO, "%s", out_buf);
    }
    else
    {
      snprintf(out_buf, sizeof(out_buf), "[%s][%s]%s", theDate, extra_msg, buf);
      printf("%s\n", out_buf);
      fflush(stdout);
    }
  }
}