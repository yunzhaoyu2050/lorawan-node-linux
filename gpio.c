#include "log.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64

int gpio_export(unsigned int gpio) {
  int fd, len;
  char buf[MAX_BUF];
  fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  if (fd < 0) {
    log(ERROR, "gpio/export");
    return fd;
  }
  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);
  return 0;
}

int gpio_unexport(unsigned int gpio) {
  int fd, len;
  char buf[MAX_BUF];
  fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  if (fd < 0) {
    log(ERROR, "gpio/export");
    return fd;
  }
  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);
  return 0;
}

int gpio_set_dir(unsigned int gpio, unsigned int out_flag) {
  int fd;
  char buf[MAX_BUF];
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    log(ERROR, "gpio/direction");
    return fd;
  }
  if (out_flag)
    write(fd, "out", 4);
  else
    write(fd, "in", 3);
  close(fd);
  return 0;
}

int gpio_set_value(unsigned int gpio, unsigned int value) {
  int fd;
  char buf[MAX_BUF];
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    log(ERROR, "gpio/set-value");
    return fd;
  }
  if (value)
    write(fd, "1", 2);
  else
    write(fd, "0", 2);
  close(fd);
  return fd;
}

int gpio_setfd_value(int fd, unsigned int value) {
  int ret = -1;
  if (value)
    ret = write(fd, "1", 2);
  else
    ret = write(fd, "0", 2);
  return ret;
}

int gpio_get_value(unsigned int gpio, unsigned int *value) {
  int fd;
  char buf[MAX_BUF];
  char ch;
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_RDONLY);
  if (fd < 0) {
    log(ERROR, "gpio/get-value");
    return fd;
  }
  read(fd, &ch, 1);
  if (ch != '0')
    *value = 1;
  else
    *value = 0;
  close(fd);
  return fd;
}

#if 0
int gpio_getfd_value(int fd, unsigned int *value) {
    int ret = -1;
    char ch;
    ret = read(fd, &ch, 1);
    if (ch != '0')
        *value = 1;
    else
        *value = 0;
    return ret;
}
#else
int gpio_getfd_value(int fd) {
  char ch;
  int value;
  read(fd, &ch, 1);
  if (ch != '0')
    value = 1;
  else
    value = 0;
  return value;
}
#endif
int gpio_set_edge(unsigned int gpio, char *edge) {
  int fd;
  char buf[MAX_BUF];
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    log(ERROR, "gpio/set-edge");
    return fd;
  }
  write(fd, edge, strlen(edge) + 1);
  close(fd);
  return 0;
}

int gpio_fd_open(unsigned int gpio) {
  int fd;
  char buf[MAX_BUF];
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_RDWR);
  if (fd < 0)
    log(ERROR, "gpio/fd_open");
  return fd;
}

int gpio_fd_close(int fd) { return close(fd); }
