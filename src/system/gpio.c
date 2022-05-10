/*!
 * \file      gpio.c
 *
 * \brief     GPIO driver implementation
 *
 * \remark: Relies on the specific board GPIO implementation as well as on
 *          IO expander driver implementation if one is available on the target
 *          board.
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
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "gpio.h"
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64

int gpio_export(int gpio) {
  if (gpio < 0)
    return -1;
  int fd, len;
  char buf[MAX_BUF];
  fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  if (fd < 0) {
    printf("ERROR gpio/export\r\n");
    return fd;
  }
  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);
  return 0;
}

int gpio_unexport(int gpio) {
  if (gpio < 0)
    return -1;
  int fd, len;
  char buf[MAX_BUF];
  fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  if (fd < 0) {
    printf("ERROR gpio/export\r\n");
    return fd;
  }
  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);
  return 0;
}

int gpio_set_dir(int gpio, uint8_t out_flag) {
  if (gpio < 0)
    return -1;
  int fd;
  char buf[MAX_BUF];
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    printf("ERROR gpio/direction\r\n");
    return fd;
  }
  if (out_flag)
    write(fd, "out", 4);
  else
    write(fd, "in", 3);
  close(fd);
  return 0;
}

int gpio_set_value(int gpio, uint8_t value) {
  if (gpio < 0)
    return -1;
  int fd;
  char buf[MAX_BUF];
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    printf("ERROR gpio/set-value\r\n");
    return fd;
  }
  if (value)
    write(fd, "1", 2);
  else
    write(fd, "0", 2);
  close(fd);
  return fd;
}

int gpio_setfd_value(int fd, uint8_t value) {
  int ret = -1;
  if (value)
    ret = write(fd, "1", 2);
  else
    ret = write(fd, "0", 2);
  return ret;
}

int gpio_get_value(int gpio, uint8_t *value) {
  if (gpio < 0)
    return -1;
  int fd;
  char buf[MAX_BUF];
  char ch;
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_RDONLY);
  if (fd < 0) {
    printf("ERROR gpio/get-value\r\n");
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
int gpio_set_edge(int gpio, char *edge) {
  if (gpio < 0)
    return -1;
  int fd;
  char buf[MAX_BUF];
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    printf("ERROR gpio/set-edge\r\n");
    return fd;
  }
  write(fd, edge, strlen(edge) + 1);
  close(fd);
  return 0;
}

int gpio_fd_open(int gpio) {
  if (gpio < 0)
    return -1;
  int fd;
  char buf[MAX_BUF];
  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_RDWR);
  if (fd < 0)
    printf("ERROR gpio/fd_open\r\n");
  return fd;
}

int gpio_fd_close(int fd) { return close(fd); }
