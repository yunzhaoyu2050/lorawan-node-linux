#ifndef __GPIO_H__
#define __GPIO_H__
#include <stdio.h>
int gpio_export(unsigned int gpio);
int gpio_unexport(unsigned int gpio);
int gpio_set_dir(unsigned int gpio, unsigned int out_flag);
int gpio_set_value(unsigned int gpio, unsigned int value);
int gpio_setfd_value(int fd, unsigned int value);
int gpio_get_value(unsigned int gpio, unsigned int *value);
int gpio_getfd_value(int fd);
int gpio_set_edge(unsigned int gpio, char *edge);
int gpio_fd_open(unsigned int gpio);
int gpio_fd_close(int fd);
#endif // __GPIO_H__
