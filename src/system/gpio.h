/*!
 * \file      gpio.h
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
#ifndef __GPIO_H__
#define __GPIO_H__

#include <stdio.h>
#include <stdint.h>

int gpio_export(int gpio);
int gpio_unexport(int gpio);
int gpio_set_dir(int gpio, uint8_t out_flag);
int gpio_set_value(int gpio, uint8_t value);
int gpio_get_value(int gpio, uint8_t *value);
int gpio_set_edge(int gpio, char *edge);

int gpio_getfd_value(int fd);
int gpio_setfd_value(int fd, uint8_t value);
int gpio_fd_open(int gpio);
int gpio_fd_close(int fd);
#endif // __GPIO_H__
