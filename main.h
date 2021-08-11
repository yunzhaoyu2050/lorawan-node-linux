#ifndef _MAIN_H_
#define _MAIN_H_
#include <stdint.h>
#include "radio.h"
typedef struct {
  /* device config */
  int busy_gp;
  int rst_gp; // reset gpio
  int dio_gp[3];
  int spidev_pt;
  char spidev_path[32];
  /* radio config */
  RadioModems_t modem;
  uint32_t freq;
  uint32_t bw;
  uint8_t sf;
  uint8_t cr;
  uint8_t nocrc;
  uint8_t prlen;
  uint8_t syncword;
  uint8_t invertio;
  uint8_t power;
  uint8_t rxContinuous;
  char desc[12];
} radiodev_t;
#endif // _MAIN_H_