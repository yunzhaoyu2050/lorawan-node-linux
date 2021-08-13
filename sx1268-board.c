/*!
 * \file      sx1268-board.c
 *
 * \brief     Target board SX126x shield driver implementation
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
 *
 * \author    Forest-Rain
 */
#include "lora-radio-config.h"
#include "radio.h"
#include "sx126x-board.h"

#include "gpio.h"
#include "log.h"
#include "main.h"
#include "utiles.h"

int LORA_RADIO_RESET_PIN = -1;
int LORA_RADIO_DIO1_PIN = -1;
int LORA_RADIO_BUSY_PIN = -1;
int LORA_RADIO_RFSW1_PIN = -1;
int LORA_RADIO_RFSW2_PIN = -1;

void SX126xIoInit(void) {
  int ret = gpio_export(LORA_RADIO_RESET_PIN);
  if (ret < 0) {
    log(ERROR, "gpio (%d)_export failed\n", LORA_RADIO_RESET_PIN);
    return;
  }
  gpio_set_dir(LORA_RADIO_RESET_PIN, 1); // out
  gpio_set_value(LORA_RADIO_RESET_PIN, 1);

  ret = gpio_export(LORA_RADIO_BUSY_PIN);
  if (ret < 0) {
    log(ERROR, "gpio (%d)_export failed\n", LORA_RADIO_BUSY_PIN);
    return;
  }
  gpio_set_dir(LORA_RADIO_BUSY_PIN, 0); // in
}

void SX126xIoIrqInit(DioIrqHandler *dioIrq) {
  radiodev.dio1_callBack = dioIrq;
  int ret = gpio_export(LORA_RADIO_DIO1_PIN);
  if (ret < 0) {
    log(ERROR, "gpio (%d)_export failed\n", LORA_RADIO_DIO1_PIN);
    return;
  }
  gpio_set_dir(LORA_RADIO_DIO1_PIN, 0);
  gpio_set_edge(LORA_RADIO_DIO1_PIN, "rising");
}

void SX126xIoDeInit(void) {
  ////    GpioInit( &SX126x.Spi.Nss, RADIO_NSS, PIN_ANALOGIC, PIN_PUSH_PULL,
  /// PIN_PULL_UP, 1 ); /    GpioInit( &SX126x.BUSY, RADIO_BUSY, PIN_ANALOGIC,
  /// PIN_PUSH_PULL, PIN_NO_PULL, 0 ); /    GpioInit( &SX126x.DIO1, RADIO_DIO_1,
  /// PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

// void SX126xIoDbgInit(void) {
// #if defined(USE_RADIO_DEBUG)
//   GpioInit(&DbgPinTx, RADIO_DBG_PIN_TX, PIN_OUTPUT, PIN_PUSH_PULL,
//   PIN_NO_PULL,
//            0);
//   GpioInit(&DbgPinRx, RADIO_DBG_PIN_RX, PIN_OUTPUT, PIN_PUSH_PULL,
//   PIN_NO_PULL,
//            0);
// #endif
// }

void SX126xIoTcxoInit(void) {
  // Initialize TCXO control
  CalibrationParams_t calibParam;

  // +clear OSC_START_ERR for reboot or cold-start from sleep
  SX126xClearDeviceErrors();

  // TCXO_CTRL_1_7V -> TCXO_CTRL_2_7V 64*15.0625US
  SX126xSetDio3AsTcxoCtrl(TCXO_CTRL_2_7V,
                          320); // SX126xGetBoardTcxoWakeupTime( ) << 6 ); //
  // convert from ms to SX126x time base

  calibParam.Value = 0x7F;
  SX126xCalibrate(calibParam);
}

uint32_t SX126xGetBoardTcxoWakeupTime(void) { return BOARD_TCXO_WAKEUP_TIME; }

void SX126xReset(void) {
  log(INFO, "wait for reset...");
  gpio_set_value(LORA_RADIO_RESET_PIN, 0);
  wait_ms(20);
  gpio_set_value(LORA_RADIO_RESET_PIN, 1);
  wait_ms(10);
}

void SX126xWaitOnBusy(void) {
  unsigned int val = 0;
  gpio_get_value(LORA_RADIO_BUSY_PIN, &val);
  while (val == 1) {
    int i = 0;
    if (i >= 200000) {
      log(DEBUG, "[sx126x] SX126xWaitOnBusy failed.\n");
      return;
    }
    i++;
  }
  return;
}

void SX126xAntSwOn(void) {
  // No need
  gpio_set_value(LORA_RADIO_RFSW1_PIN, 1);
}

void SX126xAntSwOff(void) {
  ////GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_ANALOGIC, PIN_PUSH_PULL,
  /// PIN_NO_PULL, 0 );

#if defined(LORA_RADIO_RFSW1_PIN) && defined(LORA_RADIO_RFSW2_PIN)
  gpio_set_value(LORA_RADIO_RFSW1_PIN, 0);
  gpio_set_value(LORA_RADIO_RFSW2_PIN, 0);
#endif
}

void SX126xSetAntSw(RadioOperatingModes_t mode) {
  if (mode == MODE_TX) { // Transmit
    gpio_set_value(LORA_RADIO_RFSW1_PIN, 1);
    gpio_set_value(LORA_RADIO_RFSW2_PIN, 0);
  } else {
    gpio_set_value(LORA_RADIO_RFSW1_PIN, 0);
    gpio_set_value(LORA_RADIO_RFSW2_PIN, 1);
  }
}

bool SX126xCheckRfFrequency(uint32_t frequency) {
  // Implement check. Currently all frequencies are supported
  return true;
}
