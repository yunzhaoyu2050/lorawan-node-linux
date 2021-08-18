/*!
 * \file      sx126x-board.h
 *
 * \brief     Target board SX126x driver implementation
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
#ifndef __SX126x_BOARD_H__
#define __SX126x_BOARD_H__

#include "lora-radio-config.h"
#include "sx126x.h"
#include "utiles.h"
#include <stdbool.h>
#include <stdint.h>


/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
// #define BOARD_TCXO_WAKEUP_TIME 2

#if defined(CONFIG_LORA_RADIO_USE_TCXO) || defined(LORA_RADIO_USE_TCXO)
/*!
 * Radio complete Wake-up Time with TCXO stabilisation time
 */
#define BOARD_TCXO_WAKEUP_TIME 5 // [ms]
#else
/*!
 * Radio complete Wake-up Time with TCXO stabilisation time
 */
#define BOARD_TCXO_WAKEUP_TIME 0 // No Used
#endif

#define SX126X_DELAY_MS(ms) wait_ms(ms) // rt_thread_mdelay(ms)
#define SX126X_BLOCK_DELAY_1MS() wait_ms(1) // rt_hw_us_delay(999)

/*!
 * \brief Initializes the radio I/Os pins interface
 */
void SX126xIoInit(void);

/*!
 * \brief Initializes DIO IRQ handlers
 *
 * \param [IN] irqHandlers Array containing the IRQ callback functions
 */
void SX126xIoIrqInit(DioIrqHandler *dioIrq);

/*!
 * \brief De-initializes the radio I/Os pins interface.
 *
 * \remark Useful when going in MCU low power modes
 */
void SX126xIoDeInit(void);

/*!
 * \brief Initializes the TCXO power pin.
 */
void SX126xIoTcxoInit(void);

// /*!
//  * \brief Initializes the radio debug pins.
//  */
// void SX126xIoDbgInit(void);

/*!
 * \brief HW Reset of the radio
 */
void SX126xReset(void);

/*!
 * \brief Blocking loop to wait while the Busy pin in high
 */
void SX126xWaitOnBusy(void);

/*!
 * \brief Initializes the RF Switch I/Os pins interface
 */
void SX126xAntSwOn(void);

/*!
 * \brief De-initializes the RF Switch I/Os pins interface
 *
 * \remark Needed to decrease the power consumption in MCU low power modes
 */
void SX126xAntSwOff(void);

/*!
 * \brief Set the RF Switch I/Os pins interface
 */
void SX126xSetAntSw(RadioOperatingModes_t mode);

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool SX126xCheckRfFrequency(uint32_t frequency);

/*!
 * \brief Gets the Defines the time required for the TCXO to wakeup [ms].
 *
 * \retval time Board TCXO wakeup time in ms.
 */
uint32_t SX126xGetBoardTcxoWakeupTime(void);

#endif // __SX126x_BOARD_H__
