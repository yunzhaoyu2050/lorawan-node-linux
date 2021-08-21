/*!
 * \file      sx1262mbxdas-board.c
 *
 * \brief     Target board SX1262MBXDAS shield driver implementation
 *
 * \remark    This target board is only available with the SX126xDVK1xAS
 *            development kit.
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
#include <stdlib.h>
#include "utilities.h"
#include "board-config.h"
#include "board.h"
// #include "delay.h"
#include "radio.h"
#include "sx126x-board.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "utilities.h"

#include "log.h"

#if defined( USE_RADIO_DEBUG )
/*!
 * \brief Writes new Tx debug pin state
 *
 * \param [IN] state Debug pin state
 */
static void SX126xDbgPinTxWrite( uint8_t state );

/*!
 * \brief Writes new Rx debug pin state
 *
 * \param [IN] state Debug pin state
 */
static void SX126xDbgPinRxWrite( uint8_t state );
#endif

/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t OperatingMode;

/*!
 * Antenna switch GPIO pins objects
 */
// Gpio_t AntPow;
int DeviceSel = 2;

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif

void SX126xIoInit( void )
{
    // GpioInit( &SX126x.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    // GpioInit( &SX126x.BUSY, RADIO_BUSY, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // GpioInit( &SX126x.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // GpioInit( &DeviceSel, RADIO_DEVICE_SEL, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    int ret;
    /*
    ret = gpio_export(SX126x.Reset);
    if (ret < 0)
    {
        log(ERROR, "gpio (%d)_export failed\n", SX126x.Reset);
        return;
    }
    */
    gpio_export(SX126x.Reset);
    gpio_set_dir(SX126x.Reset, 1);
    gpio_set_value(SX126x.Reset, 1);

    ret = gpio_export(SX126x.BUSY);
    if (ret < 0)
    {
        log(ERROR, "gpio (%d)_export failed\n", SX126x.BUSY);
        return;
    }
    gpio_set_dir(SX126x.BUSY, 0);
    log(INFO, "SX126xIoInit success");
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    // GpioSetInterrupt( &SX126x.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, dioIrq );
    SX126x.dio1CallBack = dioIrq;
    int ret = gpio_export(SX126x.DIO1);
    if (ret < 0)
    {
        log(ERROR, "gpio (%d)_export failed\n", SX126x.DIO1);
        return;
    }
    gpio_set_dir(SX126x.DIO1, 0);
    gpio_set_edge(SX126x.DIO1, "rising");
    log(INFO, "SX126xIoIrqInit success");
}

void SX126xIoDeInit( void )
{
    // GpioInit( &SX126x.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    // GpioInit( &SX126x.BUSY, RADIO_BUSY, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // GpioInit( &SX126x.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    gpio_unexport(SX126x.Reset);
    gpio_unexport(SX126x.BUSY);
    gpio_unexport(SX126x.BUSY);
}

void SX126xIoDbgInit( void )
{
#if defined( USE_RADIO_DEBUG )
    GpioInit( &DbgPinTx, RADIO_DBG_PIN_TX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &DbgPinRx, RADIO_DBG_PIN_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SX126xIoTcxoInit( void )
{
    CalibrationParams_t calibParam;
    // +clear OSC_START_ERR for reboot or cold-start from sleep
    SX126xClearDeviceErrors();
    log(INFO, "SX126xClearDeviceErrors success.");
    #if 0
    SX126xSetDio3AsTcxoCtrl( TCXO_CTRL_1_7V, SX126xGetBoardTcxoWakeupTime( ) << 6 ); // convert from ms to SX126x time base
    log(INFO, "SX126xSetDio3AsTcxoCtrl success. wakeup time(%d)", SX126xGetBoardTcxoWakeupTime( ));
    calibParam.Value = 0x7F;
    SX126xCalibrate( calibParam );
    #endif
    log(INFO, "SX126xCalibrate success.");
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX126xIoRfSwitchInit( void )
{
    SX126xSetDio2AsRfSwitchCtrl( true );
}

RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}

void SX126xSetOperatingMode( RadioOperatingModes_t mode )
{
    OperatingMode = mode;
#if defined( USE_RADIO_DEBUG )
    switch( mode )
    {
        case MODE_TX:
            SX126xDbgPinTxWrite( 1 );
            SX126xDbgPinRxWrite( 0 );
            break;
        case MODE_RX:
        case MODE_RX_DC:
            SX126xDbgPinTxWrite( 0 );
            SX126xDbgPinRxWrite( 1 );
            break;
        default:
            SX126xDbgPinTxWrite( 0 );
            SX126xDbgPinRxWrite( 0 );
            break;
    }
#endif
}

void SX126xReset( void )
{
    // DelayMs( 10 );
    // GpioInit( &SX126x.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // DelayMs( 20 );
    // GpioInit( &SX126x.Reset, RADIO_RESET, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 ); // internal pull-up
    // DelayMs( 10 );
    log(INFO, "wait for reset...");
    gpio_set_value(SX126x.Reset, 0);
    wait_ms(20);
    gpio_set_value(SX126x.Reset, 1);
    wait_ms(10);
    log(INFO, "and trun up...");
}

void SX126xWaitOnBusy( void )
{
    // while( GpioRead( &SX126x.BUSY ) == 1 );
    uint8_t val = 0;
    gpio_get_value(SX126x.BUSY, &val);
    int i = 0;
    while (val == 1)
    {
        if (i >= 0xffffffff)
        {
            log(DEBUG, "[sx126x] SX126xWaitOnBusy failed.\n");
            return;
        }
        i++;
    }
    return;
}

void SX126xWakeup( void )
{
    CRITICAL_SECTION_BEGIN( );

    // GpioWrite( &SX126x.Spi.Nss, 0 );

    // SpiInOut( &SX126x.Spi, RADIO_GET_STATUS );
    // SpiInOut( &SX126x.Spi, 0x00 );

    // GpioWrite( &SX126x.Spi.Nss, 1 );

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    // Update operating mode context variable
    SX126xSetOperatingMode( MODE_STDBY_RC );

    CRITICAL_SECTION_END( );
}

#define SPI_SPEED 8000000

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    // SX126xCheckDeviceReady( );

    // GpioWrite( &SX126x.Spi.Nss, 0 );

    // SpiInOut( &SX126x.Spi, ( uint8_t )command );

    // for( uint16_t i = 0; i < size; i++ )
    // {
    //     SpiInOut( &SX126x.Spi, buffer[i] );
    // }

    // GpioWrite( &SX126x.Spi.Nss, 1 );

    // if( command != RADIO_SET_SLEEP )
    // {
    //     SX126xWaitOnBusy( );
    // }
    uint8_t out_buf[1024]; // max 1024 byte
    uint8_t command_size;
    struct spi_ioc_transfer k;
    int a;
    int i;

    /* prepare frame to be sent */
    out_buf[0] = command;
    size = (size > (1024 - 1)) ? (1024 - 1) : size;
    for (i = 0; i < size; i++)
    {
        out_buf[i + 1] = buffer[i];
    }
    command_size = size + 1;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)out_buf;
    k.len = command_size;
    k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    k.bits_per_word = 8;
    a = ioctl(SX126x.Spi, SPI_IOC_MESSAGE(1), &k);
    if (command != RADIO_SET_SLEEP)
        SX126xWaitOnBusy();
    /* determine return code */
    if (a != (int)k.len)
    {
        printf("ERROR SX126xWriteCommand failed\n");
        return ;
    }
    else
    {
        return ;
    }
}

uint8_t SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    // uint8_t status = 0;

    // SX126xCheckDeviceReady( );

    // GpioWrite( &SX126x.Spi.Nss, 0 );

    // SpiInOut( &SX126x.Spi, ( uint8_t )command );
    // status = SpiInOut( &SX126x.Spi, 0x00 );
    // for( uint16_t i = 0; i < size; i++ )
    // {
    //     buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    // }

    // GpioWrite( &SX126x.Spi.Nss, 1 );

    // SX126xWaitOnBusy( );

    // return status;

    uint8_t out_buf[1024];
    uint8_t command_size;
    uint8_t in_buf[1024]; // max 1024 byte
    struct spi_ioc_transfer k;
    int a;
    int i;
    /* prepare frame to be sent */
    out_buf[0] = command;
    out_buf[1] = 0;
    size = (size > (1024 - 2)) ? (1024 - 2) : size;
    for (i = 0; i < size; i++)
    {
        out_buf[i + 2] = 0;
    }
    command_size = size + 2;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)out_buf;
    k.rx_buf = (unsigned long)in_buf;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(SX126x.Spi, SPI_IOC_MESSAGE(1), &k);
    SX126xWaitOnBusy();
    /* determine return code */
    if (a != (int)k.len)
    {
        printf("ERROR SX126xReadCommand failed\n");
        return 0;
    }
    else
    {
        for (i = 0; i < size; i++)
        {
            buffer[i] = in_buf[i + 2];
        }
        return in_buf[0]; // TODO: 检查返回值是否移植正确
    }
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint8_t out_buf[1024]; // max 1024 byte
    uint8_t command_size;
    struct spi_ioc_transfer k;
    int a;
    int i;
    /* prepare frame to be sent */
    out_buf[0] = RADIO_WRITE_REGISTER;
    out_buf[1] = (address & 0xFF00) >> 8;
    out_buf[2] = address & 0x00FF;
    size = (size > (1024 - 3)) ? (1024 - 3) : size;
    for (i = 0; i < size; i++)
    {
        out_buf[i + 3] = buffer[i];
    }
    command_size = size + 3;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)out_buf;
    k.len = command_size;
    k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    k.bits_per_word = 8;
    a = ioctl(SX126x.Spi, SPI_IOC_MESSAGE(1), &k);
    SX126xWaitOnBusy();
    /* determine return code */
    if (a != (int)k.len)
    {
        printf("ERROR SX126xWriteRegisters failed\n");
        return;
    }
    else
    {
        return;
    }
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    // SX126xWriteRegisters( address, &value, 1 );
    uint8_t out_buf[4];
    uint8_t command_size;
    struct spi_ioc_transfer k;
    int a;

    /* prepare frame to be sent */
    out_buf[0] = RADIO_WRITE_REGISTER;
    out_buf[1] = (address & 0xFF00) >> 8;
    out_buf[2] = address & 0x00FF;
    out_buf[3] = value;
    command_size = 4;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)out_buf;
    k.len = command_size;
    k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    k.bits_per_word = 8;
    a = ioctl(SX126x.Spi, SPI_IOC_MESSAGE(1), &k);
    SX126xWaitOnBusy();
    /* determine return code */
    if (a != (int)k.len)
    {
        printf("ERROR SX126xWriteRegister failed\n");
        return;
    }
    else
    {
        return;
    }
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    // SX126xCheckDeviceReady( );

    // GpioWrite( &SX126x.Spi.Nss, 0 );

    // SpiInOut( &SX126x.Spi, RADIO_READ_REGISTER );
    // SpiInOut( &SX126x.Spi, ( address & 0xFF00 ) >> 8 );
    // SpiInOut( &SX126x.Spi, address & 0x00FF );
    // SpiInOut( &SX126x.Spi, 0 );
    // for( uint16_t i = 0; i < size; i++ )
    // {
    //     buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    // }
    // GpioWrite( &SX126x.Spi.Nss, 1 );

    // SX126xWaitOnBusy( );
    uint8_t out_buf[1024];
    uint8_t command_size;
    uint8_t in_buf[1024]; // max 1024 byte
    struct spi_ioc_transfer k;
    int a;
    int i;
    /* prepare frame to be sent */
    out_buf[0] = RADIO_READ_REGISTER;
    out_buf[1] = (address & 0xFF00) >> 8;
    out_buf[2] = address & 0x00FF;
    out_buf[3] = 0;
    size = (size > (1024 - 4)) ? (1024 - 4) : size;
    for (i = 0; i < size; i++)
    {
        out_buf[i + 4] = 0;
    }
    command_size = size + 4;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)out_buf;
    k.rx_buf = (unsigned long)in_buf;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(SX126x.Spi, SPI_IOC_MESSAGE(1), &k);
    SX126xWaitOnBusy();
    /* determine return code */
    if (a != (int)k.len)
    {
        printf("ERROR SX126xReadRegisters failed\n");
        return;
    }
    else
    {
        for (i = 0; i < size; i++)
        {
            buffer[i] = in_buf[i + 4];
        }
        return;
    }
}

uint8_t SX126xReadRegister( uint16_t address )
{
    // uint8_t data;
    // SX126xReadRegisters( address, &data, 1 );
    // return data;
    uint8_t data = 0;
    uint8_t out_buf[5];
    uint8_t command_size;
    uint8_t in_buf[10];
    struct spi_ioc_transfer k;
    int a;

    /* prepare frame to be sent */
    out_buf[0] = RADIO_READ_REGISTER;
    out_buf[1] = (address & 0xFF00) >> 8;
    out_buf[2] = address & 0x00FF;
    out_buf[3] = 0;
    out_buf[4] = 0;
    command_size = 5;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)out_buf;
    k.rx_buf = (unsigned long)in_buf;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(SX126x.Spi, SPI_IOC_MESSAGE(1), &k);
    SX126xWaitOnBusy();
    /* determine return code */
    if (a != (int)k.len)
    {
        printf("ERROR SX126xReadRegister failed\n");
        return 0;
    }
    else
    {
        data = in_buf[command_size - 1];
        return data;
    }
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    // SX126xCheckDeviceReady( );

    // GpioWrite( &SX126x.Spi.Nss, 0 );

    // SpiInOut( &SX126x.Spi, RADIO_WRITE_BUFFER );
    // SpiInOut( &SX126x.Spi, offset );
    // for( uint16_t i = 0; i < size; i++ )
    // {
    //     SpiInOut( &SX126x.Spi, buffer[i] );
    // }
    // GpioWrite( &SX126x.Spi.Nss, 1 );

    // SX126xWaitOnBusy( );
    uint8_t out_buf[1024]; // max 1024 byte
    uint8_t command_size;
    struct spi_ioc_transfer k;
    int a;
    int i;
    /* prepare frame to be sent */
    out_buf[0] = RADIO_WRITE_BUFFER;
    out_buf[1] = offset;
    size = (size > (1024 - 2)) ? (1024 - 2) : size;
    for (i = 0; i < size; i++)
    {
        out_buf[i + 2] = buffer[i];
    }
    command_size = size + 2;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)out_buf;
    k.len = command_size;
    k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    k.bits_per_word = 8;
    a = ioctl(SX126x.Spi, SPI_IOC_MESSAGE(1), &k);
    SX126xWaitOnBusy();
    /* determine return code */
    if (a != (int)k.len)
    {
        printf("ERROR SX126xWriteRegisters failed\n");
        return;
    }
    else
    {
        return;
    }
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    // SX126xCheckDeviceReady( );

    // GpioWrite( &SX126x.Spi.Nss, 0 );

    // SpiInOut( &SX126x.Spi, RADIO_READ_BUFFER );
    // SpiInOut( &SX126x.Spi, offset );
    // SpiInOut( &SX126x.Spi, 0 );
    // for( uint16_t i = 0; i < size; i++ )
    // {
    //     buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    // }
    // GpioWrite( &SX126x.Spi.Nss, 1 );

    // SX126xWaitOnBusy( );
    uint8_t out_buf[1024];
    uint8_t command_size;
    uint8_t in_buf[1024]; // max 1024 byte
    struct spi_ioc_transfer k;
    int a;
    int i;
    /* prepare frame to be sent */
    out_buf[0] = RADIO_READ_BUFFER;
    out_buf[1] = offset;
    out_buf[2] = 0;
    size = (size > (1024 - 3)) ? (1024 - 3) : size;
    for (i = 0; i < size; i++)
    {
        out_buf[i + 3] = 0;
    }
    command_size = size + 3;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)out_buf;
    k.rx_buf = (unsigned long)in_buf;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(SX126x.Spi, SPI_IOC_MESSAGE(1), &k);
    SX126xWaitOnBusy();
    /* determine return code */
    if (a != (int)k.len)
    {
        printf("ERROR SX126xReadRegisters failed\n");
        return;
    }
    else
    {
        for (i = 0; i < size; i++)
        {
            buffer[i] = in_buf[i + 3];
        }
        return;
    }
}

int SX126xSpiOpen(char *spi_path)
{
    int a = 0, b = 0;
    int i, spidev = 0;

    /* open SPI device */
    spidev = open(spi_path, O_RDWR);
    if (spidev < 0)
    {
        log(ERROR, "failed to open SPI device %s\n", spi_path);
        return -1;
    }

    /* setting SPI mode to 'mode 0' */
    i = SPI_MODE_0;
    a = ioctl(spidev, SPI_IOC_WR_MODE, &i);
    b = ioctl(spidev, SPI_IOC_RD_MODE, &i);
    if ((a < 0) || (b < 0))
    {
        log(ERROR, "SPI PORT (%s) FAIL TO SET IN MODE 0\n", spi_path);
        close(spidev);
        return -1;
    }

    /* setting SPI max clk (in Hz) */
    i = SPI_SPEED;
    a = ioctl(spidev, SPI_IOC_WR_MAX_SPEED_HZ, &i);
    b = ioctl(spidev, SPI_IOC_RD_MAX_SPEED_HZ, &i);
    if ((a < 0) || (b < 0))
    {
        log(ERROR, "SPI PORT (%s) FAIL TO SET MAX SPEED\n", spi_path);
        close(spidev);
        return -1;
    }

    /* setting SPI to MSB first */
    i = 0;
    a = ioctl(spidev, SPI_IOC_WR_LSB_FIRST, &i);
    b = ioctl(spidev, SPI_IOC_RD_LSB_FIRST, &i);
    if ((a < 0) || (b < 0))
    {
        log(ERROR, "SPI PORT (%s) FAIL TO SET MSB FIRST\n", spi_path);
        close(spidev);
        return -1;
    }

    /* setting SPI to 8 bits per word */
    i = 0;
    a = ioctl(spidev, SPI_IOC_WR_BITS_PER_WORD, &i);
    b = ioctl(spidev, SPI_IOC_RD_BITS_PER_WORD, &i);
    if ((a < 0) || (b < 0))
    {
        log(ERROR, "SPI PORT (%s) FAIL TO SET 8 BITS-PER-WORD\n", spi_path);
        close(spidev);
        return -1;
    }

    return spidev;
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetDeviceId( void )
{
    // if( GpioRead( &DeviceSel ) == 1 )
    if( DeviceSel == 1 )
    {
        return SX1261;
    }
    else
    {
        return SX1262;
    }
}

void SX126xAntSwOn( void )
{
    // GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    gpio_set_value(SX126x.rfsw1, 1);
}

void SX126xAntSwOff( void )
{
    // GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    gpio_set_value(SX126x.rfsw1, 0);
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

uint32_t SX126xGetDio1PinState( void )
{
    // return GpioRead( &SX126x.DIO1 );
    uint8_t val = 0;
    gpio_get_value(SX126x.DIO1, &val);
    return (uint32_t)val;
}

#if defined( USE_RADIO_DEBUG )
static void SX126xDbgPinTxWrite( uint8_t state )
{
    GpioWrite( &DbgPinTx, state );
}

static void SX126xDbgPinRxWrite( uint8_t state )
{
    GpioWrite( &DbgPinRx, state );
}
#endif
