#include "lora-spi-sx126x.h"
#include "log.h"
#include "sx126x-board.h"
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

void SX126xWakeup(void) {
  // // Wait for chip to be ready.
  // SX126xWaitOnBusy();

  // // Update operating mode context variable
  // SX126xSetOperatingMode(MODE_STDBY_RC);
}

int SX126xWriteCommand(RadioCommands_t command, uint8_t *buffer,
                       uint16_t size) {
  uint8_t out_buf[1024]; // max 1024 byte
  uint8_t command_size;
  struct spi_ioc_transfer k;
  int a;
  int i;

  /* prepare frame to be sent */
  out_buf[0] = command;
  size = (size > (1024 - 1)) ? (1024 - 1) : size;
  for (i = 0; i < size; i++) {
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
  a = ioctl(SX126x.spi, SPI_IOC_MESSAGE(1), &k);
  if (command != RADIO_SET_SLEEP)
    SX126xWaitOnBusy();
  /* determine return code */
  if (a != (int)k.len) {
    log(ERROR, "SX126xWriteCommand failed\n");
    return -1;
  } else {
    return 0;
  }
}

int SX126xReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size) {
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
  for (i = 0; i < size; i++) {
    out_buf[i + 2] = 0;
  }
  command_size = size + 2;

  /* I/O transaction */
  memset(&k, 0, sizeof(k)); /* clear k */
  k.tx_buf = (unsigned long)out_buf;
  k.rx_buf = (unsigned long)in_buf;
  k.len = command_size;
  k.cs_change = 0;
  a = ioctl(SX126x.spi, SPI_IOC_MESSAGE(1), &k);
  SX126xWaitOnBusy();
  /* determine return code */
  if (a != (int)k.len) {
    log(ERROR, "SX126xReadCommand failed\n");
    return -1;
  } else {
    for (i = 0; i < size; i++) {
      buffer[i] = in_buf[i + 2];
    }
    return 0;
  }
}

int SX126xWriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size) {
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
  for (i = 0; i < size; i++) {
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
  a = ioctl(SX126x.spi, SPI_IOC_MESSAGE(1), &k);
  SX126xWaitOnBusy();
  /* determine return code */
  if (a != (int)k.len) {
    log(ERROR, "SX126xWriteRegisters failed\n");
    return -1;
  } else {
    return 0;
  }
}

int SX126xWriteRegister(uint16_t address, uint8_t value) {
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
  a = ioctl(SX126x.spi, SPI_IOC_MESSAGE(1), &k);
  SX126xWaitOnBusy();
  /* determine return code */
  if (a != (int)k.len) {
    log(ERROR, "SX126xWriteRegister failed\n");
    return -1;
  } else {
    return 0;
  }
}

int SX126xReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size) {

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
  for (i = 0; i < size; i++) {
    out_buf[i + 4] = 0;
  }
  command_size = size + 4;

  /* I/O transaction */
  memset(&k, 0, sizeof(k)); /* clear k */
  k.tx_buf = (unsigned long)out_buf;
  k.rx_buf = (unsigned long)in_buf;
  k.len = command_size;
  k.cs_change = 0;
  a = ioctl(SX126x.spi, SPI_IOC_MESSAGE(1), &k);
  SX126xWaitOnBusy();
  /* determine return code */
  if (a != (int)k.len) {
    log(ERROR, "SX126xReadRegisters failed\n");
    return -1;
  } else {
    for (i = 0; i < size; i++) {
      buffer[i] = in_buf[i + 4];
    }
    return 0;
  }
}

int SX126xReadRegister(uint16_t address) {
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
  a = ioctl(SX126x.spi, SPI_IOC_MESSAGE(1), &k);
  SX126xWaitOnBusy();
  /* determine return code */
  if (a != (int)k.len) {
    log(ERROR, "SX126xReadRegister failed\n");
    return -1;
  } else {
    data = in_buf[command_size - 1];
    return data;
  }
}

int SX126xWriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {

  uint8_t out_buf[1024]; // max 1024 byte
  uint8_t command_size;
  struct spi_ioc_transfer k;
  int a;
  int i;
  /* prepare frame to be sent */
  out_buf[0] = RADIO_WRITE_BUFFER;
  out_buf[1] = offset;
  size = (size > (1024 - 2)) ? (1024 - 2) : size;
  for (i = 0; i < size; i++) {
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
  a = ioctl(SX126x.spi, SPI_IOC_MESSAGE(1), &k);
  SX126xWaitOnBusy();
  /* determine return code */
  if (a != (int)k.len) {
    log(ERROR, "SX126xWriteRegisters failed\n");
    return -1;
  } else {
    return 0;
  }
}

int SX126xReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
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
  for (i = 0; i < size; i++) {
    out_buf[i + 3] = 0;
  }
  command_size = size + 3;

  /* I/O transaction */
  memset(&k, 0, sizeof(k)); /* clear k */
  k.tx_buf = (unsigned long)out_buf;
  k.rx_buf = (unsigned long)in_buf;
  k.len = command_size;
  k.cs_change = 0;
  a = ioctl(SX126x.spi, SPI_IOC_MESSAGE(1), &k);
  SX126xWaitOnBusy();
  /* determine return code */
  if (a != (int)k.len) {
    log(ERROR, "SX126xReadRegisters failed\n");
    return -1;
  } else {
    for (i = 0; i < size; i++) {
      buffer[i] = in_buf[i + 3];
    }
    return 0;
  }
}

/* SPI initialization and configuration */
static int sx126xSpiOpen(char *spi_path) {
  int a = 0, b = 0;
  int i, spidev = 0;

  /* open SPI device */
  spidev = open(spi_path, O_RDWR);
  if (spidev < 0) {
    log(ERROR, "failed to open SPI device %s\n", spi_path);
    return -1;
  }

  /* setting SPI mode to 'mode 0' */
  i = SPI_MODE_0;
  a = ioctl(spidev, SPI_IOC_WR_MODE, &i);
  b = ioctl(spidev, SPI_IOC_RD_MODE, &i);
  if ((a < 0) || (b < 0)) {
    log(ERROR, "SPI PORT (%s) FAIL TO SET IN MODE 0\n", spi_path);
    close(spidev);
    return -1;
  }

  /* setting SPI max clk (in Hz) */
  i = SPI_SPEED;
  a = ioctl(spidev, SPI_IOC_WR_MAX_SPEED_HZ, &i);
  b = ioctl(spidev, SPI_IOC_RD_MAX_SPEED_HZ, &i);
  if ((a < 0) || (b < 0)) {
    log(ERROR, "SPI PORT (%s) FAIL TO SET MAX SPEED\n", spi_path);
    close(spidev);
    return -1;
  }

  /* setting SPI to MSB first */
  i = 0;
  a = ioctl(spidev, SPI_IOC_WR_LSB_FIRST, &i);
  b = ioctl(spidev, SPI_IOC_RD_LSB_FIRST, &i);
  if ((a < 0) || (b < 0)) {
    log(ERROR, "SPI PORT (%s) FAIL TO SET MSB FIRST\n", spi_path);
    close(spidev);
    return -1;
  }

  /* setting SPI to 8 bits per word */
  i = 0;
  a = ioctl(spidev, SPI_IOC_WR_BITS_PER_WORD, &i);
  b = ioctl(spidev, SPI_IOC_RD_BITS_PER_WORD, &i);
  if ((a < 0) || (b < 0)) {
    log(ERROR, "SPI PORT (%s) FAIL TO SET 8 BITS-PER-WORD\n", spi_path);
    close(spidev);
    return -1;
  }

  return spidev;
}

int lora_radio_spi_init(char *spi_dev_path) {
  if (spi_dev_path == NULL) {
    return -1;
  }
  // TODO: check path
  return sx126xSpiOpen(spi_dev_path);
}