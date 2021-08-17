/*!
 * \file      nvmm.c
 *
 * \brief     None volatile memory management module
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
 *              (C)2013-2020 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 */

#include "utilities.h"
#include <fcntl.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// #include "eeprom-board.h"
#include "nvmm.h"

uint16_t NvmmWrite(uint8_t *src, uint16_t size, uint16_t offset) {
  int nvmm_fd = open(LORAWAN_NODE_NVMM_FILE_PATH, O_CREAT | O_WRONLY);
  if (nvmm_fd < 0)
    return 0;
  int ret = lseek(nvmm_fd, offset, SEEK_SET);
  if (ret < 0)
    return 0;
  printf("NvmmWrite:%s, %d\r\n", src, size);
  ret = write(nvmm_fd, src, size);
  if (ret < 0)
    return 0;
  close(nvmm_fd);
  return ret;
  // if (EepromMcuWriteBuffer(offset, src, size) == LMN_STATUS_OK) {
  //   return size;
  // }
  // return 0;
}

uint16_t NvmmRead(uint8_t *dest, uint16_t size, uint16_t offset) {
	int nvmm_fd = open(LORAWAN_NODE_NVMM_FILE_PATH, O_CREAT | O_RDONLY);
  if (nvmm_fd < 0)
    return 0;
  int ret = lseek(nvmm_fd, offset, SEEK_SET);
  if (ret < 0)
    return 0;
  ret = read(nvmm_fd, dest, size);
  if (ret < 0)
    return 0;
  close(nvmm_fd);
  return ret;
  // if (EepromMcuReadBuffer(offset, dest, size) == LMN_STATUS_OK) {
  //   return size;
  // }
  // return 0;
}

bool NvmmCrc32Check(uint16_t size, uint16_t offset) {
  uint16_t i;
  uint8_t data = 0;
  uint32_t calculatedCrc32 = 0;
  uint32_t readCrc32 = 0;

  if (NvmmRead((uint8_t *)&readCrc32, sizeof(readCrc32),
               (offset + (size - sizeof(readCrc32)))) == sizeof(readCrc32)) {
    // Calculate crc
    calculatedCrc32 = Crc32Init();
    for (i = 0; i < (size - sizeof(readCrc32)); i++) {
      if (NvmmRead(&data, 1, offset + i) != 1) {
        return false;
      }
      calculatedCrc32 = Crc32Update(calculatedCrc32, &data, 1);
    }
    calculatedCrc32 = Crc32Finalize(calculatedCrc32);

    if (calculatedCrc32 != readCrc32) {
      return false;
    }
  }
  return true;
}

bool NvmmReset(uint16_t size, uint16_t offset) {
	uint32_t crc32 = 0;
  int nvmm_fd = open(LORAWAN_NODE_NVMM_FILE_PATH, O_CREAT | O_WRONLY);
  if (nvmm_fd < 0)
    return 0;
  int ret = lseek(nvmm_fd, offset + size - sizeof(crc32), SEEK_SET);
  if (ret < 0)
    return 0;
  ret = write(nvmm_fd, (uint8_t *)&crc32, size);
  if (ret < 0)
    return 0;
  close(nvmm_fd);
  return ret;
  // uint32_t crc32 = 0;

  // if (EepromMcuWriteBuffer(offset + size - sizeof(crc32), (uint8_t *)&crc32,
  //                          sizeof(crc32)) == LMN_STATUS_OK) {
  //   return true;
  // }
  // return false;
}
