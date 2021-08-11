/*!
 * \file      main.c
 *
 * \brief     FUOTA interop tests - test 01
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
 *              (C)2013-2018 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 */

/*! \file fuota-test-01/NucleoL152/main.c */

#include "./common/githubVersion.h"
#include "RegionCommon.h"
#include "board.h"
#include "firmwareVersion.h"
#include "gpio.h"
#include "uart.h"
#include "utilities.h"

#include "Commissioning.h"
#include "LmHandler.h"
#include "LmHandlerMsgDisplay.h"
#include "LmhpClockSync.h"
#include "LmhpCompliance.h"
#include "LmhpFragmentation.h"
#include "LmhpRemoteMcastSetup.h"
#include "cli.h"

#include "log.h"
#include "radio.h"
#include "sx126x-board.h"
#include "timer.h"
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#ifndef ACTIVE_REGION

#warning                                                                       \
    "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * LoRaWAN default end-device class
 */
#define LORAWAN_DEFAULT_CLASS CLASS_A

/*!
 * Defines the application data transmission duty cycle. 40s, value in [ms].
 */
#define APP_TX_DUTYCYCLE 40000

/*!
 * Defines a random delay for application data transmission duty cycle. 5s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND 5000

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAMAC_HANDLER_ADR_ON

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is
 * disabled
 */
#define LORAWAN_DEFAULT_DATARATE DR_3

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE 242

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only
 * for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON true

/*!
 *
 */
typedef enum {
  LORAMAC_HANDLER_TX_ON_TIMER,
  LORAMAC_HANDLER_TX_ON_EVENT,
} LmHandlerTxEvents_t;

/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED beacon indicator
 */
static TimerEvent_t LedBeaconTimer;

static void OnMacProcessNotify(void);
static void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size);
static void OnNetworkParametersChange(CommissioningParams_t *params);
static void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq,
                             TimerTime_t nextTxIn);
static void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq,
                             TimerTime_t nextTxIn);
static void OnJoinRequest(LmHandlerJoinParams_t *params);
static void OnTxData(LmHandlerTxParams_t *params);
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);
static void OnClassChange(DeviceClass_t deviceClass);
static void OnBeaconStatusChange(LoRaMacHandlerBeaconParams_t *params);
#if (LMH_SYS_TIME_UPDATE_NEW_API == 1)
static void OnSysTimeUpdate(bool isSynchronized, int32_t timeCorrection);
#else
static void OnSysTimeUpdate(void);
#endif
#if (FRAG_DECODER_FILE_HANDLING_NEW_API == 1)
static int8_t FragDecoderWrite(uint32_t addr, uint8_t *data, uint32_t size);
static int8_t FragDecoderRead(uint32_t addr, uint8_t *data, uint32_t size);
#endif
static void OnFragProgress(uint16_t fragCounter, uint16_t fragNb,
                           uint8_t fragSize, uint16_t fragNbLost);
#if (FRAG_DECODER_FILE_HANDLING_NEW_API == 1)
static void OnFragDone(int32_t status, uint32_t size);
#else
static void OnFragDone(int32_t status, uint8_t *file, uint32_t size);
#endif
static void StartTxProcess(LmHandlerTxEvents_t txEvent);
static void UplinkProcess(void);

static void OnTxPeriodicityChanged(uint32_t periodicity);
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent(void *context);

/*!
 * Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent(void *context);

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent(void *context);

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent(void *context);

static LmHandlerCallbacks_t LmHandlerCallbacks = {
    .GetBatteryLevel = BoardGetBatteryLevel,
    .GetTemperature = NULL,
    .GetRandomSeed = BoardGetRandomSeed,
    .OnMacProcess = OnMacProcessNotify,
    .OnNvmDataChange = OnNvmDataChange,
    .OnNetworkParametersChange = OnNetworkParametersChange,
    .OnMacMcpsRequest = OnMacMcpsRequest,
    .OnMacMlmeRequest = OnMacMlmeRequest,
    .OnJoinRequest = OnJoinRequest,
    .OnTxData = OnTxData,
    .OnRxData = OnRxData,
    .OnClassChange = OnClassChange,
    .OnBeaconStatusChange = OnBeaconStatusChange,
    .OnSysTimeUpdate = OnSysTimeUpdate,
};

static LmHandlerParams_t LmHandlerParams = {
    .Region = ACTIVE_REGION,
    .AdrEnable = LORAWAN_ADR_STATE,
    .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
    .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
    .DataBuffer = AppDataBuffer,
    .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
};

static LmhpComplianceParams_t LmhpComplianceParams = {
    .FwVersion.Value = FIRMWARE_VERSION,
    .OnTxPeriodicityChanged = OnTxPeriodicityChanged,
    .OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
    .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
};

/*!
 * Defines the maximum size for the buffer receiving the fragmentation result.
 *
 * \remark By default FragDecoder.h defines:
 *         \ref FRAG_MAX_NB   21
 *         \ref FRAG_MAX_SIZE 50
 *
 *         FileSize = FRAG_MAX_NB * FRAG_MAX_SIZE
 *
 *         If bigger file size is to be received or is fragmented differently
 *         one must update those parameters.
 */
#define UNFRAGMENTED_DATA_SIZE (21 * 50)

/*
 * Un-fragmented data storage.
 */
static uint8_t UnfragmentedData[UNFRAGMENTED_DATA_SIZE];

static LmhpFragmentationParams_t FragmentationParams = {
#if (FRAG_DECODER_FILE_HANDLING_NEW_API == 1)
    .DecoderCallbacks =
        {
            .FragDecoderWrite = FragDecoderWrite,
            .FragDecoderRead = FragDecoderRead,
        },
#else
    .Buffer = UnfragmentedData,
    .BufferSize = UNFRAGMENTED_DATA_SIZE,
#endif
    .OnProgress = OnFragProgress,
    .OnDone = OnFragDone};

/*!
 * Indicates if LoRaMacProcess call is pending.
 *
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;

static volatile uint8_t IsTxFramePending = 0;

static volatile uint32_t TxPeriodicity = 0;

/*
 * Indicates if the system time has been synchronized
 */
static volatile bool IsClockSynched = false;

/*
 * MC Session Started
 */
static volatile bool IsMcSessionStarted = false;

/*
 * Indicates if the file transfer is done
 */
static volatile bool IsFileTransferDone = false;

/*
 *  Received file computed CRC32
 */
static volatile uint32_t FileRxCrc = 0;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1; // Tx
extern Gpio_t Led2; // Rx

/*!
 * UART object used for command line interface handling
 */
extern Uart_t Uart2;

static const char *g_spi_device = "/dev/spidev1.1";
static unsigned int g_loop_tv = 0;

static void print_usage(const char *prog) {
  printf("Usage: %s [-dth]\n", prog);
  puts("  -d --device   device to use (default /dev/spidev1.1)\n"
       "  -t --timer    timer timing value (default 10ms)\n"
       "  -h            help\n");
  exit(1);
}

static void parse_opts(int argc, char *argv[]) {
  while (1) {
    static const struct option lopts[] = {
        {"device", 1, 0, 'd'},
        {"timer", 1, 0, 't'},
        {NULL, 0, 0, 0},
    };
    int c;
    c = getopt_long(argc, argv, "d:t:h", lopts, NULL);
    if (c == -1)
      break;

    switch (c) {
    case 'd':
      g_spi_device = optarg;
      break;
    case 't':
      g_loop_tv = atoi(optarg);
      break;
    default:
      print_usage(argv[0]);
      break;
    }
  }
}

// test
LORA_RADIO_RESET_PIN = 0;
LORA_RADIO_BUSY_PIN = 0;
LORA_RADIO_DIO1_PIN = 0;

radiodev_t radiodev; // 对接成全局参数以供多个模块使用

/*!
 * Main application entry point.
 */
int main(int argc, char *argv[]) {
  parse_opts(argc, argv);

  radiodev.rst_gp = LORA_RADIO_RESET_PIN;
  radiodev.busy_gp = LORA_RADIO_BUSY_PIN;
  radiodev.dio_gp[0] = LORA_RADIO_DIO1_PIN;
  memset(radiodev.spidev_path, 0, sizeof(radiodev.spidev_path));
  strncpy(radiodev.spidev_path, g_spi_device, sizeof(radiodev.spidev_path));
  radiodev.spidev_pt = lora_radio_spi_init((char *)g_spi_device);
  if (radiodev.spidev_pt < 0) {
    log(ERROR, "spidev is error path, please check.(input:%s)", g_spi_device);
    goto exit0;
  }

  unsigned char keep_running = 1;
  int max_fd = 0;
  fd_set fd_mask;
  FD_ZERO(&fd_mask);
  // add dio1 gpio
  int dio1_fd = gpio_fd_open(radiodev.dio_gp[0]);
  if (dio1_fd < 0) {
    log(ERROR, "dio1 gpio fd open failed.");
    goto exit1;
  }
  FD_SET(dio1_fd, &fd_mask);
  max_fd = MAX(max_fd, dio1_fd);
  // BoardInitMcu();
  // BoardInitPeriph();

  TimerInit(&Led1Timer, OnLed1TimerEvent, NULL);
  TimerSetValue(&Led1Timer, 0, 25);

  TimerInit(&Led2Timer, OnLed2TimerEvent, NULL);
  TimerSetValue(&Led2Timer, 0, 100);

  TimerInit(&LedBeaconTimer, OnLedBeaconTimerEvent, NULL);
  TimerSetValue(&LedBeaconTimer, 0, 5000);

  // Initialize transmission periodicity variable
  TxPeriodicity =
      APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);

  const Version_t appVersion = {.Value = FIRMWARE_VERSION};
  const Version_t gitHubVersion = {.Value = GITHUB_VERSION};
  DisplayAppInfo("fuota-test-01", &appVersion, &gitHubVersion);

  if (LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams) !=
      LORAMAC_HANDLER_SUCCESS) {
    printf("LoRaMac wasn't properly initialized\n");
    // Fatal error, endless loop.
    while (1) {
    }
  }

  // Set system maximum tolerated rx error in milliseconds
  LmHandlerSetSystemMaxRxError(20);

  // The LoRa-Alliance Compliance protocol package should always be
  // initialized and activated.
  LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams);
  LmHandlerPackageRegister(PACKAGE_ID_CLOCK_SYNC, NULL);
  LmHandlerPackageRegister(PACKAGE_ID_REMOTE_MCAST_SETUP, NULL);
  LmHandlerPackageRegister(PACKAGE_ID_FRAGMENTATION, &FragmentationParams);

  IsClockSynched = false;
  IsFileTransferDone = false;

  LmHandlerJoin();

  StartTxProcess(LORAMAC_HANDLER_TX_ON_TIMER);

  time_t lastTime = 0;
  struct timespec last_ts = {0};
  struct timespec now_ts = {0};

  struct timeval timeout = {0};
  timeout.tv_sec = 0;
  timeout.tv_usec = g_loop_tv; // 10ms loop
  while (keep_running) {
    int ret = select(max_fd + 1, &fd_mask, NULL, NULL, &timeout);
    if (ret < 0) {
      log(ERROR, "select error ret = %d", ret);
      keep_running = 0;
      continue;
    } else if (ret == 0) {
      // timeout
    } else {
      // dio1 irq
      if (FD_ISSET(dio1_fd, &fd_mask)) {
        // dio1 irq callback function
        srdio1.callBack();
      }
    }
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    int sums = 0;
    int tms = ts_diff(&now_ts, &last_ts);
    // TODO: check 100ms is on timer process
    if (tms >= 10 && tms < 100) {
      // timer.h -> #define CFG_TIMER_1_TICK_N_MS   10
      timer_ticks(); // 10ms, 1 tick
      timer_loop();  // 10ms loop check
      sums += tms;
    } else if (sums >= 100) { // 100 ms process
      // Process characters sent over the command line interface
      CliProcess(&Uart2);
      // Processes the LoRaMac events
      LmHandlerProcess();
      // Process application uplinks management
      UplinkProcess();
      CRITICAL_SECTION_BEGIN();
      if (IsMacProcessPending == 1) {
        // Clear flag and prevent MCU to go into low power modes.
        IsMacProcessPending = 0;
      } else {
        // The MCU wakes up through events
        BoardLowPowerHandler();
      }
      CRITICAL_SECTION_END();
      sums = 0;
    }
    last_ts = now_ts;
  }
  close(dio1_fd);
  close(radiodev.spidev_pt);
  return 0;
exit2:
  close(dio1_fd);
exit1:
  close(radiodev.spidev_pt);
exit0:
  return -1;
}

static void OnMacProcessNotify(void) { IsMacProcessPending = 1; }

static void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size) {
  DisplayNvmDataChange(state, size);
}

static void OnNetworkParametersChange(CommissioningParams_t *params) {
  DisplayNetworkParametersUpdate(params);
}

static void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq,
                             TimerTime_t nextTxIn) {
  DisplayMacMcpsRequestUpdate(status, mcpsReq, nextTxIn);
}

static void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq,
                             TimerTime_t nextTxIn) {
  DisplayMacMlmeRequestUpdate(status, mlmeReq, nextTxIn);
}

static void OnJoinRequest(LmHandlerJoinParams_t *params) {
  DisplayJoinRequestUpdate(params);
  if (params->Status == LORAMAC_HANDLER_ERROR) {
    LmHandlerJoin();
  } else {
    LmHandlerRequestClass(LORAWAN_DEFAULT_CLASS);
  }
}

static void OnTxData(LmHandlerTxParams_t *params) { DisplayTxUpdate(params); }

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params) {
  DisplayRxUpdate(appData, params);
}

static void OnClassChange(DeviceClass_t deviceClass) {
  DisplayClassUpdate(deviceClass);

  switch (deviceClass) {
  default:
  case CLASS_A: {
    IsMcSessionStarted = false;
    break;
  }
  case CLASS_B: {
    // Inform the server as soon as possible that the end-device has switched to
    // ClassB
    LmHandlerAppData_t appData = {
        .Buffer = NULL,
        .BufferSize = 0,
        .Port = 0,
    };
    LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG);
    IsMcSessionStarted = true;
    break;
  }
  case CLASS_C: {
    IsMcSessionStarted = true;
    // Switch LED 2 ON
    GpioWrite(&Led2, 1);
    break;
  }
  }
}

static void OnBeaconStatusChange(LoRaMacHandlerBeaconParams_t *params) {
  switch (params->State) {
  case LORAMAC_HANDLER_BEACON_RX: {
    TimerStart(&LedBeaconTimer);
    break;
  }
  case LORAMAC_HANDLER_BEACON_LOST:
  case LORAMAC_HANDLER_BEACON_NRX: {
    TimerStop(&LedBeaconTimer);
    break;
  }
  default: {
    break;
  }
  }

  DisplayBeaconUpdate(params);
}

#if (LMH_SYS_TIME_UPDATE_NEW_API == 1)
static void OnSysTimeUpdate(bool isSynchronized, int32_t timeCorrection) {
  IsClockSynched = isSynchronized;
}
#else
static void OnSysTimeUpdate(void) { IsClockSynched = true; }
#endif

#if (FRAG_DECODER_FILE_HANDLING_NEW_API == 1)
static int8_t FragDecoderWrite(uint32_t addr, uint8_t *data, uint32_t size) {
  if (size >= UNFRAGMENTED_DATA_SIZE) {
    return -1; // Fail
  }
  for (uint32_t i = 0; i < size; i++) {
    UnfragmentedData[addr + i] = data[i];
  }
  return 0; // Success
}

static int8_t FragDecoderRead(uint32_t addr, uint8_t *data, uint32_t size) {
  if (size >= UNFRAGMENTED_DATA_SIZE) {
    return -1; // Fail
  }
  for (uint32_t i = 0; i < size; i++) {
    data[i] = UnfragmentedData[addr + i];
  }
  return 0; // Success
}
#endif

static void OnFragProgress(uint16_t fragCounter, uint16_t fragNb,
                           uint8_t fragSize, uint16_t fragNbLost) {
  // Switch LED 2 OFF for each received downlink
  GpioWrite(&Led2, 0);
  TimerStart(&Led2Timer);

  printf("\n###### =========== FRAG_DECODER ============ ######\n");
  printf("######               PROGRESS                ######\n");
  printf("###### ===================================== ######\n");
  printf("RECEIVED    : %5d / %5d Fragments\n", fragCounter, fragNb);
  printf("              %5d / %5d Bytes\n", fragCounter * fragSize,
         fragNb * fragSize);
  printf("LOST        :       %7d Fragments\n\n", fragNbLost);
}

#if (FRAG_DECODER_FILE_HANDLING_NEW_API == 1)
static void OnFragDone(int32_t status, uint32_t size) {
  FileRxCrc = Crc32(UnfragmentedData, size);
  IsFileTransferDone = true;
  // Switch LED 2 OFF
  GpioWrite(&Led2, 0);

  printf("\n###### =========== FRAG_DECODER ============ ######\n");
  printf("######               FINISHED                ######\n");
  printf("###### ===================================== ######\n");
  printf("STATUS      : %ld\n", status);
  printf("CRC         : %08lX\n\n", FileRxCrc);
}
#else
static void OnFragDone(int32_t status, uint8_t *file, uint32_t size) {
  FileRxCrc = Crc32(file, size);
  IsFileTransferDone = true;
  // Switch LED 2 OFF
  GpioWrite(&Led2, 0);

  printf("\n###### =========== FRAG_DECODER ============ ######\n");
  printf("######               FINISHED                ######\n");
  printf("###### ===================================== ######\n");
  printf("STATUS      : %ld\n", status);
  printf("CRC         : %08lX\n\n", FileRxCrc);
}
#endif

static void StartTxProcess(LmHandlerTxEvents_t txEvent) {
  switch (txEvent) {
  default:
    // Intentional fall through
  case LORAMAC_HANDLER_TX_ON_TIMER: {
    // Schedule 1st packet transmission
    TimerInit(&TxTimer, OnTxTimerEvent);
    TimerSetValue(&TxTimer, TxPeriodicity);
    OnTxTimerEvent(NULL);
  } break;
  case LORAMAC_HANDLER_TX_ON_EVENT: {
  } break;
  }
}

static void UplinkProcess(void) {
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  if (LmHandlerIsBusy() == true) {
    return;
  }

  uint8_t isPending = 0;
  CRITICAL_SECTION_BEGIN();
  isPending = IsTxFramePending;
  IsTxFramePending = 0;
  CRITICAL_SECTION_END();
  if (isPending == 1) {
    if (IsMcSessionStarted == false) {
      if (IsFileTransferDone == false) {
        if (IsClockSynched == false) {
          status = LmhpClockSyncAppTimeReq();
        } else {
          AppDataBuffer[0] = randr(0, 255);
          // Send random packet
          LmHandlerAppData_t appData = {
              .Buffer = AppDataBuffer,
              .BufferSize = 1,
              .Port = 1,
          };
          status = LmHandlerSend(&appData, LmHandlerParams.IsTxConfirmed);
        }
      } else {
        AppDataBuffer[0] = 0x05; // FragDataBlockAuthReq
        AppDataBuffer[1] = FileRxCrc & 0x000000FF;
        AppDataBuffer[2] = (FileRxCrc >> 8) & 0x000000FF;
        AppDataBuffer[3] = (FileRxCrc >> 16) & 0x000000FF;
        AppDataBuffer[4] = (FileRxCrc >> 24) & 0x000000FF;

        // Send FragAuthReq
        LmHandlerAppData_t appData = {
            .Buffer = AppDataBuffer,
            .BufferSize = 5,
            .Port = 201,
        };
        status = LmHandlerSend(&appData, LmHandlerParams.IsTxConfirmed);
      }
      if (status == LORAMAC_HANDLER_SUCCESS) {
        // Switch LED 1 ON
        GpioWrite(&Led1, 1);
        TimerStart(&Led1Timer);
      }
    }
  }
}

static void OnTxPeriodicityChanged(uint32_t periodicity) {
  TxPeriodicity = periodicity;

  if (TxPeriodicity == 0) { // Revert to application default periodicity
    TxPeriodicity =
        APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
  }

  // Update timer periodicity
  TimerStop(&TxTimer);
  TimerSetValue(&TxTimer, TxPeriodicity);
  TimerStart(&TxTimer);
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed) {
  LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity) {
  LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent(void *context) {
  TimerStop(&TxTimer);

  IsTxFramePending = 1;

  // Schedule next transmission
  TimerSetValue(&TxTimer, TxPeriodicity);
  TimerStart(&TxTimer);
}

/*!
 * Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent(void *context) {
  TimerStop(&Led1Timer);
  // Switch LED 1 OFF
  GpioWrite(&Led1, 0);
}

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent(void *context) {
  TimerStop(&Led2Timer);
  // Switch LED 2 ON
  GpioWrite(&Led2, 1);
}

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent(void *context) {
  GpioWrite(&Led2, 1);
  TimerStart(&Led2Timer);

  TimerStart(&LedBeaconTimer);
}
