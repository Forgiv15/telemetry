#include <Arduino.h>
#include <stdlib.h>
#include <ctype.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <ASM330LHHSensor.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>
#include <stdarg.h>

#include <driver/uart.h>

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024

#include <MQTT.h>
#include <mcp_can.h>
#include <TinyGsmClient.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

namespace {

constexpr uint32_t kSerialBaud = 2000000;
constexpr uint32_t kHeartbeatHalfPeriodMs = 25;
constexpr uint32_t kSdErrorHalfPeriodMs = 250;
constexpr uint32_t kAsm330DebugPeriodMs = 1000;
constexpr uint32_t kAsm330RetryPeriodMs = 1000;
constexpr uint32_t kTwaiStatusPeriodMs = 1000;
constexpr uint32_t kAsm330StartupRetries = 10;
constexpr uint32_t kHardStopLatchMs = 1000;
constexpr float kHardStopThresholdG = 10.0f;

constexpr int kGreenLedPin = 41;
constexpr int kRedLedPin = 42;
constexpr int kRecordButtonPin = 38;

constexpr int kTwaiTxPin = 4;
constexpr int kTwaiRxPin = 5;

constexpr int kAsm330SdaPin = 6;
constexpr int kAsm330Sa0Pin = 7;
constexpr int kAsm330SclPin = 8;
constexpr int kAsm330DrdyPin = 45;
constexpr int kAsm330CsPin = 35;  // bodge wire: was GND, now IO35
constexpr bool kAsm330Sa0High = false;
constexpr uint32_t kAsm330I2cHz = 400000;
constexpr bool kEnableAsm330Runtime = true;  // I2C ASM330LHH enabled

constexpr int kSdCsPin = 9;
constexpr int kMcp2515CsPin = 10;
constexpr int kMcp2515MosiPin = 11;
constexpr int kMcp2515SckPin = 12;
constexpr int kMcp2515MisoPin = 13;
constexpr int kMcp2515IntPin = 3;
constexpr int kGnssEspRxPin = 14;
constexpr int kModemCtsPin = 15;
constexpr int kModemRtsPin = 16;
constexpr int kModemEspTxPin = 17;
constexpr int kModemEspRxPin = 18;
constexpr int kModemResetPin = 1;
constexpr int kModemPwrKeyPin = 2;
constexpr int kGnssEspTxPin = 21;
constexpr int kGnssTimePulsePin = 47;
constexpr int kGnssResetPin = 48;
constexpr uint32_t kMcp2515SpiHz = 4000000;
constexpr uint32_t kMcp2515StatusPeriodMs = 1000;
constexpr bool kMcp2515EnableClockRecovery = false;
constexpr uint32_t kMcpServiceTaskStack = 4096;
constexpr UBaseType_t kMcpServiceTaskPriority = 4;
constexpr BaseType_t kMcpServiceTaskCore = 0;
constexpr size_t kMcpRxQueueDepth = 2048;
constexpr uint32_t kMcpErrorReportPeriodMs = 250;
constexpr uint32_t kTwaiRxBudgetPerLoop = 64;
constexpr uint32_t kMcpQueueDrainBudgetPerLoop = 128;
constexpr uint32_t kSdHealthCheckPeriodMs = 30000;
constexpr size_t kSdLogQueueDepth = 512;
constexpr uint32_t kSdLogWriteBudgetPerLoop = 4;
constexpr uint32_t kGnssTargetUartBaud = 921600;
constexpr uint32_t kGnssUartBaudCandidates[] = {kGnssTargetUartBaud, 460800, 115200, 38400, 9600};
constexpr uint32_t kGnssNavigationFrequencyHz = 2;
constexpr uint32_t kGnssReportPeriodMs = 1000;
constexpr uint32_t kGnssTaskStack = 6144;
constexpr UBaseType_t kGnssTaskPriority = 1;
constexpr BaseType_t kGnssTaskCore = 1;
constexpr uint32_t kModemStandardUartBaud = 115200;
constexpr uint32_t kModemTargetUartBaud = 921600;
constexpr uint32_t kModemUartBaudCandidates[] = {kModemStandardUartBaud, kModemTargetUartBaud, 460800};
constexpr uint32_t kModemPowerKeyAssertMs = 500;
constexpr uint32_t kModemPowerKeyBootWaitMs = 14000;
constexpr uint32_t kModemPowerKeyBootMaxMs = 18000;
constexpr uint32_t kModemAtProbeTimeoutMs = 750;
constexpr uint32_t kModemReadyProbePeriodMs = 2000;
constexpr uint32_t kModemSnapshotPeriodMs = 1000;
constexpr uint32_t kModemRecoveryRetryMs = 1500;
constexpr uint32_t kModemReportPeriodMs = 1500;
constexpr bool kModemUseHardwareFlowControl = true;
constexpr bool kModemAutoPowerKeyOnBoot = true;
constexpr uint32_t kModemTaskStack = 16384;
constexpr UBaseType_t kModemTaskPriority = 1;
constexpr BaseType_t kModemTaskCore = 0;
constexpr uint32_t kMqttReconnectPeriodMs = 5000;
constexpr uint32_t kMqttPublishPeriodMs = 2000;
constexpr uint16_t kMqttDefaultPort = 8883;
constexpr size_t kMqttTopicBufferSize = 128;
constexpr size_t kMqttPayloadBufferSize = 320;
constexpr size_t kMqttClientBufferSize = 512;
constexpr uint32_t kNtripReconnectPeriodMs = 5000;
constexpr uint32_t kNtripIdleTimeoutMs = 15000;
constexpr uint32_t kNtripGgaPeriodMs = 10000;
constexpr size_t kSerialCommandBufferSize = 192;

constexpr uint32_t kRecordButtonDebounceMs = 35;

constexpr uint16_t kImuCanIdAccel = 0x700;
constexpr uint16_t kImuCanIdGyro = 0x701;
constexpr uint16_t kImuCanIdStatus = 0x702;

constexpr uint8_t kAsm330RegInt1Ctrl = 0x0D;
constexpr uint8_t kAsm330RegWhoAmI = 0x0F;
constexpr uint8_t kAsm330RegCtrl1Xl = 0x10;
constexpr uint8_t kAsm330RegCtrl2G = 0x11;
constexpr uint8_t kAsm330RegCtrl3C = 0x12;
constexpr uint8_t kAsm330RegCtrl4C = 0x13;
constexpr uint8_t kAsm330RegCtrl9Xl = 0x18;
constexpr uint8_t kAsm330RegStatus = 0x1E;
constexpr uint8_t kAsm330RegOutTempL = 0x20;
constexpr uint8_t kAsm330WhoAmIValue = 0x6B;
constexpr uint8_t kAsm330I2cAddressLow = 0x6A;
constexpr uint8_t kAsm330I2cAddressHigh = 0x6B;
constexpr uint8_t kAsm330DriverAddressLow = ASM330LHH_I2C_ADD_L;
constexpr uint8_t kAsm330DriverAddressHigh = ASM330LHH_I2C_ADD_H;

constexpr uint8_t kAsm330Ctrl1Xl416Hz16g = 0x64;
constexpr uint8_t kAsm330Ctrl2G416Hz2000dps = 0x6C;
constexpr uint8_t kAsm330Ctrl3CSwReset = 0x01;
constexpr uint8_t kAsm330Ctrl3CBduIfInc = 0x44;
constexpr uint8_t kAsm330Ctrl4CI2cDisable = 0x04;
constexpr uint8_t kAsm330Ctrl4CI2cEnable = 0x00;
constexpr uint8_t kAsm330Ctrl9XlDeviceConf = 0x02;
constexpr uint8_t kAsm330Int1DataReady = 0x03;
constexpr uint32_t kAsm330ResetTimeoutMs = 100;
constexpr float kAsm330TargetOdrHz = 417.0f;
constexpr int32_t kAsm330TargetAccelRangeG = 16;
constexpr int32_t kAsm330TargetGyroRangeDps = 2000;
constexpr uint8_t kAsm330StatusAccelReady = 0x01;
constexpr uint8_t kAsm330StatusGyroReady = 0x02;
constexpr uint8_t kAsm330StatusAccelGyroReady = kAsm330StatusAccelReady | kAsm330StatusGyroReady;

constexpr float kAccelScaleGPerLsb = 0.000488f;
constexpr float kGyroScaleDpsPerLsb = 0.07f;

using canid_t = uint32_t;

constexpr canid_t CAN_EFF_FLAG = 0x80000000UL;
constexpr canid_t CAN_RTR_FLAG = 0x40000000UL;
constexpr canid_t CAN_SFF_MASK = 0x000007FFUL;
constexpr canid_t CAN_EFF_MASK = 0x1FFFFFFFUL;
constexpr uint8_t CAN_MAX_DLEN = 8;

struct can_frame {
  canid_t can_id = 0;
  uint8_t can_dlc = 0;
  alignas(8) uint8_t data[CAN_MAX_DLEN] = {};
};

struct QueuedCanFrame {
  can_frame frame = {};
  uint32_t timestampMs = 0;
};

struct SdLogEntry {
  char text[96] = {};
};

struct GnssState {
  bool uartReady = false;
  bool moduleReady = false;
  bool configured = false;
  bool fixValid = false;
  bool invalidLlh = true;
  bool timePulseHigh = false;
  uint32_t uartBaud = 0;
  uint8_t fixType = 0;
  uint8_t carrierSolution = 0;
  uint8_t siv = 0;
  int32_t latitudeE7 = 0;
  int32_t longitudeE7 = 0;
  int32_t altitudeMslMm = 0;
  int32_t horizontalAccuracyMm = 0;
  int32_t verticalAccuracyMm = 0;
  uint32_t timeOfWeekMs = 0;
  uint32_t pvtCount = 0;
  uint32_t lastPvtMillis = 0;
  char lastEvent[48] = "BOOT";
  char lastError[64] = "-";
};

struct CellularState {
  bool uartReady = false;
  bool modemReady = false;
  bool simReady = false;
  bool networkReady = false;
  bool gprsReady = false;
  bool internetOk = false;
  bool mqttConfigured = false;
  bool mqttEnabled = false;
  bool mqttConnected = false;
  bool ntripConfigured = false;
  bool ntripEnabled = false;
  bool ntripConnected = false;
  bool uartHardwareFlow = kModemUseHardwareFlowControl;
  bool uartHighSpeedActive = false;
  int8_t uartCtsLevel = -1;
  int8_t uartRtsLevel = -1;
  int signalQuality = -1;
  uint16_t mqttPort = kMqttDefaultPort;
  uint16_t ntripPort = 2101;
  uint32_t uartBaud = kModemStandardUartBaud;
  uint32_t uartTargetBaud = kModemTargetUartBaud;
  uint32_t uartLastProbeBaud = 0;
  uint32_t lastNetworkMillis = 0;
  uint32_t lastHttpTestMillis = 0;
  uint32_t lastMqttRxMillis = 0;
  uint32_t lastMqttTxMillis = 0;
  uint32_t httpStatusCode = 0;
  uint32_t mqttPublishCount = 0;
  uint32_t mqttReceiveCount = 0;
  uint32_t mqttReconnectCount = 0;
  uint32_t ntripBytesRx = 0;
  uint32_t ntripGgaTxCount = 0;
  uint32_t ntripReconnectCount = 0;
  uint32_t lastNtripDataMillis = 0;
  char modemInfo[48] = "-";
  char operatorName[32] = "-";
  char ipAddress[24] = "-";
  char apn[40] = "";
  char apnUser[32] = "";
  char apnPass[32] = "";
  char httpHost[48] = "example.com";
  char httpPath[64] = "/";
  char mqttHost[64] = "9ddfaf6f481045449c7efc293f3a389f.s1.eu.hivemq.cloud";
  char mqttClientId[48] = "telemetry-node";
  char mqttUser[32] = "";
  char mqttPass[32] = "";
  char mqttTopicPrefix[64] = "szen/telemetry/node";
  char ntripHost[48] = "";
  char ntripMount[32] = "";
  char ntripUser[32] = "";
  char ntripPass[32] = "";
  char lastModemEvent[48] = "BOOT";
  char lastModemError[64] = "-";
  char lastMqttEvent[48] = "IDLE";
  char lastMqttError[64] = "-";
  char lastNtripEvent[48] = "IDLE";
  char lastNtripError[64] = "-";
};

struct Mcp2515BitTimingProfile {
  uint8_t cnf1;
  uint8_t cnf2;
  uint8_t cnf3;
  const char *label;
};

struct Mcp2515TxBufferConfig {
  uint8_t ctrl;
  uint8_t sidh;
  uint8_t rtsInstruction;
};

struct Mcp2515RxBufferConfig {
  uint8_t ctrl;
  uint8_t sidh;
  uint8_t data;
  uint8_t interruptFlag;
};

bool takeCanSpiMutex();
void giveCanSpiMutex();

constexpr uint8_t kMcp2515OpcodeWrite = 0x02;
constexpr uint8_t kMcp2515OpcodeRead = 0x03;
constexpr uint8_t kMcp2515OpcodeBitModify = 0x05;
constexpr uint8_t kMcp2515OpcodeReadStatus = 0xA0;
constexpr uint8_t kMcp2515OpcodeReset = 0xC0;

constexpr uint8_t kMcp2515RegCanstat = 0x0E;
constexpr uint8_t kMcp2515RegCanctrl = 0x0F;
constexpr uint8_t kMcp2515RegTec = 0x1C;
constexpr uint8_t kMcp2515RegRec = 0x1D;
constexpr uint8_t kMcp2515RegCnf3 = 0x28;
constexpr uint8_t kMcp2515RegCnf2 = 0x29;
constexpr uint8_t kMcp2515RegCnf1 = 0x2A;
constexpr uint8_t kMcp2515RegCaninte = 0x2B;
constexpr uint8_t kMcp2515RegCanintf = 0x2C;
constexpr uint8_t kMcp2515RegEflg = 0x2D;

constexpr uint8_t kMcp2515CanctrlReqOpMask = 0xE0;
constexpr uint8_t kMcp2515CanctrlOneShot = 0x08;
constexpr uint8_t kMcp2515CanctrlClken = 0x04;
constexpr uint8_t kMcp2515CanctrlClkPreMask = 0x03;

constexpr uint8_t kMcp2515ModeNormal = 0x00;
constexpr uint8_t kMcp2515ModeSleep = 0x20;
constexpr uint8_t kMcp2515ModeLoopback = 0x40;
constexpr uint8_t kMcp2515ModeListenOnly = 0x60;
constexpr uint8_t kMcp2515ModeConfig = 0x80;

constexpr uint8_t kMcp2515Cnf3Sof = 0x80;

constexpr uint8_t kMcp2515InterruptRx0 = 0x01;
constexpr uint8_t kMcp2515InterruptRx1 = 0x02;
constexpr uint8_t kMcp2515InterruptTx0 = 0x04;
constexpr uint8_t kMcp2515InterruptTx1 = 0x08;
constexpr uint8_t kMcp2515InterruptTx2 = 0x10;
constexpr uint8_t kMcp2515InterruptErr = 0x20;
constexpr uint8_t kMcp2515InterruptMerr = 0x80;
constexpr uint8_t kMcp2515InterruptTxMask =
    kMcp2515InterruptTx0 | kMcp2515InterruptTx1 | kMcp2515InterruptTx2;

constexpr uint8_t kMcp2515EflgRx1Ovr = 0x80;
constexpr uint8_t kMcp2515EflgRx0Ovr = 0x40;

constexpr uint8_t kMcp2515ReadStatusRx0If = 0x01;
constexpr uint8_t kMcp2515ReadStatusRx1If = 0x02;

constexpr uint8_t kMcp2515TxbExideMask = 0x08;
constexpr uint8_t kMcp2515DlcMask = 0x0F;
constexpr uint8_t kMcp2515RtrMask = 0x40;

constexpr uint8_t kMcp2515RxModeMask = 0x60;
constexpr uint8_t kMcp2515RxModeAny = 0x60;
constexpr uint8_t kMcp2515RxModeStdExt = 0x00;
constexpr uint8_t kMcp2515RxB0Bukt = 0x04;
constexpr uint8_t kMcp2515RxBnCtrlRtr = 0x08;

constexpr uint8_t kMcp2515TxCtrlAbtf = 0x40;
constexpr uint8_t kMcp2515TxCtrlMloa = 0x20;
constexpr uint8_t kMcp2515TxCtrlTxErr = 0x10;
constexpr uint8_t kMcp2515TxCtrlTxReq = 0x08;

constexpr uint8_t kMcp2515IdSidH = 0;
constexpr uint8_t kMcp2515IdSidL = 1;
constexpr uint8_t kMcp2515IdEid8 = 2;
constexpr uint8_t kMcp2515IdEid0 = 3;
constexpr uint8_t kMcp2515IdDlc = 4;
constexpr uint8_t kMcp2515IdData = 5;

constexpr Mcp2515TxBufferConfig kMcp2515TxBuffers[] = {
    {0x30, 0x31, 0x81},
    {0x40, 0x41, 0x82},
    {0x50, 0x51, 0x84},
};

constexpr Mcp2515RxBufferConfig kMcp2515RxBuffers[] = {
    {0x60, 0x61, 0x66, kMcp2515InterruptRx0},
    {0x70, 0x71, 0x76, kMcp2515InterruptRx1},
};

constexpr uint8_t kMcp2515RxFilterRegisters[] = {0x00, 0x04, 0x08, 0x10, 0x14, 0x18};
constexpr uint8_t kMcp2515RxMaskRegisters[] = {0x20, 0x24};

constexpr Mcp2515BitTimingProfile kMcp2515ClockCandidates[] = {
    {0x00, 0xA0, 0x02, "10MHZ"},
    {0x00, 0x90, 0x82, "8MHZ"},
    {0x00, 0xF0, 0x86, "16MHZ"},
    {0x00, 0xFA, 0x87, "20MHZ"},
};
constexpr size_t kMcp2515DefaultClockCandidateIndex = 0;

class Mcp2515Driver {
 public:
  enum class Error : uint8_t {
    Ok = 0,
    Fail = 1,
    AllTxBusy = 2,
    NoMessage = 3,
  };

  Mcp2515Driver(const uint8_t csPin, const uint32_t spiHz, SPIClass *spi)
      : csPin_(csPin), spiHz_(spiHz), spi_(spi) {}

  void begin(const int sckPin, const int misoPin, const int mosiPin) {
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);
    spi_->begin(sckPin, misoPin, mosiPin, csPin_);
  }

  Error initialize(const Mcp2515BitTimingProfile &timing) {
    if (!reset()) {
      return Error::Fail;
    }
    if (!requestMode(kMcp2515ModeConfig)) {
      return Error::Fail;
    }

    writeRegister(kMcp2515RegCnf1, timing.cnf1);
    writeRegister(kMcp2515RegCnf2, timing.cnf2);
    writeRegister(kMcp2515RegCnf3, timing.cnf3);

    bitModify(kMcp2515RegCanctrl, kMcp2515CanctrlClkPreMask, 0x00);
    bitModify(kMcp2515RegCanctrl, kMcp2515CanctrlClken, 0x00);
    bitModify(kMcp2515RegCnf3, kMcp2515Cnf3Sof, kMcp2515Cnf3Sof);

    const uint8_t zeroBlock[14] = {};
    for (const Mcp2515TxBufferConfig &txBuffer : kMcp2515TxBuffers) {
      writeRegisters(txBuffer.ctrl, zeroBlock, sizeof(zeroBlock));
    }

    writeRegister(kMcp2515RxBuffers[0].ctrl, kMcp2515RxModeAny | kMcp2515RxB0Bukt);
    writeRegister(kMcp2515RxBuffers[1].ctrl, kMcp2515RxModeAny);

    const uint8_t zeroId[4] = {};
    for (const uint8_t reg : kMcp2515RxFilterRegisters) {
      writeRegisters(reg, zeroId, sizeof(zeroId));
    }
    for (const uint8_t reg : kMcp2515RxMaskRegisters) {
      writeRegisters(reg, zeroId, sizeof(zeroId));
    }

    writeRegister(kMcp2515RegCaninte,
                  kMcp2515InterruptRx0 | kMcp2515InterruptRx1 | kMcp2515InterruptErr | kMcp2515InterruptMerr);
    writeRegister(kMcp2515RegCanintf, 0x00);
    bitModify(kMcp2515RegEflg, kMcp2515EflgRx0Ovr | kMcp2515EflgRx1Ovr, 0x00);

    return requestMode(kMcp2515ModeNormal) ? Error::Ok : Error::Fail;
  }

  Error readMessage(can_frame *frame) {
    const uint8_t intf = readRegister(kMcp2515RegCanintf);
    if ((intf & kMcp2515InterruptRx0) != 0U) {
      return readMessageFromBuffer(0, frame);
    }
    if ((intf & kMcp2515InterruptRx1) != 0U) {
      return readMessageFromBuffer(1, frame);
    }

    const uint8_t status = readStatus();
    if ((status & kMcp2515ReadStatusRx0If) != 0U) {
      return readMessageFromBuffer(0, frame);
    }
    if ((status & kMcp2515ReadStatusRx1If) != 0U) {
      return readMessageFromBuffer(1, frame);
    }
    return Error::NoMessage;
  }

  Error sendMessage(const can_frame *frame) {
    if (frame == nullptr || frame->can_dlc > CAN_MAX_DLEN) {
      return Error::Fail;
    }

    for (size_t index = 0; index < (sizeof(kMcp2515TxBuffers) / sizeof(kMcp2515TxBuffers[0])); ++index) {
      if ((readRegister(kMcp2515TxBuffers[index].ctrl) & kMcp2515TxCtrlTxReq) == 0U) {
        return sendMessage(index, *frame);
      }
    }

    return Error::AllTxBusy;
  }

  uint8_t getErrorFlags() {
    return readRegister(kMcp2515RegEflg);
  }

  uint8_t getInterruptFlags() {
    return readRegister(kMcp2515RegCanintf);
  }

  uint8_t getReceiveErrorCount() {
    return readRegister(kMcp2515RegRec);
  }

  uint8_t getTransmitErrorCount() {
    return readRegister(kMcp2515RegTec);
  }

  uint8_t getOperatingMode() {
    return readRegister(kMcp2515RegCanstat) & kMcp2515CanctrlReqOpMask;
  }

  void clearRxOverflow() {
    bitModify(kMcp2515RegEflg, kMcp2515EflgRx0Ovr | kMcp2515EflgRx1Ovr, 0x00);
  }

  void clearErrorInterrupts() {
    bitModify(kMcp2515RegCanintf, kMcp2515InterruptErr | kMcp2515InterruptMerr, 0x00);
  }

  bool setMode(const uint8_t mode) {
    return requestMode(mode);
  }

 private:
  void beginTransfer() {
    takeCanSpiMutex();
    // Keep SD deselected while talking to MCP2515 on the shared HSPI bus.
    digitalWrite(kSdCsPin, HIGH);
    spi_->beginTransaction(SPISettings(spiHz_, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
  }

  void endTransfer() {
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
    giveCanSpiMutex();
  }

  bool reset() {
    beginTransfer();
    spi_->transfer(kMcp2515OpcodeReset);
    endTransfer();
    delay(10);
    return true;
  }

  bool requestMode(const uint8_t mode) {
    bitModify(kMcp2515RegCanctrl, kMcp2515CanctrlReqOpMask | kMcp2515CanctrlOneShot, mode);

    const uint32_t deadline = millis() + 10U;
    while (millis() < deadline) {
      if ((readRegister(kMcp2515RegCanstat) & kMcp2515CanctrlReqOpMask) == mode) {
        return true;
      }
    }
    return false;
  }

  uint8_t readRegister(const uint8_t reg) {
    beginTransfer();
    spi_->transfer(kMcp2515OpcodeRead);
    spi_->transfer(reg);
    const uint8_t value = spi_->transfer(0x00);
    endTransfer();
    return value;
  }

  void readRegisters(const uint8_t reg, uint8_t *values, const uint8_t length) {
    beginTransfer();
    spi_->transfer(kMcp2515OpcodeRead);
    spi_->transfer(reg);
    for (uint8_t index = 0; index < length; ++index) {
      values[index] = spi_->transfer(0x00);
    }
    endTransfer();
  }

  void writeRegister(const uint8_t reg, const uint8_t value) {
    beginTransfer();
    spi_->transfer(kMcp2515OpcodeWrite);
    spi_->transfer(reg);
    spi_->transfer(value);
    endTransfer();
  }

  void writeRegisters(const uint8_t reg, const uint8_t *values, const uint8_t length) {
    beginTransfer();
    spi_->transfer(kMcp2515OpcodeWrite);
    spi_->transfer(reg);
    for (uint8_t index = 0; index < length; ++index) {
      spi_->transfer(values[index]);
    }
    endTransfer();
  }

  void bitModify(const uint8_t reg, const uint8_t mask, const uint8_t value) {
    beginTransfer();
    spi_->transfer(kMcp2515OpcodeBitModify);
    spi_->transfer(reg);
    spi_->transfer(mask);
    spi_->transfer(value);
    endTransfer();
  }

  uint8_t readStatus() {
    beginTransfer();
    spi_->transfer(kMcp2515OpcodeReadStatus);
    const uint8_t status = spi_->transfer(0x00);
    endTransfer();
    return status;
  }

  void prepareId(uint8_t *buffer, const bool extended, const uint32_t identifier) {
    uint16_t canId = static_cast<uint16_t>(identifier & 0xFFFFU);
    if (extended) {
      buffer[kMcp2515IdEid0] = static_cast<uint8_t>(canId & 0xFFU);
      buffer[kMcp2515IdEid8] = static_cast<uint8_t>(canId >> 8);
      canId = static_cast<uint16_t>(identifier >> 16);
      buffer[kMcp2515IdSidL] = static_cast<uint8_t>(canId & 0x03U);
      buffer[kMcp2515IdSidL] |= static_cast<uint8_t>((canId & 0x1CU) << 3);
      buffer[kMcp2515IdSidL] |= kMcp2515TxbExideMask;
      buffer[kMcp2515IdSidH] = static_cast<uint8_t>(canId >> 5);
      return;
    }

    buffer[kMcp2515IdSidH] = static_cast<uint8_t>(canId >> 3);
    buffer[kMcp2515IdSidL] = static_cast<uint8_t>((canId & 0x07U) << 5);
    buffer[kMcp2515IdEid8] = 0;
    buffer[kMcp2515IdEid0] = 0;
  }

  Error sendMessage(const size_t bufferIndex, const can_frame &frame) {
    if (bufferIndex >= (sizeof(kMcp2515TxBuffers) / sizeof(kMcp2515TxBuffers[0]))) {
      return Error::Fail;
    }

    const Mcp2515TxBufferConfig &txBuffer = kMcp2515TxBuffers[bufferIndex];
    uint8_t packet[13] = {};
    const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0U;
    const bool remote = (frame.can_id & CAN_RTR_FLAG) != 0U;
    const uint32_t identifier = frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);

    prepareId(packet, extended, identifier);
    packet[kMcp2515IdDlc] = static_cast<uint8_t>(frame.can_dlc & kMcp2515DlcMask);
    if (remote) {
      packet[kMcp2515IdDlc] |= kMcp2515RtrMask;
    }
    memcpy(&packet[kMcp2515IdData], frame.data, frame.can_dlc);

    writeRegisters(txBuffer.sidh, packet, static_cast<uint8_t>(5U + frame.can_dlc));

    beginTransfer();
    spi_->transfer(txBuffer.rtsInstruction);
    endTransfer();

    const uint8_t ctrl = readRegister(txBuffer.ctrl);
    return ((ctrl & (kMcp2515TxCtrlAbtf | kMcp2515TxCtrlMloa | kMcp2515TxCtrlTxErr)) == 0U)
               ? Error::Ok
               : Error::Fail;
  }

  Error readMessageFromBuffer(const size_t bufferIndex, can_frame *frame) {
    if (frame == nullptr || bufferIndex >= (sizeof(kMcp2515RxBuffers) / sizeof(kMcp2515RxBuffers[0]))) {
      return Error::Fail;
    }

    const Mcp2515RxBufferConfig &rxBuffer = kMcp2515RxBuffers[bufferIndex];
    uint8_t header[5] = {};
    readRegisters(rxBuffer.sidh, header, sizeof(header));

    uint32_t identifier = (static_cast<uint32_t>(header[kMcp2515IdSidH]) << 3) |
                          (static_cast<uint32_t>(header[kMcp2515IdSidL]) >> 5);
    if ((header[kMcp2515IdSidL] & kMcp2515TxbExideMask) != 0U) {
      identifier = (identifier << 2) | (header[kMcp2515IdSidL] & 0x03U);
      identifier = (identifier << 8) | header[kMcp2515IdEid8];
      identifier = (identifier << 8) | header[kMcp2515IdEid0];
      identifier |= CAN_EFF_FLAG;
    }

    const uint8_t dlc = static_cast<uint8_t>(header[kMcp2515IdDlc] & kMcp2515DlcMask);
    if (dlc > CAN_MAX_DLEN) {
      return Error::Fail;
    }

    if ((readRegister(rxBuffer.ctrl) & kMcp2515RxBnCtrlRtr) != 0U) {
      identifier |= CAN_RTR_FLAG;
    }

    frame->can_id = identifier;
    frame->can_dlc = dlc;
    if (dlc > 0U) {
      readRegisters(rxBuffer.data, frame->data, dlc);
    }

    bitModify(kMcp2515RegCanintf, rxBuffer.interruptFlag, 0x00);
    return Error::Ok;
  }

  uint8_t csPin_;
  uint32_t spiHz_;
  SPIClass *spi_;
};

TwoWire imuI2c(0);
ASM330LHHSensor asm330SensorHigh(&imuI2c, kAsm330DriverAddressHigh);
ASM330LHHSensor asm330SensorLow(&imuI2c, kAsm330DriverAddressLow);
ASM330LHHSensor *g_asm330Sensor = &asm330SensorHigh;
SPIClass canSpi(HSPI);
HardwareSerial gnssSerial(1);
HardwareSerial modemSerial(2);
MCP_CAN mcp2515(&canSpi, kMcp2515CsPin);
Mcp2515Driver mcp2515Fast(kMcp2515CsPin, kMcp2515SpiHz, &canSpi);
SFE_UBLOX_GNSS g_gnss;
TinyGsm g_modem(modemSerial);
TinyGsmClient g_httpClient(g_modem);
TinyGsmClient g_mqttTransport(g_modem);
TinyGsmClient g_ntripClient(g_modem);
MQTTClient g_mqttClient(kMqttClientBufferSize, kMqttClientBufferSize);
SemaphoreHandle_t g_canSpiMutex = nullptr;
SemaphoreHandle_t g_gnssStateMutex = nullptr;
SemaphoreHandle_t g_cellularStateMutex = nullptr;
SemaphoreHandle_t g_gnssUartMutex = nullptr;
TaskHandle_t g_mcpServiceTaskHandle = nullptr;
TaskHandle_t g_gnssTaskHandle = nullptr;
TaskHandle_t g_modemTaskHandle = nullptr;
QueueHandle_t g_mcpRxQueue = nullptr;
QueueHandle_t g_sdLogQueue = nullptr;

volatile bool g_imuDataReady = false;
bool g_twaiReady = false;
bool g_mcpReady = false;
bool g_imuReady = false;
bool g_sdReady = false;
bool g_sdError = false;
bool g_recordingRequested = false;
bool g_buttonStablePressed = false;
bool g_buttonLastReadPressed = false;
const char *g_mcpClockLabel = "UNKNOWN";

uint32_t g_lastHeartbeatToggleMs = 0;
bool g_greenLedState = false;
uint32_t g_lastRedBlinkToggleMs = 0;
bool g_redBlinkState = false;
uint32_t g_lastHardStopMs = 0;
uint32_t g_lastMcpProfileSwitchMs = 0;
uint32_t g_lastMcpStatusMs = 0;
uint32_t g_lastImuPollUs = 0;
uint32_t g_sampleCounter = 0;
uint32_t g_buttonLastTransitionMs = 0;
uint16_t g_logFileIndex = 0;
uint32_t g_lastAsm330DebugMs = 0;
uint32_t g_lastTwaiStatusMs = 0;
uint32_t g_lastSdHealthCheckMs = 0;
uint32_t g_logLineCount = 0;
uint32_t g_lastGnssReportMs = 0;
uint32_t g_lastModemReportMs = 0;
uint32_t g_twaiRxFrameCount = 0;
volatile uint32_t g_mcpRxFrameCount = 0;
volatile uint32_t g_mcpRxOverrunCount = 0;
volatile uint32_t g_mcpRxReadErrorCount = 0;
volatile uint32_t g_mcpQueueDropCount = 0;
volatile uint8_t g_mcpLastErrorFlags = 0;
volatile uint8_t g_mcpLastOverrunEflg = 0;
volatile uint8_t g_mcpLastReadErrEflg = 0;
volatile uint8_t g_mcpLastReadErrCanintf = 0;
uint32_t g_lastMcpErrorReportMs = 0;
uint32_t g_reportedMcpRxOverrunCount = 0;
uint32_t g_reportedMcpRxReadErrorCount = 0;
uint32_t g_reportedMcpQueueDropCount = 0;
size_t g_mcpClockCandidateIndex = 0;

volatile uint32_t g_asm330IrqCount = 0;
uint32_t g_asm330ReadAttemptCount = 0;
uint32_t g_asm330SampleCount = 0;
uint32_t g_asm330InterruptSourceCount = 0;
uint32_t g_asm330PinReadyCount = 0;
uint32_t g_asm330FallbackPollCount = 0;
uint32_t g_asm330AllZeroSampleCount = 0;
uint32_t g_asm330ConsecutiveNoSampleCount = 0;
uint32_t g_asm330InitCycleCount = 0;
uint32_t g_asm330InitSuccessCount = 0;
uint32_t g_asm330InitFailureCount = 0;
uint32_t g_lastAsm330InitAttemptMs = 0;
uint8_t g_asm330LastCtrl1Xl = 0;
uint8_t g_asm330LastCtrl2G = 0;
uint8_t g_asm330LastCtrl3C = 0;
uint8_t g_asm330LastWhoAmI = 0;
uint8_t g_asm330LastStatus = 0;
uint8_t g_asm330LastRawBytes[14] = {};
uint8_t g_asm330LastCtrl4C = 0;
uint8_t g_asm330LastCtrl9Xl = 0;
uint8_t g_asm330LastInt1Ctrl = 0;
uint8_t g_asm330I2cAddress = kAsm330I2cAddressHigh;
char g_serialCommandBuffer[kSerialCommandBufferSize] = {};
size_t g_serialCommandLength = 0;
bool g_asm330InitRetryEnabled = false;
bool g_asm330LastInitSucceeded = false;
bool g_sdCardPresent = false;
bool g_sdReadOk = false;
bool g_sdWriteOk = false;
uint32_t g_sdLogEnqueueDropCount = 0;
volatile bool g_modemConnectRequested = false;
volatile bool g_modemDisconnectRequested = false;
volatile bool g_modemSetupRequested = false;
volatile bool g_modemPowerKeyRequested = false;
volatile bool g_modemResetRequested = false;
volatile bool g_modemHttpTestRequested = false;
volatile bool g_bridgeOnlyMode = false;
volatile bool g_mqttEnableRequested = false;
volatile bool g_mqttDisableRequested = false;
volatile bool g_mqttPublishStateRequested = false;
volatile bool g_gnssResetRequested = false;
volatile bool g_ntripEnableRequested = false;
volatile bool g_ntripDisableRequested = false;
char g_pendingMqttCommand[kSerialCommandBufferSize] = {};
bool g_pendingMqttCommandReady = false;
uint32_t g_lastModemSnapshotMs = 0;
uint32_t g_lastModemReadyProbeMs = 0;
uint32_t g_lastModemRecoveryAttemptMs = 0;
uint32_t g_lastModemPowerKeyPulseMs = 0;
uint8_t g_modemAtFailureCount = 0;
bool g_modemInitialized = false;
bool g_modemHighSpeedConfigured = false;
bool g_modemPowerKeyBootPending = false;
bool g_modemFlowControlEnabled = kModemUseHardwareFlowControl;
bool g_modemBootPowerKeyPending = kModemAutoPowerKeyOnBoot;
size_t g_modemSyncCandidateIndex = 0;
bool g_modemSyncFlowFallback = false;

GnssState g_gnssState = {};
CellularState g_cellularState = {};

File g_logFile;

bool takeMutex(SemaphoreHandle_t mutex, const TickType_t waitTicks = portMAX_DELAY) {
  return mutex == nullptr || xSemaphoreTake(mutex, waitTicks) == pdTRUE;
}

void giveMutex(SemaphoreHandle_t mutex) {
  if (mutex != nullptr) {
    xSemaphoreGive(mutex);
  }
}

bool takeMcpBus() {
  if (g_canSpiMutex != nullptr && xSemaphoreTake(g_canSpiMutex, 0) != pdTRUE) {
    return false;
  }

  digitalWrite(kSdCsPin, HIGH);
  return true;
}

void giveMcpBus() {
  giveCanSpiMutex();
}

bool takeCanSpiMutex() {
  return g_canSpiMutex == nullptr || xSemaphoreTake(g_canSpiMutex, portMAX_DELAY) == pdTRUE;
}

void giveCanSpiMutex() {
  if (g_canSpiMutex != nullptr) {
    xSemaphoreGive(g_canSpiMutex);
  }
}

struct ImuSample {
  int16_t accelRaw[3];
  int16_t gyroRaw[3];
  int16_t tempRaw;
  float accelG[3];
  float gyroDps[3];
  float accelMagnitudeG;
  bool hardStop;
  uint32_t timestampMs;
};

void asm330ReadRegisters(uint8_t startReg, uint8_t *buffer, size_t length);
void prepareAsm330I2cPins();
void printAsm330I2cScan(const char *prefix);

void IRAM_ATTR onAsm330DataReady() {
  g_imuDataReady = true;
  g_asm330IrqCount++;
}

void IRAM_ATTR onMcp2515Interrupt() {
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if (g_mcpServiceTaskHandle != nullptr) {
    vTaskNotifyGiveFromISR(g_mcpServiceTaskHandle, &higherPriorityTaskWoken);
  }
  if (higherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void printHexByte(const uint8_t value) {
  Serial.printf("%02X", value);
}

void printAsm330RawBuffer(const uint8_t *buffer, const size_t length) {
  for (size_t index = 0; index < length; ++index) {
    printHexByte(buffer[index]);
    if (index + 1U < length) {
      Serial.print(' ');
    }
  }
}

void setLed(const int pin, const bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

void copyText(char *destination, const size_t destinationSize, const char *source) {
  if (destination == nullptr || destinationSize == 0U) {
    return;
  }

  const char *resolved = (source != nullptr && source[0] != '\0') ? source : "-";
  strncpy(destination, resolved, destinationSize - 1U);
  destination[destinationSize - 1U] = '\0';
}

void sanitizeText(const char *source, char *destination, const size_t destinationSize) {
  if (destination == nullptr || destinationSize == 0U) {
    return;
  }

  if (source == nullptr || source[0] == '\0') {
    copyText(destination, destinationSize, "-");
    return;
  }

  size_t outIndex = 0;
  for (size_t inIndex = 0; source[inIndex] != '\0' && outIndex + 1U < destinationSize; ++inIndex) {
    const char ch = source[inIndex];
    if (ch == ',' || ch == '\r' || ch == '\n' || !isprint(static_cast<unsigned char>(ch))) {
      destination[outIndex++] = '_';
    } else if (isspace(static_cast<unsigned char>(ch))) {
      destination[outIndex++] = '_';
    } else {
      destination[outIndex++] = ch;
    }
  }

  if (outIndex == 0U) {
    destination[outIndex++] = '-';
  }
  destination[outIndex] = '\0';
}

void sanitizeString(const String &source, char *destination, const size_t destinationSize) {
  sanitizeText(source.c_str(), destination, destinationSize);
}

GnssState copyGnssState() {
  GnssState snapshot = {};
  if (takeMutex(g_gnssStateMutex, pdMS_TO_TICKS(10))) {
    snapshot = g_gnssState;
    giveMutex(g_gnssStateMutex);
  }
  return snapshot;
}

CellularState copyCellularState() {
  CellularState snapshot = {};
  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    snapshot = g_cellularState;
    giveMutex(g_cellularStateMutex);
  }
  return snapshot;
}

void setGnssEvent(const char *eventText, const char *errorText = nullptr) {
  if (!takeMutex(g_gnssStateMutex, pdMS_TO_TICKS(10))) {
    return;
  }

  copyText(g_gnssState.lastEvent, sizeof(g_gnssState.lastEvent), eventText);
  if (errorText != nullptr) {
    copyText(g_gnssState.lastError, sizeof(g_gnssState.lastError), errorText);
  }
  giveMutex(g_gnssStateMutex);
}

void setModemEvent(const char *eventText, const char *errorText = nullptr) {
  if (!takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    return;
  }

  copyText(g_cellularState.lastModemEvent, sizeof(g_cellularState.lastModemEvent), eventText);
  if (errorText != nullptr) {
    copyText(g_cellularState.lastModemError, sizeof(g_cellularState.lastModemError), errorText);
  }
  giveMutex(g_cellularStateMutex);
}

void setMqttEvent(const char *eventText, const char *errorText = nullptr) {
  if (!takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    return;
  }

  copyText(g_cellularState.lastMqttEvent, sizeof(g_cellularState.lastMqttEvent), eventText);
  if (errorText != nullptr) {
    copyText(g_cellularState.lastMqttError, sizeof(g_cellularState.lastMqttError), errorText);
  }
  giveMutex(g_cellularStateMutex);
}

void setNtripEvent(const char *eventText, const char *errorText = nullptr) {
  if (!takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    return;
  }

  copyText(g_cellularState.lastNtripEvent, sizeof(g_cellularState.lastNtripEvent), eventText);
  if (errorText != nullptr) {
    copyText(g_cellularState.lastNtripError, sizeof(g_cellularState.lastNtripError), errorText);
  }
  giveMutex(g_cellularStateMutex);
}

void clearSdLogQueue() {
  if (g_sdLogQueue == nullptr) {
    return;
  }

  SdLogEntry entry = {};
  while (xQueueReceive(g_sdLogQueue, &entry, 0) == pdTRUE) {
  }
}

bool runSdQuickProbe() {
  bool readOk = false;
  bool writeOk = false;
  if (!takeCanSpiMutex()) {
    return false;
  }

  // Keep MCP2515 deselected while probing SD on shared HSPI.
  digitalWrite(kMcp2515CsPin, HIGH);

  File root = SD.open("/");
  if (root) {
    readOk = root.isDirectory();
    root.close();
  }

  if (readOk) {
    File probe = SD.open("/.SDCHK", FILE_WRITE);
    if (probe) {
      writeOk = true;
      probe.close();
    }
  }

  giveCanSpiMutex();

  g_sdReadOk = readOk;
  g_sdWriteOk = writeOk;
  return readOk && writeOk;
}

bool isRecordingActive() {
  return g_recordingRequested && g_sdReady && !g_sdError && static_cast<bool>(g_logFile);
}

void printSdState(const char *prefix) {
  Serial.printf(
    "%s,CARD=%u,READY=%u,ERR=%u,READ_OK=%u,WRITE_OK=%u,REQ=%u,ACTIVE=%u,OPEN=%u,NEXT_INDEX=%u,LOGQ_DROP=%lu\n",
      prefix,
    g_sdCardPresent ? 1U : 0U,
      g_sdReady ? 1U : 0U,
      g_sdError ? 1U : 0U,
    g_sdReadOk ? 1U : 0U,
    g_sdWriteOk ? 1U : 0U,
      g_recordingRequested ? 1U : 0U,
      isRecordingActive() ? 1U : 0U,
      static_cast<bool>(g_logFile) ? 1U : 0U,
    static_cast<unsigned int>(g_logFileIndex),
    static_cast<unsigned long>(g_sdLogEnqueueDropCount));
}

bool openLogFile() {
  if (!g_sdReady || g_sdError || static_cast<bool>(g_logFile)) {
    return false;
  }

  char fileName[20] = {};
  for (uint16_t idx = g_logFileIndex; idx < 1000; ++idx) {
    snprintf(fileName, sizeof(fileName), "/LOG%03u.CSV", idx);
    if (!takeCanSpiMutex()) {
      g_sdError = true;
      Serial.println("SD,ERR,MUTEX");
      return false;
    }

    // Keep MCP2515 deselected while talking to SD on shared HSPI.
    digitalWrite(kMcp2515CsPin, HIGH);

    const bool exists = SD.exists(fileName);
    if (!exists) {
      g_logFile = SD.open(fileName, FILE_WRITE);
      giveCanSpiMutex();
      if (g_logFile) {
        g_logFileIndex = idx + 1;
        g_logLineCount = 0;
        clearSdLogQueue();
        g_logFile.println("TYPE,MS,IDTYPE,ID,DLC,DATA");
        g_logFile.flush();
        Serial.printf("SD,LOG_OPEN,%s\n", fileName);
        return true;
      }

      g_sdError = true;
      Serial.printf("SD,ERR,OPEN,%s\n", fileName);
      return false;
    }

    giveCanSpiMutex();
  }

  g_sdError = true;
  Serial.println("SD,ERR,NO_FILENAME");
  return false;
}

void closeLogFile() {
  if (g_logFile) {
    if (takeCanSpiMutex()) {
      g_logFile.flush();
      g_logFile.close();
      giveCanSpiMutex();
    }
    Serial.println("SD,LOG_CLOSED");
  }
  clearSdLogQueue();
}

bool logLine(const String &line) {
  if (!isRecordingActive()) {
    return false;
  }

  if (g_sdLogQueue == nullptr) {
    return false;
  }

  SdLogEntry entry = {};
  line.toCharArray(entry.text, sizeof(entry.text));
  if (xQueueSend(g_sdLogQueue, &entry, 0) != pdPASS) {
    g_sdLogEnqueueDropCount++;
    return false;
  }

  return true;
}

void serviceSdLogWriter() {
  if (!isRecordingActive() || g_sdLogQueue == nullptr) {
    return;
  }

  if (!takeCanSpiMutex()) {
    g_sdError = true;
    Serial.println("SD,ERR,MUTEX");
    return;
  }

  // Keep MCP2515 deselected while talking to SD on shared HSPI.
  digitalWrite(kMcp2515CsPin, HIGH);

  SdLogEntry entry = {};
  bool writeError = false;
  uint32_t writtenCount = 0;
  while (writtenCount < kSdLogWriteBudgetPerLoop && xQueueReceive(g_sdLogQueue, &entry, 0) == pdTRUE) {
    if (g_logFile.println(entry.text) == 0) {
      writeError = true;
      break;
    }

    ++g_logLineCount;
    ++writtenCount;
    if ((g_logLineCount % 128U) == 0U) {
      g_logFile.flush();
    }
  }

  giveCanSpiMutex();

  if (writeError) {
    g_sdError = true;
    closeLogFile();
    Serial.println("SD,ERR,WRITE");
    printSdState("SD,STATE");
  }
}

const char kBase64Alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

bool base64Encode(const char *source, char *destination, const size_t destinationSize) {
  if (source == nullptr || destination == nullptr || destinationSize == 0U) {
    return false;
  }

  const size_t sourceLength = strlen(source);
  const size_t requiredLength = ((sourceLength + 2U) / 3U) * 4U;
  if (destinationSize <= requiredLength) {
    destination[0] = '\0';
    return false;
  }

  size_t inIndex = 0;
  size_t outIndex = 0;
  while (inIndex < sourceLength) {
    const uint32_t octetA = static_cast<uint8_t>(source[inIndex++]);
    const bool hasB = inIndex < sourceLength;
    const uint32_t octetB = hasB ? static_cast<uint8_t>(source[inIndex++]) : 0U;
    const bool hasC = inIndex < sourceLength;
    const uint32_t octetC = hasC ? static_cast<uint8_t>(source[inIndex++]) : 0U;

    const uint32_t triple = (octetA << 16) | (octetB << 8) | octetC;
    destination[outIndex++] = kBase64Alphabet[(triple >> 18) & 0x3FU];
    destination[outIndex++] = kBase64Alphabet[(triple >> 12) & 0x3FU];
    destination[outIndex++] = hasB ? kBase64Alphabet[(triple >> 6) & 0x3FU] : '=';
    destination[outIndex++] = hasC ? kBase64Alphabet[triple & 0x3FU] : '=';
  }

  destination[outIndex] = '\0';
  return true;
}

bool readClientLine(Client &client, char *buffer, const size_t bufferSize, const uint32_t timeoutMs) {
  if (buffer == nullptr || bufferSize == 0U) {
    return false;
  }

  size_t index = 0;
  const uint32_t startMs = millis();
  while ((millis() - startMs) < timeoutMs) {
    while (client.available() > 0) {
      const int value = client.read();
      if (value < 0) {
        continue;
      }

      const char ch = static_cast<char>(value);
      if (ch == '\r') {
        continue;
      }
      if (ch == '\n') {
        buffer[index] = '\0';
        return true;
      }
      if (index + 1U < bufferSize) {
        buffer[index++] = ch;
      }
    }

    delay(1);
  }

  buffer[index] = '\0';
  return index > 0U;
}

void formatNmeaCoordinate(const int32_t valueE7, const bool latitude, char *buffer, const size_t bufferSize, char &hemisphere) {
  const int32_t absoluteValue = valueE7 < 0 ? -valueE7 : valueE7;
  const int32_t degrees = absoluteValue / 10000000L;
  const int32_t fractionalE7 = absoluteValue % 10000000L;
  const uint32_t scaledMinutes = static_cast<uint32_t>((static_cast<int64_t>(fractionalE7) * 6000000LL) / 10000000LL);
  const uint32_t minuteWhole = scaledMinutes / 100000U;
  const uint32_t minuteFrac = scaledMinutes % 100000U;

  hemisphere = latitude ? (valueE7 >= 0 ? 'N' : 'S') : (valueE7 >= 0 ? 'E' : 'W');
  snprintf(buffer,
           bufferSize,
           latitude ? "%02ld%02lu.%05lu" : "%03ld%02lu.%05lu",
           static_cast<long>(degrees),
           static_cast<unsigned long>(minuteWhole),
           static_cast<unsigned long>(minuteFrac));
}

bool buildGgaSentence(const GnssState &state, char *buffer, const size_t bufferSize) {
  if (buffer == nullptr || bufferSize == 0U || state.latitudeE7 == 0 || state.longitudeE7 == 0) {
    return false;
  }

  char latitudeText[20] = {};
  char longitudeText[20] = {};
  char latHemisphere = 'N';
  char lonHemisphere = 'E';
  formatNmeaCoordinate(state.latitudeE7, true, latitudeText, sizeof(latitudeText), latHemisphere);
  formatNmeaCoordinate(state.longitudeE7, false, longitudeText, sizeof(longitudeText), lonHemisphere);

  const int quality = (state.carrierSolution == 2U) ? 4 : ((state.carrierSolution == 1U) ? 5 : (state.fixType >= 3U ? 1 : 0));
  const int altitudeWhole = state.altitudeMslMm / 1000;
  const int altitudeFrac = abs(state.altitudeMslMm % 1000);

  char body[128] = {};
  snprintf(body,
           sizeof(body),
           "GPGGA,000000.00,%s,%c,%s,%c,%d,%u,1.0,%d.%03d,M,0.0,M,,",
           latitudeText,
           latHemisphere,
           longitudeText,
           lonHemisphere,
           quality,
           static_cast<unsigned int>(state.siv),
           altitudeWhole,
           altitudeFrac);

  uint8_t checksum = 0;
  for (size_t index = 0; body[index] != '\0'; ++index) {
    checksum ^= static_cast<uint8_t>(body[index]);
  }

  snprintf(buffer, bufferSize, "$%s*%02X\r\n", body, checksum);
  return true;
}

void printGnssState(const char *prefix) {
  const GnssState state = copyGnssState();
  Serial.printf(
      "%s,UART=%u,READY=%u,CFG=%u,UART_BAUD=%lu,FIX_VALID=%u,FIX=%u,CARR=%u,SIV=%u,LAT_E7=%ld,LON_E7=%ld,ALT_MM=%ld,HACC_MM=%ld,VACC_MM=%ld,TOW=%lu,PVT=%lu,TIMEPULSE=%u,EVENT=%s,ERR=%s\n",
      prefix,
      state.uartReady ? 1U : 0U,
      state.moduleReady ? 1U : 0U,
      state.configured ? 1U : 0U,
      static_cast<unsigned long>(state.uartBaud),
      state.fixValid ? 1U : 0U,
      state.fixType,
      state.carrierSolution,
      state.siv,
      static_cast<long>(state.latitudeE7),
      static_cast<long>(state.longitudeE7),
      static_cast<long>(state.altitudeMslMm),
      static_cast<long>(state.horizontalAccuracyMm),
      static_cast<long>(state.verticalAccuracyMm),
      static_cast<unsigned long>(state.timeOfWeekMs),
      static_cast<unsigned long>(state.pvtCount),
      state.timePulseHigh ? 1U : 0U,
      state.lastEvent,
      state.lastError);
}

void printModemState(const char *prefix) {
  const CellularState state = copyCellularState();
  const uint32_t uptimeMs = millis();
  const bool bootWindowElapsed = uptimeMs >= kModemPowerKeyBootWaitMs;
  Serial.printf(
  "%s,UART=%u,UART_BAUD=%lu,UART_TARGET=%lu,UART_PROBE=%lu,FLOW=%u,HIGH=%u,CTS=%d,RTS=%d,TX_PIN=%d,RX_PIN=%d,READY=%u,SIM=%u,NET=%u,GPRS=%u,INTERNET=%u,CSQ=%d,HTTP_CODE=%lu,APN=%s,IP=%s,OP=%s,INFO=%s,BOOT14=%u,UP_MS=%lu,EVENT=%s,ERR=%s\n",
      prefix,
      state.uartReady ? 1U : 0U,
  static_cast<unsigned long>(state.uartBaud),
  static_cast<unsigned long>(state.uartTargetBaud),
  static_cast<unsigned long>(state.uartLastProbeBaud),
  state.uartHardwareFlow ? 1U : 0U,
  state.uartHighSpeedActive ? 1U : 0U,
      static_cast<int>(state.uartCtsLevel),
      static_cast<int>(state.uartRtsLevel),
      kModemEspTxPin,
      kModemEspRxPin,
      state.modemReady ? 1U : 0U,
      state.simReady ? 1U : 0U,
      state.networkReady ? 1U : 0U,
      state.gprsReady ? 1U : 0U,
      state.internetOk ? 1U : 0U,
      state.signalQuality,
      static_cast<unsigned long>(state.httpStatusCode),
      state.apn,
      state.ipAddress,
      state.operatorName,
      state.modemInfo,
      bootWindowElapsed ? 1U : 0U,
      static_cast<unsigned long>(uptimeMs),
      state.lastModemEvent,
      state.lastModemError);
}

void printNtripState(const char *prefix) {
  const CellularState state = copyCellularState();
  Serial.printf(
      "%s,CFG=%u,EN=%u,SOCK=%u,HOST=%s,PORT=%u,MOUNT=%s,RTCM_BYTES=%lu,GGA_TX=%lu,RECONNECTS=%lu,EVENT=%s,ERR=%s\n",
      prefix,
      state.ntripConfigured ? 1U : 0U,
      state.ntripEnabled ? 1U : 0U,
      state.ntripConnected ? 1U : 0U,
      state.ntripHost,
      static_cast<unsigned int>(state.ntripPort),
      state.ntripMount,
      static_cast<unsigned long>(state.ntripBytesRx),
      static_cast<unsigned long>(state.ntripGgaTxCount),
      static_cast<unsigned long>(state.ntripReconnectCount),
      state.lastNtripEvent,
      state.lastNtripError);
}

    void printMqttState(const char *prefix) {
      const CellularState state = copyCellularState();
      Serial.printf(
      "%s,CFG=%u,EN=%u,SOCK=%u,HOST=%s,PORT=%u,CLIENT=%s,PREFIX=%s,TX=%lu,RX=%lu,RECONNECTS=%lu,DROPPED=%lu,EVENT=%s,ERR=%s\n",
      prefix,
      state.mqttConfigured ? 1U : 0U,
      state.mqttEnabled ? 1U : 0U,
      state.mqttConnected ? 1U : 0U,
      state.mqttHost,
      static_cast<unsigned int>(state.mqttPort),
      state.mqttClientId,
      state.mqttTopicPrefix,
      static_cast<unsigned long>(state.mqttPublishCount),
      static_cast<unsigned long>(state.mqttReceiveCount),
      static_cast<unsigned long>(state.mqttReconnectCount),
      static_cast<unsigned long>(g_mqttClient.droppedMessages()),
      state.lastMqttEvent,
      state.lastMqttError);
    }

void requestGnssHardwareReset() {
  g_gnssResetRequested = true;
}

void applyGnssHardwareReset() {
  digitalWrite(kGnssResetPin, LOW);
  delay(50);
  digitalWrite(kGnssResetPin, HIGH);
  delay(250);
}

bool configureGnssModule() {
  for (size_t index = 0; index < (sizeof(kGnssUartBaudCandidates) / sizeof(kGnssUartBaudCandidates[0])); ++index) {
    const uint32_t baudRate = kGnssUartBaudCandidates[index];
    gnssSerial.end();
    gnssSerial.begin(baudRate, SERIAL_8N1, kGnssEspRxPin, kGnssEspTxPin);
    delay(100);

    bool started = false;
    bool configured = false;
    if (takeMutex(g_gnssUartMutex, pdMS_TO_TICKS(50))) {
      started = g_gnss.begin(gnssSerial);
      if (started) {
        if (baudRate != kGnssTargetUartBaud) {
          g_gnss.setSerialRate(kGnssTargetUartBaud, COM_PORT_UART1);
          delay(100);
          gnssSerial.flush();
          gnssSerial.end();
          gnssSerial.begin(kGnssTargetUartBaud, SERIAL_8N1, kGnssEspRxPin, kGnssEspTxPin);
          delay(100);
          started = g_gnss.begin(gnssSerial);
        }
        if (started) {
          configured = g_gnss.setUART1Output(COM_TYPE_UBX) &&
                       g_gnss.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_RTCM3) &&
                       g_gnss.setNavigationFrequency(static_cast<uint8_t>(kGnssNavigationFrequencyHz)) &&
                       g_gnss.setAutoPVT(true) &&
                       g_gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
        }
      }
      giveMutex(g_gnssUartMutex);
    }

    if (!started) {
      continue;
    }

    if (takeMutex(g_gnssStateMutex, pdMS_TO_TICKS(10))) {
      g_gnssState.uartReady = true;
      g_gnssState.moduleReady = true;
      g_gnssState.configured = configured;
      g_gnssState.uartBaud = started ? kGnssTargetUartBaud : baudRate;
      copyText(g_gnssState.lastEvent, sizeof(g_gnssState.lastEvent), configured ? "GNSS_OK" : "GNSS_PARTIAL_CFG");
      copyText(g_gnssState.lastError, sizeof(g_gnssState.lastError), configured ? "-" : "CFG_FAIL");
      giveMutex(g_gnssStateMutex);
    }

    return true;
  }

  if (takeMutex(g_gnssStateMutex, pdMS_TO_TICKS(10))) {
    g_gnssState.uartReady = true;
    g_gnssState.moduleReady = false;
    g_gnssState.configured = false;
    copyText(g_gnssState.lastEvent, sizeof(g_gnssState.lastEvent), "GNSS_NOT_FOUND");
    copyText(g_gnssState.lastError, sizeof(g_gnssState.lastError), "UART_LINK_FAIL");
    giveMutex(g_gnssStateMutex);
  }
  return false;
}

void updateGnssSolution() {
  bool gotPvt = false;
  GnssState updatedState = copyGnssState();
  updatedState.timePulseHigh = digitalRead(kGnssTimePulsePin) == HIGH;

  if (takeMutex(g_gnssUartMutex, pdMS_TO_TICKS(20))) {
    gotPvt = g_gnss.getPVT();
    if (gotPvt) {
      updatedState.fixType = g_gnss.getFixType();
      updatedState.carrierSolution = g_gnss.getCarrierSolutionType();
      updatedState.siv = g_gnss.getSIV();
      updatedState.latitudeE7 = g_gnss.getLatitude();
      updatedState.longitudeE7 = g_gnss.getLongitude();
      updatedState.altitudeMslMm = g_gnss.getAltitudeMSL();
      updatedState.horizontalAccuracyMm = g_gnss.getHorizontalAccEst();
      updatedState.verticalAccuracyMm = g_gnss.getVerticalAccEst();
      updatedState.timeOfWeekMs = g_gnss.getTimeOfWeek();
      updatedState.invalidLlh = g_gnss.getInvalidLlh();
    }
    giveMutex(g_gnssUartMutex);
  }

  if (!gotPvt) {
    if (takeMutex(g_gnssStateMutex, pdMS_TO_TICKS(10))) {
      g_gnssState.timePulseHigh = updatedState.timePulseHigh;
      giveMutex(g_gnssStateMutex);
    }
    return;
  }

  updatedState.fixValid = !updatedState.invalidLlh && updatedState.fixType >= 3U;
  updatedState.lastPvtMillis = millis();
  updatedState.pvtCount++;
  copyText(updatedState.lastEvent, sizeof(updatedState.lastEvent), "PVT_OK");
  copyText(updatedState.lastError, sizeof(updatedState.lastError), "-");

  if (takeMutex(g_gnssStateMutex, pdMS_TO_TICKS(10))) {
    g_gnssState = updatedState;
    giveMutex(g_gnssStateMutex);
  }
}

void gnssServiceTask(void *parameter) {
  (void)parameter;
  for (;;) {
    if (g_gnssResetRequested) {
      g_gnssResetRequested = false;
      setGnssEvent("GNSS_RESET", "-");
      applyGnssHardwareReset();
      if (takeMutex(g_gnssStateMutex, pdMS_TO_TICKS(10))) {
        g_gnssState.moduleReady = false;
        g_gnssState.configured = false;
        giveMutex(g_gnssStateMutex);
      }
    }

    const GnssState state = copyGnssState();
    if (!state.moduleReady) {
      configureGnssModule();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    updateGnssSolution();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void updateModemUartState(const uint32_t activeBaud, const uint32_t probeBaud) {
  const int ctsLevel = digitalRead(kModemCtsPin);
  const int rtsLevel = digitalRead(kModemRtsPin);
  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.uartReady = true;
    g_cellularState.uartBaud = activeBaud;
    g_cellularState.uartTargetBaud = kModemTargetUartBaud;
    g_cellularState.uartLastProbeBaud = probeBaud;
    g_cellularState.uartHardwareFlow = g_modemFlowControlEnabled;
    g_cellularState.uartHighSpeedActive = g_modemHighSpeedConfigured && activeBaud == kModemTargetUartBaud;
    g_cellularState.uartCtsLevel = static_cast<int8_t>(ctsLevel);
    g_cellularState.uartRtsLevel = static_cast<int8_t>(rtsLevel);
    giveMutex(g_cellularStateMutex);
  }
}

void configureModemUart(const uint32_t baudRate = kModemStandardUartBaud, const bool useFlowControl = kModemUseHardwareFlowControl) {
  g_modemFlowControlEnabled = useFlowControl;
  modemSerial.end();
  modemSerial.begin(baudRate, SERIAL_8N1, kModemEspRxPin, kModemEspTxPin);
  uart_set_mode(UART_NUM_2, UART_MODE_UART);
  if (g_modemFlowControlEnabled) {
    // ESP32 RTS output must drive the module CTS pin, and ESP32 CTS input
    // must read the module RTS pin.
    uart_set_pin(UART_NUM_2, kModemEspTxPin, kModemEspRxPin, kModemCtsPin, kModemRtsPin);
    uart_set_hw_flow_ctrl(UART_NUM_2, UART_HW_FLOWCTRL_CTS_RTS, 64);
  } else {
    uart_set_pin(UART_NUM_2, kModemEspTxPin, kModemEspRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_hw_flow_ctrl(UART_NUM_2, UART_HW_FLOWCTRL_DISABLE, 0);
  }
  updateModemUartState(baudRate, baudRate);
}

bool promoteModemUartToTarget() {
  if (g_modemHighSpeedConfigured || kModemTargetUartBaud == kModemStandardUartBaud) {
    return true;
  }

  if (!g_modem.setBaud(kModemTargetUartBaud)) {
    setModemEvent("MODEM_UART_FAIL", "AT+IPR");
    return false;
  }

  delay(100);
  g_modemHighSpeedConfigured = true;
  configureModemUart(kModemTargetUartBaud, g_modemFlowControlEnabled);
  delay(100);
  if (!g_modem.testAT(kModemAtProbeTimeoutMs)) {
    g_modemHighSpeedConfigured = false;
    configureModemUart(kModemStandardUartBaud, g_modemFlowControlEnabled);
    setModemEvent("MODEM_UART_FAIL", "BAUD_SWITCH");
    return false;
  }

  updateModemUartState(kModemTargetUartBaud, kModemStandardUartBaud);
  setModemEvent("MODEM_UART_OK", "-");
  return true;
}

void pulseModemPowerKey() {
  // IO2 drives an NMOS low-side switch, so HIGH on the ESP32 pulls the
  // SIM7600 PWRKEY pin low. The module datasheet calls for a 100-500 ms
  // active-low pulse, then roughly 12 s before the UART is ready.
  digitalWrite(kModemPwrKeyPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(kModemPowerKeyAssertMs));
  digitalWrite(kModemPwrKeyPin, LOW);
  g_lastModemPowerKeyPulseMs = millis();
  g_modemHighSpeedConfigured = false;
  g_modemPowerKeyBootPending = true;
}

void pulseModemResetPin() {
  digitalWrite(kModemResetPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(150));
  digitalWrite(kModemResetPin, LOW);
  g_modemHighSpeedConfigured = false;
  vTaskDelay(pdMS_TO_TICKS(2500));
}

bool synchronizeModemUartWithFlow(const bool useFlowControl) {
  if (g_modemSyncCandidateIndex >= (sizeof(kModemUartBaudCandidates) / sizeof(kModemUartBaudCandidates[0]))) {
    return false;
  }

  const uint32_t baudRate = kModemUartBaudCandidates[g_modemSyncCandidateIndex++];
  configureModemUart(baudRate, useFlowControl);
  updateModemUartState(baudRate, baudRate);
  vTaskDelay(pdMS_TO_TICKS(1));
  delay(100);
  if (!g_modem.testAT(kModemAtProbeTimeoutMs)) {
    return false;
  }

  g_modemHighSpeedConfigured = baudRate == kModemTargetUartBaud;
  updateModemUartState(baudRate, baudRate);

  if (baudRate != kModemTargetUartBaud && !promoteModemUartToTarget()) {
    return false;
  }

  g_modemSyncCandidateIndex = 0;
  g_modemSyncFlowFallback = false;
  setModemEvent("MODEM_UART_OK", "-");
  return true;
}

bool synchronizeModemUart() {
  if (synchronizeModemUartWithFlow(g_modemSyncFlowFallback ? false : kModemUseHardwareFlowControl)) {
    return true;
  }

  if (g_modemSyncCandidateIndex < (sizeof(kModemUartBaudCandidates) / sizeof(kModemUartBaudCandidates[0]))) {
    setModemEvent("MODEM_UART_SCAN", g_modemSyncFlowFallback ? "FLOW_OFF" : "FLOW_ON");
    return false;
  }

  // If CTS/RTS wiring keeps UART blocked, retry without HW flow control.
  if (kModemUseHardwareFlowControl && !g_modemSyncFlowFallback) {
    g_modemSyncFlowFallback = true;
    g_modemSyncCandidateIndex = 0;
    setModemEvent("MODEM_UART_SCAN", "FLOW_FALLBACK");
    return false;
  }

  g_modemSyncCandidateIndex = 0;
  g_modemSyncFlowFallback = false;
  setModemEvent("MODEM_UART_FAIL", "AT_TIMEOUT");
  return false;
}

void refreshModemNetworkSnapshot(const bool force = false) {
  const uint32_t nowMs = millis();
  if (!force && (nowMs - g_lastModemSnapshotMs) < kModemSnapshotPeriodMs) {
    return;
  }

  CellularState updatedState = copyCellularState();
  updatedState.signalQuality = g_modem.getSignalQuality();
  updatedState.networkReady = g_modem.isNetworkConnected();
  updatedState.gprsReady = g_modem.isGprsConnected();
  updatedState.simReady = g_modem.getSimStatus() == SIM_READY;
  updatedState.lastNetworkMillis = nowMs;
  sanitizeString(g_modem.getOperator(), updatedState.operatorName, sizeof(updatedState.operatorName));
  sanitizeString(g_modem.localIP().toString(), updatedState.ipAddress, sizeof(updatedState.ipAddress));
  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.signalQuality = updatedState.signalQuality;
    g_cellularState.networkReady = updatedState.networkReady;
    g_cellularState.gprsReady = updatedState.gprsReady;
    g_cellularState.simReady = updatedState.simReady;
    g_cellularState.lastNetworkMillis = updatedState.lastNetworkMillis;
    copyText(g_cellularState.operatorName, sizeof(g_cellularState.operatorName), updatedState.operatorName);
    copyText(g_cellularState.ipAddress, sizeof(g_cellularState.ipAddress), updatedState.ipAddress);
    giveMutex(g_cellularStateMutex);
  }
  g_lastModemSnapshotMs = nowMs;
}

bool isModemBootWindowElapsed() {
  if (!g_modemPowerKeyBootPending) {
    return true;
  }

  return (millis() - g_lastModemPowerKeyPulseMs) >= kModemPowerKeyBootWaitMs;
}

bool ensureModemReady() {
  const uint32_t nowMs = millis();
  const CellularState state = copyCellularState();

  if (g_modemPowerKeyBootPending) {
    const uint32_t bootElapsedMs = nowMs - g_lastModemPowerKeyPulseMs;
    if (bootElapsedMs < kModemPowerKeyBootWaitMs) {
      setModemEvent("MODEM_BOOT_WAIT", "PWRKEY_BOOT");
      return false;
    }
    g_modemPowerKeyBootPending = false;
    setModemEvent(bootElapsedMs <= kModemPowerKeyBootMaxMs ? "MODEM_BOOT_READY" : "MODEM_BOOT_LATE", "-");
  }

  if (state.modemReady && g_modemInitialized && (nowMs - g_lastModemReadyProbeMs) < kModemReadyProbePeriodMs) {
    return true;
  }

  g_lastModemReadyProbeMs = nowMs;

  if (!g_modem.testAT(kModemAtProbeTimeoutMs)) {
    g_modemInitialized = false;
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      g_cellularState.modemReady = false;
      g_cellularState.networkReady = false;
      g_cellularState.gprsReady = false;
      g_cellularState.internetOk = false;
      copyText(g_cellularState.lastModemError, sizeof(g_cellularState.lastModemError), "MODEM_NO_AT");
      giveMutex(g_cellularStateMutex);
    }

    if ((nowMs - g_lastModemRecoveryAttemptMs) < kModemRecoveryRetryMs) {
      setModemEvent("MODEM_AT_WAIT", "AT_TIMEOUT");
      return false;
    }

    g_lastModemRecoveryAttemptMs = nowMs;
    ++g_modemAtFailureCount;
    if (!synchronizeModemUart()) {
      return false;
    }
    g_modemAtFailureCount = 0;
    if (g_modemPowerKeyBootPending) {
      setModemEvent("MODEM_BOOT_READY", "AT_OK");
    }
  }

  if (g_modemPowerKeyBootPending) {
    setModemEvent("MODEM_BOOT_READY", "AT_OK");
  }

  if (!g_modem.testAT(kModemAtProbeTimeoutMs)) {
    if (g_modemAtFailureCount == 2U) {
      setModemEvent("MODEM_PWRKEY", "AT_TIMEOUT");
      g_modemInitialized = false;
      pulseModemPowerKey();
      return false;
    } else if (g_modemAtFailureCount >= 4U) {
      setModemEvent("MODEM_RESET", "AT_TIMEOUT");
      pulseModemResetPin();
      g_modemAtFailureCount = 0;
      return false;
    }

    setModemEvent("MODEM_AT_WAIT", "AT_TIMEOUT");
    return false;
  }

  CellularState currentState = copyCellularState();
  if (currentState.uartBaud == kModemTargetUartBaud) {
    g_modemHighSpeedConfigured = true;
    updateModemUartState(currentState.uartBaud, currentState.uartLastProbeBaud);
  } else if (!promoteModemUartToTarget()) {
    return false;
  }

  g_modemAtFailureCount = 0;
  g_modemSyncCandidateIndex = 0;
  g_modemSyncFlowFallback = false;

  if (!g_modemInitialized) {
    const bool initialized = g_modem.init();
    if (!initialized) {
      if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
        g_cellularState.modemReady = false;
        copyText(g_cellularState.lastModemError, sizeof(g_cellularState.lastModemError), "MODEM_INIT_FAIL");
        giveMutex(g_cellularStateMutex);
      }
      return false;
    }
    g_modemInitialized = true;
  }

  CellularState updatedState = copyCellularState();
  updatedState.modemReady = true;
  updatedState.simReady = g_modem.getSimStatus() == SIM_READY;
  sanitizeString(g_modem.getModemInfo(), updatedState.modemInfo, sizeof(updatedState.modemInfo));
  copyText(updatedState.lastModemEvent, sizeof(updatedState.lastModemEvent), "MODEM_OK");
  copyText(updatedState.lastModemError, sizeof(updatedState.lastModemError), "-");

  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.modemReady = updatedState.modemReady;
    g_cellularState.simReady = updatedState.simReady;
    copyText(g_cellularState.modemInfo, sizeof(g_cellularState.modemInfo), updatedState.modemInfo);
    copyText(g_cellularState.lastModemEvent, sizeof(g_cellularState.lastModemEvent), updatedState.lastModemEvent);
    copyText(g_cellularState.lastModemError, sizeof(g_cellularState.lastModemError), updatedState.lastModemError);
    giveMutex(g_cellularStateMutex);
  }

  refreshModemNetworkSnapshot(true);
  return true;
}

bool ensurePacketDataSession() {
  const CellularState state = copyCellularState();
  if (!ensureModemReady()) {
    return false;
  }

  if (!g_modem.isNetworkConnected() && !g_modem.waitForNetwork(30000L, true)) {
    setModemEvent("NETWORK_WAIT_FAIL", "NO_NETWORK");
    refreshModemNetworkSnapshot(true);
    return false;
  }

  const char *apn = state.apn;
  const char *apnUser = state.apnUser;
  const char *apnPass = state.apnPass;
  if (!g_modem.isGprsConnected() && !g_modem.gprsConnect(apn, apnUser, apnPass)) {
    setModemEvent("GPRS_CONNECT_FAIL", "NO_PDP");
    refreshModemNetworkSnapshot(true);
    return false;
  }

  refreshModemNetworkSnapshot(true);
  setModemEvent("GPRS_OK", "-");
  return true;
}

bool runHttpConnectivityTest() {
  const CellularState state = copyCellularState();
  if (!ensurePacketDataSession()) {
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      g_cellularState.internetOk = false;
      g_cellularState.httpStatusCode = 0;
      g_cellularState.lastHttpTestMillis = millis();
      giveMutex(g_cellularStateMutex);
    }
    return false;
  }

  g_httpClient.stop();
  if (!g_httpClient.connect(state.httpHost, 80)) {
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      g_cellularState.internetOk = false;
      g_cellularState.httpStatusCode = 0;
      g_cellularState.lastHttpTestMillis = millis();
      copyText(g_cellularState.lastModemEvent, sizeof(g_cellularState.lastModemEvent), "HTTP_CONNECT_FAIL");
      copyText(g_cellularState.lastModemError, sizeof(g_cellularState.lastModemError), "HTTP_SOCKET");
      giveMutex(g_cellularStateMutex);
    }
    return false;
  }

  g_httpClient.print("GET ");
  g_httpClient.print((state.httpPath[0] != '\0') ? state.httpPath : "/");
  g_httpClient.print(" HTTP/1.0\r\nHost: ");
  g_httpClient.print(state.httpHost);
  g_httpClient.print("\r\nConnection: close\r\n\r\n");

  char statusLine[128] = {};
  int statusCode = 0;
  const bool gotStatusLine = readClientLine(g_httpClient, statusLine, sizeof(statusLine), 5000U);
  if (gotStatusLine) {
    if (sscanf(statusLine, "HTTP/%*s %d", &statusCode) != 1 && strstr(statusLine, "ICY 200") != nullptr) {
      statusCode = 200;
    }
  }

  const bool ok = statusCode >= 200 && statusCode < 400;
  g_httpClient.stop();

  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.internetOk = ok;
    g_cellularState.httpStatusCode = static_cast<uint32_t>(statusCode);
    g_cellularState.lastHttpTestMillis = millis();
    copyText(g_cellularState.lastModemEvent, sizeof(g_cellularState.lastModemEvent), ok ? "HTTP_OK" : "HTTP_FAIL");
    copyText(g_cellularState.lastModemError, sizeof(g_cellularState.lastModemError), ok ? "-" : (gotStatusLine ? statusLine : "HTTP_TIMEOUT"));
    giveMutex(g_cellularStateMutex);
  }

  return ok;
}

void disconnectNtripSocket(const char *eventText, const char *errorText) {
  g_ntripClient.stop();
  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.ntripConnected = false;
    copyText(g_cellularState.lastNtripEvent, sizeof(g_cellularState.lastNtripEvent), eventText);
    copyText(g_cellularState.lastNtripError, sizeof(g_cellularState.lastNtripError), errorText);
    giveMutex(g_cellularStateMutex);
  }
}

bool connectNtripSocket() {
  CellularState state = copyCellularState();
  if (!state.ntripConfigured || !state.ntripEnabled) {
    return false;
  }
  if (!ensurePacketDataSession()) {
    return false;
  }

  g_ntripClient.stop();
  if (!g_ntripClient.connect(state.ntripHost, static_cast<int>(state.ntripPort))) {
    disconnectNtripSocket("CONNECT_FAIL", "NTRIP_SOCKET");
    return false;
  }

  char authPlain[96] = {};
  char authEncoded[160] = {};
  if (state.ntripUser[0] != '\0' || state.ntripPass[0] != '\0') {
    snprintf(authPlain, sizeof(authPlain), "%s:%s", state.ntripUser, state.ntripPass);
    if (!base64Encode(authPlain, authEncoded, sizeof(authEncoded))) {
      disconnectNtripSocket("AUTH_FAIL", "NTRIP_AUTH_ENCODE");
      return false;
    }
  }

  g_ntripClient.print("GET /");
  g_ntripClient.print(state.ntripMount);
  g_ntripClient.print(" HTTP/1.0\r\nUser-Agent: NTRIP ESP32S3\r\nAccept: */*\r\nConnection: close\r\n");
  if (authEncoded[0] != '\0') {
    g_ntripClient.print("Authorization: Basic ");
    g_ntripClient.print(authEncoded);
    g_ntripClient.print("\r\n");
  }
  g_ntripClient.print("\r\n");

  char headerLine[128] = {};
  if (!readClientLine(g_ntripClient, headerLine, sizeof(headerLine), 5000U)) {
    disconnectNtripSocket("HEADER_FAIL", "NTRIP_TIMEOUT");
    return false;
  }

  if (strstr(headerLine, "200") == nullptr && strstr(headerLine, "ICY") == nullptr) {
    disconnectNtripSocket("HEADER_FAIL", headerLine);
    return false;
  }

  while (readClientLine(g_ntripClient, headerLine, sizeof(headerLine), 1000U)) {
    if (headerLine[0] == '\0') {
      break;
    }
  }

  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.ntripConnected = true;
    g_cellularState.ntripReconnectCount++;
    g_cellularState.lastNtripDataMillis = millis();
    copyText(g_cellularState.lastNtripEvent, sizeof(g_cellularState.lastNtripEvent), "NTRIP_OK");
    copyText(g_cellularState.lastNtripError, sizeof(g_cellularState.lastNtripError), "-");
    giveMutex(g_cellularStateMutex);
  }

  return true;
}

void maybeSendNtripGga() {
  static uint32_t lastGgaMs = 0;
  const uint32_t nowMs = millis();
  if ((nowMs - lastGgaMs) < kNtripGgaPeriodMs) {
    return;
  }

  const GnssState gnssState = copyGnssState();
  char ggaSentence[160] = {};
  if (!buildGgaSentence(gnssState, ggaSentence, sizeof(ggaSentence))) {
    return;
  }

  g_ntripClient.print(ggaSentence);
  lastGgaMs = nowMs;
  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.ntripGgaTxCount++;
    giveMutex(g_cellularStateMutex);
  }
}

void serviceNtripStream() {
  CellularState state = copyCellularState();

  if (g_ntripDisableRequested) {
    g_ntripDisableRequested = false;
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      g_cellularState.ntripEnabled = false;
      giveMutex(g_cellularStateMutex);
    }
    disconnectNtripSocket("NTRIP_OFF", "-");
    return;
  }

  if (g_ntripEnableRequested) {
    g_ntripEnableRequested = false;
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      g_cellularState.ntripEnabled = true;
      giveMutex(g_cellularStateMutex);
    }
    state = copyCellularState();
  }

  if (!state.ntripEnabled) {
    if (state.ntripConnected) {
      disconnectNtripSocket("NTRIP_OFF", "-");
    }
    return;
  }

  const uint32_t nowMs = millis();
  static uint32_t lastConnectAttemptMs = 0;
  if (!state.ntripConnected) {
    if ((nowMs - lastConnectAttemptMs) >= kNtripReconnectPeriodMs) {
      lastConnectAttemptMs = nowMs;
      connectNtripSocket();
    }
    return;
  }

  maybeSendNtripGga();

  uint8_t rtcmBuffer[256] = {};
  size_t bufferCount = 0;
  while (g_ntripClient.available() > 0 && bufferCount < sizeof(rtcmBuffer)) {
    const int value = g_ntripClient.read();
    if (value >= 0) {
      rtcmBuffer[bufferCount++] = static_cast<uint8_t>(value);
    }
  }

  if (bufferCount > 0U) {
    if (takeMutex(g_gnssUartMutex, pdMS_TO_TICKS(20))) {
      gnssSerial.write(rtcmBuffer, bufferCount);
      giveMutex(g_gnssUartMutex);
    }

    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      g_cellularState.ntripBytesRx += static_cast<uint32_t>(bufferCount);
      g_cellularState.lastNtripDataMillis = nowMs;
      giveMutex(g_cellularStateMutex);
    }
  }

  state = copyCellularState();
  if (!g_ntripClient.connected() || (state.lastNtripDataMillis != 0U && (nowMs - state.lastNtripDataMillis) > kNtripIdleTimeoutMs)) {
    disconnectNtripSocket("NTRIP_RETRY", g_ntripClient.connected() ? "IDLE_TIMEOUT" : "SOCKET_CLOSED");
  }
}

bool dispatchControlCommand(const char *line);

bool buildMqttTopic(const CellularState &state, const char *suffix, char *buffer, const size_t bufferSize) {
  if (suffix == nullptr || buffer == nullptr || bufferSize == 0U) {
    return false;
  }

  const char *prefix = (state.mqttTopicPrefix[0] != '\0') ? state.mqttTopicPrefix : "szen/telemetry/node";
  const int written = snprintf(buffer, bufferSize, "%s/%s", prefix, suffix);
  return written > 0 && static_cast<size_t>(written) < bufferSize;
}

void markMqttPublish(const bool published) {
  if (!published || !takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    return;
  }

  g_cellularState.mqttPublishCount++;
  g_cellularState.lastMqttTxMillis = millis();
  giveMutex(g_cellularStateMutex);
}

bool mqttPublishText(const char *suffix, const char *payload, const bool retained, const int qos) {
  const CellularState state = copyCellularState();
  if (!state.mqttConnected || payload == nullptr) {
    return false;
  }

  char topic[kMqttTopicBufferSize] = {};
  if (!buildMqttTopic(state, suffix, topic, sizeof(topic))) {
    return false;
  }

  const bool published = g_mqttClient.publish(topic, payload, retained, qos);
  markMqttPublish(published);
  if (!published) {
    setMqttEvent("PUB_FAIL", "MQTT_PUBLISH");
  }
  return published;
}

void publishMqttStateSnapshots() {
  const CellularState cellularState = copyCellularState();
  const GnssState gnssState = copyGnssState();
  char payload[kMqttPayloadBufferSize] = {};

  snprintf(
      payload,
      sizeof(payload),
      "MODEM,STATE,UART=%u,UART_BAUD=%lu,UART_TARGET=%lu,UART_PROBE=%lu,FLOW=%u,HIGH=%u,CTS=%d,RTS=%d,TX_PIN=%d,RX_PIN=%d,READY=%u,SIM=%u,NET=%u,GPRS=%u,INTERNET=%u,CSQ=%d,HTTP_CODE=%lu,APN=%s,IP=%s,OP=%s,INFO=%s,BOOT14=%u,UP_MS=%lu,EVENT=%s,ERR=%s",
      cellularState.uartReady ? 1U : 0U,
      static_cast<unsigned long>(cellularState.uartBaud),
      static_cast<unsigned long>(cellularState.uartTargetBaud),
      static_cast<unsigned long>(cellularState.uartLastProbeBaud),
      cellularState.uartHardwareFlow ? 1U : 0U,
      cellularState.uartHighSpeedActive ? 1U : 0U,
      static_cast<int>(cellularState.uartCtsLevel),
      static_cast<int>(cellularState.uartRtsLevel),
      kModemEspTxPin,
      kModemEspRxPin,
      cellularState.modemReady ? 1U : 0U,
      cellularState.simReady ? 1U : 0U,
      cellularState.networkReady ? 1U : 0U,
      cellularState.gprsReady ? 1U : 0U,
      cellularState.internetOk ? 1U : 0U,
      cellularState.signalQuality,
      static_cast<unsigned long>(cellularState.httpStatusCode),
      cellularState.apn,
      cellularState.ipAddress,
      cellularState.operatorName,
      cellularState.modemInfo,
      millis() >= kModemPowerKeyBootWaitMs ? 1U : 0U,
      static_cast<unsigned long>(millis()),
      cellularState.lastModemEvent,
      cellularState.lastModemError);
  mqttPublishText("state/modem", payload, true, 1);

  snprintf(
      payload,
      sizeof(payload),
      "GNSS,STATE,UART=%u,READY=%u,CFG=%u,FIX_VALID=%u,FIX=%u,CARR=%u,SIV=%u,LAT_E7=%ld,LON_E7=%ld,ALT_MM=%ld,HACC_MM=%ld,VACC_MM=%ld,TOW_MS=%lu,PVT_COUNT=%lu,TP=%u,EVENT=%s,ERR=%s",
      gnssState.uartReady ? 1U : 0U,
      gnssState.moduleReady ? 1U : 0U,
      gnssState.configured ? 1U : 0U,
      gnssState.fixValid ? 1U : 0U,
      static_cast<unsigned int>(gnssState.fixType),
      static_cast<unsigned int>(gnssState.carrierSolution),
      static_cast<unsigned int>(gnssState.siv),
      static_cast<long>(gnssState.latitudeE7),
      static_cast<long>(gnssState.longitudeE7),
      static_cast<long>(gnssState.altitudeMslMm),
      static_cast<long>(gnssState.horizontalAccuracyMm),
      static_cast<long>(gnssState.verticalAccuracyMm),
      static_cast<unsigned long>(gnssState.timeOfWeekMs),
      static_cast<unsigned long>(gnssState.pvtCount),
      gnssState.timePulseHigh ? 1U : 0U,
      gnssState.lastEvent,
      gnssState.lastError);
  mqttPublishText("state/gnss", payload, true, 1);

  snprintf(
      payload,
      sizeof(payload),
      "NTRIP,STATE,CFG=%u,EN=%u,SOCK=%u,HOST=%s,PORT=%u,MOUNT=%s,RTCM_BYTES=%lu,GGA_TX=%lu,RECONNECTS=%lu,EVENT=%s,ERR=%s",
      cellularState.ntripConfigured ? 1U : 0U,
      cellularState.ntripEnabled ? 1U : 0U,
      cellularState.ntripConnected ? 1U : 0U,
      cellularState.ntripHost,
      static_cast<unsigned int>(cellularState.ntripPort),
      cellularState.ntripMount,
      static_cast<unsigned long>(cellularState.ntripBytesRx),
      static_cast<unsigned long>(cellularState.ntripGgaTxCount),
      static_cast<unsigned long>(cellularState.ntripReconnectCount),
      cellularState.lastNtripEvent,
      cellularState.lastNtripError);
  mqttPublishText("state/ntrip", payload, true, 1);

  snprintf(
      payload,
      sizeof(payload),
      "MQTT,STATE,CFG=%u,EN=%u,SOCK=%u,HOST=%s,PORT=%u,CLIENT=%s,PREFIX=%s,TX=%lu,RX=%lu,RECONNECTS=%lu,DROPPED=%lu,EVENT=%s,ERR=%s",
      cellularState.mqttConfigured ? 1U : 0U,
      cellularState.mqttEnabled ? 1U : 0U,
      cellularState.mqttConnected ? 1U : 0U,
      cellularState.mqttHost,
      static_cast<unsigned int>(cellularState.mqttPort),
      cellularState.mqttClientId,
      cellularState.mqttTopicPrefix,
      static_cast<unsigned long>(cellularState.mqttPublishCount),
      static_cast<unsigned long>(cellularState.mqttReceiveCount),
      static_cast<unsigned long>(cellularState.mqttReconnectCount),
      static_cast<unsigned long>(g_mqttClient.droppedMessages()),
      cellularState.lastMqttEvent,
      cellularState.lastMqttError);
  mqttPublishText("state/mqtt", payload, true, 1);
}

void disconnectMqttBroker(const char *eventText, const char *errorText) {
  const CellularState state = copyCellularState();
  if (state.mqttConnected) {
    mqttPublishText("availability", "offline", true, 1);
  }
  g_mqttClient.disconnect();
  g_mqttTransport.stop();

  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.mqttConnected = false;
    copyText(g_cellularState.lastMqttEvent, sizeof(g_cellularState.lastMqttEvent), eventText);
    copyText(g_cellularState.lastMqttError, sizeof(g_cellularState.lastMqttError), errorText);
    giveMutex(g_cellularStateMutex);
  }
}

void onMqttMessage(MQTTClient *client, char topic[], char bytes[], int length) {
  (void)client;
  (void)topic;
  if (length <= 0) {
    return;
  }

  int copyLength = length;
  if (copyLength >= static_cast<int>(sizeof(g_pendingMqttCommand))) {
    copyLength = static_cast<int>(sizeof(g_pendingMqttCommand)) - 1;
  }

  memcpy(g_pendingMqttCommand, bytes, static_cast<size_t>(copyLength));
  while (copyLength > 0 && (g_pendingMqttCommand[copyLength - 1] == '\r' || g_pendingMqttCommand[copyLength - 1] == '\n')) {
    copyLength--;
  }
  g_pendingMqttCommand[copyLength] = '\0';
  g_pendingMqttCommandReady = copyLength > 0;

  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.mqttReceiveCount++;
    g_cellularState.lastMqttRxMillis = millis();
    copyText(g_cellularState.lastMqttEvent, sizeof(g_cellularState.lastMqttEvent), "CMD_RX");
    copyText(g_cellularState.lastMqttError, sizeof(g_cellularState.lastMqttError), "-");
    giveMutex(g_cellularStateMutex);
  }
}

bool connectMqttBroker() {
  const CellularState state = copyCellularState();
  if (!state.mqttConfigured || !state.mqttEnabled || state.mqttHost[0] == '\0' || state.mqttClientId[0] == '\0') {
    return false;
  }

  if (!ensurePacketDataSession()) {
    return false;
  }

  char availabilityTopic[kMqttTopicBufferSize] = {};
  char controlTopic[kMqttTopicBufferSize] = {};
  if (!buildMqttTopic(state, "availability", availabilityTopic, sizeof(availabilityTopic)) ||
      !buildMqttTopic(state, "control/command", controlTopic, sizeof(controlTopic))) {
    setMqttEvent("CFG_FAIL", "TOPIC_PATH");
    return false;
  }

  g_mqttClient.begin(state.mqttHost, static_cast<int>(state.mqttPort), g_mqttTransport);
  g_mqttClient.onMessageAdvanced(onMqttMessage);
  g_mqttClient.setOptions(30, false, 1000);
  g_mqttClient.dropOverflow(false);
  g_mqttClient.setWill(availabilityTopic, "offline", true, 1);

  const bool connected = (state.mqttUser[0] != '\0' || state.mqttPass[0] != '\0')
      ? g_mqttClient.connect(state.mqttClientId, state.mqttUser, state.mqttPass)
      : g_mqttClient.connect(state.mqttClientId);
  if (!connected) {
    char errorText[32] = {};
    snprintf(
        errorText,
        sizeof(errorText),
        "RC=%d ERR=%d",
        static_cast<int>(g_mqttClient.returnCode()),
        static_cast<int>(g_mqttClient.lastError()));
    setMqttEvent("CONNECT_FAIL", errorText);
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      g_cellularState.mqttConnected = false;
      giveMutex(g_cellularStateMutex);
    }
    return false;
  }

  g_mqttClient.subscribe(controlTopic, 1);
  if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
    g_cellularState.mqttConnected = true;
    g_cellularState.mqttReconnectCount++;
    copyText(g_cellularState.lastMqttEvent, sizeof(g_cellularState.lastMqttEvent), "MQTT_OK");
    copyText(g_cellularState.lastMqttError, sizeof(g_cellularState.lastMqttError), "-");
    giveMutex(g_cellularStateMutex);
  }

  mqttPublishText("availability", "online", true, 1);
  publishMqttStateSnapshots();
  return true;
}

void processPendingMqttCommand() {
  if (!g_pendingMqttCommandReady) {
    return;
  }

  char command[kSerialCommandBufferSize] = {};
  strncpy(command, g_pendingMqttCommand, sizeof(command) - 1U);
  g_pendingMqttCommandReady = false;

  if (dispatchControlCommand(command)) {
    mqttPublishText("event/command", command, false, 1);
    g_mqttPublishStateRequested = true;
  } else {
    mqttPublishText("event/command_error", command, false, 1);
  }
}

void serviceMqttLink() {
  CellularState state = copyCellularState();

  if (g_mqttDisableRequested) {
    g_mqttDisableRequested = false;
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      g_cellularState.mqttEnabled = false;
      giveMutex(g_cellularStateMutex);
    }
    disconnectMqttBroker("MQTT_OFF", "-");
    return;
  }

  if (g_mqttEnableRequested) {
    g_mqttEnableRequested = false;
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      g_cellularState.mqttEnabled = true;
      giveMutex(g_cellularStateMutex);
    }
    state = copyCellularState();
  }

  if (!state.mqttEnabled) {
    if (state.mqttConnected) {
      disconnectMqttBroker("MQTT_OFF", "-");
    }
    return;
  }

  const uint32_t nowMs = millis();
  static uint32_t lastConnectAttemptMs = 0;
  static uint32_t lastPublishMs = 0;

  if (!state.mqttConnected) {
    if ((nowMs - lastConnectAttemptMs) >= kMqttReconnectPeriodMs) {
      lastConnectAttemptMs = nowMs;
      connectMqttBroker();
    }
    return;
  }

  if (!g_mqttClient.loop() || !g_mqttClient.connected()) {
    disconnectMqttBroker("MQTT_RETRY", "SOCKET_CLOSED");
    return;
  }

  processPendingMqttCommand();

  if (g_mqttPublishStateRequested || (nowMs - lastPublishMs) >= kMqttPublishPeriodMs) {
    lastPublishMs = nowMs;
    g_mqttPublishStateRequested = false;
    publishMqttStateSnapshots();
  }
}

bool modemServiceNeeded(const CellularState &state) {
  return g_modemBootPowerKeyPending ||
         g_modemPowerKeyBootPending ||
         g_modemSetupRequested ||
         g_modemPowerKeyRequested ||
         g_modemResetRequested ||
         g_modemConnectRequested ||
         g_modemDisconnectRequested ||
         g_modemHttpTestRequested ||
         state.mqttEnabled ||
         state.ntripEnabled ||
         state.gprsReady ||
         state.modemReady;
}

void modemServiceTask(void *parameter) {
  (void)parameter;
  configureModemUart(kModemStandardUartBaud, kModemUseHardwareFlowControl);
  for (;;) {
    const CellularState state = copyCellularState();
    if (!modemServiceNeeded(state)) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (g_modemBootPowerKeyPending) {
      g_modemBootPowerKeyPending = false;
      // Always issue a controlled startup pulse so modem power-on is deterministic.
      g_modemInitialized = false;
      g_modemPowerKeyBootPending = false;
      pulseModemPowerKey();
      setModemEvent("MODEM_PWRKEY", "BOOT");
      continue;
    }

    if (g_modemPowerKeyRequested) {
      g_modemPowerKeyRequested = false;
      g_modemInitialized = false;
      g_modemPowerKeyBootPending = false;
      pulseModemPowerKey();
      setModemEvent("MODEM_PWRKEY", "MANUAL");
      continue;
    }

    if (g_modemResetRequested) {
      g_modemResetRequested = false;
      pulseModemResetPin();
      if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
        g_cellularState.modemReady = false;
        g_cellularState.networkReady = false;
        g_cellularState.gprsReady = false;
        g_cellularState.internetOk = false;
        giveMutex(g_cellularStateMutex);
      }
      g_modemInitialized = false;
      g_modemSetupRequested = false;
      setModemEvent("MODEM_RESET", "-");
    }

    const bool needModemSetup = g_modemSetupRequested ||
                                g_modemConnectRequested ||
                                g_modemDisconnectRequested ||
                                g_modemHttpTestRequested ||
                                state.mqttEnabled ||
                                state.ntripEnabled ||
                                state.gprsReady ||
                                state.modemReady;
    if (!needModemSetup) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (!isModemBootWindowElapsed()) {
      setModemEvent("MODEM_BOOT_WAIT", "SETUP_GATED");
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (!ensureModemReady()) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (g_modemSetupRequested) {
      g_modemSetupRequested = false;
      setModemEvent("MODEM_SETUP_OK", "-");
    }

    if (g_modemDisconnectRequested) {
      g_modemDisconnectRequested = false;
      g_modem.gprsDisconnect();
      refreshModemNetworkSnapshot(true);
      setModemEvent("GPRS_OFF", "-");
    }

    if (g_modemConnectRequested) {
      g_modemConnectRequested = false;
      ensurePacketDataSession();
    }

    if (g_modemHttpTestRequested) {
      g_modemHttpTestRequested = false;
      runHttpConnectivityTest();
    }

    refreshModemNetworkSnapshot();
    serviceMqttLink();
    serviceNtripStream();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void putInt16Le(uint8_t *buffer, const int index, const int16_t value) {
  buffer[index] = static_cast<uint8_t>(value & 0xFF);
  buffer[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void putUint16Le(uint8_t *buffer, const int index, const uint16_t value) {
  buffer[index] = static_cast<uint8_t>(value & 0xFF);
  buffer[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void putUint32Le(uint8_t *buffer, const int index, const uint32_t value) {
  buffer[index] = static_cast<uint8_t>(value & 0xFF);
  buffer[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[index + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buffer[index + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

bool asm330WriteBytes(const uint8_t reg, const uint8_t *data, const size_t length) {
  return g_asm330Sensor->IO_Write(const_cast<uint8_t *>(data), reg, static_cast<uint16_t>(length)) == 0;
}

uint8_t asm330ReadRegister(const uint8_t reg) {
  uint8_t value = 0;
  if (g_asm330Sensor->Read_Reg(reg, &value) != ASM330LHH_OK) {
    return 0;
  }
  return value;
}

void asm330ReadRegisters(const uint8_t startReg, uint8_t *buffer, const size_t length) {
  memset(buffer, 0, length);
  g_asm330Sensor->IO_Read(buffer, startReg, static_cast<uint16_t>(length));
}

void asm330WriteRegister(const uint8_t reg, const uint8_t value) {
  uint8_t mutableValue = value;
  g_asm330Sensor->Write_Reg(reg, mutableValue);
}

bool asm330WriteRegisterChecked(const uint8_t reg, const uint8_t value) {
  uint8_t mutableValue = value;
  return g_asm330Sensor->Write_Reg(reg, mutableValue) == ASM330LHH_OK;
}

bool asm330UpdateRegisterBits(const uint8_t reg, const uint8_t mask, const uint8_t value) {
  const uint8_t current = asm330ReadRegister(reg);
  const uint8_t updated = static_cast<uint8_t>((current & ~mask) | (value & mask));
  return asm330WriteRegisterChecked(reg, updated);
}

bool asm330ResetDevice() {
  if (!asm330UpdateRegisterBits(kAsm330RegCtrl3C, kAsm330Ctrl3CSwReset, kAsm330Ctrl3CSwReset)) {
    return false;
  }

  const uint32_t startMs = millis();
  while ((millis() - startMs) < kAsm330ResetTimeoutMs) {
    if ((asm330ReadRegister(kAsm330RegCtrl3C) & kAsm330Ctrl3CSwReset) == 0U) {
      return true;
    }
    delay(1);
  }

  return false;
}

void printAsm330KeyRegisters(const char *prefix) {
  const uint8_t whoAmI = asm330ReadRegister(kAsm330RegWhoAmI);
  const uint8_t int1Ctrl = asm330ReadRegister(kAsm330RegInt1Ctrl);
  const uint8_t ctrl1Xl = asm330ReadRegister(kAsm330RegCtrl1Xl);
  const uint8_t ctrl2G = asm330ReadRegister(kAsm330RegCtrl2G);
  const uint8_t ctrl3C = asm330ReadRegister(kAsm330RegCtrl3C);
  const uint8_t ctrl4C = asm330ReadRegister(kAsm330RegCtrl4C);
  const uint8_t ctrl9Xl = asm330ReadRegister(kAsm330RegCtrl9Xl);
  const uint8_t status = asm330ReadRegister(kAsm330RegStatus);

  g_asm330LastWhoAmI = whoAmI;
  g_asm330LastStatus = status;
  g_asm330LastCtrl1Xl = ctrl1Xl;
  g_asm330LastCtrl2G = ctrl2G;
  g_asm330LastCtrl3C = ctrl3C;
  g_asm330LastCtrl4C = ctrl4C;
  g_asm330LastCtrl9Xl = ctrl9Xl;
  g_asm330LastInt1Ctrl = int1Ctrl;

  Serial.printf(
      "%s,ADDR=0x%02X,WHOAMI=0x%02X,CTRL1_XL=0x%02X,CTRL2_G=0x%02X,CTRL3_C=0x%02X,CTRL4_C=0x%02X,CTRL9_XL=0x%02X,INT1_CTRL=0x%02X,STATUS=0x%02X,DRDY_PIN=%d\n",
      prefix,
      g_asm330I2cAddress,
      whoAmI,
      ctrl1Xl,
      ctrl2G,
      ctrl3C,
      ctrl4C,
      ctrl9Xl,
      int1Ctrl,
      status,
      digitalRead(kAsm330DrdyPin));
}

void printAsm330InitState(const char *prefix) {
  Serial.printf(
  "%s,READY=%u,RETRY=%u,LAST_OK=%u,INIT_CYCLES=%lu,INIT_OK=%lu,INIT_FAIL=%lu,ADDR=0x%02X,WHOAMI=0x%02X,CTRL3_C=0x%02X,CTRL4_C=0x%02X,CTRL9_XL=0x%02X\n",
      prefix,
      g_imuReady ? 1U : 0U,
      g_asm330InitRetryEnabled ? 1U : 0U,
      g_asm330LastInitSucceeded ? 1U : 0U,
      static_cast<unsigned long>(g_asm330InitCycleCount),
      static_cast<unsigned long>(g_asm330InitSuccessCount),
      static_cast<unsigned long>(g_asm330InitFailureCount),
      g_asm330I2cAddress,
      g_asm330LastWhoAmI,
      g_asm330LastCtrl3C,
      g_asm330LastCtrl4C,
      g_asm330LastCtrl9Xl);
}

void printAsm330DebugSnapshot(const char *prefix) {
  Serial.printf(
  "%s,READY=%u,WHOAMI=0x%02X,IRQ=%lu,READ_ATTEMPTS=%lu,SAMPLES=%lu,INT_SRC=%lu,PIN_SRC=%lu,POLL_SRC=%lu,ZERO=%lu,NOSAMPLE=%lu,STATUS=0x%02X,DRDY_PIN=%d,SDA=%d,SCL=%d,SA0=%d,CS=%d\n",
      prefix,
      g_imuReady ? 1U : 0U,
      g_asm330LastWhoAmI,
      static_cast<unsigned long>(g_asm330IrqCount),
      static_cast<unsigned long>(g_asm330ReadAttemptCount),
      static_cast<unsigned long>(g_asm330SampleCount),
      static_cast<unsigned long>(g_asm330InterruptSourceCount),
      static_cast<unsigned long>(g_asm330PinReadyCount),
      static_cast<unsigned long>(g_asm330FallbackPollCount),
      static_cast<unsigned long>(g_asm330AllZeroSampleCount),
      static_cast<unsigned long>(g_asm330ConsecutiveNoSampleCount),
      g_asm330LastStatus,
      digitalRead(kAsm330DrdyPin),
      digitalRead(kAsm330SdaPin),
      digitalRead(kAsm330SclPin),
      digitalRead(kAsm330Sa0Pin),
      (kAsm330CsPin >= 0) ? digitalRead(kAsm330CsPin) : -1);

  Serial.print(prefix);
  Serial.print(",LAST_RAW,");
  printAsm330RawBuffer(g_asm330LastRawBytes, sizeof(g_asm330LastRawBytes));
  Serial.println();
}

void prepareAsm330I2cPins() {
  pinMode(kAsm330DrdyPin, INPUT);
  if (kAsm330CsPin >= 0) {
    pinMode(kAsm330CsPin, OUTPUT);
    digitalWrite(kAsm330CsPin, HIGH);
  }

  pinMode(kAsm330Sa0Pin, OUTPUT);
  digitalWrite(kAsm330Sa0Pin, kAsm330Sa0High ? HIGH : LOW);
}

void selectAsm330I2cAddress(const bool sa0High) {
  digitalWrite(kAsm330Sa0Pin, sa0High ? HIGH : LOW);
  delay(2);
  g_asm330I2cAddress = sa0High ? kAsm330I2cAddressHigh : kAsm330I2cAddressLow;
  g_asm330Sensor = sa0High ? &asm330SensorHigh : &asm330SensorLow;
}

void printAsm330I2cScan(const char *prefix) {
  const uint8_t addresses[] = {kAsm330I2cAddressLow, kAsm330I2cAddressHigh};
  for (size_t index = 0; index < (sizeof(addresses) / sizeof(addresses[0])); ++index) {
    const uint8_t address = addresses[index];
    imuI2c.beginTransmission(address);
    const uint8_t error = imuI2c.endTransmission(true);
    Serial.printf(
        "%s,ADDR=0x%02X,ACK=%u,ERR=%u,SDA=%d,SCL=%d,SA0=%d,CS=%d\n",
        prefix,
        address,
        error == 0U ? 1U : 0U,
        error,
        digitalRead(kAsm330SdaPin),
        digitalRead(kAsm330SclPin),
        digitalRead(kAsm330Sa0Pin),
        (kAsm330CsPin >= 0) ? digitalRead(kAsm330CsPin) : -1);
  }
}

bool initAsm330() {
  g_asm330InitCycleCount++;
  g_lastAsm330InitAttemptMs = millis();
  g_imuDataReady = false;
  g_asm330LastInitSucceeded = false;

  detachInterrupt(digitalPinToInterrupt(kAsm330DrdyPin));

  prepareAsm330I2cPins();

  imuI2c.begin(kAsm330SdaPin, kAsm330SclPin, kAsm330I2cHz);
  imuI2c.setTimeOut(20);
  selectAsm330I2cAddress(kAsm330Sa0High);
  delay(10);
  printAsm330I2cScan("BOOT,ASM330,I2C_SCAN");

  Serial.printf(
      "BOOT,ASM330,CFG,SDA=%d,SCL=%d,SA0_PIN=%d,SA0_LEVEL=%d,ADDR=0x%02X,CS=%d,CS_LEVEL=%d,SDA_LEVEL=%d,SCL_LEVEL=%d,DRDY=%d,I2C_HZ=%lu\n",
      kAsm330SdaPin,
      kAsm330SclPin,
      kAsm330Sa0Pin,
      kAsm330Sa0High ? 1 : 0,
      g_asm330I2cAddress,
      kAsm330CsPin,
      (kAsm330CsPin >= 0) ? digitalRead(kAsm330CsPin) : -1,
      digitalRead(kAsm330SdaPin),
      digitalRead(kAsm330SclPin),
      kAsm330DrdyPin,
      static_cast<unsigned long>(kAsm330I2cHz));

  uint8_t whoAmI = 0;
  const bool sa0Candidates[] = {kAsm330Sa0High, !kAsm330Sa0High};
  uint32_t attempt = 0;
  for (const bool sa0High : sa0Candidates) {
    selectAsm330I2cAddress(sa0High);
    Serial.printf(
        "BOOT,ASM330,PROBE,SA0=%u,ADDR=0x%02X,SDA=%d,SCL=%d,CS=%d\n",
        sa0High ? 1U : 0U,
        g_asm330I2cAddress,
        digitalRead(kAsm330SdaPin),
        digitalRead(kAsm330SclPin),
        (kAsm330CsPin >= 0) ? digitalRead(kAsm330CsPin) : -1);

    for (uint32_t retry = 1; retry <= kAsm330StartupRetries; ++retry) {
      ++attempt;
      whoAmI = asm330ReadRegister(kAsm330RegWhoAmI);
      g_asm330LastWhoAmI = whoAmI;
      Serial.printf(
          "BOOT,ASM330,WHOAMI_ATTEMPT,%lu,SA0=%u,ADDR=0x%02X,WHOAMI=0x%02X\n",
          static_cast<unsigned long>(attempt),
          sa0High ? 1U : 0U,
          g_asm330I2cAddress,
          whoAmI);
      if (whoAmI == kAsm330WhoAmIValue) {
        break;
      }
      delay(10);
    }

    if (whoAmI == kAsm330WhoAmIValue) {
      break;
    }
  }

  if (whoAmI != kAsm330WhoAmIValue) {
    g_asm330InitFailureCount++;
    Serial.printf("BOOT,ASM330,ERR,WHOAMI,0x%02X\n", whoAmI);
    printAsm330I2cScan("BOOT,ASM330,I2C_SCAN_FAIL");
    printAsm330KeyRegisters("BOOT,ASM330,REGS_FAIL");
    printAsm330InitState("ASM330,INIT,STATE");
    return false;
  }

  if (!asm330ResetDevice()) {
    g_asm330InitFailureCount++;
    Serial.println("BOOT,ASM330,ERR,RESET_TIMEOUT");
    printAsm330KeyRegisters("BOOT,ASM330,REGS_RESET_FAIL");
    printAsm330InitState("ASM330,INIT,STATE");
    return false;
  }

  g_asm330LastCtrl4C = asm330ReadRegister(kAsm330RegCtrl4C);
  Serial.printf("BOOT,ASM330,CTRL4_AFTER_RESET,0x%02X\n", g_asm330LastCtrl4C);
  if ((g_asm330LastCtrl4C & kAsm330Ctrl4CI2cDisable) != 0U) {
    g_asm330InitFailureCount++;
    Serial.printf("BOOT,ASM330,ERR,I2C_DISABLED,CTRL4_C=0x%02X\n", g_asm330LastCtrl4C);
    printAsm330I2cScan("BOOT,ASM330,I2C_SCAN_DISABLED");
    printAsm330KeyRegisters("BOOT,ASM330,REGS_I2C_DISABLED");
    printAsm330InitState("ASM330,INIT,STATE");
    return false;
  }

  if (g_asm330Sensor->begin() != ASM330LHH_OK ||
      g_asm330Sensor->Set_X_FS(kAsm330TargetAccelRangeG) != ASM330LHH_OK ||
      g_asm330Sensor->Set_G_FS(kAsm330TargetGyroRangeDps) != ASM330LHH_OK ||
      g_asm330Sensor->Set_X_ODR(kAsm330TargetOdrHz) != ASM330LHH_OK ||
      g_asm330Sensor->Set_G_ODR(kAsm330TargetOdrHz) != ASM330LHH_OK ||
      g_asm330Sensor->Enable_X() != ASM330LHH_OK ||
      g_asm330Sensor->Enable_G() != ASM330LHH_OK ||
      !asm330WriteRegisterChecked(kAsm330RegInt1Ctrl, kAsm330Int1DataReady)) {
    g_asm330InitFailureCount++;
    Serial.println("BOOT,ASM330,ERR,DRIVER_INIT_FAIL");
    printAsm330KeyRegisters("BOOT,ASM330,REGS_WRITE_FAIL");
    printAsm330InitState("ASM330,INIT,STATE");
    return false;
  }

  delay(5);

  g_asm330LastCtrl1Xl = asm330ReadRegister(kAsm330RegCtrl1Xl);
  g_asm330LastCtrl2G = asm330ReadRegister(kAsm330RegCtrl2G);
  g_asm330LastCtrl3C = asm330ReadRegister(kAsm330RegCtrl3C);
  g_asm330LastCtrl4C = asm330ReadRegister(kAsm330RegCtrl4C);
  g_asm330LastCtrl9Xl = asm330ReadRegister(kAsm330RegCtrl9Xl);
  g_asm330LastInt1Ctrl = asm330ReadRegister(kAsm330RegInt1Ctrl);

  const bool ctrl1Ok = g_asm330LastCtrl1Xl == kAsm330Ctrl1Xl416Hz16g;
  const bool ctrl2Ok = g_asm330LastCtrl2G == kAsm330Ctrl2G416Hz2000dps;
  const bool ctrl3Ok = (g_asm330LastCtrl3C & kAsm330Ctrl3CBduIfInc) == kAsm330Ctrl3CBduIfInc &&
                       (g_asm330LastCtrl3C & kAsm330Ctrl3CSwReset) == 0U;
  const bool ctrl4Ok = (g_asm330LastCtrl4C & kAsm330Ctrl4CI2cDisable) == kAsm330Ctrl4CI2cEnable;
  const bool ctrl9Ok = (g_asm330LastCtrl9Xl & kAsm330Ctrl9XlDeviceConf) != 0U;
  const bool int1Ok = g_asm330LastInt1Ctrl == kAsm330Int1DataReady;
  if (!ctrl1Ok || !ctrl2Ok || !ctrl3Ok || !ctrl4Ok || !ctrl9Ok || !int1Ok) {
    g_asm330LastInitSucceeded = false;
    g_asm330InitFailureCount++;
    Serial.printf(
        "BOOT,ASM330,ERR,CFG_VERIFY,CTRL1_XL=0x%02X,CTRL2_G=0x%02X,CTRL3_C=0x%02X,CTRL4_C=0x%02X,CTRL9_XL=0x%02X,INT1_CTRL=0x%02X\n",
        g_asm330LastCtrl1Xl,
        g_asm330LastCtrl2G,
        g_asm330LastCtrl3C,
        g_asm330LastCtrl4C,
        g_asm330LastCtrl9Xl,
        g_asm330LastInt1Ctrl);
    printAsm330KeyRegisters("BOOT,ASM330,REGS_CFG_FAIL");
    printAsm330InitState("ASM330,INIT,STATE");
    return false;
  }

  printAsm330KeyRegisters("BOOT,ASM330,REGS_AFTER_CFG");

  attachInterrupt(digitalPinToInterrupt(kAsm330DrdyPin), onAsm330DataReady, RISING);
  g_asm330LastInitSucceeded = true;
  g_asm330InitSuccessCount++;
  g_asm330InitRetryEnabled = false;
  printAsm330InitState("ASM330,INIT,STATE");
  Serial.println("BOOT,ASM330,OK,ODR=417HZ,ACC=16G,GYRO=2000DPS,DRV=ST");
  return true;
}

void serviceAsm330InitRetry() {
  if (!g_asm330InitRetryEnabled || g_imuReady) {
    return;
  }

  const uint32_t nowMs = millis();
  if ((nowMs - g_lastAsm330InitAttemptMs) < kAsm330RetryPeriodMs) {
    return;
  }

  Serial.printf("ASM330,INIT,ATTEMPT,%lu\n", static_cast<unsigned long>(g_asm330InitCycleCount + 1U));
  g_imuReady = initAsm330();
}

bool initTwai() {
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(kTwaiTxPin),
      static_cast<gpio_num_t>(kTwaiRxPin),
      TWAI_MODE_NORMAL);
  generalConfig.tx_queue_len = 16;
  generalConfig.rx_queue_len = 32;

  const twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
  const twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t result = twai_driver_install(&generalConfig, &timingConfig, &filterConfig);
  if (result != ESP_OK) {
    Serial.printf("BOOT,TWAI,ERR,INSTALL,%d\n", static_cast<int>(result));
    return false;
  }

  result = twai_start();
  if (result != ESP_OK) {
    Serial.printf("BOOT,TWAI,ERR,START,%d\n", static_cast<int>(result));
    twai_driver_uninstall();
    return false;
  }

  Serial.println("BOOT,TWAI,OK,500KBPS,NORMAL");
  return true;
}

bool initMcp2515() {
  pinMode(kSdCsPin, OUTPUT);
  digitalWrite(kSdCsPin, HIGH);

  pinMode(kMcp2515CsPin, OUTPUT);
  digitalWrite(kMcp2515CsPin, HIGH);
  pinMode(kMcp2515IntPin, INPUT_PULLUP);

  canSpi.begin(kMcp2515SckPin, kMcp2515MisoPin, kMcp2515MosiPin, kMcp2515CsPin);

  if (!takeMcpBus()) {
    Serial.println("BOOT,MCP2515,ERR,MUTEX");
    return false;
  }

  const uint8_t beginResult = mcp2515.begin(MCP_ANY, CAN_500KBPS, MCP_10MHZ);
  const uint8_t oneShotResult = (beginResult == CAN_OK) ? mcp2515.disOneShotTX() : CAN_FAILINIT;
  const uint8_t modeResult = (beginResult == CAN_OK) ? mcp2515.setMode(MCP_NORMAL) : CAN_FAILINIT;
  giveMcpBus();

  if (beginResult != CAN_OK || oneShotResult != CAN_OK || modeResult != CAN_OK) {
    Serial.printf(
        "BOOT,MCP2515,ERR,INIT,BEGIN=%u,ONESHOT=%u,MODE=%u\n",
        static_cast<unsigned int>(beginResult),
        static_cast<unsigned int>(oneShotResult),
        static_cast<unsigned int>(modeResult));
    return false;
  }

  g_mcpClockLabel = "10MHZ";
  g_lastMcpProfileSwitchMs = millis();
  attachInterrupt(digitalPinToInterrupt(kMcp2515IntPin), onMcp2515Interrupt, FALLING);
  Serial.printf("BOOT,MCP2515,CFG,CLOCK=%s,INT_PIN=%d,PULLUP=1\n", g_mcpClockLabel, kMcp2515IntPin);
  Serial.printf("BOOT,MCP2515,OK,500KBPS,OSC=%s,NORMAL,RX_INT\n", g_mcpClockLabel);
  return true;
}

void serviceMcpRecovery() {
  if (!kMcp2515EnableClockRecovery || !g_mcpReady || g_mcpRxFrameCount > 0U || g_twaiRxFrameCount == 0U) {
    return;
  }
}

void serviceMcpStatus() {
  if (!g_mcpReady) {
    return;
  }

  const uint32_t nowMs = millis();
  if ((nowMs - g_lastMcpStatusMs) < kMcp2515StatusPeriodMs) {
    return;
  }

  g_lastMcpStatusMs = nowMs;

  uint8_t errorFlags = g_mcpLastErrorFlags;
  uint8_t txErrorCount = 0;
  uint8_t rxErrorCount = 0;
  uint8_t rxPending = 0;
  errorFlags = mcp2515Fast.getErrorFlags();
  txErrorCount = mcp2515Fast.getTransmitErrorCount();
  rxErrorCount = mcp2515Fast.getReceiveErrorCount();
  rxPending = (mcp2515Fast.getInterruptFlags() & (kMcp2515InterruptRx0 | kMcp2515InterruptRx1)) != 0U ? 1U : 0U;
  g_mcpLastErrorFlags = errorFlags;

  Serial.printf(
      "MCP2515,STATE,INT=%u,PENDING=%u,EFLG=0x%02X,TEC=%u,REC=%u,RX_FRAMES=%lu,QUEUE_DROP=%lu,OSC=%s\n",
      digitalRead(kMcp2515IntPin) == LOW ? 1U : 0U,
      rxPending,
      errorFlags,
      txErrorCount,
      rxErrorCount,
      static_cast<unsigned long>(g_mcpRxFrameCount),
      static_cast<unsigned long>(g_mcpQueueDropCount),
      g_mcpClockLabel);
}

void serviceMcpErrorReport() {
  if (!g_mcpReady) {
    return;
  }

  const uint32_t nowMs = millis();
  if ((nowMs - g_lastMcpErrorReportMs) < kMcpErrorReportPeriodMs) {
    return;
  }
  g_lastMcpErrorReportMs = nowMs;

  const uint32_t overrunCount = g_mcpRxOverrunCount;
  if (overrunCount != g_reportedMcpRxOverrunCount) {
    g_reportedMcpRxOverrunCount = overrunCount;
    Serial.printf(
        "MCP2515,ERR,RX_OVERRUN,EFLG=0x%02X,COUNT=%lu,DROPPED=%lu\n",
        static_cast<uint8_t>(g_mcpLastOverrunEflg),
        static_cast<unsigned long>(overrunCount),
        static_cast<unsigned long>(g_mcpQueueDropCount));
  }

  const uint32_t readErrorCount = g_mcpRxReadErrorCount;
  if (readErrorCount != g_reportedMcpRxReadErrorCount) {
    g_reportedMcpRxReadErrorCount = readErrorCount;
    Serial.printf(
      "MCP2515,ERR,RX_READ,EFLG=0x%02X,STATUS=0x%02X,COUNT=%lu\n",
        static_cast<uint8_t>(g_mcpLastReadErrEflg),
        static_cast<uint8_t>(g_mcpLastReadErrCanintf),
        static_cast<unsigned long>(readErrorCount));
  }

  const uint32_t droppedCount = g_mcpQueueDropCount;
  if (droppedCount != g_reportedMcpQueueDropCount) {
    g_reportedMcpQueueDropCount = droppedCount;
    Serial.printf("MCP2515,WARN,RX_QUEUE_DROP,COUNT=%lu\n", static_cast<unsigned long>(droppedCount));
  }
}

bool initSdCard() {
  pinMode(kSdCsPin, OUTPUT);
  digitalWrite(kSdCsPin, HIGH);
  pinMode(kMcp2515CsPin, OUTPUT);
  digitalWrite(kMcp2515CsPin, HIGH);

  if (!takeCanSpiMutex()) {
    Serial.println("SD,ERR,MUTEX");
    return false;
  }

  const bool started = SD.begin(kSdCsPin, canSpi);
  // Ensure SD is deselected after init on the shared HSPI bus.
  digitalWrite(kSdCsPin, HIGH);
  giveCanSpiMutex();
  if (!started) {
    g_sdCardPresent = false;
    g_sdReadOk = false;
    g_sdWriteOk = false;
    Serial.println("SD,ERR,INIT");
    return false;
  }

  if (SD.cardType() == CARD_NONE) {
    g_sdCardPresent = false;
    g_sdReadOk = false;
    g_sdWriteOk = false;
    Serial.println("SD,ERR,NO_CARD");
    return false;
  }

  g_sdCardPresent = true;
  if (!runSdQuickProbe()) {
    Serial.println("SD,ERR,PROBE");
    return false;
  }
  Serial.println("SD,OK");
  return true;
}

void serviceSdHealthCheck() {
  const uint32_t nowMs = millis();
  if ((nowMs - g_lastSdHealthCheckMs) < kSdHealthCheckPeriodMs) {
    return;
  }
  g_lastSdHealthCheckMs = nowMs;

  // Fast path when we already know a card is present: just verify it is still visible.
  if (g_sdCardPresent) {
    if (!takeCanSpiMutex()) {
      return;
    }

    digitalWrite(kMcp2515CsPin, HIGH);
    const bool stillPresent = SD.cardType() != CARD_NONE;
    giveCanSpiMutex();

    if (!stillPresent) {
      Serial.println("SD,CARD,REMOVED");
      g_sdCardPresent = false;
      g_sdReady = false;
      g_sdError = true;
      g_sdReadOk = false;
      g_sdWriteOk = false;
      g_recordingRequested = false;
      closeLogFile();
      printSdState("SD,STATE");
      return;
    }

    // Recover readiness if we had a previous SD error while card remains inserted.
    if (!g_sdReady || !g_sdReadOk || !g_sdWriteOk) {
      g_sdReady = runSdQuickProbe();
      g_sdError = !g_sdReady;
      printSdState("SD,STATE");
    }
    return;
  }

  // Slow path for insertion detection when no card is currently present.
  if (!takeCanSpiMutex()) {
    return;
  }

  digitalWrite(kMcp2515CsPin, HIGH);
  const bool started = SD.begin(kSdCsPin, canSpi);
  const bool cardPresentNow = started && (SD.cardType() != CARD_NONE);
  giveCanSpiMutex();

  if (!cardPresentNow) {
    return;
  }

  Serial.println("SD,CARD,INSERTED");
  g_sdCardPresent = true;
  g_sdReady = runSdQuickProbe();
  g_sdError = !g_sdReady;
  printSdState("SD,STATE");
}

void serviceRecordButton() {
  const uint32_t nowMs = millis();
  const bool readPressed = digitalRead(kRecordButtonPin) == LOW;

  if (readPressed != g_buttonLastReadPressed) {
    g_buttonLastReadPressed = readPressed;
    g_buttonLastTransitionMs = nowMs;
  }

  if ((nowMs - g_buttonLastTransitionMs) < kRecordButtonDebounceMs) {
    return;
  }

  if (readPressed == g_buttonStablePressed) {
    return;
  }

  g_buttonStablePressed = readPressed;
  if (!g_buttonStablePressed) {
    return;
  }

  const bool enableRecording = !g_recordingRequested;
  if (!enableRecording) {
    g_recordingRequested = false;
    Serial.println("SD,RECORD,OFF,SRC=GPIO");
    closeLogFile();
    printSdState("SD,STATE");
    return;
  }

  g_recordingRequested = true;
  Serial.println("SD,RECORD,ON,SRC=GPIO");

  if (!g_sdReady || g_sdError) {
    Serial.println("SD,RECORD,REJECTED");
    g_recordingRequested = false;
    printSdState("SD,STATE");
    return;
  }

  if (!static_cast<bool>(g_logFile) && !openLogFile()) {
    Serial.println("SD,RECORD,REJECTED");
    g_recordingRequested = false;
  }
  printSdState("SD,STATE");
}

void serviceRecordingState() {
  if (!g_recordingRequested) {
    return;
  }

  if (!g_sdReady || g_sdError || static_cast<bool>(g_logFile)) {
    return;
  }

  if (!openLogFile()) {
    g_recordingRequested = false;
    Serial.println("SD,RECORD,REJECTED");
    printSdState("SD,STATE");
  }
}

bool readAsm330Sample(ImuSample &sample) {
  if (!g_imuReady) {
    return false;
  }

  const bool interruptReady = g_imuDataReady;
  const bool pinReady = digitalRead(kAsm330DrdyPin) == HIGH;
  const uint32_t nowUs = micros();
  const bool fallbackPoll = (nowUs - g_lastImuPollUs) >= 2500;

  g_asm330ConsecutiveNoSampleCount++;

  if (!interruptReady && !pinReady && !fallbackPoll) {
    return false;
  }

  g_asm330LastStatus = asm330ReadRegister(kAsm330RegStatus);
  const bool statusReady = (g_asm330LastStatus & kAsm330StatusAccelGyroReady) == kAsm330StatusAccelGyroReady;
  if (!statusReady) {
    if (interruptReady || pinReady) {
      g_imuDataReady = false;
    }
    g_lastImuPollUs = nowUs;
    return false;
  }

  g_asm330ReadAttemptCount++;
  if (interruptReady) {
    g_asm330InterruptSourceCount++;
  }
  if (pinReady) {
    g_asm330PinReadyCount++;
  }
  if (!interruptReady && !pinReady && fallbackPoll) {
    g_asm330FallbackPollCount++;
  }

  g_imuDataReady = false;
  g_lastImuPollUs = nowUs;

  if (g_asm330Sensor->Get_G_AxesRaw(sample.gyroRaw) != ASM330LHH_OK ||
      g_asm330Sensor->Get_X_AxesRaw(sample.accelRaw) != ASM330LHH_OK) {
    return false;
  }

  uint8_t rawBytes[14] = {};
  asm330ReadRegisters(kAsm330RegOutTempL, rawBytes, sizeof(rawBytes));
  memcpy(g_asm330LastRawBytes, rawBytes, sizeof(rawBytes));

  sample.tempRaw = static_cast<int16_t>((static_cast<uint16_t>(rawBytes[1]) << 8) | rawBytes[0]);

  for (int axis = 0; axis < 3; ++axis) {
    sample.gyroDps[axis] = static_cast<float>(sample.gyroRaw[axis]) * kGyroScaleDpsPerLsb;
    sample.accelG[axis] = static_cast<float>(sample.accelRaw[axis]) * kAccelScaleGPerLsb;
  }

  sample.accelMagnitudeG = sqrtf(
      (sample.accelG[0] * sample.accelG[0]) +
      (sample.accelG[1] * sample.accelG[1]) +
      (sample.accelG[2] * sample.accelG[2]));
  sample.hardStop = sample.accelMagnitudeG >= kHardStopThresholdG;
  sample.timestampMs = millis();

  if (sample.hardStop) {
    g_lastHardStopMs = sample.timestampMs;
  }

  bool allZero = true;
  for (size_t index = 0; index < sizeof(rawBytes); ++index) {
    if (rawBytes[index] != 0U) {
      allZero = false;
      break;
    }
  }

  if (allZero) {
    g_asm330AllZeroSampleCount++;
  }

  g_asm330SampleCount++;
  g_asm330ConsecutiveNoSampleCount = 0;

  return true;
}

void serviceAsm330Debug() {
  const uint32_t nowMs = millis();
  if ((nowMs - g_lastAsm330DebugMs) < kAsm330DebugPeriodMs) {
    return;
  }

  g_lastAsm330DebugMs = nowMs;
  printAsm330DebugSnapshot("ASMDBG");

  if ((g_asm330SampleCount == 0U) || (g_asm330ConsecutiveNoSampleCount > 2000U) || (g_asm330AllZeroSampleCount > 0U)) {
    printAsm330I2cScan("ASMDBG,I2C_SCAN");
    printAsm330KeyRegisters("ASMDBG,REGS");
  }
}

void setSdRecordingFromSerial(const bool enable) {
  if (enable) {
    g_recordingRequested = true;
    Serial.println("SD,RECORD,ON,SRC=SERIAL");

    if (!g_sdReady || g_sdError) {
      Serial.println("SD,RECORD,REJECTED");
      g_recordingRequested = false;
      printSdState("SD,STATE");
      return;
    }

    if (!static_cast<bool>(g_logFile) && !openLogFile()) {
      Serial.println("SD,RECORD,REJECTED");
      g_recordingRequested = false;
    }

    printSdState("SD,STATE");
    return;
  }

  g_recordingRequested = false;
  closeLogFile();
  Serial.println("SD,RECORD,OFF,SRC=SERIAL");
  printSdState("SD,STATE");
}

bool handleSdSerialCommand(const char *line) {
  if (strcmp(line, "SDSTATE") == 0) {
    printSdState("SD,STATE");
    return true;
  }

  if (strcmp(line, "SDREC ON") == 0) {
    setSdRecordingFromSerial(true);
    return true;
  }

  if (strcmp(line, "SDREC OFF") == 0) {
    setSdRecordingFromSerial(false);
    return true;
  }

  if (strcmp(line, "SDREC TOGGLE") == 0) {
    setSdRecordingFromSerial(!g_recordingRequested);
    return true;
  }

  if (strcmp(line, "SDHELP") == 0) {
    Serial.println("SD,CMD,SDSTATE|SDREC ON|SDREC OFF|SDREC TOGGLE");
    return true;
  }

  if (strncmp(line, "SD", 2) == 0) {
    Serial.printf("SD,ERR,UNKNOWN_CMD,%s\n", line);
    return true;
  }

  return false;
}

bool handleModemSerialCommand(const char *line) {
  if (strcmp(line, "MODEMSTATE") == 0) {
    // Trigger a fresh AT probe / modem health check via the modem task.
    g_modemSetupRequested = true;
    // Print whatever state we have right now; the modem task will update
    // the cached state on its next iteration for subsequent reads.
    printModemState("MODEM,STATE");
    printNtripState("NTRIP,STATE");
    printMqttState("MQTT,STATE");
    return true;
  }

  // Lightweight inline AT probe – prints MODEM,PROBE,OK or MODEM,PROBE,FAIL
  // without going through the full setup path.
  if (strcmp(line, "MODEMPROBE") == 0) {
    // Give the modem task a chance to configure UART2 first.
    vTaskDelay(pdMS_TO_TICKS(1));
    const bool atOk = g_modem.testAT(kModemAtProbeTimeoutMs);
    Serial.printf("MODEM,PROBE,%s\n", atOk ? "OK" : "FAIL");
    if (atOk) {
      // Update cached state so MODEMSTATE reflects the live result.
      if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
        g_cellularState.uartReady = true;
        giveMutex(g_cellularStateMutex);
      }
    }
    return true;
  }

  if (strcmp(line, "MQTTSTATE") == 0) {
    printMqttState("MQTT,STATE");
    return true;
  }

  if (strcmp(line, "MQTTON") == 0) {
    g_mqttEnableRequested = true;
    Serial.println("MQTT,CMD,ON");
    return true;
  }

  if (strcmp(line, "MQTTOFF") == 0) {
    g_mqttDisableRequested = true;
    Serial.println("MQTT,CMD,OFF");
    return true;
  }

  if (strcmp(line, "MODEMCONNECT") == 0) {
    g_modemConnectRequested = true;
    Serial.println("MODEM,CMD,CONNECT");
    return true;
  }

  if (strcmp(line, "MODEMSETUP") == 0) {
    g_modemSetupRequested = true;
    Serial.println("MODEM,CMD,SETUP");
    return true;
  }

  if (strcmp(line, "MODEMDISCONNECT") == 0) {
    g_modemDisconnectRequested = true;
    Serial.println("MODEM,CMD,DISCONNECT");
    return true;
  }

  if (strcmp(line, "MODEMPWRKEY") == 0) {
    g_modemPowerKeyRequested = true;
    Serial.println("MODEM,CMD,PWRKEY");
    return true;
  }

  if (strcmp(line, "MODEMRESET") == 0) {
    g_modemResetRequested = true;
    Serial.println("MODEM,CMD,RESET");
    return true;
  }

  if (strcmp(line, "BRIDGEONLY") == 0) {
    g_bridgeOnlyMode = true;
    Serial.println("BRIDGE,MODE,ONLY,CAN_UART_USB");
    return true;
  }

  if (strcmp(line, "BRIDGEFULL") == 0) {
    g_bridgeOnlyMode = false;
    Serial.println("BRIDGE,MODE,FULL,ALL_SUBSYSTEMS");
    return true;
  }

  if (strncmp(line, "MODEMAPN,", 9) == 0) {
    char buffer[kSerialCommandBufferSize] = {};
    strncpy(buffer, line + 9, sizeof(buffer) - 1U);
    char *savePtr = nullptr;
    char *apn = strtok_r(buffer, ",", &savePtr);
    char *user = strtok_r(nullptr, ",", &savePtr);
    char *pass = strtok_r(nullptr, "", &savePtr);
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      copyText(g_cellularState.apn, sizeof(g_cellularState.apn), apn == nullptr ? "" : apn);
      copyText(g_cellularState.apnUser, sizeof(g_cellularState.apnUser), user == nullptr ? "" : user);
      copyText(g_cellularState.apnPass, sizeof(g_cellularState.apnPass), pass == nullptr ? "" : pass);
      giveMutex(g_cellularStateMutex);
    }
    Serial.println("MODEM,CMD,APN_SET");
    printModemState("MODEM,STATE");
    return true;
  }

  if (strncmp(line, "MODEMHTTP,", 10) == 0) {
    char buffer[kSerialCommandBufferSize] = {};
    strncpy(buffer, line + 10, sizeof(buffer) - 1U);
    char *savePtr = nullptr;
    char *host = strtok_r(buffer, ",", &savePtr);
    char *path = strtok_r(nullptr, "", &savePtr);
    if (host == nullptr) {
      Serial.println("MODEM,ERR,HTTP_ARGS");
      return true;
    }

    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      copyText(g_cellularState.httpHost, sizeof(g_cellularState.httpHost), host);
      copyText(g_cellularState.httpPath, sizeof(g_cellularState.httpPath), (path == nullptr || path[0] == '\0') ? "/" : path);
      giveMutex(g_cellularStateMutex);
    }
    g_modemHttpTestRequested = true;
    Serial.println("MODEM,CMD,HTTP_TEST");
    return true;
  }

  if (strncmp(line, "MQTTCFG,", 8) == 0) {
    char buffer[kSerialCommandBufferSize] = {};
    strncpy(buffer, line + 8, sizeof(buffer) - 1U);
    char *savePtr = nullptr;
    char *host = strtok_r(buffer, ",", &savePtr);
    char *port = strtok_r(nullptr, ",", &savePtr);
    char *clientId = strtok_r(nullptr, ",", &savePtr);
    char *topicPrefix = strtok_r(nullptr, "", &savePtr);
    if (host == nullptr || port == nullptr || clientId == nullptr || topicPrefix == nullptr) {
      Serial.println("MQTT,ERR,CFG_ARGS");
      return true;
    }

    const unsigned long parsedPort = strtoul(port, nullptr, 10);
    if (parsedPort == 0UL || parsedPort > 65535UL) {
      Serial.println("MQTT,ERR,CFG_PORT");
      return true;
    }

    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      copyText(g_cellularState.mqttHost, sizeof(g_cellularState.mqttHost), host);
      g_cellularState.mqttPort = static_cast<uint16_t>(parsedPort);
      copyText(g_cellularState.mqttClientId, sizeof(g_cellularState.mqttClientId), clientId);
      copyText(g_cellularState.mqttTopicPrefix, sizeof(g_cellularState.mqttTopicPrefix), topicPrefix);
      g_cellularState.mqttConfigured = true;
      copyText(g_cellularState.lastMqttEvent, sizeof(g_cellularState.lastMqttEvent), "CFG_SET");
      copyText(g_cellularState.lastMqttError, sizeof(g_cellularState.lastMqttError), "-");
      giveMutex(g_cellularStateMutex);
    }

    Serial.println("MQTT,CMD,CFG_SET");
    printMqttState("MQTT,STATE");
    return true;
  }

  if (strncmp(line, "MQTTAUTH,", 9) == 0) {
    char buffer[kSerialCommandBufferSize] = {};
    strncpy(buffer, line + 9, sizeof(buffer) - 1U);
    char *savePtr = nullptr;
    char *user = strtok_r(buffer, ",", &savePtr);
    char *pass = strtok_r(nullptr, "", &savePtr);
    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      copyText(g_cellularState.mqttUser, sizeof(g_cellularState.mqttUser), (user == nullptr || strcmp(user, "-") == 0) ? "" : user);
      copyText(g_cellularState.mqttPass, sizeof(g_cellularState.mqttPass), (pass == nullptr || strcmp(pass, "-") == 0) ? "" : pass);
      copyText(g_cellularState.lastMqttEvent, sizeof(g_cellularState.lastMqttEvent), "AUTH_SET");
      copyText(g_cellularState.lastMqttError, sizeof(g_cellularState.lastMqttError), "-");
      giveMutex(g_cellularStateMutex);
    }

    Serial.println("MQTT,CMD,AUTH_SET");
    printMqttState("MQTT,STATE");
    return true;
  }

  if (strcmp(line, "MODEMHELP") == 0) {
    Serial.println("MODEM,CMD,MODEMSTATE|MODEMSETUP|MODEMCONNECT|MODEMDISCONNECT|MODEMPWRKEY|MODEMRESET|BRIDGEONLY|BRIDGEFULL|MODEMAPN,<apn>,<user>,<pass>|MODEMHTTP,<host>,<path>|MQTTSTATE|MQTTON|MQTTOFF|MQTTCFG,<host>,<port>,<client>,<prefix>|MQTTAUTH,<user>,<pass>");
    return true;
  }

  if (strncmp(line, "MODEM", 5) == 0 || strncmp(line, "MQTT", 4) == 0) {
    Serial.printf("MODEM,ERR,UNKNOWN_CMD,%s\n", line);
    return true;
  }

  return false;
}

bool handleGnssSerialCommand(const char *line) {
  if (strcmp(line, "GNSSSTATE") == 0) {
    printGnssState("GNSS,STATE");
    return true;
  }

  if (strcmp(line, "GNSSRESET") == 0) {
    requestGnssHardwareReset();
    Serial.println("GNSS,CMD,RESET");
    return true;
  }

  if (strncmp(line, "NTRIPCFG,", 9) == 0) {
    char buffer[kSerialCommandBufferSize] = {};
    strncpy(buffer, line + 9, sizeof(buffer) - 1U);
    char *savePtr = nullptr;
    char *host = strtok_r(buffer, ",", &savePtr);
    char *port = strtok_r(nullptr, ",", &savePtr);
    char *mount = strtok_r(nullptr, ",", &savePtr);
    char *user = strtok_r(nullptr, ",", &savePtr);
    char *pass = strtok_r(nullptr, "", &savePtr);
    if (host == nullptr || port == nullptr || mount == nullptr) {
      Serial.println("NTRIP,ERR,CFG_ARGS");
      return true;
    }

    const unsigned long parsedPort = strtoul(port, nullptr, 10);
    if (parsedPort == 0UL || parsedPort > 65535UL) {
      Serial.println("NTRIP,ERR,CFG_PORT");
      return true;
    }

    if (takeMutex(g_cellularStateMutex, pdMS_TO_TICKS(10))) {
      copyText(g_cellularState.ntripHost, sizeof(g_cellularState.ntripHost), host);
      copyText(g_cellularState.ntripMount, sizeof(g_cellularState.ntripMount), mount);
      copyText(g_cellularState.ntripUser, sizeof(g_cellularState.ntripUser), user == nullptr ? "" : user);
      copyText(g_cellularState.ntripPass, sizeof(g_cellularState.ntripPass), pass == nullptr ? "" : pass);
      g_cellularState.ntripPort = static_cast<uint16_t>(parsedPort);
      g_cellularState.ntripConfigured = true;
      copyText(g_cellularState.lastNtripEvent, sizeof(g_cellularState.lastNtripEvent), "CFG_SET");
      copyText(g_cellularState.lastNtripError, sizeof(g_cellularState.lastNtripError), "-");
      giveMutex(g_cellularStateMutex);
    }

    Serial.println("NTRIP,CMD,CFG_SET");
    printNtripState("NTRIP,STATE");
    return true;
  }

  if (strcmp(line, "NTRIPON") == 0) {
    g_ntripEnableRequested = true;
    Serial.println("NTRIP,CMD,ON");
    return true;
  }

  if (strcmp(line, "NTRIPOFF") == 0) {
    g_ntripDisableRequested = true;
    Serial.println("NTRIP,CMD,OFF");
    return true;
  }

  if (strcmp(line, "NTRIPSTATE") == 0) {
    printNtripState("NTRIP,STATE");
    return true;
  }

  if (strcmp(line, "GNSSHELP") == 0) {
    Serial.println("GNSS,CMD,GNSSSTATE|GNSSRESET|NTRIPCFG,<host>,<port>,<mount>,<user>,<pass>|NTRIPON|NTRIPOFF|NTRIPSTATE");
    return true;
  }

  if (strncmp(line, "GNSS", 4) == 0 || strncmp(line, "NTRIP", 5) == 0) {
    Serial.printf("GNSS,ERR,UNKNOWN_CMD,%s\n", line);
    return true;
  }

  return false;
}

void handleAsm330SerialCommand(const char *line) {
  if (strcmp(line, "ASMDBG") == 0) {
    printAsm330DebugSnapshot("ASMDBG,CMD");
    return;
  }

  if (strcmp(line, "ASMDUMP") == 0) {
    printAsm330KeyRegisters("ASMDBG,DUMP");
    printAsm330DebugSnapshot("ASMDBG,DUMP");
    return;
  }

  if (strcmp(line, "ASMRAW") == 0) {
    uint8_t rawBytes[14] = {};
    asm330ReadRegisters(kAsm330RegOutTempL, rawBytes, sizeof(rawBytes));
    memcpy(g_asm330LastRawBytes, rawBytes, sizeof(rawBytes));
    Serial.print("ASMDBG,RAW,");
    printAsm330RawBuffer(rawBytes, sizeof(rawBytes));
    Serial.println();
    return;
  }

  if (strcmp(line, "ASMINIT") == 0) {
    g_asm330InitRetryEnabled = true;
    g_imuReady = false;
    g_asm330LastInitSucceeded = false;
    g_lastAsm330InitAttemptMs = 0;
    Serial.println("ASM330,INIT,START");
    printAsm330InitState("ASM330,INIT,STATE");
    return;
  }

  if (strcmp(line, "ASMINITSTOP") == 0) {
    g_asm330InitRetryEnabled = false;
    Serial.println("ASM330,INIT,STOP");
    printAsm330InitState("ASM330,INIT,STATE");
    return;
  }

  if (strcmp(line, "ASMSTATE") == 0) {
    printAsm330InitState("ASM330,INIT,STATE");
    printAsm330KeyRegisters("ASM330,INIT,REGS");
    return;
  }

  if (strncmp(line, "ASMREG ", 7) == 0) {
    const unsigned long reg = strtoul(line + 7, nullptr, 16);
    if (reg <= 0x7FUL) {
      const uint8_t value = asm330ReadRegister(static_cast<uint8_t>(reg));
      Serial.printf("ASMDBG,REG,0x%02lX,0x%02X\n", reg, value);
      return;
    }
  }

  Serial.printf("ASMDBG,ERR,UNKNOWN_CMD,%s\n", line);
}

void handleTxSerialCommand(const char *line);

bool dispatchControlCommand(const char *line) {
  if (line == nullptr || line[0] == '\0') {
    return false;
  }

  if (strncmp(line, "TX,", 3) == 0) {
    handleTxSerialCommand(line);
    return true;
  }

  if (handleSdSerialCommand(line) || handleModemSerialCommand(line) || handleGnssSerialCommand(line)) {
    return true;
  }

  if (strncmp(line, "ASM", 3) == 0) {
    handleAsm330SerialCommand(line);
    return true;
  }

  Serial.printf("CMD,ERR,UNKNOWN_CMD,%s\n", line);
  return false;
}

void serviceSerialCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if ((ch == '\r') || (ch == '\n')) {
      if (g_serialCommandLength == 0U) {
        continue;
      }

      g_serialCommandBuffer[g_serialCommandLength] = '\0';
      dispatchControlCommand(g_serialCommandBuffer);
      g_serialCommandLength = 0U;
      continue;
    }

    if (g_serialCommandLength < (sizeof(g_serialCommandBuffer) - 1U)) {
      g_serialCommandBuffer[g_serialCommandLength++] = ch;
    } else {
      g_serialCommandLength = 0U;
      Serial.println("CMD,ERR,CMD_TOO_LONG");
    }
  }
}

void printCanPayload(const uint8_t *data, const uint8_t dlc) {
  for (uint8_t index = 0; index < dlc; ++index) {
    Serial.printf("%02X", data[index]);
    if (index + 1U < dlc) {
      Serial.print(' ');
    }
  }
}

void printTwaiFrame(const twai_message_t &message) {
  Serial.print("CAN1,");
  Serial.print(millis());
  Serial.print(',');
  Serial.print(message.extd ? "E," : "S,");
  if (message.extd) {
    Serial.printf("%08lX,", static_cast<unsigned long>(message.identifier & 0x1FFFFFFFUL));
  } else {
    Serial.printf("%03lX,", static_cast<unsigned long>(message.identifier & 0x7FFU));
  }
  Serial.print(message.data_length_code);
  Serial.print(',');
  printCanPayload(message.data, message.data_length_code);
  Serial.println();

  String line = "CAN1,";
  line += String(millis());
  line += ",";
  line += (message.extd ? "E," : "S,");
  line += String(message.identifier, HEX);
  line += ",";
  line += String(message.data_length_code);
  line += ",";
  for (uint8_t index = 0; index < message.data_length_code; ++index) {
    if (message.data[index] < 16U) {
      line += '0';
    }
    line += String(message.data[index], HEX);
    if (index + 1U < message.data_length_code) {
      line += ' ';
    }
  }
  logLine(line);
}

void printMcpFrame(const can_frame &frame, const uint32_t timestampMs) {
  const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  const uint32_t identifier = frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);

  Serial.print("CAN2,");
  Serial.print(timestampMs);
  Serial.print(',');
  Serial.print(extended ? "E," : "S,");
  if (extended) {
    Serial.printf("%08lX,", static_cast<unsigned long>(identifier));
  } else {
    Serial.printf("%03lX,", static_cast<unsigned long>(identifier));
  }
  Serial.print(frame.can_dlc);
  Serial.print(',');
  printCanPayload(frame.data, frame.can_dlc);
  Serial.println();

  String line = "CAN2,";
  line += String(timestampMs);
  line += ",";
  line += (extended ? "E," : "S,");
  line += String(identifier, HEX);
  line += ",";
  line += String(frame.can_dlc);
  line += ",";
  for (uint8_t index = 0; index < frame.can_dlc; ++index) {
    if (frame.data[index] < 16U) {
      line += '0';
    }
    line += String(frame.data[index], HEX);
    if (index + 1U < frame.can_dlc) {
      line += ' ';
    }
  }
  logLine(line);
}

void serviceTwaiRx(const uint32_t maxFrames) {
  if (!g_twaiReady) {
    return;
  }

  twai_message_t message = {};
  uint32_t drainedCount = 0;
  while (twai_receive(&message, 0) == ESP_OK) {
    ++g_twaiRxFrameCount;
    printTwaiFrame(message);
    ++drainedCount;
    if (drainedCount >= maxFrames) {
      break;
    }
  }
}

void serviceTwaiStatus() {
  if (!g_twaiReady) {
    return;
  }

  const uint32_t nowMs = millis();
  if ((nowMs - g_lastTwaiStatusMs) < kTwaiStatusPeriodMs) {
    return;
  }

  g_lastTwaiStatusMs = nowMs;

  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    Serial.println("TWAI,STATUS,ERR");
    return;
  }

  Serial.printf(
      "TWAI,STATE=%d,RX_QUEUE=%lu,TX_QUEUE=%lu,RX_ERR=%lu,TX_ERR=%lu,RX_FRAMES=%lu\n",
      static_cast<int>(status.state),
      static_cast<unsigned long>(status.msgs_to_rx),
      static_cast<unsigned long>(status.msgs_to_tx),
      static_cast<unsigned long>(status.rx_error_counter),
      static_cast<unsigned long>(status.tx_error_counter),
      static_cast<unsigned long>(g_twaiRxFrameCount));
}

uint32_t serviceMcpRx() {
  if (!g_mcpReady) {
    return 0;
  }

  can_frame frame = {};
  uint32_t drainedCount = 0;
  while (true) {
    const Mcp2515Driver::Error readStatus = mcp2515Fast.readMessage(&frame);

    if (readStatus == Mcp2515Driver::Error::NoMessage) {
      break;
    }

    if (readStatus != Mcp2515Driver::Error::Ok) {
      ++g_mcpRxReadErrorCount;
      g_mcpLastReadErrEflg = g_mcpLastErrorFlags;
      g_mcpLastReadErrCanintf = static_cast<uint8_t>(readStatus);
      break;
    }

    ++drainedCount;
    ++g_mcpRxFrameCount;
    if (g_mcpServiceTaskHandle != nullptr && g_mcpRxQueue != nullptr) {
      QueuedCanFrame queuedFrame = {};
      queuedFrame.frame = frame;
      queuedFrame.timestampMs = millis();
      if (xQueueSend(g_mcpRxQueue, &queuedFrame, 0) != pdPASS) {
        ++g_mcpQueueDropCount;
      }
    } else {
      printMcpFrame(frame, millis());
    }
  }

  if (drainedCount > 0U || digitalRead(kMcp2515IntPin) == LOW) {
    const uint8_t errorFlags = mcp2515Fast.getErrorFlags();
    if ((errorFlags & (kMcp2515EflgRx0Ovr | kMcp2515EflgRx1Ovr)) != 0U) {
      ++g_mcpRxOverrunCount;
      g_mcpLastOverrunEflg = errorFlags;
      mcp2515Fast.clearRxOverflow();
    }
    mcp2515Fast.clearErrorInterrupts();
    g_mcpLastErrorFlags = errorFlags;
  }

  return drainedCount;
}

void serviceMcpRxQueue(const uint32_t maxFrames) {
  if (g_mcpRxQueue == nullptr) {
    return;
  }

  QueuedCanFrame queuedFrame = {};
  uint32_t drainedCount = 0;
  while (xQueueReceive(g_mcpRxQueue, &queuedFrame, 0) == pdTRUE) {
    printMcpFrame(queuedFrame.frame, queuedFrame.timestampMs);
    ++drainedCount;
    if (drainedCount >= maxFrames) {
      break;
    }
  }
}

void mcpServiceTask(void *parameter) {
  (void)parameter;
  for (;;) {
    if (digitalRead(kMcp2515IntPin) != LOW) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    } else {
      ulTaskNotifyTake(pdTRUE, 0);
    }
    const uint32_t drainedCount = serviceMcpRx();
    if (drainedCount > 0U || digitalRead(kMcp2515IntPin) == LOW) {
      taskYIELD();
    }
  }
}

void sendTwaiFrameRaw(const uint32_t identifier, const bool extended, const uint8_t *payload, const uint8_t length) {
  if (!g_twaiReady) {
    return;
  }

  twai_message_t message = {};
  message.identifier = identifier;
  message.extd = extended ? 1 : 0;
  message.rtr = 0;
  message.ss = 0;
  message.self = 0;
  message.dlc_non_comp = 0;
  const uint8_t clampedLength = (length <= 8U) ? length : 8U;
  message.data_length_code = clampedLength;
  memcpy(message.data, payload, clampedLength);

  const esp_err_t result = twai_transmit(&message, 0);
  if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
    Serial.printf("ERR,TWAI_TX,%d\n", static_cast<int>(result));
  }
}

void sendMcpFrameRaw(const uint32_t identifier, const bool extended, const uint8_t *payload, const uint8_t length) {
  if (!g_mcpReady) {
    return;
  }

  const uint8_t clampedLength = (length <= 8U) ? length : 8U;
  Mcp2515Driver::Error sendStatus = Mcp2515Driver::Error::Fail;
  uint8_t errorFlags = g_mcpLastErrorFlags;
  uint8_t txErrorCount = 0;

  can_frame frame = {};
  frame.can_id = extended ? ((identifier & CAN_EFF_MASK) | CAN_EFF_FLAG) : (identifier & CAN_SFF_MASK);
  frame.can_dlc = clampedLength;
  memcpy(frame.data, payload, clampedLength);
  sendStatus = mcp2515Fast.sendMessage(&frame);
  errorFlags = mcp2515Fast.getErrorFlags();
  txErrorCount = mcp2515Fast.getTransmitErrorCount();

  g_mcpLastErrorFlags = errorFlags;

  if (sendStatus != Mcp2515Driver::Error::Ok) {
    Serial.printf("ERR,MCP_TX,EFLG=0x%02X,STATUS=%u,TEC=%u\n", errorFlags, static_cast<uint8_t>(sendStatus), txErrorCount);
  }
}

void sendTwaiFrame(const uint16_t identifier, const uint8_t *payload, const uint8_t length) {
  sendTwaiFrameRaw(identifier, false, payload, length);
}

void sendMcpFrame(const uint16_t identifier, const uint8_t *payload, const uint8_t length) {
  sendMcpFrameRaw(identifier, false, payload, length);
}

bool parseUIntToken(const char *token, const int base, unsigned long &value) {
  if (token == nullptr || token[0] == '\0') {
    return false;
  }

  char *endPtr = nullptr;
  const unsigned long parsed = strtoul(token, &endPtr, base);
  if ((endPtr == token) || (*endPtr != '\0')) {
    return false;
  }

  value = parsed;
  return true;
}

bool parseHexByteToken(const char *token, uint8_t &value) {
  unsigned long parsed = 0;
  if (!parseUIntToken(token, 16, parsed) || parsed > 0xFFUL) {
    return false;
  }

  value = static_cast<uint8_t>(parsed);
  return true;
}

bool parsePayloadList(char *payloadText, const uint8_t expectedLength, uint8_t *payloadOut) {
  if (expectedLength == 0U) {
    return true;
  }

  if (payloadText == nullptr || payloadText[0] == '\0') {
    return false;
  }

  uint8_t parsedLength = 0;
  char *token = strtok(payloadText, " ");
  while (token != nullptr) {
    if (parsedLength >= expectedLength) {
      return false;
    }

    uint8_t value = 0;
    if (!parseHexByteToken(token, value)) {
      return false;
    }

    payloadOut[parsedLength++] = value;
    token = strtok(nullptr, " ");
  }

  return parsedLength == expectedLength;
}

void handleTxSerialCommand(const char *line) {
  char commandBuffer[96] = {};
  strncpy(commandBuffer, line, sizeof(commandBuffer) - 1U);

  char *savePtr = nullptr;
  char *token = strtok_r(commandBuffer, ",", &savePtr);
  if ((token == nullptr) || (strcmp(token, "TX") != 0)) {
    Serial.println("CMD,ERR,TX_FORMAT");
    return;
  }

  char *token2 = strtok_r(nullptr, ",", &savePtr);
  if (token2 == nullptr) {
    Serial.println("CMD,ERR,TX_FORMAT");
    return;
  }

  char busSelector = 'B';
  char frameType = 'S';
  char *idToken = nullptr;
  char *dlcToken = nullptr;
  char *payloadToken = nullptr;

  if ((strcmp(token2, "S") == 0) || (strcmp(token2, "E") == 0)) {
    frameType = token2[0];
    idToken = strtok_r(nullptr, ",", &savePtr);
    dlcToken = strtok_r(nullptr, ",", &savePtr);
    payloadToken = strtok_r(nullptr, "", &savePtr);
  } else {
    busSelector = static_cast<char>(toupper(static_cast<unsigned char>(token2[0])));

    char *frameToken = strtok_r(nullptr, ",", &savePtr);
    idToken = strtok_r(nullptr, ",", &savePtr);
    dlcToken = strtok_r(nullptr, ",", &savePtr);
    payloadToken = strtok_r(nullptr, "", &savePtr);

    if (frameToken == nullptr || ((strcmp(frameToken, "S") != 0) && (strcmp(frameToken, "E") != 0))) {
      Serial.println("CMD,ERR,TX_FRAME_TYPE");
      return;
    }
    frameType = frameToken[0];
  }

  if (idToken == nullptr || dlcToken == nullptr) {
    Serial.println("CMD,ERR,TX_ARGS");
    return;
  }

  unsigned long parsedId = 0;
  if (!parseUIntToken(idToken, 16, parsedId)) {
    Serial.println("CMD,ERR,TX_ID");
    return;
  }

  unsigned long parsedDlc = 0;
  if (!parseUIntToken(dlcToken, 10, parsedDlc) || parsedDlc > 8UL) {
    Serial.println("CMD,ERR,TX_DLC");
    return;
  }

  const bool extended = frameType == 'E';
  if ((!extended && parsedId > 0x7FFUL) || (extended && parsedId > 0x1FFFFFFFUL)) {
    Serial.println("CMD,ERR,TX_ID_RANGE");
    return;
  }

  uint8_t payload[8] = {};
  if (parsedDlc > 0UL) {
    if (!parsePayloadList(payloadToken, static_cast<uint8_t>(parsedDlc), payload)) {
      Serial.println("CMD,ERR,TX_PAYLOAD");
      return;
    }
  }

  if (busSelector == '2') {
    Serial.println("CMD,ERR,TX_CAN2_DISABLED");
    return;
  }

  const bool sendTwai = (busSelector == '1') || (busSelector == 'A') || (busSelector == 'B');
  const bool sendMcp = false;
  if (!sendTwai) {
    Serial.println("CMD,ERR,TX_BUS");
    return;
  }

  if (sendTwai) {
    sendTwaiFrameRaw(static_cast<uint32_t>(parsedId), extended, payload, static_cast<uint8_t>(parsedDlc));
  }
  if (sendMcp) {
    sendMcpFrameRaw(static_cast<uint32_t>(parsedId), extended, payload, static_cast<uint8_t>(parsedDlc));
  }

  Serial.printf("CMD,OK,TX,%c,%c,%lX,%lu\n", busSelector, frameType, parsedId, parsedDlc);
}

void publishImuToCan(const ImuSample &sample) {
  uint8_t accelPayload[8] = {};
  uint8_t gyroPayload[8] = {};
  uint8_t statusPayload[8] = {};

  const int16_t accelMgX = static_cast<int16_t>(sample.accelG[0] * 1000.0f);
  const int16_t accelMgY = static_cast<int16_t>(sample.accelG[1] * 1000.0f);
  const int16_t accelMgZ = static_cast<int16_t>(sample.accelG[2] * 1000.0f);
  const int16_t gyroDps10X = static_cast<int16_t>(sample.gyroDps[0] * 10.0f);
  const int16_t gyroDps10Y = static_cast<int16_t>(sample.gyroDps[1] * 10.0f);
  const int16_t gyroDps10Z = static_cast<int16_t>(sample.gyroDps[2] * 10.0f);
  const uint16_t accelMagnitudeMg = static_cast<uint16_t>(sample.accelMagnitudeG * 1000.0f);

  putInt16Le(accelPayload, 0, accelMgX);
  putInt16Le(accelPayload, 2, accelMgY);
  putInt16Le(accelPayload, 4, accelMgZ);
  accelPayload[6] = static_cast<uint8_t>(g_sampleCounter & 0xFFU);
  accelPayload[7] = sample.hardStop ? 0x01 : 0x00;

  putInt16Le(gyroPayload, 0, gyroDps10X);
  putInt16Le(gyroPayload, 2, gyroDps10Y);
  putInt16Le(gyroPayload, 4, gyroDps10Z);
  gyroPayload[6] = static_cast<uint8_t>(g_sampleCounter & 0xFFU);
  gyroPayload[7] = sample.hardStop ? 0x01 : 0x00;

  putUint32Le(statusPayload, 0, sample.timestampMs);
  putUint16Le(statusPayload, 4, accelMagnitudeMg);
  putUint16Le(statusPayload, 6, static_cast<uint16_t>(g_sampleCounter & 0xFFFFU));

  sendTwaiFrame(kImuCanIdAccel, accelPayload, sizeof(accelPayload));
  sendTwaiFrame(kImuCanIdGyro, gyroPayload, sizeof(gyroPayload));
  sendTwaiFrame(kImuCanIdStatus, statusPayload, sizeof(statusPayload));
}

void printImuSample(const ImuSample &sample) {
  Serial.printf(
      "IMU,%lu,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.3f,%u\n",
      static_cast<unsigned long>(sample.timestampMs),
      sample.accelG[0],
      sample.accelG[1],
      sample.accelG[2],
      sample.gyroDps[0],
      sample.gyroDps[1],
      sample.gyroDps[2],
      sample.accelMagnitudeG,
      sample.hardStop ? 1U : 0U);

  String line = "IMU,";
  line += String(sample.timestampMs);
  line += ",S,700,8,";
  line += String(sample.accelG[0], 4);
  line += ' ';
  line += String(sample.accelG[1], 4);
  line += ' ';
  line += String(sample.accelG[2], 4);
  line += ' ';
  line += String(sample.gyroDps[0], 2);
  line += ' ';
  line += String(sample.gyroDps[1], 2);
  line += ' ';
  line += String(sample.gyroDps[2], 2);
  line += ' ';
  line += String(sample.accelMagnitudeG, 3);
  line += ' ';
  line += String(sample.hardStop ? 1 : 0);
  logLine(line);
}

void updateLeds() {
  const uint32_t nowMs = millis();

  if (isRecordingActive()) {
    setLed(kGreenLedPin, true);
  } else {
    if ((nowMs - g_lastHeartbeatToggleMs) >= kHeartbeatHalfPeriodMs) {
      g_lastHeartbeatToggleMs = nowMs;
      g_greenLedState = !g_greenLedState;
    }
    setLed(kGreenLedPin, g_greenLedState);
  }

  if (g_recordingRequested && (g_sdError || !g_sdReady)) {
    if ((nowMs - g_lastRedBlinkToggleMs) >= kSdErrorHalfPeriodMs) {
      g_lastRedBlinkToggleMs = nowMs;
      g_redBlinkState = !g_redBlinkState;
    }
    setLed(kRedLedPin, g_redBlinkState);
    return;
  }

  const bool hardStopLatched = (nowMs - g_lastHardStopMs) <= kHardStopLatchMs;
  setLed(kRedLedPin, hardStopLatched);
}

void serviceGnssStatusReport() {
  const uint32_t nowMs = millis();
  if ((nowMs - g_lastGnssReportMs) < kGnssReportPeriodMs) {
    return;
  }

  g_lastGnssReportMs = nowMs;
  printGnssState("GNSS,STATE");
}

void serviceModemStatusReport() {
  const uint32_t nowMs = millis();
  if ((nowMs - g_lastModemReportMs) < kModemReportPeriodMs) {
    return;
  }

  g_lastModemReportMs = nowMs;
  printModemState("MODEM,STATE");
  printNtripState("NTRIP,STATE");
  printMqttState("MQTT,STATE");
}

}  // namespace

void setup() {
  pinMode(kRecordButtonPin, INPUT_PULLUP);
  pinMode(kGreenLedPin, OUTPUT);
  pinMode(kRedLedPin, OUTPUT);
  setLed(kGreenLedPin, false);
  setLed(kRedLedPin, false);
  prepareAsm330I2cPins();

  Serial.begin(kSerialBaud);
  delay(500);

  Serial.println("BOOT,TELEMETRY,START");

  pinMode(kGnssResetPin, OUTPUT);
  digitalWrite(kGnssResetPin, HIGH);
  pinMode(kGnssTimePulsePin, INPUT);
  pinMode(kModemResetPin, OUTPUT);
  // Keep RESET_N deasserted except explicit reset pulses.
  digitalWrite(kModemResetPin, LOW);
  pinMode(kModemPwrKeyPin, OUTPUT);
  digitalWrite(kModemPwrKeyPin, LOW);
  g_modemBootPowerKeyPending = kModemAutoPowerKeyOnBoot;

  g_canSpiMutex = xSemaphoreCreateMutex();
  if (g_canSpiMutex == nullptr) {
    Serial.println("BOOT,ERR,CAN_SPI_MUTEX");
  }

  g_gnssStateMutex = xSemaphoreCreateMutex();
  g_cellularStateMutex = xSemaphoreCreateMutex();
  g_gnssUartMutex = xSemaphoreCreateMutex();
  if (g_gnssStateMutex == nullptr || g_cellularStateMutex == nullptr || g_gnssUartMutex == nullptr) {
    Serial.println("BOOT,ERR,MODEM_GNSS_MUTEX");
  }

  g_sdLogQueue = xQueueCreate(kSdLogQueueDepth, sizeof(SdLogEntry));
  if (g_sdLogQueue == nullptr) {
    Serial.println("BOOT,ERR,SD_LOG_QUEUE");
  }

  g_twaiReady = initTwai();
  g_mcpReady = initMcp2515();
  if (g_mcpReady) {
    g_mcpRxQueue = xQueueCreate(kMcpRxQueueDepth, sizeof(QueuedCanFrame));
    if (g_mcpRxQueue == nullptr) {
      Serial.println("BOOT,MCP2515,ERR,RX_QUEUE");
    }
  }

  // SD init – non-blocking: only probe if already in full mode, skip in bridge-only
  if (!g_bridgeOnlyMode) {
    g_sdReady = initSdCard();
    g_sdError = !g_sdReady;
    printSdState("SD,STATE");
  } else {
    g_sdReady = false;
    g_sdError = true;
    Serial.println("SD,DISABLED,BRIDGE_ONLY");
  }

  // ASM330 I2C init – probe on startup with auto-retry on failure
  if (kEnableAsm330Runtime) {
    g_imuReady = initAsm330();
    if (!g_imuReady) {
      g_asm330LastInitSucceeded = false;
      g_asm330InitRetryEnabled = true;
      Serial.println("BOOT,ASM330,RETRY_ENABLED");
      printAsm330InitState("ASM330,INIT,STATE");
    }
  } else {
    g_imuReady = false;
    Serial.println("BOOT,ASM330,DISABLED");
  }

  if (g_mcpReady && g_mcpRxQueue != nullptr) {
    if (xTaskCreatePinnedToCore(
            mcpServiceTask,
            "mcp-service",
            kMcpServiceTaskStack,
            nullptr,
            kMcpServiceTaskPriority,
            &g_mcpServiceTaskHandle,
            kMcpServiceTaskCore) != pdPASS) {
      Serial.println("BOOT,MCP2515,ERR,TASK");
      g_mcpServiceTaskHandle = nullptr;
    vQueueDelete(g_mcpRxQueue);
    g_mcpRxQueue = nullptr;
    }
  }

  if (!g_bridgeOnlyMode) {
    if (xTaskCreatePinnedToCore(
            gnssServiceTask,
            "gnss-service",
            kGnssTaskStack,
            nullptr,
            kGnssTaskPriority,
            &g_gnssTaskHandle,
            kGnssTaskCore) != pdPASS) {
      Serial.println("BOOT,GNSS,ERR,TASK");
      g_gnssTaskHandle = nullptr;
    }

    if (xTaskCreatePinnedToCore(
            modemServiceTask,
            "modem-service",
            kModemTaskStack,
            nullptr,
            kModemTaskPriority,
            &g_modemTaskHandle,
            kModemTaskCore) != pdPASS) {
      Serial.println("BOOT,MODEM,ERR,TASK");
      g_modemTaskHandle = nullptr;
    }
  } else {
    Serial.println("BOOT,BRIDGE_ONLY,GNSS_AND_MODEM_DISABLED");
    g_gnssTaskHandle = nullptr;
    g_modemTaskHandle = nullptr;
  }

  if (g_recordingRequested && g_sdReady && !g_sdError) {
    openLogFile();
  }

  Serial.printf("BOOT,RECORD,%s\n", g_recordingRequested ? "ON" : "OFF");

  if (!g_twaiReady || !g_mcpReady || (kEnableAsm330Runtime && !g_imuReady)) {
    Serial.println("BOOT,WARN,SUBSYSTEM_NOT_READY");
  }
}

void loop() {
  serviceSerialCommands();
  serviceRecordButton();
  updateLeds();

  // CAN-UART bridge is always active (top priority).
  serviceMcpRxQueue(kMcpQueueDrainBudgetPerLoop);
  serviceTwaiRx(kTwaiRxBudgetPerLoop);
  serviceMcpRxQueue(kMcpQueueDrainBudgetPerLoop);
  serviceTwaiStatus();
  if (g_mcpServiceTaskHandle == nullptr) {
    serviceMcpRx();
  }
  serviceMcpRxQueue(kMcpQueueDrainBudgetPerLoop);
  serviceMcpErrorReport();
  serviceMcpStatus();
  serviceMcpRecovery();

  // Everything below is gated behind full mode.
  if (g_bridgeOnlyMode) {
    delay(1);
    return;
  }

  serviceRecordingState();
  serviceGnssStatusReport();
  serviceModemStatusReport();
  serviceSdHealthCheck();
  serviceSdLogWriter();
  serviceSerialCommands();
  serviceRecordButton();

  // ASM330 I2C runtime
  if (kEnableAsm330Runtime) {
    serviceAsm330InitRetry();
    serviceAsm330Debug();

    ImuSample sample = {};
    if (readAsm330Sample(sample)) {
      ++g_sampleCounter;
      printImuSample(sample);
      publishImuToCan(sample);
    }
  }

  delay(1);
}
