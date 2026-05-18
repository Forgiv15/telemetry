#include <Arduino.h>
#include <stdlib.h>
#include <ctype.h>
#include <SD.h>
#include <SPI.h>
#include <driver/twai.h>
#include <math.h>
#include <mcp2515.h>

namespace {

constexpr uint32_t kSerialBaud = 921600;
constexpr uint32_t kHeartbeatHalfPeriodMs = 25;
constexpr uint32_t kSdErrorHalfPeriodMs = 250;
constexpr uint32_t kAsm330DebugPeriodMs = 100;
constexpr uint32_t kAsm330RetryPeriodMs = 250;
constexpr uint32_t kTwaiStatusPeriodMs = 1000;
constexpr uint32_t kAsm330StartupRetries = 20;
constexpr uint32_t kHardStopLatchMs = 1000;
constexpr float kHardStopThresholdG = 10.0f;

constexpr int kGreenLedPin = 41;
constexpr int kRedLedPin = 42;
constexpr int kRecordButtonPin = 38;

constexpr int kTwaiTxPin = 4;
constexpr int kTwaiRxPin = 5;

constexpr int kAsm330MosiPin = 6;
constexpr int kAsm330MisoPin = 7;
constexpr int kAsm330SckPin = 8;
constexpr int kAsm330DrdyPin = 45;
constexpr int kAsm330CsPin = 35;  // bodge wire: was GND, now IO35

constexpr int kSdCsPin = 9;
constexpr int kMcp2515CsPin = 10;
constexpr int kMcp2515MosiPin = 11;
constexpr int kMcp2515SckPin = 12;
constexpr int kMcp2515MisoPin = 13;

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

constexpr uint8_t kAsm330Ctrl1Xl416Hz16g = 0x64;
constexpr uint8_t kAsm330Ctrl2G416Hz2000dps = 0x6C;
constexpr uint8_t kAsm330Ctrl3CBduIfInc = 0x44;
constexpr uint8_t kAsm330Ctrl4CI2cDisable = 0x04;
constexpr uint8_t kAsm330Ctrl9XlDeviceConf = 0x02;
constexpr uint8_t kAsm330Int1DataReady = 0x03;
constexpr uint8_t kAsm330StatusAccelReady = 0x01;
constexpr uint8_t kAsm330StatusGyroReady = 0x02;
constexpr uint8_t kAsm330StatusAccelGyroReady = kAsm330StatusAccelReady | kAsm330StatusGyroReady;

constexpr float kAccelScaleGPerLsb = 0.000488f;
constexpr float kGyroScaleDpsPerLsb = 0.07f;

SPIClass imuSpi(FSPI);
SPIClass canSpi(HSPI);
MCP2515 mcp2515(kMcp2515CsPin, 8000000, &canSpi);

volatile bool g_imuDataReady = false;
bool g_twaiReady = false;
bool g_mcpReady = false;
bool g_imuReady = false;
bool g_sdReady = false;
bool g_sdError = false;
bool g_recordingRequested = false;
bool g_buttonStablePressed = false;
bool g_buttonLastReadPressed = false;

uint32_t g_lastHeartbeatToggleMs = 0;
bool g_greenLedState = false;
uint32_t g_lastRedBlinkToggleMs = 0;
bool g_redBlinkState = false;
uint32_t g_lastHardStopMs = 0;
uint32_t g_lastImuPollUs = 0;
uint32_t g_sampleCounter = 0;
uint32_t g_buttonLastTransitionMs = 0;
uint16_t g_logFileIndex = 0;
uint32_t g_lastAsm330DebugMs = 0;
uint32_t g_lastTwaiStatusMs = 0;
uint32_t g_twaiRxFrameCount = 0;

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
char g_serialCommandBuffer[64] = {};
size_t g_serialCommandLength = 0;
bool g_asm330InitRetryEnabled = false;
bool g_asm330LastInitSucceeded = false;

File g_logFile;

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

void IRAM_ATTR onAsm330DataReady() {
  g_imuDataReady = true;
  g_asm330IrqCount++;
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

bool isRecordingActive() {
  return g_recordingRequested && g_sdReady && !g_sdError && static_cast<bool>(g_logFile);
}

bool openLogFile() {
  if (!g_sdReady || g_sdError || static_cast<bool>(g_logFile)) {
    return false;
  }

  char fileName[20] = {};
  for (uint16_t idx = g_logFileIndex; idx < 1000; ++idx) {
    snprintf(fileName, sizeof(fileName), "/LOG%03u.CSV", idx);
    if (!SD.exists(fileName)) {
      g_logFile = SD.open(fileName, FILE_WRITE);
      if (g_logFile) {
        g_logFileIndex = idx + 1;
        g_logFile.println("TYPE,MS,IDTYPE,ID,DLC,DATA");
        g_logFile.flush();
        Serial.printf("SD,LOG_OPEN,%s\n", fileName);
        return true;
      }

      g_sdError = true;
      Serial.printf("SD,ERR,OPEN,%s\n", fileName);
      return false;
    }
  }

  g_sdError = true;
  Serial.println("SD,ERR,NO_FILENAME");
  return false;
}

void closeLogFile() {
  if (g_logFile) {
    g_logFile.flush();
    g_logFile.close();
    Serial.println("SD,LOG_CLOSED");
  }
}

bool logLine(const String &line) {
  if (!isRecordingActive()) {
    return false;
  }

  if (g_logFile.println(line) == 0) {
    g_sdError = true;
    closeLogFile();
    Serial.println("SD,ERR,WRITE");
    return false;
  }

  if ((g_sampleCounter % 32U) == 0U) {
    g_logFile.flush();
  }

  return true;
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

void asm330Select() {
  if (kAsm330CsPin >= 0) {
    digitalWrite(kAsm330CsPin, LOW);
  }
}

void asm330Deselect() {
  if (kAsm330CsPin >= 0) {
    digitalWrite(kAsm330CsPin, HIGH);
  }
}

uint8_t asm330ReadRegister(const uint8_t reg) {
  imuSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  asm330Select();
  imuSpi.transfer(static_cast<uint8_t>(reg | 0x80));
  const uint8_t value = imuSpi.transfer(0x00);
  asm330Deselect();
  imuSpi.endTransaction();
  return value;
}

void asm330ReadRegisters(const uint8_t startReg, uint8_t *buffer, const size_t length) {
  imuSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  asm330Select();
  imuSpi.transfer(static_cast<uint8_t>(startReg | 0x80));
  for (size_t index = 0; index < length; ++index) {
    buffer[index] = imuSpi.transfer(0x00);
  }
  asm330Deselect();
  imuSpi.endTransaction();
}

void asm330WriteRegister(const uint8_t reg, const uint8_t value) {
  imuSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  asm330Select();
  imuSpi.transfer(static_cast<uint8_t>(reg & 0x7F));
  imuSpi.transfer(value);
  asm330Deselect();
  imuSpi.endTransaction();
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
      "%s,WHOAMI=0x%02X,CTRL1_XL=0x%02X,CTRL2_G=0x%02X,CTRL3_C=0x%02X,CTRL4_C=0x%02X,CTRL9_XL=0x%02X,INT1_CTRL=0x%02X,STATUS=0x%02X,DRDY_PIN=%d\n",
      prefix,
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
      "%s,READY=%u,RETRY=%u,LAST_OK=%u,INIT_CYCLES=%lu,INIT_OK=%lu,INIT_FAIL=%lu,WHOAMI=0x%02X,CTRL4_C=0x%02X,CTRL9_XL=0x%02X\n",
      prefix,
      g_imuReady ? 1U : 0U,
      g_asm330InitRetryEnabled ? 1U : 0U,
      g_asm330LastInitSucceeded ? 1U : 0U,
      static_cast<unsigned long>(g_asm330InitCycleCount),
      static_cast<unsigned long>(g_asm330InitSuccessCount),
      static_cast<unsigned long>(g_asm330InitFailureCount),
      g_asm330LastWhoAmI,
      g_asm330LastCtrl3C,
      g_asm330LastCtrl4C,
      g_asm330LastCtrl9Xl);
}

void printAsm330DebugSnapshot(const char *prefix) {
  Serial.printf(
      "%s,READY=%u,WHOAMI=0x%02X,IRQ=%lu,READ_ATTEMPTS=%lu,SAMPLES=%lu,INT_SRC=%lu,PIN_SRC=%lu,POLL_SRC=%lu,ZERO=%lu,NOSAMPLE=%lu,STATUS=0x%02X,DRDY_PIN=%d\n",
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
      digitalRead(kAsm330DrdyPin));

  Serial.print(prefix);
  Serial.print(",LAST_RAW,");
  printAsm330RawBuffer(g_asm330LastRawBytes, sizeof(g_asm330LastRawBytes));
  Serial.println();
}

bool initAsm330() {
  g_asm330InitCycleCount++;
  g_lastAsm330InitAttemptMs = millis();
  g_imuDataReady = false;

  pinMode(kAsm330DrdyPin, INPUT);
  if (kAsm330CsPin >= 0) {
    pinMode(kAsm330CsPin, OUTPUT);
    digitalWrite(kAsm330CsPin, HIGH);
  }

  imuSpi.begin(kAsm330SckPin, kAsm330MisoPin, kAsm330MosiPin, kAsm330CsPin >= 0 ? kAsm330CsPin : -1);
  delay(10);

  Serial.printf(
      "BOOT,ASM330,CFG,MOSI=%d,MISO=%d,SCK=%d,CS=%d,DRDY=%d,SPI_MODE=3,SPI_HZ=1000000\n",
      kAsm330MosiPin,
      kAsm330MisoPin,
      kAsm330SckPin,
      kAsm330CsPin,
      kAsm330DrdyPin);

  uint8_t whoAmI = 0;
  for (uint32_t attempt = 1; attempt <= kAsm330StartupRetries; ++attempt) {
    whoAmI = asm330ReadRegister(kAsm330RegWhoAmI);
    g_asm330LastWhoAmI = whoAmI;
    Serial.printf("BOOT,ASM330,WHOAMI_ATTEMPT,%lu,0x%02X\n", static_cast<unsigned long>(attempt), whoAmI);
    if (whoAmI == kAsm330WhoAmIValue) {
      break;
    }
    delay(10);
  }

  if (whoAmI != kAsm330WhoAmIValue) {
    Serial.printf("BOOT,ASM330,ERR,WHOAMI,0x%02X\n", whoAmI);
    printAsm330KeyRegisters("BOOT,ASM330,REGS_FAIL");
    return false;
  }

  asm330WriteRegister(kAsm330RegCtrl3C, kAsm330Ctrl3CBduIfInc);
  asm330WriteRegister(kAsm330RegCtrl4C, kAsm330Ctrl4CI2cDisable);
  asm330WriteRegister(kAsm330RegCtrl9Xl, kAsm330Ctrl9XlDeviceConf);
  asm330WriteRegister(kAsm330RegCtrl1Xl, kAsm330Ctrl1Xl416Hz16g);
  asm330WriteRegister(kAsm330RegCtrl2G, kAsm330Ctrl2G416Hz2000dps);
  asm330WriteRegister(kAsm330RegInt1Ctrl, kAsm330Int1DataReady);
  delay(5);

  g_asm330LastCtrl1Xl = asm330ReadRegister(kAsm330RegCtrl1Xl);
  g_asm330LastCtrl2G = asm330ReadRegister(kAsm330RegCtrl2G);
  g_asm330LastCtrl3C = asm330ReadRegister(kAsm330RegCtrl3C);
  g_asm330LastCtrl4C = asm330ReadRegister(kAsm330RegCtrl4C);
  g_asm330LastCtrl9Xl = asm330ReadRegister(kAsm330RegCtrl9Xl);
  g_asm330LastInt1Ctrl = asm330ReadRegister(kAsm330RegInt1Ctrl);

  const bool ctrl1Ok = g_asm330LastCtrl1Xl == kAsm330Ctrl1Xl416Hz16g;
  const bool ctrl2Ok = g_asm330LastCtrl2G == kAsm330Ctrl2G416Hz2000dps;
  const bool ctrl3Ok = g_asm330LastCtrl3C == kAsm330Ctrl3CBduIfInc;
  const bool ctrl4Ok = (g_asm330LastCtrl4C & kAsm330Ctrl4CI2cDisable) != 0U;
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
  Serial.println("BOOT,ASM330,OK,ODR=416HZ,ACC=16G,GYRO=2000DPS");
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
  if (!g_imuReady) {
    g_asm330LastInitSucceeded = false;
    g_asm330InitFailureCount++;
    printAsm330InitState("ASM330,INIT,STATE");
  }
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

  canSpi.begin(kMcp2515SckPin, kMcp2515MisoPin, kMcp2515MosiPin, kMcp2515CsPin);

  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("BOOT,MCP2515,ERR,RESET");
    return false;
  }

  // autowp-mcp2515 has no MCP_10MHZ enum; this profile yields 500 kbps when Fosc=10 MHz.
  if (mcp2515.setBitrate(CAN_1000KBPS, MCP_20MHZ) != MCP2515::ERROR_OK) {
    Serial.println("BOOT,MCP2515,ERR,BITRATE");
    return false;
  }

  if (mcp2515.setNormalOneShotMode() != MCP2515::ERROR_OK) {
    Serial.println("BOOT,MCP2515,ERR,MODE");
    return false;
  }

  Serial.println("BOOT,MCP2515,OK,500KBPS,OSC=10MHZ,ONE_SHOT");
  return true;
}

bool initSdCard() {
  pinMode(kSdCsPin, OUTPUT);
  digitalWrite(kSdCsPin, HIGH);

  if (!SD.begin(kSdCsPin, canSpi)) {
    Serial.println("SD,ERR,INIT");
    return false;
  }

  if (SD.cardType() == CARD_NONE) {
    Serial.println("SD,ERR,NO_CARD");
    return false;
  }

  Serial.println("SD,OK");
  return true;
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

  g_recordingRequested = !g_recordingRequested;
  Serial.printf("SD,RECORD,%s\n", g_recordingRequested ? "ON" : "OFF");

  if (!g_recordingRequested) {
    closeLogFile();
    return;
  }

  if (!g_sdReady || g_sdError) {
    Serial.println("SD,RECORD,REJECTED");
    return;
  }

  openLogFile();
}

void serviceRecordingState() {
  if (!g_recordingRequested) {
    return;
  }

  if (!g_sdReady || g_sdError || static_cast<bool>(g_logFile)) {
    return;
  }

  openLogFile();
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

  uint8_t rawBytes[14] = {};
  asm330ReadRegisters(kAsm330RegOutTempL, rawBytes, sizeof(rawBytes));
  memcpy(g_asm330LastRawBytes, rawBytes, sizeof(rawBytes));

  sample.tempRaw = static_cast<int16_t>((static_cast<uint16_t>(rawBytes[1]) << 8) | rawBytes[0]);

  for (int axis = 0; axis < 3; ++axis) {
    const int gyroIndex = 2 + (axis * 2);
    const int accelIndex = 8 + (axis * 2);

    sample.gyroRaw[axis] = static_cast<int16_t>((static_cast<uint16_t>(rawBytes[gyroIndex + 1]) << 8) | rawBytes[gyroIndex]);
    sample.accelRaw[axis] = static_cast<int16_t>((static_cast<uint16_t>(rawBytes[accelIndex + 1]) << 8) | rawBytes[accelIndex]);

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
    printAsm330KeyRegisters("ASMDBG,REGS");
  }
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

void serviceSerialCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if ((ch == '\r') || (ch == '\n')) {
      if (g_serialCommandLength == 0U) {
        continue;
      }

      g_serialCommandBuffer[g_serialCommandLength] = '\0';
      if (strncmp(g_serialCommandBuffer, "TX,", 3) == 0) {
        handleTxSerialCommand(g_serialCommandBuffer);
      } else {
        handleAsm330SerialCommand(g_serialCommandBuffer);
      }
      g_serialCommandLength = 0U;
      continue;
    }

    if (g_serialCommandLength < (sizeof(g_serialCommandBuffer) - 1U)) {
      g_serialCommandBuffer[g_serialCommandLength++] = ch;
    } else {
      g_serialCommandLength = 0U;
      Serial.println("ASMDBG,ERR,CMD_TOO_LONG");
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

void printMcpFrame(const can_frame &frame) {
  const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  const uint32_t identifier = frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);

  Serial.print("CAN2,");
  Serial.print(millis());
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
  line += String(millis());
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

void serviceTwaiRx() {
  if (!g_twaiReady) {
    return;
  }

  twai_message_t message = {};
  while (twai_receive(&message, 0) == ESP_OK) {
    ++g_twaiRxFrameCount;
    printTwaiFrame(message);
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

void serviceMcpRx() {
  if (!g_mcpReady) {
    return;
  }

  can_frame frame = {};
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    printMcpFrame(frame);
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

  can_frame frame = {};
  const uint8_t clampedLength = (length <= 8U) ? length : 8U;
  frame.can_id = extended ? ((identifier & CAN_EFF_MASK) | CAN_EFF_FLAG) : (identifier & CAN_SFF_MASK);
  frame.can_dlc = clampedLength;
  memcpy(frame.data, payload, clampedLength);

  const MCP2515::ERROR result = mcp2515.sendMessage(&frame);
  if (result != MCP2515::ERROR_OK && result != MCP2515::ERROR_ALLTXBUSY) {
    Serial.printf("ERR,MCP_TX,%d\n", static_cast<int>(result));
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

  const bool sendTwai = (busSelector == '1') || (busSelector == 'A') || (busSelector == 'B');
  const bool sendMcp = (busSelector == '2') || (busSelector == 'B');
  if (!sendTwai && !sendMcp) {
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

  sendMcpFrame(kImuCanIdAccel, accelPayload, sizeof(accelPayload));
  sendMcpFrame(kImuCanIdGyro, gyroPayload, sizeof(gyroPayload));
  sendMcpFrame(kImuCanIdStatus, statusPayload, sizeof(statusPayload));
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

}  // namespace

void setup() {
  pinMode(kRecordButtonPin, INPUT_PULLUP);
  pinMode(kGreenLedPin, OUTPUT);
  pinMode(kRedLedPin, OUTPUT);
  setLed(kGreenLedPin, false);
  setLed(kRedLedPin, false);

  Serial.begin(kSerialBaud);
  delay(500);

  Serial.println("BOOT,TELEMETRY,START");

  g_twaiReady = initTwai();
  g_mcpReady = initMcp2515();
  g_sdReady = initSdCard();
  g_sdError = !g_sdReady;
  g_imuReady = initAsm330();
  if (!g_imuReady) {
    g_asm330LastInitSucceeded = false;
    g_asm330InitFailureCount++;
    printAsm330InitState("ASM330,INIT,STATE");
  }

  if (g_recordingRequested && g_sdReady && !g_sdError) {
    openLogFile();
  }

  Serial.printf("BOOT,RECORD,%s\n", g_recordingRequested ? "ON" : "OFF");

  if (!g_imuReady || !g_twaiReady || !g_mcpReady) {
    Serial.println("BOOT,WARN,SUBSYSTEM_NOT_READY");
  }
}

void loop() {
  serviceSerialCommands();
  serviceRecordButton();
  serviceRecordingState();
  updateLeds();
  serviceTwaiRx();
  serviceTwaiStatus();
  serviceMcpRx();
  serviceAsm330InitRetry();
  serviceAsm330Debug();

  ImuSample sample = {};
  if (readAsm330Sample(sample)) {
    ++g_sampleCounter;
    printImuSample(sample);
    publishImuToCan(sample);
  }
}
