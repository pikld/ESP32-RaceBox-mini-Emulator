#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

// ============================================================================
// User Settings
// Change values in this section when adapting the sketch to different hardware
// or tuning behavior. Everything below is implementation detail or protocol
// layout and should usually stay unchanged.
// ============================================================================
namespace UserSettings {

constexpr uint32_t kSerialBaudRate = 115200UL;

namespace Device {
constexpr char kName[] = "RaceBox Mini 0123456789";
constexpr int kStatusLedPin = 2;
}  // namespace Device

namespace Gnss {
constexpr int kRxPin = 16;
constexpr int kTxPin = 17;
constexpr uint32_t kBaudRate = 115200UL;
constexpr uint32_t kFactoryBaudRate = 9600UL;
constexpr uint8_t kNavigationRateHz = 25U;

struct ConstellationSetting {
  sfe_ublox_gnss_ids_e id;
  const char *name;
  bool enabled;
};

// Available constellations depend on the installed u-blox receiver.
constexpr ConstellationSetting kConstellations[] = {
  {SFE_UBLOX_GNSS_ID_GPS, "GPS", true},
  {SFE_UBLOX_GNSS_ID_GALILEO, "Galileo", true},
  {SFE_UBLOX_GNSS_ID_GLONASS, "GLONASS", false},
  {SFE_UBLOX_GNSS_ID_BEIDOU, "BEIDOU", false},
  {SFE_UBLOX_GNSS_ID_SBAS, "SBAS", false},
  {SFE_UBLOX_GNSS_ID_QZSS, "QZSS", false},
};
}  // namespace Gnss

namespace Imu {
constexpr uint32_t kSampleIntervalMs = 10UL;  // 100 Hz
constexpr float kAccelFilterAlpha = 0.8f;
constexpr float kGyroFilterAlpha = 0.8f;
constexpr mpu6050_accel_range_t kAccelRange = MPU6050_RANGE_8_G;
constexpr mpu6050_gyro_range_t kGyroRange = MPU6050_RANGE_500_DEG;
constexpr mpu6050_bandwidth_t kFilterBandwidth = MPU6050_BAND_21_HZ;
}  // namespace Imu

namespace Ble {
constexpr uint16_t kRequestedMtu = 128U;
}  // namespace Ble

}  // namespace UserSettings

// ============================================================================
// Fixed Constants
// Protocol layout, timing, device identity metadata, and conversion factors.
// These are not day-to-day user settings.
// ============================================================================
namespace InternalConstants {

namespace DeviceIdentity {
constexpr char kExpectedNamePrefix[] = "RaceBox Mini ";
constexpr size_t kNamePrefixLength = sizeof(kExpectedNamePrefix) - 1U;
constexpr size_t kSerialDigits = 10U;
constexpr unsigned long kMaxAllowedSerial = 3999999999UL;

constexpr char kModelName[] = "RaceBox Mini";
constexpr char kFirmwareRevision[] = "3.3";
constexpr char kHardwareRevision[] = "1";
constexpr char kManufacturerName[] = "RaceBox";
}  // namespace DeviceIdentity

namespace BleUuids {
constexpr char kRaceBoxService[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kRaceBoxRxCharacteristic[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kRaceBoxTxCharacteristic[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kNmeaService[] = "00001101-0000-1000-8000-00805F9B34FB";
constexpr char kNmeaRxCharacteristic[] = "00001102-0000-1000-8000-00805F9B34FB";
constexpr char kNmeaTxCharacteristic[] = "00001103-0000-1000-8000-00805F9B34FB";
constexpr char kDeviceInfoService[] = "0000180a-0000-1000-8000-00805f9b34fb";
constexpr char kModelCharacteristic[] = "00002a24-0000-1000-8000-00805f9b34fb";
constexpr char kSerialCharacteristic[] = "00002a25-0000-1000-8000-00805f9b34fb";
constexpr char kFirmwareCharacteristic[] = "00002a26-0000-1000-8000-00805f9b34fb";
constexpr char kHardwareCharacteristic[] = "00002a27-0000-1000-8000-00805f9b34fb";
constexpr char kManufacturerCharacteristic[] = "00002a29-0000-1000-8000-00805f9b34fb";
}  // namespace BleUuids

namespace BleTransport {
constexpr uint16_t kDefaultPeerMtu = 23U;
constexpr size_t kNotificationOverhead = 3U;
constexpr size_t kFallbackNotificationPayload =
    kDefaultPeerMtu - kNotificationOverhead;
constexpr size_t kMinimumUbxFrameSize = 8U;
constexpr size_t kMaxUbxFrameSize = 1024U;
}  // namespace BleTransport

namespace Timing {
constexpr uint32_t kRateReportIntervalMs = 5000UL;
constexpr float kRateReportIntervalSeconds =
    static_cast<float>(kRateReportIntervalMs) / 1000.0f;
constexpr uint32_t kDisconnectedBlinkIntervalMs = 500UL;
constexpr uint32_t kRestartAdvertisingDelayMs = 500UL;
constexpr uint32_t kGnssSerialRxBufferSize = 512UL;
constexpr uint32_t kHaltLoopDelayMs = 100UL;
}  // namespace Timing

namespace Conversion {
constexpr float kMetersPerSecondSquaredToMilliG = 1000.0f / 9.80665f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadiansPerSecondToCentiDegrees = 18000.0f / kPi;
constexpr double kLatLonToDegrees = 1e-7;
}  // namespace Conversion

namespace Protocol {
constexpr uint8_t kUbxSyncChar1 = 0xB5U;
constexpr uint8_t kUbxSyncChar2 = 0x62U;
constexpr uint8_t kMessageClass = 0xFFU;
constexpr uint8_t kMessageId = 0x01U;
constexpr uint8_t kAckMessageId = 0x02U;
constexpr uint8_t kNackMessageId = 0x03U;
constexpr uint8_t kGnssConfigurationMessageId = 0x27U;
constexpr size_t kHeaderSize = 6U;
constexpr size_t kPayloadSize = 80U;
constexpr size_t kChecksumSize = 2U;
constexpr size_t kPacketSize = kHeaderSize + kPayloadSize + kChecksumSize;
constexpr size_t kPayloadOffset = kHeaderSize;
constexpr size_t kChecksumOffset = kPayloadOffset + kPayloadSize;
constexpr bool kReportedBatteryCharging = false;
constexpr uint8_t kReportedBatteryPercent = 100U;

namespace GnssConfiguration {
constexpr uint8_t kMinimumDynamicPlatformModel = 0U;
constexpr uint8_t kMaximumDynamicPlatformModel = 8U;
constexpr size_t kPayloadSize = 3U;
constexpr uint8_t kDefaultDynamicPlatformModel =
    static_cast<uint8_t>(DYN_MODEL_AUTOMOTIVE);
constexpr bool kDefaultEnable3dSpeed = false;
constexpr uint8_t kDefaultMinimumHorizontalAccuracyDecimeters = 30U;
constexpr uint32_t kDecimeterToMillimeter = 100U;
}  // namespace GnssConfiguration

namespace Offset {
constexpr size_t kITow = 0U;
constexpr size_t kYear = 4U;
constexpr size_t kMonth = 6U;
constexpr size_t kDay = 7U;
constexpr size_t kHour = 8U;
constexpr size_t kMinute = 9U;
constexpr size_t kSecond = 10U;
constexpr size_t kValidityFlags = 11U;
constexpr size_t kTimeAccuracy = 12U;
constexpr size_t kNanoseconds = 16U;
constexpr size_t kFixType = 20U;
constexpr size_t kFixStatusFlags = 21U;
constexpr size_t kDateTimeFlags = 22U;
constexpr size_t kNumSv = 23U;
constexpr size_t kLongitude = 24U;
constexpr size_t kLatitude = 28U;
constexpr size_t kHeight = 32U;
constexpr size_t kHmsl = 36U;
constexpr size_t kHorizontalAccuracy = 40U;
constexpr size_t kVerticalAccuracy = 44U;
constexpr size_t kGroundSpeed = 48U;
constexpr size_t kHeadingOfMotion = 52U;
constexpr size_t kSpeedAccuracy = 56U;
constexpr size_t kHeadingAccuracy = 60U;
constexpr size_t kPdop = 64U;
constexpr size_t kLatLonFlags = 66U;
constexpr size_t kBatteryStatus = 67U;
constexpr size_t kAccelX = 68U;
constexpr size_t kAccelY = 70U;
constexpr size_t kAccelZ = 72U;
constexpr size_t kGyroX = 74U;
constexpr size_t kGyroY = 76U;
constexpr size_t kGyroZ = 78U;
}  // namespace Offset

}  // namespace Protocol

}  // namespace InternalConstants

static_assert(
    sizeof(UserSettings::Device::kName) ==
        (InternalConstants::DeviceIdentity::kNamePrefixLength +
         InternalConstants::DeviceIdentity::kSerialDigits + 1U),
    "Device name must be 'RaceBox Mini ' followed by 10 digits.");
static_assert(
    InternalConstants::Protocol::kPacketSize ==
        (InternalConstants::Protocol::kHeaderSize +
         InternalConstants::Protocol::kPayloadSize +
         InternalConstants::Protocol::kChecksumSize),
    "Packet size must equal header + payload + checksum.");
static_assert(
    InternalConstants::Protocol::kChecksumOffset +
            InternalConstants::Protocol::kChecksumSize ==
        InternalConstants::Protocol::kPacketSize,
    "Checksum offset must point at the last two bytes.");
static_assert(
    InternalConstants::Protocol::Offset::kGyroZ + sizeof(int16_t) ==
        InternalConstants::Protocol::kPayloadSize,
    "Payload layout no longer ends at gyro Z.");

// ============================================================================
// Runtime State
// ============================================================================
struct FilteredImuState {
  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
};

struct RuntimeCounters {
  unsigned int blePacketCount = 0U;
  unsigned int gnssUpdateCount = 0U;
};

struct RuntimeTimers {
  unsigned long lastAccelReadMs = 0UL;
  unsigned long lastDisconnectedBlinkMs = 0UL;
  unsigned long lastRateReportMs = 0UL;
};

struct RaceBoxGnssConfiguration {
  uint8_t dynamicPlatformModel =
      InternalConstants::Protocol::GnssConfiguration::kDefaultDynamicPlatformModel;
  bool enable3dSpeed =
      InternalConstants::Protocol::GnssConfiguration::kDefaultEnable3dSpeed;
  uint8_t minimumHorizontalAccuracyDecimeters =
      InternalConstants::Protocol::GnssConfiguration::
          kDefaultMinimumHorizontalAccuracyDecimeters;
};

struct RuntimeState {
  char deviceSerialNumber[InternalConstants::DeviceIdentity::kSerialDigits + 1U] = {};
  FilteredImuState filteredImu{};
  RuntimeCounters counters{};
  RuntimeTimers timers{};
  RaceBoxGnssConfiguration gnssConfiguration{};
  bool deviceConnected = false;
  bool previousDeviceConnected = false;
  uint32_t lastITow = 0U;
};

struct BleHandles {
  BLEServer *server = nullptr;
  BLECharacteristic *tx = nullptr;
  BLECharacteristic *rx = nullptr;
  BLECharacteristic *nmeaTx = nullptr;
  BLECharacteristic *nmeaRx = nullptr;
};

struct EncodedImuSample {
  int16_t accelX = 0;
  int16_t accelY = 0;
  int16_t accelZ = 0;
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
};

struct UbxFrameAssembler {
  std::array<uint8_t, InternalConstants::BleTransport::kMaxUbxFrameSize> buffer = {};
  size_t length = 0U;
  size_t expectedLength = 0U;
};

SFE_UBLOX_GNSS gGnss;
HardwareSerial gGpsSerial(2);
Adafruit_MPU6050 gMpu;

RuntimeState gState;
BleHandles gBle;
UbxFrameAssembler gRaceBoxRxAssembler;

std::array<uint8_t, InternalConstants::Protocol::kPayloadSize> gTxPayloadBuffer = {};
std::array<uint8_t, InternalConstants::Protocol::kPacketSize> gTxPacketBuffer = {{
    InternalConstants::Protocol::kUbxSyncChar1,
    InternalConstants::Protocol::kUbxSyncChar2,
    InternalConstants::Protocol::kMessageClass,
    InternalConstants::Protocol::kMessageId,
    static_cast<uint8_t>(InternalConstants::Protocol::kPayloadSize & 0xFFU),
    static_cast<uint8_t>((InternalConstants::Protocol::kPayloadSize >> 8U) & 0xFFU),
}};

// ============================================================================
// BLE Callbacks
// ============================================================================
void consumeUbxTransportBytes(UbxFrameAssembler &assembler, const uint8_t *data,
                              size_t length);
void dispatchReceivedUbxFrame(const uint8_t *frame, size_t length);

class RaceBoxServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer *server) override {
    gState.deviceConnected = true;

    if (server != nullptr) {
      server->updatePeerMTU(server->getConnId(), UserSettings::Ble::kRequestedMtu);
    }

    digitalWrite(UserSettings::Device::kStatusLedPin, HIGH);
    Serial.println("BLE client connected; MTU update requested.");
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    gState.deviceConnected = false;
    digitalWrite(UserSettings::Device::kStatusLedPin, LOW);
    Serial.println("BLE client disconnected.");
  }
};

class RaceBoxRxCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic *characteristic) override {
    if (characteristic == nullptr) {
      return;
    }

    uint8_t *rxValue = characteristic->getData();
    const size_t rxLength = characteristic->getLength();

    if ((rxValue == nullptr) || (rxLength == 0U)) {
      return;
    }

    consumeUbxTransportBytes(gRaceBoxRxAssembler, rxValue, rxLength);
  }
};

class NmeaRxCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic *characteristic) override {
    (void)characteristic;
  }
};

// ============================================================================
// Utility Helpers
// ============================================================================
[[noreturn]] void haltWithMessage(const char *message) {
  Serial.println(message);
  while (true) {
    delay(InternalConstants::Timing::kHaltLoopDelayMs);
  }
}

template <typename TObject>
TObject *requireCreatedObject(TObject *object, const char *description) {
  if (object == nullptr) {
    Serial.print("Failed to create ");
    Serial.println(description);
    while (true) {
      delay(InternalConstants::Timing::kHaltLoopDelayMs);
    }
  }

  return object;
}

template <typename TValue, size_t kBufferSize>
void writeLittleEndian(std::array<uint8_t, kBufferSize> &buffer, size_t offset,
                       TValue value) {
  memcpy(buffer.data() + offset, &value, sizeof(TValue));
}

void resetUbxFrameAssembler(UbxFrameAssembler &assembler) {
  assembler.length = 0U;
  assembler.expectedLength = 0U;
}

size_t getExpectedUbxFrameLength(const UbxFrameAssembler &assembler) {
  if (assembler.length < InternalConstants::Protocol::kHeaderSize) {
    return 0U;
  }

  if ((assembler.buffer[0] != InternalConstants::Protocol::kUbxSyncChar1) ||
      (assembler.buffer[1] != InternalConstants::Protocol::kUbxSyncChar2)) {
    return 0U;
  }

  const uint16_t payloadLength =
      static_cast<uint16_t>(assembler.buffer[4]) |
      (static_cast<uint16_t>(assembler.buffer[5]) << 8U);
  const size_t frameLength = InternalConstants::Protocol::kHeaderSize +
                             static_cast<size_t>(payloadLength) +
                             InternalConstants::Protocol::kChecksumSize;

  if ((frameLength < InternalConstants::BleTransport::kMinimumUbxFrameSize) ||
      (frameLength > InternalConstants::BleTransport::kMaxUbxFrameSize)) {
    return 0U;
  }

  return frameLength;
}

void handleReassembledRaceBoxFrame(const uint8_t *frame, size_t length) {
  if ((frame == nullptr) || (length == 0U)) {
    return;
  }

  Serial.printf("Reassembled BLE RX frame (%u bytes): ",
                static_cast<unsigned int>(length));
  for (size_t i = 0; i < length; ++i) {
    Serial.printf("0x%02X ", frame[i]);
  }
  Serial.println();

  dispatchReceivedUbxFrame(frame, length);
}

void consumeUbxTransportBytes(UbxFrameAssembler &assembler, const uint8_t *data,
                              size_t length) {
  if ((data == nullptr) || (length == 0U)) {
    return;
  }

  for (size_t index = 0U; index < length; ++index) {
    const uint8_t byte = data[index];

    if (assembler.length == 0U) {
      if (byte == InternalConstants::Protocol::kUbxSyncChar1) {
        assembler.buffer[0] = byte;
        assembler.length = 1U;
      }
      continue;
    }

    if (assembler.length == 1U) {
      if (byte == InternalConstants::Protocol::kUbxSyncChar2) {
        assembler.buffer[assembler.length++] = byte;
      } else if (byte == InternalConstants::Protocol::kUbxSyncChar1) {
        assembler.buffer[0] = byte;
        assembler.length = 1U;
      } else {
        resetUbxFrameAssembler(assembler);
      }
      continue;
    }

    if (assembler.length >= assembler.buffer.size()) {
      Serial.println("Dropping BLE RX frame because it exceeded the assembler buffer.");
      resetUbxFrameAssembler(assembler);
      if (byte == InternalConstants::Protocol::kUbxSyncChar1) {
        assembler.buffer[0] = byte;
        assembler.length = 1U;
      }
      continue;
    }

    assembler.buffer[assembler.length++] = byte;

    if (assembler.length == InternalConstants::Protocol::kHeaderSize) {
      assembler.expectedLength = getExpectedUbxFrameLength(assembler);
      if (assembler.expectedLength == 0U) {
        Serial.println("Dropping BLE RX frame with an invalid UBX header.");
        resetUbxFrameAssembler(assembler);
      }
      continue;
    }

    if ((assembler.expectedLength != 0U) &&
        (assembler.length == assembler.expectedLength)) {
      handleReassembledRaceBoxFrame(assembler.buffer.data(), assembler.length);
      resetUbxFrameAssembler(assembler);
    }
  }
}

void calculateUbxChecksum(const uint8_t *payload, uint16_t payloadLength,
                          uint8_t messageClass, uint8_t messageId, uint8_t *ckA,
                          uint8_t *ckB) {
  *ckA = 0U;
  *ckB = 0U;

  *ckA += messageClass;
  *ckB += *ckA;
  *ckA += messageId;
  *ckB += *ckA;
  *ckA += payloadLength & 0xFFU;
  *ckB += *ckA;
  *ckA += payloadLength >> 8U;
  *ckB += *ckA;

  for (uint16_t i = 0U; i < payloadLength; ++i) {
    *ckA += payload[i];
    *ckB += *ckA;
  }
}

float applyExponentialMovingAverage(float alpha, float sample, float filteredValue) {
  return (alpha * sample) + ((1.0f - alpha) * filteredValue);
}

int16_t saturatingFloatToInt16(float value) {
  if (value > static_cast<float>(std::numeric_limits<int16_t>::max())) {
    return std::numeric_limits<int16_t>::max();
  }

  if (value < static_cast<float>(std::numeric_limits<int16_t>::min())) {
    return std::numeric_limits<int16_t>::min();
  }

  return static_cast<int16_t>(value);
}

size_t getBleNotifyPayloadLimit() {
  if ((gBle.server == nullptr) || !gState.deviceConnected) {
    return InternalConstants::BleTransport::kFallbackNotificationPayload;
  }

  const uint16_t connId = gBle.server->getConnId();
  uint16_t mtu = gBle.server->getPeerMTU(connId);
  if (mtu <= InternalConstants::BleTransport::kNotificationOverhead) {
    mtu = InternalConstants::BleTransport::kDefaultPeerMtu;
  }

  return static_cast<size_t>(mtu) - InternalConstants::BleTransport::kNotificationOverhead;
}

bool notifyCharacteristicValue(BLECharacteristic *characteristic, const uint8_t *data,
                               size_t length, bool allowChunking) {
  if ((characteristic == nullptr) || (data == nullptr) || (length == 0U)) {
    return false;
  }

  const size_t payloadLimit = getBleNotifyPayloadLimit();
  if (!allowChunking && (length > payloadLimit)) {
    Serial.printf(
        "Skipping BLE notify because %u bytes exceeds the current ATT payload limit of %u bytes.\n",
        static_cast<unsigned int>(length), static_cast<unsigned int>(payloadLimit));
    return false;
  }

  size_t offset = 0U;
  while (offset < length) {
    const size_t remaining = length - offset;
    const size_t chunkLength = allowChunking ? std::min(remaining, payloadLimit) : remaining;
    characteristic->setValue(const_cast<uint8_t *>(data + offset), chunkLength);
    characteristic->notify();
    offset += chunkLength;
  }

  return true;
}

bool isValidUbxFrame(const uint8_t *frame, size_t length) {
  if ((frame == nullptr) ||
      (length < InternalConstants::BleTransport::kMinimumUbxFrameSize)) {
    return false;
  }

  if ((frame[0] != InternalConstants::Protocol::kUbxSyncChar1) ||
      (frame[1] != InternalConstants::Protocol::kUbxSyncChar2)) {
    return false;
  }

  const uint16_t payloadLength =
      static_cast<uint16_t>(frame[4]) |
      (static_cast<uint16_t>(frame[5]) << 8U);
  const size_t expectedLength = InternalConstants::Protocol::kHeaderSize +
                                static_cast<size_t>(payloadLength) +
                                InternalConstants::Protocol::kChecksumSize;
  if (expectedLength != length) {
    return false;
  }

  uint8_t ckA = 0U;
  uint8_t ckB = 0U;
  calculateUbxChecksum(frame + InternalConstants::Protocol::kHeaderSize, payloadLength,
                       frame[2], frame[3], &ckA, &ckB);
  return (frame[length - 2U] == ckA) && (frame[length - 1U] == ckB);
}

bool sendUbxPacket(BLECharacteristic *characteristic, uint8_t messageClass,
                   uint8_t messageId, const uint8_t *payload,
                   size_t payloadLength, bool allowChunking) {
  const size_t maxPayloadLength =
      InternalConstants::BleTransport::kMaxUbxFrameSize -
      InternalConstants::Protocol::kHeaderSize -
      InternalConstants::Protocol::kChecksumSize;
  if (payloadLength > maxPayloadLength) {
    return false;
  }

  std::array<uint8_t, InternalConstants::BleTransport::kMaxUbxFrameSize> packet = {};
  packet[0] = InternalConstants::Protocol::kUbxSyncChar1;
  packet[1] = InternalConstants::Protocol::kUbxSyncChar2;
  packet[2] = messageClass;
  packet[3] = messageId;
  packet[4] = static_cast<uint8_t>(payloadLength & 0xFFU);
  packet[5] = static_cast<uint8_t>((payloadLength >> 8U) & 0xFFU);

  if ((payload != nullptr) && (payloadLength > 0U)) {
    memcpy(packet.data() + InternalConstants::Protocol::kHeaderSize, payload,
           payloadLength);
  }

  uint8_t ckA = 0U;
  uint8_t ckB = 0U;
  calculateUbxChecksum(payload, static_cast<uint16_t>(payloadLength), messageClass,
                       messageId, &ckA, &ckB);

  const size_t checksumOffset =
      InternalConstants::Protocol::kHeaderSize + payloadLength;
  packet[checksumOffset] = ckA;
  packet[checksumOffset + 1U] = ckB;

  return notifyCharacteristicValue(characteristic, packet.data(),
                                   checksumOffset + InternalConstants::Protocol::kChecksumSize,
                                   allowChunking);
}

// ============================================================================
// Device Identity
// ============================================================================
void validateDeviceNameOrHalt() {
  const char *configuredName = UserSettings::Device::kName;
  const char *serialSuffix =
      configuredName + InternalConstants::DeviceIdentity::kNamePrefixLength;

  if (strncmp(configuredName, InternalConstants::DeviceIdentity::kExpectedNamePrefix,
              InternalConstants::DeviceIdentity::kNamePrefixLength) != 0) {
    haltWithMessage("Device name must start with 'RaceBox Mini '.");
  }

  if (strspn(serialSuffix, "0123456789") !=
      InternalConstants::DeviceIdentity::kSerialDigits) {
    haltWithMessage("Device name suffix must contain exactly 10 digits.");
  }

  char *parsedEnd = nullptr;
  const unsigned long serialValue = strtoul(serialSuffix, &parsedEnd, 10);

  if ((parsedEnd == nullptr) || (*parsedEnd != '\0') ||
      (serialValue > InternalConstants::DeviceIdentity::kMaxAllowedSerial)) {
    haltWithMessage("RaceBox Mini number must be 3999999999 or lower.");
  }

  memcpy(gState.deviceSerialNumber, serialSuffix,
         InternalConstants::DeviceIdentity::kSerialDigits);
  gState.deviceSerialNumber[InternalConstants::DeviceIdentity::kSerialDigits] = '\0';
}

// ============================================================================
// IMU Setup and Sampling
// ============================================================================
void seedImuFilterState() {
  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t temperatureEvent;
  gMpu.getEvent(&accelEvent, &gyroEvent, &temperatureEvent);

  gState.filteredImu.accelX = accelEvent.acceleration.x;
  gState.filteredImu.accelY = accelEvent.acceleration.y;
  gState.filteredImu.accelZ = accelEvent.acceleration.z;
  gState.filteredImu.gyroX = gyroEvent.gyro.x;
  gState.filteredImu.gyroY = gyroEvent.gyro.y;
  gState.filteredImu.gyroZ = gyroEvent.gyro.z;
}

void initializeImuOrHalt() {
  if (!gMpu.begin()) {
    haltWithMessage("Failed to find MPU6050 chip.");
  }

  gMpu.setAccelerometerRange(UserSettings::Imu::kAccelRange);
  gMpu.setGyroRange(UserSettings::Imu::kGyroRange);
  gMpu.setFilterBandwidth(UserSettings::Imu::kFilterBandwidth);
  seedImuFilterState();
}

void updateFilteredImu(unsigned long now) {
  if (gState.timers.lastAccelReadMs == 0UL) {
    gState.timers.lastAccelReadMs = now;
  }

  bool shouldSample = false;
  while ((now - gState.timers.lastAccelReadMs) >= UserSettings::Imu::kSampleIntervalMs) {
    gState.timers.lastAccelReadMs += UserSettings::Imu::kSampleIntervalMs;
    shouldSample = true;
  }

  if (!shouldSample) {
    return;
  }

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t temperatureEvent;
  gMpu.getEvent(&accelEvent, &gyroEvent, &temperatureEvent);

  gState.filteredImu.accelX = applyExponentialMovingAverage(
      UserSettings::Imu::kAccelFilterAlpha, accelEvent.acceleration.x,
      gState.filteredImu.accelX);
  gState.filteredImu.accelY = applyExponentialMovingAverage(
      UserSettings::Imu::kAccelFilterAlpha, accelEvent.acceleration.y,
      gState.filteredImu.accelY);
  gState.filteredImu.accelZ = applyExponentialMovingAverage(
      UserSettings::Imu::kAccelFilterAlpha, accelEvent.acceleration.z,
      gState.filteredImu.accelZ);

  gState.filteredImu.gyroX = applyExponentialMovingAverage(
      UserSettings::Imu::kGyroFilterAlpha, gyroEvent.gyro.x,
      gState.filteredImu.gyroX);
  gState.filteredImu.gyroY = applyExponentialMovingAverage(
      UserSettings::Imu::kGyroFilterAlpha, gyroEvent.gyro.y,
      gState.filteredImu.gyroY);
  gState.filteredImu.gyroZ = applyExponentialMovingAverage(
      UserSettings::Imu::kGyroFilterAlpha, gyroEvent.gyro.z,
      gState.filteredImu.gyroZ);
}

EncodedImuSample encodeFilteredImuSample() {
  EncodedImuSample encoded;

  encoded.accelX = saturatingFloatToInt16(
      gState.filteredImu.accelX *
      InternalConstants::Conversion::kMetersPerSecondSquaredToMilliG);
  encoded.accelY = saturatingFloatToInt16(
      gState.filteredImu.accelY *
      InternalConstants::Conversion::kMetersPerSecondSquaredToMilliG);
  encoded.accelZ = saturatingFloatToInt16(
      gState.filteredImu.accelZ *
      InternalConstants::Conversion::kMetersPerSecondSquaredToMilliG);

  encoded.gyroX = saturatingFloatToInt16(
      gState.filteredImu.gyroX *
      InternalConstants::Conversion::kRadiansPerSecondToCentiDegrees);
  encoded.gyroY = saturatingFloatToInt16(
      gState.filteredImu.gyroY *
      InternalConstants::Conversion::kRadiansPerSecondToCentiDegrees);
  encoded.gyroZ = saturatingFloatToInt16(
      gState.filteredImu.gyroZ *
      InternalConstants::Conversion::kRadiansPerSecondToCentiDegrees);

  return encoded;
}

// ============================================================================
// GNSS Setup
// ============================================================================
bool configureGnssUartOutput() {
  if (gGnss.setUART1Output(COM_TYPE_UBX)) {
    return true;
  }

  Serial.println("Failed to set GNSS UART output to UBX only.");
  return false;
}

bool resetGnssBaudRate() {
  Serial.println("Attempting to set correct GNSS baud rate.");
  gGpsSerial.updateBaudRate(UserSettings::Gnss::kFactoryBaudRate);
  delay(500);

  if (!gGnss.begin(gGpsSerial)) {
    Serial.print("u-blox GNSS not detected at ");
    Serial.print(UserSettings::Gnss::kFactoryBaudRate);
    Serial.println(" baud.");
    Serial.println("Check GNSS wiring and the factory baud rate for your module.");
    return false;
  }

  Serial.print("GNSS detected at ");
  Serial.print(UserSettings::Gnss::kFactoryBaudRate);
  Serial.println(" baud.");

  Serial.print("Setting baud rate to ");
  Serial.print(UserSettings::Gnss::kBaudRate);
  Serial.println("...");
  gGnss.setSerialRate(UserSettings::Gnss::kBaudRate);

  gGpsSerial.updateBaudRate(UserSettings::Gnss::kBaudRate);
  delay(500);

  if (!gGnss.begin(gGpsSerial)) {
    Serial.print("GNSS not detected at ");
    Serial.print(UserSettings::Gnss::kBaudRate);
    Serial.println(" baud.");
    Serial.println("Check GNSS wiring and the configured baud rate.");
    return false;
  }

  Serial.print("GNSS detected at ");
  Serial.print(UserSettings::Gnss::kBaudRate);
  Serial.println(" baud. Saving UART settings to flash.");

  if (!configureGnssUartOutput()) {
    return false;
  }

  gGnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  return true;
}

void configureGnssConstellations() {
  const size_t constellationCount =
      sizeof(UserSettings::Gnss::kConstellations) /
      sizeof(UserSettings::Gnss::kConstellations[0]);

  for (size_t i = 0; i < constellationCount; ++i) {
    const UserSettings::Gnss::ConstellationSetting &config =
        UserSettings::Gnss::kConstellations[i];
    const bool configured = gGnss.enableGNSS(config.enabled, config.id);

    if (configured) {
      Serial.print(config.name);
      Serial.println(config.enabled ? " enabled." : " disabled.");
      continue;
    }

    Serial.print("Failed to ");
    Serial.print(config.enabled ? "enable " : "disable ");
    Serial.print(config.name);
    Serial.println(".");
  }
}

void initializeGnssOrHalt() {
  gGpsSerial.setRxBufferSize(InternalConstants::Timing::kGnssSerialRxBufferSize);
  gGpsSerial.begin(UserSettings::Gnss::kBaudRate, SERIAL_8N1, UserSettings::Gnss::kRxPin,
                   UserSettings::Gnss::kTxPin);

  if (!gGnss.begin(gGpsSerial)) {
    Serial.println("GNSS not detected at the configured baud rate; attempting recovery.");
    if (!resetGnssBaudRate()) {
      haltWithMessage("Unable to initialize the GNSS module.");
    }
  }

  configureGnssUartOutput();
  gGnss.setAutoPVT(true);
  if (!gGnss.setDynamicModel(
          static_cast<dynModel>(gState.gnssConfiguration.dynamicPlatformModel))) {
    Serial.println("Failed to apply the configured GNSS dynamic platform model.");
  }

  if (gGnss.setNavigationFrequency(UserSettings::Gnss::kNavigationRateHz)) {
    Serial.printf("GPS update rate set to %u Hz.\n",
                  static_cast<unsigned int>(UserSettings::Gnss::kNavigationRateHz));
  } else {
    Serial.println("Failed to set GPS update rate.");
  }

  configureGnssConstellations();
}

// ============================================================================
// RaceBox Command Handling
// ============================================================================
bool sendRaceBoxReply(uint8_t messageId, const uint8_t *payload,
                      size_t payloadLength) {
  return sendUbxPacket(gBle.tx, InternalConstants::Protocol::kMessageClass, messageId,
                       payload, payloadLength, true);
}

void sendRaceBoxAckNack(bool acknowledged, uint8_t messageId) {
  const uint8_t payload[2] = {InternalConstants::Protocol::kMessageClass, messageId};
  const uint8_t replyId = acknowledged ? InternalConstants::Protocol::kAckMessageId
                                       : InternalConstants::Protocol::kNackMessageId;

  if (!sendRaceBoxReply(replyId, payload, sizeof(payload))) {
    Serial.printf("Failed to send RaceBox %s for 0x%02X.\n",
                  acknowledged ? "ACK" : "NACK",
                  static_cast<unsigned int>(messageId));
  }
}

bool isValidDynamicPlatformModel(uint8_t dynamicPlatformModel) {
  return dynamicPlatformModel >=
             InternalConstants::Protocol::GnssConfiguration::
                 kMinimumDynamicPlatformModel &&
         dynamicPlatformModel <=
             InternalConstants::Protocol::GnssConfiguration::
                 kMaximumDynamicPlatformModel;
}

bool applyGnssConfiguration(const RaceBoxGnssConfiguration &configuration) {
  if (!isValidDynamicPlatformModel(configuration.dynamicPlatformModel)) {
    return false;
  }

  if (!gGnss.setDynamicModel(
          static_cast<dynModel>(configuration.dynamicPlatformModel))) {
    Serial.printf("Failed to apply GNSS dynamic platform model %u.\n",
                  static_cast<unsigned int>(configuration.dynamicPlatformModel));
    return false;
  }

  if (!gGnss.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF)) {
    Serial.println("Failed to save GNSS navigation configuration.");
    return false;
  }

  gState.gnssConfiguration = configuration;
  Serial.printf(
      "Applied GNSS config: dynModel=%u 3dSpeed=%u minHAcc=%.1fm\n",
      static_cast<unsigned int>(configuration.dynamicPlatformModel),
      configuration.enable3dSpeed ? 1U : 0U,
      static_cast<float>(configuration.minimumHorizontalAccuracyDecimeters) / 10.0f);
  return true;
}

void sendGnssConfigurationReply() {
  const uint8_t payload[InternalConstants::Protocol::GnssConfiguration::kPayloadSize] = {
      gState.gnssConfiguration.dynamicPlatformModel,
      static_cast<uint8_t>(gState.gnssConfiguration.enable3dSpeed ? 1U : 0U),
      gState.gnssConfiguration.minimumHorizontalAccuracyDecimeters,
  };

  if (!sendRaceBoxReply(InternalConstants::Protocol::kGnssConfigurationMessageId,
                        payload, sizeof(payload))) {
    Serial.println("Failed to send the current GNSS configuration.");
  }
}

void handleGnssConfigurationCommand(const uint8_t *payload, size_t payloadLength) {
  if (payloadLength == 0U) {
    sendGnssConfigurationReply();
    return;
  }

  if ((payload == nullptr) ||
      (payloadLength !=
       InternalConstants::Protocol::GnssConfiguration::kPayloadSize)) {
    sendRaceBoxAckNack(false, InternalConstants::Protocol::kGnssConfigurationMessageId);
    return;
  }

  RaceBoxGnssConfiguration configuration;
  configuration.dynamicPlatformModel = payload[0];
  configuration.enable3dSpeed = payload[1] != 0U;
  configuration.minimumHorizontalAccuracyDecimeters = payload[2];

  if (applyGnssConfiguration(configuration)) {
    sendRaceBoxAckNack(true, InternalConstants::Protocol::kGnssConfigurationMessageId);
    return;
  }

  sendRaceBoxAckNack(false, InternalConstants::Protocol::kGnssConfigurationMessageId);
}

void dispatchReceivedUbxFrame(const uint8_t *frame, size_t length) {
  if (!isValidUbxFrame(frame, length)) {
    Serial.println("Dropping BLE RX frame with an invalid checksum.");
    return;
  }

  const uint8_t messageClass = frame[2];
  const uint8_t messageId = frame[3];
  const uint16_t payloadLength =
      static_cast<uint16_t>(frame[4]) |
      (static_cast<uint16_t>(frame[5]) << 8U);
  const uint8_t *payload = frame + InternalConstants::Protocol::kHeaderSize;

  if (messageClass != InternalConstants::Protocol::kMessageClass) {
    Serial.printf(
        "Ignoring UBX command 0x%02X 0x%02X until generic pass-through is implemented.\n",
        static_cast<unsigned int>(messageClass),
        static_cast<unsigned int>(messageId));
    return;
  }

  if (messageId == InternalConstants::Protocol::kGnssConfigurationMessageId) {
    handleGnssConfigurationCommand(payload, payloadLength);
    return;
  }

  sendRaceBoxAckNack(false, messageId);
}

// ============================================================================
// BLE Setup
// ============================================================================
BLECharacteristic *createReadOnlyCharacteristic(BLEService *service, const char *uuid,
                                                const char *description) {
  return requireCreatedObject(
      service->createCharacteristic(uuid, BLECharacteristic::PROPERTY_READ), description);
}

void initializeRaceBoxService(BLEServer *server) {
  BLEService *service = requireCreatedObject(
      server->createService(InternalConstants::BleUuids::kRaceBoxService),
      "RaceBox BLE service");

  gBle.tx = requireCreatedObject(
      service->createCharacteristic(InternalConstants::BleUuids::kRaceBoxTxCharacteristic,
                                    BLECharacteristic::PROPERTY_NOTIFY),
      "RaceBox TX characteristic");
  gBle.tx->addDescriptor(requireCreatedObject(new BLE2902(), "RaceBox TX descriptor"));

  gBle.rx = requireCreatedObject(
      service->createCharacteristic(InternalConstants::BleUuids::kRaceBoxRxCharacteristic,
                                    BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_WRITE_NR),
      "RaceBox RX characteristic");
  gBle.rx->setCallbacks(new RaceBoxRxCallbacks());

  service->start();
}

void initializeNmeaService(BLEServer *server) {
  BLEService *service = requireCreatedObject(
      server->createService(InternalConstants::BleUuids::kNmeaService),
      "NMEA BLE service");

  gBle.nmeaTx = requireCreatedObject(
      service->createCharacteristic(InternalConstants::BleUuids::kNmeaTxCharacteristic,
                                    BLECharacteristic::PROPERTY_NOTIFY),
      "NMEA TX characteristic");
  gBle.nmeaTx->addDescriptor(requireCreatedObject(new BLE2902(), "NMEA TX descriptor"));

  gBle.nmeaRx = requireCreatedObject(
      service->createCharacteristic(InternalConstants::BleUuids::kNmeaRxCharacteristic,
                                    BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_WRITE_NR),
      "NMEA RX characteristic");
  gBle.nmeaRx->setCallbacks(new NmeaRxCallbacks());

  service->start();
}

void initializeDeviceInformationService(BLEServer *server) {
  BLEService *service = requireCreatedObject(
      server->createService(InternalConstants::BleUuids::kDeviceInfoService),
      "Device Information service");

  createReadOnlyCharacteristic(service, InternalConstants::BleUuids::kModelCharacteristic,
                               "Device model characteristic")
      ->setValue(InternalConstants::DeviceIdentity::kModelName);

  createReadOnlyCharacteristic(service, InternalConstants::BleUuids::kSerialCharacteristic,
                               "Device serial characteristic")
      ->setValue(reinterpret_cast<uint8_t *>(gState.deviceSerialNumber),
                 InternalConstants::DeviceIdentity::kSerialDigits);

  createReadOnlyCharacteristic(
      service, InternalConstants::BleUuids::kFirmwareCharacteristic,
      "Device firmware characteristic")
      ->setValue(InternalConstants::DeviceIdentity::kFirmwareRevision);

  createReadOnlyCharacteristic(
      service, InternalConstants::BleUuids::kHardwareCharacteristic,
      "Device hardware characteristic")
      ->setValue(InternalConstants::DeviceIdentity::kHardwareRevision);

  createReadOnlyCharacteristic(
      service, InternalConstants::BleUuids::kManufacturerCharacteristic,
      "Device manufacturer characteristic")
      ->setValue(InternalConstants::DeviceIdentity::kManufacturerName);

  service->start();
}

void startBleAdvertising() {
  BLEAdvertising *advertising =
      requireCreatedObject(BLEDevice::getAdvertising(), "BLE advertising");
  advertising->addServiceUUID(InternalConstants::BleUuids::kRaceBoxService);
  advertising->addServiceUUID(InternalConstants::BleUuids::kNmeaService);
  advertising->addServiceUUID(InternalConstants::BleUuids::kDeviceInfoService);
  advertising->setScanResponse(true);

  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started.");
}

void initializeBleOrHalt() {
  BLEDevice::init(UserSettings::Device::kName);

  gBle.server = requireCreatedObject(BLEDevice::createServer(), "BLE server");
  gBle.server->setCallbacks(new RaceBoxServerCallbacks());

  initializeRaceBoxService(gBle.server);
  initializeNmeaService(gBle.server);
  initializeDeviceInformationService(gBle.server);
  startBleAdvertising();
}

// ============================================================================
// RaceBox Packet Assembly
// ============================================================================
uint32_t getMinimumHorizontalAccuracyThresholdMillimeters() {
  return static_cast<uint32_t>(
             gState.gnssConfiguration.minimumHorizontalAccuracyDecimeters) *
         InternalConstants::Protocol::GnssConfiguration::kDecimeterToMillimeter;
}

bool isReportedFixValid(const UBX_NAV_PVT_t *navPvtPacket) {
  if (navPvtPacket == nullptr) {
    return false;
  }

  const auto &navData = navPvtPacket->data;
  return navData.flags.bits.gnssFixOK &&
         (navData.hAcc <= getMinimumHorizontalAccuracyThresholdMillimeters());
}

int32_t getReportedSpeedMillimetersPerSecond(const UBX_NAV_PVT_t *navPvtPacket) {
  if (navPvtPacket == nullptr) {
    return 0;
  }

  const auto &navData = navPvtPacket->data;
  if (!gState.gnssConfiguration.enable3dSpeed) {
    return navData.gSpeed;
  }

  const int64_t horizontalSpeed = static_cast<int64_t>(navData.gSpeed);
  const int64_t verticalSpeed = static_cast<int64_t>(navData.velD);
  const double speed3d = std::sqrt(static_cast<double>(
      (horizontalSpeed * horizontalSpeed) + (verticalSpeed * verticalSpeed)));
  return static_cast<int32_t>(std::lround(speed3d));
}

uint8_t buildValidityFlags(const UBX_NAV_PVT_t *navPvtPacket) {
  const auto &navData = navPvtPacket->data;
  uint8_t flags = 0U;

  if (navData.valid.bits.validDate) {
    flags |= (1U << 0U);
  }
  if (navData.valid.bits.validTime) {
    flags |= (1U << 1U);
  }
  if (navData.valid.bits.fullyResolved) {
    flags |= (1U << 2U);
  }

  return flags;
}

uint8_t buildFixStatusFlags(const UBX_NAV_PVT_t *navPvtPacket) {
  const auto &navData = navPvtPacket->data;
  uint8_t flags = 0U;

  if (isReportedFixValid(navPvtPacket)) {
    flags |= (1U << 0U);
  }
  if (navData.flags.bits.diffSoln) {
    flags |= (1U << 1U);
  }
  if (navData.flags.bits.headVehValid) {
    flags |= (1U << 5U);
  }
  flags |= static_cast<uint8_t>(navData.flags.bits.carrSoln << 6U);

  return flags;
}

uint8_t buildDateTimeFlags(const UBX_NAV_PVT_t *navPvtPacket) {
  const auto &navData = navPvtPacket->data;
  uint8_t flags = 0U;

  if (navData.flags2.bits.confirmedAvai) {
    flags |= (1U << 5U);
  }
  if (navData.flags2.bits.confirmedDate) {
    flags |= (1U << 6U);
  }
  if (navData.flags2.bits.confirmedTime) {
    flags |= (1U << 7U);
  }

  return flags;
}

uint8_t buildLatLonFlags(const UBX_NAV_PVT_t *navPvtPacket) {
  const auto &navData = navPvtPacket->data;
  uint8_t flags = 0U;

  if (navData.flags3.bits.invalidLlh) {
    flags |= (1U << 0U);
  }

  return flags;
}

uint8_t buildBatteryStatusByte() {
  uint8_t batteryStatus =
      InternalConstants::Protocol::kReportedBatteryPercent & 0x7FU;

  if (InternalConstants::Protocol::kReportedBatteryCharging) {
    batteryStatus |= 0x80U;
  }

  return batteryStatus;
}

void populateRaceBoxPayload(const UBX_NAV_PVT_t *navPvtPacket) {
  gTxPayloadBuffer.fill(0U);

  const auto &navData = navPvtPacket->data;
  const EncodedImuSample imuSample = encodeFilteredImuSample();

  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kITow,
                    navData.iTOW);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kYear,
                    navData.year);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kMonth,
                    navData.month);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kDay,
                    navData.day);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kHour,
                    navData.hour);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kMinute,
                    navData.min);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kSecond,
                    navData.sec);
  writeLittleEndian(gTxPayloadBuffer,
                    InternalConstants::Protocol::Offset::kValidityFlags,
                    buildValidityFlags(navPvtPacket));
  writeLittleEndian(gTxPayloadBuffer,
                    InternalConstants::Protocol::Offset::kTimeAccuracy,
                    navData.tAcc);
  writeLittleEndian(gTxPayloadBuffer,
                    InternalConstants::Protocol::Offset::kNanoseconds, navData.nano);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kFixType,
                    navData.fixType);
  writeLittleEndian(gTxPayloadBuffer,
                    InternalConstants::Protocol::Offset::kFixStatusFlags,
                    buildFixStatusFlags(navPvtPacket));
  writeLittleEndian(gTxPayloadBuffer,
                    InternalConstants::Protocol::Offset::kDateTimeFlags,
                    buildDateTimeFlags(navPvtPacket));
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kNumSv,
                    navData.numSV);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kLongitude,
                    navData.lon);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kLatitude,
                    navData.lat);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kHeight,
                    navData.height);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kHmsl,
                    navData.hMSL);
  writeLittleEndian(
      gTxPayloadBuffer, InternalConstants::Protocol::Offset::kHorizontalAccuracy,
      navData.hAcc);
  writeLittleEndian(
      gTxPayloadBuffer, InternalConstants::Protocol::Offset::kVerticalAccuracy,
      navData.vAcc);
  writeLittleEndian(gTxPayloadBuffer,
                    InternalConstants::Protocol::Offset::kGroundSpeed,
                    getReportedSpeedMillimetersPerSecond(navPvtPacket));
  writeLittleEndian(
      gTxPayloadBuffer, InternalConstants::Protocol::Offset::kHeadingOfMotion,
      navData.headMot);
  writeLittleEndian(gTxPayloadBuffer,
                    InternalConstants::Protocol::Offset::kSpeedAccuracy,
                    navData.sAcc);
  writeLittleEndian(
      gTxPayloadBuffer, InternalConstants::Protocol::Offset::kHeadingAccuracy,
      navData.headAcc);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kPdop,
                    navData.pDOP);
  writeLittleEndian(gTxPayloadBuffer,
                    InternalConstants::Protocol::Offset::kLatLonFlags,
                    buildLatLonFlags(navPvtPacket));
  writeLittleEndian(gTxPayloadBuffer,
                    InternalConstants::Protocol::Offset::kBatteryStatus,
                    buildBatteryStatusByte());

  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kAccelX,
                    imuSample.accelX);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kAccelY,
                    imuSample.accelY);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kAccelZ,
                    imuSample.accelZ);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kGyroX,
                    imuSample.gyroX);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kGyroY,
                    imuSample.gyroY);
  writeLittleEndian(gTxPayloadBuffer, InternalConstants::Protocol::Offset::kGyroZ,
                    imuSample.gyroZ);
}

void finalizeRaceBoxPacket() {
  memcpy(gTxPacketBuffer.data() + InternalConstants::Protocol::kPayloadOffset,
         gTxPayloadBuffer.data(), InternalConstants::Protocol::kPayloadSize);

  uint8_t ckA = 0U;
  uint8_t ckB = 0U;
  calculateUbxChecksum(
      gTxPayloadBuffer.data(), InternalConstants::Protocol::kPayloadSize,
      InternalConstants::Protocol::kMessageClass,
      InternalConstants::Protocol::kMessageId, &ckA, &ckB);

  gTxPacketBuffer[InternalConstants::Protocol::kChecksumOffset] = ckA;
  gTxPacketBuffer[InternalConstants::Protocol::kChecksumOffset + 1U] = ckB;
}

void sendRaceBoxPacket(const UBX_NAV_PVT_t *navPvtPacket) {
  if (!gState.deviceConnected || (gBle.tx == nullptr) || (navPvtPacket == nullptr)) {
    return;
  }

  populateRaceBoxPayload(navPvtPacket);
  finalizeRaceBoxPacket();

  if (notifyCharacteristicValue(gBle.tx, gTxPacketBuffer.data(),
                                InternalConstants::Protocol::kPacketSize, true)) {
    gState.counters.blePacketCount++;
  }
}

// ============================================================================
// Loop Tasks
// ============================================================================
void updateConnectionLed(unsigned long now) {
  if (gState.deviceConnected) {
    digitalWrite(UserSettings::Device::kStatusLedPin, HIGH);
    return;
  }

  if ((now - gState.timers.lastDisconnectedBlinkMs) >
      InternalConstants::Timing::kDisconnectedBlinkIntervalMs) {
    gState.timers.lastDisconnectedBlinkMs = now;
    digitalWrite(UserSettings::Device::kStatusLedPin,
                 !digitalRead(UserSettings::Device::kStatusLedPin));
  }
}

bool isNewNavigationEpoch(const UBX_NAV_PVT_t *navPvtPacket) {
  if (navPvtPacket == nullptr) {
    return false;
  }

  const uint32_t currentITow = navPvtPacket->data.iTOW;
  if (currentITow == gState.lastITow) {
    return false;
  }

  gState.lastITow = currentITow;
  return true;
}

void reportRatesIfDue(unsigned long now, const UBX_NAV_PVT_t *navPvtPacket) {
  if ((now - gState.timers.lastRateReportMs) <
      InternalConstants::Timing::kRateReportIntervalMs) {
    return;
  }

  float bleRate = static_cast<float>(gState.counters.blePacketCount) /
                  InternalConstants::Timing::kRateReportIntervalSeconds;
  float gnssRate = static_cast<float>(gState.counters.gnssUpdateCount) /
                   InternalConstants::Timing::kRateReportIntervalSeconds;

  unsigned int satellites = 0U;
  unsigned int fixType = 0U;
  unsigned long horizontalAccuracy = 0UL;
  double latitude = 0.0;
  double longitude = 0.0;

  if (navPvtPacket != nullptr) {
    const auto &navData = navPvtPacket->data;
    satellites = static_cast<unsigned int>(navData.numSV);
    fixType = static_cast<unsigned int>(navData.fixType);
    horizontalAccuracy = static_cast<unsigned long>(navData.hAcc);
    latitude = static_cast<double>(navData.lat) *
               InternalConstants::Conversion::kLatLonToDegrees;
    longitude = static_cast<double>(navData.lon) *
                InternalConstants::Conversion::kLatLonToDegrees;
  }

  Serial.printf(
      "BLE packet rate: %.2f Hz | GNSS update rate: %.2f Hz | SVs: %u | Fix: %u | "
      "HAcc: %lu mm | Lat: %.7f Lon: %.7f\n",
      bleRate, gnssRate, satellites, fixType, horizontalAccuracy, latitude,
      longitude);

  gState.counters.blePacketCount = 0U;
  gState.counters.gnssUpdateCount = 0U;
  gState.timers.lastRateReportMs = now;
}

void updateBleAdvertisingState() {
  if (!gState.deviceConnected && gState.previousDeviceConnected) {
    delay(InternalConstants::Timing::kRestartAdvertisingDelayMs);
    if (gBle.server != nullptr) {
      gBle.server->startAdvertising();
    }
    gState.previousDeviceConnected = false;
    return;
  }

  if (gState.deviceConnected && !gState.previousDeviceConnected) {
    gState.previousDeviceConnected = true;
  }
}

void processGnssData(unsigned long now) {
  if (!gGnss.getPVT()) {
    return;
  }

  const UBX_NAV_PVT_t *navPvtPacket = gGnss.packetUBXNAVPVT;

  if (isNewNavigationEpoch(navPvtPacket)) {
    gState.counters.gnssUpdateCount++;
    sendRaceBoxPacket(navPvtPacket);
  }

  reportRatesIfDue(now, navPvtPacket);
}

// ============================================================================
// Arduino Entry Points
// ============================================================================
void setup() {
  Serial.begin(UserSettings::kSerialBaudRate);
  pinMode(UserSettings::Device::kStatusLedPin, OUTPUT);

  validateDeviceNameOrHalt();
  initializeImuOrHalt();
  initializeGnssOrHalt();
  initializeBleOrHalt();

  gState.timers.lastRateReportMs = millis();
}

void loop() {
  const unsigned long now = millis();

  gGnss.checkUblox();
  updateFilteredImu(now);
  updateConnectionLed(now);
  processGnssData(now);
  updateBleAdvertisingState();
}
