#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <array>
#include <cstdlib>
#include <cstring>
#include <limits>

// --- GPS Configuration ---
constexpr uint32_t kSerialBaudRate = 115200UL;
constexpr int kGpsRxPin = 16;
constexpr int kGpsTxPin = 17;
constexpr uint32_t kGpsBaudRate = 115200UL;
constexpr uint32_t kFactoryGpsBaudRate = 9600UL;
constexpr uint8_t kMaxNavigationRateHz = 25U;

SFE_UBLOX_GNSS myGNSS;
HardwareSerial GPS_Serial(2);
// --- Enable GNSS constellations ---
// The specific constellations available and how many you can turn on depend on your u-blox module 
// check this out for which constellations to enable https://app.qzss.go.jp/GNSSView/gnssview.html

struct GnssConstellationConfig {
  sfe_ublox_gnss_ids_e id;
  const char *name;
  bool enabled;
};

constexpr GnssConstellationConfig kGnssConstellations[] = {
  {SFE_UBLOX_GNSS_ID_GPS, "GPS", true},
  {SFE_UBLOX_GNSS_ID_GALILEO, "Galileo", true},
  {SFE_UBLOX_GNSS_ID_GLONASS, "GLONASS", false},
  {SFE_UBLOX_GNSS_ID_BEIDOU, "BEIDOU", false},
  {SFE_UBLOX_GNSS_ID_SBAS, "SBAS", false},
  {SFE_UBLOX_GNSS_ID_QZSS, "QZSS", false},
};

constexpr char kDeviceName[] = "RaceBox Mini 0123456789";
constexpr size_t kDeviceNamePrefixLength = sizeof("RaceBox Mini ") - 1U;
constexpr size_t kDeviceNameSuffixLength = 10U;
constexpr unsigned long kMaxAllowedDeviceSuffix = 3999999999UL;

static_assert(sizeof(kDeviceName) == (kDeviceNamePrefixLength + kDeviceNameSuffixLength + 1U),
              "Device name must be 'RaceBox Mini ' followed by 10 digits.");

char deviceSerialNumber[kDeviceNameSuffixLength + 1U] = {};

constexpr int kOnboardLedPin = 2;

Adafruit_MPU6050 mpu;
constexpr uint32_t kAccelSampleIntervalMs = 10UL; // 10ms = 100Hz
// --- Smoothing Configuration ---
// alpha = 1.0: No filtering (raw data)
// alpha = 0.5: 50% current reading, 50% previous (moderate)
// alpha = 0.8: Very snappy, just kills high-frequency "buzz"
float accelAlpha = 0.8; 
float gyroAlpha = 0.8; 
// Storage for the filtered values
float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

// --- BLE Configuration ---
constexpr char kRaceBoxServiceUuid[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kRaceBoxCharacteristicRxUuid[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kRaceBoxCharacteristicTxUuid[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kDeviceInfoServiceUuid[] = "0000180a-0000-1000-8000-00805f9b34fb";
constexpr char kModelCharacteristicUuid[] = "00002a24-0000-1000-8000-00805f9b34fb";
constexpr char kSerialCharacteristicUuid[] = "00002a25-0000-1000-8000-00805f9b34fb";
constexpr char kFirmwareCharacteristicUuid[] = "00002a26-0000-1000-8000-00805f9b34fb";
constexpr char kHardwareCharacteristicUuid[] = "00002a27-0000-1000-8000-00805f9b34fb";
constexpr char kManufacturerCharacteristicUuid[] = "00002a29-0000-1000-8000-00805f9b34fb";
constexpr char kModelName[] = "RaceBox Mini";
constexpr char kFirmwareRevision[] = "3.3";
constexpr char kHardwareRevision[] = "1";
constexpr char kManufacturerName[] = "RaceBox";
constexpr uint16_t kRequestedBleMtu = 128U;

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicTx = NULL;
BLECharacteristic *pCharacteristicRx = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// --- Packet Timing ---
unsigned long lastPacketSendTime = 0;
constexpr uint32_t kPacketSendIntervalMs = 40UL;
unsigned long lastGpsRateCheckTime = 0;
unsigned int gpsUpdateCount = 0;
constexpr uint32_t kGpsRateReportIntervalMs = 5000UL;
unsigned int gnssUpdateCount = 0;
constexpr uint32_t kDisconnectedBlinkIntervalMs = 500UL;
constexpr uint32_t kRestartAdvertisingDelayMs = 500UL;

// --- RaceBox Packet Layout ---
constexpr uint8_t kUbxSyncChar1 = 0xB5U;
constexpr uint8_t kUbxSyncChar2 = 0x62U;
constexpr uint8_t kRaceBoxMessageClass = 0xFFU;
constexpr uint8_t kRaceBoxMessageId = 0x01U;
constexpr size_t kRaceBoxHeaderSize = 6U;
constexpr size_t kRaceBoxPayloadSize = 80U;
constexpr size_t kRaceBoxChecksumSize = 2U;
constexpr size_t kRaceBoxPacketSize = kRaceBoxHeaderSize + kRaceBoxPayloadSize + kRaceBoxChecksumSize;
constexpr int kPacketPayloadOffset = 6;
constexpr int kPacketChecksumOffset = kPacketPayloadOffset + static_cast<int>(kRaceBoxPayloadSize);

constexpr int kPayloadOffsetITow = 0;
constexpr int kPayloadOffsetYear = 4;
constexpr int kPayloadOffsetMonth = 6;
constexpr int kPayloadOffsetDay = 7;
constexpr int kPayloadOffsetHour = 8;
constexpr int kPayloadOffsetMinute = 9;
constexpr int kPayloadOffsetSecond = 10;
constexpr int kPayloadOffsetValidityFlags = 11;
constexpr int kPayloadOffsetTimeAccuracy = 12;
constexpr int kPayloadOffsetNanoseconds = 16;
constexpr int kPayloadOffsetFixType = 20;
constexpr int kPayloadOffsetFixStatusFlags = 21;
constexpr int kPayloadOffsetDateTimeFlags = 22;
constexpr int kPayloadOffsetNumSv = 23;
constexpr int kPayloadOffsetLongitude = 24;
constexpr int kPayloadOffsetLatitude = 28;
constexpr int kPayloadOffsetHeight = 32;
constexpr int kPayloadOffsetHmsl = 36;
constexpr int kPayloadOffsetHorizontalAccuracy = 40;
constexpr int kPayloadOffsetVerticalAccuracy = 44;
constexpr int kPayloadOffsetGroundSpeed = 48;
constexpr int kPayloadOffsetHeadingOfMotion = 52;
constexpr int kPayloadOffsetSpeedAccuracy = 56;
constexpr int kPayloadOffsetHeadingAccuracy = 60;
constexpr int kPayloadOffsetPdop = 64;
constexpr int kPayloadOffsetLatLonFlags = 66;
constexpr int kPayloadOffsetBatteryStatus = 67;
constexpr int kPayloadOffsetAccelX = 68;
constexpr int kPayloadOffsetAccelY = 70;
constexpr int kPayloadOffsetAccelZ = 72;
constexpr int kPayloadOffsetGyroX = 74;
constexpr int kPayloadOffsetGyroY = 76;
constexpr int kPayloadOffsetGyroZ = 78;
constexpr uint8_t kReportedBatteryPercent = 100U;
constexpr float kAccelerationToMilliG = 1000.0f / 9.80665f;
constexpr float kGyroRadiansToCentiDegrees = 18000.0f / static_cast<float>(M_PI);
constexpr float kGpsRateReportIntervalSeconds = kGpsRateReportIntervalMs / 1000.0f;
constexpr double kLatLonToDegrees = 1e-7;

std::array<uint8_t, kRaceBoxPayloadSize> txPayloadBuffer = {};
std::array<uint8_t, kRaceBoxPacketSize> txPacketBuffer = {{
  kUbxSyncChar1,
  kUbxSyncChar2,
  kRaceBoxMessageClass,
  kRaceBoxMessageId,
  static_cast<uint8_t>(kRaceBoxPayloadSize),
  0U
}};

static_assert(kRaceBoxPacketSize == (kRaceBoxHeaderSize + kRaceBoxPayloadSize + kRaceBoxChecksumSize),
              "RaceBox packet size must match header + payload + checksum.");
static_assert(kPacketChecksumOffset + static_cast<int>(kRaceBoxChecksumSize) == static_cast<int>(kRaceBoxPacketSize),
              "RaceBox checksum offset must point to the final two bytes.");
static_assert(kPayloadOffsetGyroZ + static_cast<int>(sizeof(int16_t)) == static_cast<int>(kRaceBoxPayloadSize),
              "RaceBox payload layout no longer ends at gyro Z.");


// --- BLE Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    // Request a larger MTU to fit an 88-byte packet + headers in one go
    if (pServer != NULL) {
      pServer->updatePeerMTU(pServer->getConnId(), kRequestedBleMtu);
    }
    digitalWrite(kOnboardLedPin, HIGH);
    Serial.println("✅ BLE Client connected & MTU update requested");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    digitalWrite(kOnboardLedPin, LOW);
    Serial.println("❌ BLE Client disconnected");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic == NULL) {
      return;
    }

    uint8_t *rxValue = pCharacteristic->getData();
    const size_t rxLength = pCharacteristic->getLength();

    if ((rxValue != NULL) && (rxLength > 0U)) {
      Serial.print("📨 Received BLE command: ");
      for (size_t i = 0; i < rxLength; ++i) {
        Serial.printf("0x%02X ", rxValue[i]);
      }
      Serial.println();
    }
  }
};

// --- UBX Packet Construction Helpers ---
void writeLittleEndian(uint8_t* buffer, int offset, uint32_t value) { memcpy(buffer + offset, &value, 4); }
void writeLittleEndian(uint8_t* buffer, int offset, int32_t value)  { memcpy(buffer + offset, &value, 4); }
void writeLittleEndian(uint8_t* buffer, int offset, uint16_t value) { memcpy(buffer + offset, &value, 2); }
void writeLittleEndian(uint8_t* buffer, int offset, int16_t value)  { memcpy(buffer + offset, &value, 2); }
void writeLittleEndian(uint8_t* buffer, int offset, uint8_t value)  { buffer[offset] = value; }
void writeLittleEndian(uint8_t* buffer, int offset, int8_t value)   { buffer[offset] = (uint8_t)value; }

void calculateChecksum(uint8_t* payload, uint16_t len, uint8_t cls, uint8_t id, uint8_t* ckA, uint8_t* ckB) {
  *ckA = *ckB = 0;
  *ckA += cls; *ckB += *ckA;
  *ckA += id; *ckB += *ckA;
  *ckA += len & 0xFF; *ckB += *ckA;
  *ckA += len >> 8; *ckB += *ckA;
  for (uint16_t i = 0; i < len; i++) {
    *ckA += payload[i];
    *ckB += *ckA;
  }
}

void* requireBleObject(void *object, const char *description) {
  if (object == NULL) {
    Serial.print("❌ Failed to create ");
    Serial.println(description);
    while (1) {
      delay(100);
    }
  }

  return object;
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

void resetGpsBaudRate() {
  Serial.println("Attempting to set Correct Baud Rate");
  GPS_Serial.begin(kFactoryGpsBaudRate, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
  delay(500);

  if (!myGNSS.begin(GPS_Serial)) {
    Serial.print("u-blox GNSS not detected at ");
    Serial.print(kFactoryGpsBaudRate);
    Serial.println(" baud.");
    Serial.print("u-blox GNSS not detected, Check documentation for factory baud rate and/or check your wiring");
    while(1) delay(100);
  } else {
    Serial.print("GNSS detected at ");
    Serial.print(kFactoryGpsBaudRate);
    Serial.println(" baud!");
  }
  delay(500);

  // Now switch baud rate
  Serial.print("Setting baud rate to ");
  Serial.print(kGpsBaudRate);
  Serial.println("...");
  myGNSS.setSerialRate(kGpsBaudRate);
  Serial.print("Baud rate changed to ");
  Serial.println(kGpsBaudRate);

  GPS_Serial.end();
  delay(100);
  // Re-initialize the serial port at the new baud rate
  GPS_Serial.begin(kGpsBaudRate, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
  delay(500);

  if (!myGNSS.begin(GPS_Serial)) {
    Serial.print("GNSS not detected at ");
    Serial.print(kGpsBaudRate);
    Serial.println(" baud.");
    Serial.print("u-blox GNSS not detected, Check documentation for factory baud rate and/or check your wiring");
    while (1) delay(100);
  }
  Serial.print("GNSS detected at ");
  Serial.print(kGpsBaudRate);
  Serial.println(" baud! Saving to Flash");
  myGNSS.saveConfiguration(); // Save to flash
  GPS_Serial.end();
}

void validateDeviceNameOrHalt() {
  const char *suffix = kDeviceName + kDeviceNamePrefixLength;

  if (strncmp(kDeviceName, "RaceBox Mini ", kDeviceNamePrefixLength) != 0) {
    Serial.println("❌ Device name must start with 'RaceBox Mini '.");
    while (1) {
      delay(100);
    }
  }

  if (strspn(suffix, "0123456789") != kDeviceNameSuffixLength) {
    Serial.println("❌ Device name suffix must contain exactly 10 digits.");
    while (1) {
      delay(100);
    }
  }

  char *parsedEnd = NULL;
  const unsigned long suffixValue = strtoul(suffix, &parsedEnd, 10);

  if ((parsedEnd == NULL) || (*parsedEnd != '\0') || (suffixValue > kMaxAllowedDeviceSuffix)) {
    Serial.println("❌ RaceBox Mini number must be 3999999999 or lower.");
    while (1) {
      delay(100);
    }
  }

  memcpy(deviceSerialNumber, suffix, kDeviceNameSuffixLength);
  deviceSerialNumber[kDeviceNameSuffixLength] = '\0';
}

void configureGnssConstellations() {
  for (size_t i = 0; i < (sizeof(kGnssConstellations) / sizeof(kGnssConstellations[0])); ++i) {
    const GnssConstellationConfig &config = kGnssConstellations[i];
    const bool configured = myGNSS.enableGNSS(config.enabled, config.id);

    if (configured) {
      Serial.print(config.enabled ? "✅ " : "🚫 ");
      Serial.print(config.name);
      Serial.println(config.enabled ? " enabled." : " disabled.");
    } else {
      Serial.print("❌ Failed to ");
      Serial.print(config.enabled ? "enable " : "disable ");
      Serial.print(config.name);
      Serial.println(".");
    }
  }
}

void setup() {
  Serial.begin(kSerialBaudRate);
  pinMode(kOnboardLedPin, OUTPUT);
  validateDeviceNameOrHalt();
  if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050 chip");
    while (1) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Initialize filters with the first real reading so they don't start at zero
  filtered_ax = a.acceleration.x;
  filtered_ay = a.acceleration.y;
  filtered_az = a.acceleration.z;
  filtered_gx = g.gyro.x;
  filtered_gy = g.gyro.y;
  filtered_gz = g.gyro.z;

  GPS_Serial.begin(kGpsBaudRate, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
  if (!myGNSS.begin(GPS_Serial)) {
    Serial.println("❌ GNSS not detected. Attempting to configure.");
    GPS_Serial.end();
    resetGpsBaudRate();
    GPS_Serial.begin(kGpsBaudRate, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
  }

  // Set GNSS output to PVT only
  myGNSS.setAutoPVT(true);
  myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
    // --- Configure GPS update rate to kMaxNavigationRateHz Hz ---
  if (myGNSS.setNavigationFrequency(kMaxNavigationRateHz)) {
  Serial.printf("✅ GPS update rate set to %d Hz.\n",kMaxNavigationRateHz );
  } else {
    Serial.println("❌ Failed to set GPS update rate.");
  }

  // --- GNSS Constellation Setup ---
  configureGnssConstellations();

  // --- BLE Setup ---
  BLEDevice::init(kDeviceName);
  pServer = static_cast<BLEServer*>(requireBleObject(BLEDevice::createServer(), "BLE server"));
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = static_cast<BLEService*>(requireBleObject(pServer->createService(kRaceBoxServiceUuid), "RaceBox BLE service"));
  pCharacteristicTx = static_cast<BLECharacteristic*>(requireBleObject(pService->createCharacteristic(kRaceBoxCharacteristicTxUuid, BLECharacteristic::PROPERTY_NOTIFY), "RaceBox TX characteristic"));
  pCharacteristicTx->addDescriptor(static_cast<BLE2902*>(requireBleObject(new BLE2902(), "RaceBox TX descriptor")));
  pCharacteristicRx = static_cast<BLECharacteristic*>(requireBleObject(pService->createCharacteristic(kRaceBoxCharacteristicRxUuid, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR), "RaceBox RX characteristic"));
  pCharacteristicRx->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();
  // --- Device Information Service ---
  BLEService *pDeviceInfo = static_cast<BLEService*>(requireBleObject(pServer->createService(kDeviceInfoServiceUuid), "Device Information service"));
  // Model
  BLECharacteristic *pModel = static_cast<BLECharacteristic*>(requireBleObject(pDeviceInfo->createCharacteristic(kModelCharacteristicUuid, BLECharacteristic::PROPERTY_READ), "Device model characteristic"));
  pModel->setValue(kModelName);
  // Serial number (last 10 digits of device name)
  BLECharacteristic *pSerial = static_cast<BLECharacteristic*>(requireBleObject(pDeviceInfo->createCharacteristic(kSerialCharacteristicUuid, BLECharacteristic::PROPERTY_READ), "Device serial characteristic"));
  pSerial->setValue(reinterpret_cast<uint8_t*>(deviceSerialNumber), kDeviceNameSuffixLength);
  // Firmware revision
  BLECharacteristic *pFirm = static_cast<BLECharacteristic*>(requireBleObject(pDeviceInfo->createCharacteristic(kFirmwareCharacteristicUuid, BLECharacteristic::PROPERTY_READ), "Device firmware characteristic"));
  pFirm->setValue(kFirmwareRevision);
  // Hardware revision
  BLECharacteristic *pHardware = static_cast<BLECharacteristic*>(requireBleObject(pDeviceInfo->createCharacteristic(kHardwareCharacteristicUuid, BLECharacteristic::PROPERTY_READ), "Device hardware characteristic"));
  pHardware->setValue(kHardwareRevision);
  // Manufacturer
  BLECharacteristic *pManufacturer = static_cast<BLECharacteristic*>(requireBleObject(pDeviceInfo->createCharacteristic(kManufacturerCharacteristicUuid, BLECharacteristic::PROPERTY_READ), "Device manufacturer characteristic"));
  pManufacturer->setValue(kManufacturerName);
  pDeviceInfo->start();
  BLEAdvertising *pAdvertising = static_cast<BLEAdvertising*>(requireBleObject(BLEDevice::getAdvertising(), "BLE advertising"));
  pAdvertising->addServiceUUID(kRaceBoxServiceUuid);
  // Advertise Device Information Service to help official apps discover the device
  pAdvertising->addServiceUUID(kDeviceInfoServiceUuid);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("📡 BLE advertising started.");

  lastGpsRateCheckTime = millis();
}

void loop() {
  const unsigned long now = millis();
  myGNSS.checkUblox(); // Required to keep GNSS data flowing
  static unsigned long lastAccelReadMs = 0;

  if (lastAccelReadMs == 0UL) {
    lastAccelReadMs = now;
  }

  // Update Accelrometer readings at fixed interval
  bool shouldReadAccel = false;
  while ((now - lastAccelReadMs) >= kAccelSampleIntervalMs) {
    lastAccelReadMs += kAccelSampleIntervalMs; // Stay on the fixed timing grid if the loop slips.
    shouldReadAccel = true;
  }

  if (shouldReadAccel) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Apply Exponential Moving Average (Complementary Filter logic)
    filtered_ax = (accelAlpha * a.acceleration.x) + ((1.0 - accelAlpha) * filtered_ax);
    filtered_ay = (accelAlpha * a.acceleration.y) + ((1.0 - accelAlpha) * filtered_ay);
    filtered_az = (accelAlpha * a.acceleration.z) + ((1.0 - accelAlpha) * filtered_az);

    filtered_gx = (gyroAlpha * g.gyro.x) + ((1.0 - gyroAlpha) * filtered_gx);
    filtered_gy = (gyroAlpha * g.gyro.y) + ((1.0 - gyroAlpha) * filtered_gy);
    filtered_gz = (gyroAlpha * g.gyro.z) + ((1.0 - gyroAlpha) * filtered_gz);
  }
  // LED Blink Logic
  if (!deviceConnected) {
    static unsigned long lastBlinkMs = 0;
    if ((now - lastBlinkMs) > kDisconnectedBlinkIntervalMs) {
      lastBlinkMs = now;
      digitalWrite(kOnboardLedPin, !digitalRead(kOnboardLedPin));
    }
  } else {
    digitalWrite(kOnboardLedPin, HIGH);
  }
  if (myGNSS.getPVT()) {
    const UBX_NAV_PVT_t *navPvtPacket = myGNSS.packetUBXNAVPVT;
    static uint32_t lastITOW = 0;
    uint32_t currentITOW = 0;

    if (navPvtPacket != NULL) {
      currentITOW = navPvtPacket->data.iTOW;
    }

    if ((navPvtPacket != NULL) && (currentITOW != lastITOW)) {
      lastITOW = currentITOW;
      gnssUpdateCount++;

      if (deviceConnected && (pCharacteristicTx != NULL)) {
        lastPacketSendTime = now;
        gpsUpdateCount++;
        const auto &navData = navPvtPacket->data;

        // Convert accelerometer to milli-g (1g = 9.80665 m/s^2)
        const float accelXMilliG = filtered_ax * kAccelerationToMilliG;
        const float accelYMilliG = filtered_ay * kAccelerationToMilliG;
        const float accelZMilliG = filtered_az * kAccelerationToMilliG;
        const int16_t gX = saturatingFloatToInt16(accelXMilliG);
        const int16_t gY = saturatingFloatToInt16(accelYMilliG);
        const int16_t gZ = saturatingFloatToInt16(accelZMilliG);

        // Convert gyro to centi-deg/sec
        const float gyroXCentiDegrees = filtered_gx * kGyroRadiansToCentiDegrees;
        const float gyroYCentiDegrees = filtered_gy * kGyroRadiansToCentiDegrees;
        const float gyroZCentiDegrees = filtered_gz * kGyroRadiansToCentiDegrees;
        const int16_t rX = saturatingFloatToInt16(gyroXCentiDegrees);
        const int16_t rY = saturatingFloatToInt16(gyroYCentiDegrees);
        const int16_t rZ = saturatingFloatToInt16(gyroZCentiDegrees);

        uint8_t *payload = txPayloadBuffer.data();
        uint8_t *packet = txPacketBuffer.data();

        // Access data directly from navData
        writeLittleEndian(payload, kPayloadOffsetITow, navData.iTOW);
        writeLittleEndian(payload, kPayloadOffsetYear, navData.year);
        writeLittleEndian(payload, kPayloadOffsetMonth, navData.month);
        writeLittleEndian(payload, kPayloadOffsetDay, navData.day);
        writeLittleEndian(payload, kPayloadOffsetHour, navData.hour);
        writeLittleEndian(payload, kPayloadOffsetMinute, navData.min);
        writeLittleEndian(payload, kPayloadOffsetSecond, navData.sec);

        // Offset 11: Validity Flags (RaceBox Protocol) 
        uint8_t raceboxValidityFlags = 0;
        if (navData.valid.bits.validDate) raceboxValidityFlags |= (1 << 0); // Bit 0: valid date 
        if (navData.valid.bits.validTime) raceboxValidityFlags |= (1 << 1); // Bit 1: valid time 
        if (navData.valid.bits.fullyResolved) raceboxValidityFlags |= (1 << 2); // Bit 2: fully resolved 
        writeLittleEndian(payload, kPayloadOffsetValidityFlags, raceboxValidityFlags);

        // Offset 12: Time Accuracy (RaceBox Protocol) 
        writeLittleEndian(payload, kPayloadOffsetTimeAccuracy, navData.tAcc);

        // Offset 16: Nanoseconds (RaceBox Protocol) 
        writeLittleEndian(payload, kPayloadOffsetNanoseconds, navData.nano);

        // Offset 20: Fix Status (RaceBox Protocol) 
        writeLittleEndian(payload, kPayloadOffsetFixType, navData.fixType);

        // Offset 21: Fix Status Flags (RaceBox Protocol)
        uint8_t fixStatusFlagsRacebox = 0;

        if (navData.fixType == 3) {
            fixStatusFlagsRacebox |= (1 << 0); // Bit 0: valid fix
        }

        if (myGNSS.getHeadVehValid()) { // Use the confirmed function to check for valid heading
            fixStatusFlagsRacebox |= (1 << 5); // Bit 5: valid heading (as per RaceBox Protocol)
        }
        writeLittleEndian(payload, kPayloadOffsetFixStatusFlags, fixStatusFlagsRacebox);

        // Offset 22: Date/Time Flags (RaceBox Protocol) 
        uint8_t raceboxDateTimeFlags = 0;
        if (navData.valid.bits.validTime) raceboxDateTimeFlags |= (1 << 5); // Available confirmation of Date/Time Validity
        if (navData.valid.bits.validDate) raceboxDateTimeFlags |= (1 << 6); // Confirmed UTC Date Validity
        if (navData.valid.bits.validTime && navData.valid.bits.fullyResolved) raceboxDateTimeFlags |= (1 << 7); // Confirmed UTC Time Validity
        writeLittleEndian(payload, kPayloadOffsetDateTimeFlags, raceboxDateTimeFlags);

        // Offset 23: Number of SVs (RaceBox Protocol) 
        writeLittleEndian(payload, kPayloadOffsetNumSv, navData.numSV);

        // Remaining fields, mostly direct mappings from u-blox data
        writeLittleEndian(payload, kPayloadOffsetLongitude, navData.lon);
        writeLittleEndian(payload, kPayloadOffsetLatitude, navData.lat);
        writeLittleEndian(payload, kPayloadOffsetHeight, navData.height);
        writeLittleEndian(payload, kPayloadOffsetHmsl, navData.hMSL);

        writeLittleEndian(payload, kPayloadOffsetHorizontalAccuracy, navData.hAcc);
        writeLittleEndian(payload, kPayloadOffsetVerticalAccuracy, navData.vAcc);
        writeLittleEndian(payload, kPayloadOffsetGroundSpeed, navData.gSpeed);
        writeLittleEndian(payload, kPayloadOffsetHeadingOfMotion, navData.headMot);
        writeLittleEndian(payload, kPayloadOffsetSpeedAccuracy, navData.sAcc);
        writeLittleEndian(payload, kPayloadOffsetHeadingAccuracy, navData.headAcc);

        writeLittleEndian(payload, kPayloadOffsetPdop, navData.pDOP);

        // Offset 66: Lat/Lon Flags (RaceBox Protocol) 
        uint8_t latLonFlags = 0;
        if (navData.fixType < 2) { // If no 2D/3D fix, then coordinates are considered invalid 
            latLonFlags |= (1 << 0); // Bit 0: Invalid Latitude, Longitude, WGS Altitude, and MSL Altitude
        }
        writeLittleEndian(payload, kPayloadOffsetLatLonFlags, latLonFlags);

        // Offset 67: Battery status (1 byte) - report 100% to avoid low battery warnings
        writeLittleEndian(payload, kPayloadOffsetBatteryStatus, kReportedBatteryPercent);

        writeLittleEndian(payload, kPayloadOffsetAccelX, gX);
        writeLittleEndian(payload, kPayloadOffsetAccelY, gY);
        writeLittleEndian(payload, kPayloadOffsetAccelZ, gZ);
        writeLittleEndian(payload, kPayloadOffsetGyroX, rX);
        writeLittleEndian(payload, kPayloadOffsetGyroY, rY);
        writeLittleEndian(payload, kPayloadOffsetGyroZ, rZ);

        // Wrap in UBX (standard RaceBox header and checksum)
        memcpy(packet + kPacketPayloadOffset, payload, kRaceBoxPayloadSize);
        uint8_t ckA, ckB; 
        calculateChecksum(payload, kRaceBoxPayloadSize, kRaceBoxMessageClass, kRaceBoxMessageId, &ckA, &ckB);
        packet[kPacketChecksumOffset] = ckA;
        packet[kPacketChecksumOffset + 1] = ckB;

        pCharacteristicTx->setValue(packet, kRaceBoxPacketSize);
        pCharacteristicTx->notify();
      }
    }

    // Report packet send rate
    if ((now - lastGpsRateCheckTime) >= kGpsRateReportIntervalMs) {
      const float bleRate = gpsUpdateCount / kGpsRateReportIntervalSeconds;
      const float gnssRate = gnssUpdateCount / kGpsRateReportIntervalSeconds;
      // Additional satellite info for debugging: number of satellites, fix type, horizontal accuracy, and lat/lon
      uint8_t sats = 0;
      uint8_t fix = 0;
      uint32_t hAcc = 0;
      double lat = 0.0, lon = 0.0;
      if (navPvtPacket != NULL) {
        const auto &navData = navPvtPacket->data;
        sats = navData.numSV;
        fix = navData.fixType;
        hAcc = navData.hAcc;
        lat = navData.lat * kLatLonToDegrees;
        lon = navData.lon * kLatLonToDegrees;
      }
      Serial.printf("BLE Packet Rate: %.2f Hz | GNSS Update Rate: %.2f Hz | SVs: %u | Fix: %u | HAcc: %u mm | Lat: %.7f Lon: %.7f\n",
                    bleRate, gnssRate, sats, fix, hAcc, lat, lon);
      gpsUpdateCount = 0;
      gnssUpdateCount = 0;
      lastGpsRateCheckTime = now;
    }

    if (!deviceConnected && oldDeviceConnected) {
      delay(kRestartAdvertisingDelayMs);
      if (pServer != NULL) {
        pServer->startAdvertising();
      }
      oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
    }
  }
}
