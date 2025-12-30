#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>

#define XSHUT_PIN A1
#define SERIAL_BAUD 115200
constexpr uint8_t kMaxTargets = VL53L4CX_MAX_RANGE_RESULTS;
constexpr uint8_t kSensorCount = 2;
constexpr uint8_t kMuxAddress = 0x70;
constexpr uint8_t kMuxPorts[kSensorCount] = {0, 1};
constexpr uint16_t kSensorSpacingMm = 100;
constexpr uint16_t kObjectMaxRangeMm = 300;
constexpr float kHoScaleRatio = 87.0f;
constexpr float kMmPerMeter = 1000.0f;
constexpr float kMsPerSecond = 1000.0f;
constexpr float kMpsToMph = 2.23693629f;
constexpr uint32_t kLedBlinkMs = 50;

VL53L4CX tof0(&Wire, XSHUT_PIN);
VL53L4CX tof1(&Wire, XSHUT_PIN);
VL53L4CX *const kSensors[kSensorCount] = {&tof0, &tof1};

static bool objectBlocked[kSensorCount] = {};
static bool waitingForSecondSensor = false;
static uint8_t firstSensorIndex = 0;
static uint32_t firstBlockMs = 0;
static bool ledOn = false;
static uint32_t ledOffMs = 0;

static bool selectMuxChannel(uint8_t channel)
{
  if (channel > 7) {
    return false;
  }
  Wire.beginTransmission(kMuxAddress);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

static void printUid(uint64_t uid)
{
  for (int i = 7; i >= 0; --i) {
    uint8_t byte = (uid >> (static_cast<uint8_t>(i) * 8)) & 0xFF;
    if (byte < 16) {
      Serial.print('0');
    }
    Serial.print(byte, HEX);
  }
}

static void printSensorInfo(VL53L4CX &sensor, uint8_t sensorIndex)
{
  VL53L4CX_Version_t version;
  Serial.print("Sensor ");
  Serial.print(sensorIndex);
  Serial.println();
  if (sensor.VL53L4CX_GetVersion(&version) == VL53L4CX_ERROR_NONE) {
    Serial.print("Driver version: ");
    Serial.print(version.major);
    Serial.print('.');
    Serial.print(version.minor);
    Serial.print('.');
    Serial.print(version.build);
    Serial.print(" (rev ");
    Serial.print(version.revision);
    Serial.println(')');
  } else {
    Serial.println("Driver version: read failed");
  }

  VL53L4CX_DeviceInfo_t info;
  if (sensor.VL53L4CX_GetDeviceInfo(&info) == VL53L4CX_ERROR_NONE) {
    Serial.print("Device type: 0x");
    Serial.print(info.ProductType, HEX);
    Serial.print(" rev ");
    Serial.print(info.ProductRevisionMajor);
    Serial.print('.');
    Serial.println(info.ProductRevisionMinor);
  } else {
    Serial.println("Device info: read failed");
  }

  uint64_t uid = 0;
  if (sensor.VL53L4CX_GetUID(&uid) == VL53L4CX_ERROR_NONE) {
    Serial.print("Device UID: 0x");
    printUid(uid);
    Serial.println();
  } else {
    Serial.println("Device UID: read failed");
  }
}


static bool findNearestObjectWithin(const VL53L4CX_MultiRangingData_t &data,
                                    uint16_t maxRangeMm,
                                    uint16_t *nearestRangeMm)
{
  bool found = false;
  uint16_t best = 0;
  for (uint8_t i = 0; i < data.NumberOfObjectsFound && i < kMaxTargets; ++i) {
    uint16_t range = data.RangeData[i].RangeMilliMeter;
    if (range == 0 || range > maxRangeMm) {
      continue;
    }
    if (!found || range < best) {
      best = range;
      found = true;
    }
  }

  if (found && nearestRangeMm != nullptr) {
    *nearestRangeMm = best;
  }
  return found;
}

static void reportTransitSpeed(uint8_t startSensor,
                               uint8_t endSensor,
                               uint32_t deltaMs)
{
  if (deltaMs == 0) {
    return;
  }
  float deltaSeconds = static_cast<float>(deltaMs) / kMsPerSecond;
  float distanceMeters = static_cast<float>(kSensorSpacingMm) / kMmPerMeter;
  float speedMps = distanceMeters / deltaSeconds;
  float scaleMph = speedMps * kMpsToMph * kHoScaleRatio;

  Serial.print("Transit dir=");
  Serial.print(startSensor);
  Serial.print("->");
  Serial.print(endSensor);
  Serial.print(" delta=");
  Serial.print(deltaMs);
  Serial.print("ms: ");
  Serial.print(speedMps, 3);
  Serial.print(" m/s, ");
  Serial.print(scaleMph, 1);
  Serial.println(" scale mph");
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000)) {
    delay(10);
  }

  Serial.println("VL53L4CX initialization");
  Serial.println("Wiring: SDA=D14, SCL=D15, XSHUT=A1, VIN=3V3, GND=GND");
  Serial.println("Qwiic mux: address 0x70, sensors on ports 0 and 1");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();

  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (!selectMuxChannel(kMuxPorts[i])) {
      Serial.print("Mux select failed for port ");
      Serial.println(kMuxPorts[i]);
      while (true) {
        delay(100);
      }
    }

    VL53L4CX &sensor = *kSensors[i];
    sensor.begin();
    sensor.VL53L4CX_Off();

    VL53L4CX_Error status = sensor.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
    if (status != VL53L4CX_ERROR_NONE) {
      Serial.print("InitSensor failed on sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(status);
      while (true) {
        delay(100);
      }
    }

    printSensorInfo(sensor, i);

    status = sensor.VL53L4CX_StopMeasurement();
    if (status != VL53L4CX_ERROR_NONE) {
      Serial.print("StopMeasurement warning on sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(status);
    }

    status = sensor.VL53L4CX_StartMeasurement();
    if (status != VL53L4CX_ERROR_NONE) {
      Serial.print("StartMeasurement failed on sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(status);
      while (true) {
        delay(100);
      }
    }
  }
}

void loop()
{
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (!selectMuxChannel(kMuxPorts[i])) {
      Serial.print("Mux select failed for port ");
      Serial.println(kMuxPorts[i]);
      delay(100);
      continue;
    }

    VL53L4CX &sensor = *kSensors[i];
    uint8_t dataReady = 0;
    VL53L4CX_Error status = sensor.VL53L4CX_GetMeasurementDataReady(&dataReady);
    if (status != VL53L4CX_ERROR_NONE) {
      Serial.print("DataReady error on sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(status);
      delay(100);
      continue;
    }

    if (dataReady) {
      uint32_t sampleMs = millis();
      VL53L4CX_MultiRangingData_t data;
      status = sensor.VL53L4CX_GetMultiRangingData(&data);
      if (status == VL53L4CX_ERROR_NONE) {
        bool objectPresent =
            findNearestObjectWithin(data, kObjectMaxRangeMm, nullptr);
        if (objectPresent) {
          if (!objectBlocked[i]) {
            objectBlocked[i] = true;
            ledOn = true;
            ledOffMs = sampleMs + kLedBlinkMs;
            digitalWrite(LED_BUILTIN, HIGH);
            if (!waitingForSecondSensor) {
              waitingForSecondSensor = true;
              firstSensorIndex = i;
              firstBlockMs = sampleMs;
            } else if (i != firstSensorIndex) {
              uint32_t deltaMs = sampleMs - firstBlockMs;
              reportTransitSpeed(firstSensorIndex, i, deltaMs);
              waitingForSecondSensor = false;
            }
          }
        } else {
          objectBlocked[i] = false;
        }
      } else {
        Serial.print("GetMultiRangingData error on sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(status);
      }

      status = sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
      if (status != VL53L4CX_ERROR_NONE) {
        Serial.print("ClearInterrupt error on sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(status);
        delay(100);
      }
    }
  }

  uint32_t nowMs = millis();
  if (ledOn && static_cast<int32_t>(nowMs - ledOffMs) >= 0) {
    digitalWrite(LED_BUILTIN, LOW);
    ledOn = false;
  }
}
