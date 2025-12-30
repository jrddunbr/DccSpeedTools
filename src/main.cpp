#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>

#define XSHUT_PIN A1
#define SERIAL_BAUD 115200
constexpr uint8_t kMaxTargets = VL53L4CX_MAX_RANGE_RESULTS;
constexpr uint8_t kSensorCount = 2;
constexpr uint8_t kMuxAddress = 0x70;
constexpr uint8_t kMuxPorts[kSensorCount] = {0, 1};
constexpr uint32_t kI2cClockHz = 1000000;
constexpr uint32_t kTimingBudgetUs = 15000;
constexpr uint16_t kSensorSpacingMm = 270;
constexpr uint16_t kObjectMaxRangeMm = 150;
constexpr uint8_t kClearSamplesRequired = 3;
constexpr float kHoScaleRatio = 87.0f;
constexpr float kMmPerMeter = 1000.0f;
constexpr float kMsPerSecond = 1000.0f;
constexpr float kUsPerSecond = 1000000.0f;
constexpr float kMphToMps = 0.44704f;
constexpr float kMpsToMph = 2.23693629f;
constexpr float kMinScaleMph = 0.0f;
constexpr float kMaxScaleMph = 160.0f;
constexpr float kSensorSpacingMeters =
    static_cast<float>(kSensorSpacingMm) / kMmPerMeter;
constexpr float kMaxSpeedMps = (kMaxScaleMph / kHoScaleRatio) * kMphToMps;
constexpr uint32_t kMinTransitUs = static_cast<uint32_t>(
    (kSensorSpacingMeters / kMaxSpeedMps) * kUsPerSecond + 0.5f);
constexpr uint32_t kStatsIntervalUs = 15000000;
constexpr uint32_t kTransitTimeoutUs = 30000000;
constexpr uint32_t kWaitLogIntervalUs = 1000000;
constexpr uint8_t kDebounceSamples = 3;
constexpr bool kDebugSensorState = true;
constexpr bool kDebugTransitState = true;

VL53L4CX tof0(&Wire, XSHUT_PIN);
VL53L4CX tof1(&Wire, XSHUT_PIN);
VL53L4CX *const kSensors[kSensorCount] = {&tof0, &tof1};

static bool objectBlocked[kSensorCount] = {};
static bool objectPresentState[kSensorCount] = {};
static bool lastObjectPresentState[kSensorCount] = {};
static uint8_t presentStreak[kSensorCount] = {};
static uint8_t clearStreak[kSensorCount] = {};
static uint8_t clearSamples[kSensorCount] = {};
static bool transitArmed = false;
static bool waitingForSecondSensor = false;
static uint8_t firstSensorIndex = 0;
static uint32_t firstBlockUs = 0;
static uint32_t lastWaitLogUs = 0;
static uint32_t measurementStartUs[kSensorCount] = {};
static uint32_t lastStatsReportUs = 0;
static uint32_t lastMeasAvgUs[kSensorCount] = {};
static uint32_t lastMeasMaxUs[kSensorCount] = {};

struct TimingStats {
  uint32_t min;
  uint32_t max;
  uint64_t sum;
  uint32_t count;
};

static TimingStats measurementStats[kSensorCount];
static TimingStats transferStats[kSensorCount];

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

static void resetTimingStats(TimingStats &stats)
{
  stats.min = UINT32_MAX;
  stats.max = 0;
  stats.sum = 0;
  stats.count = 0;
}

static void updateTimingStats(TimingStats &stats, uint32_t value)
{
  if (value < stats.min) {
    stats.min = value;
  }
  if (value > stats.max) {
    stats.max = value;
  }
  stats.sum += value;
  stats.count += 1;
}

static void cacheMeasurementStats(uint8_t sensorIndex)
{
  if (sensorIndex >= kSensorCount) {
    return;
  }

  const TimingStats &stats = measurementStats[sensorIndex];
  if (stats.count > 0) {
    lastMeasAvgUs[sensorIndex] = static_cast<uint32_t>(stats.sum / stats.count);
    lastMeasMaxUs[sensorIndex] = stats.max;
  }
}

static bool getMeasurementAvgMaxUs(uint8_t sensorIndex,
                                   uint32_t *avgUs,
                                   uint32_t *maxUs)
{
  if (sensorIndex >= kSensorCount) {
    return false;
  }

  const TimingStats &stats = measurementStats[sensorIndex];
  if (stats.count > 0) {
    if (avgUs != nullptr) {
      *avgUs = static_cast<uint32_t>(stats.sum / stats.count);
    }
    if (maxUs != nullptr) {
      *maxUs = stats.max;
    }
    return true;
  }

  if (lastMeasAvgUs[sensorIndex] > 0) {
    if (avgUs != nullptr) {
      *avgUs = lastMeasAvgUs[sensorIndex];
    }
    if (maxUs != nullptr) {
      *maxUs = lastMeasMaxUs[sensorIndex];
    }
    return true;
  }

  return false;
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
                               uint32_t deltaUs)
{
  if (deltaUs == 0) {
    return;
  }
  uint32_t avgStartUs = 0;
  uint32_t maxStartUs = 0;
  uint32_t avgEndUs = 0;
  uint32_t maxEndUs = 0;
  bool haveStartStats =
      getMeasurementAvgMaxUs(startSensor, &avgStartUs, &maxStartUs);
  bool haveEndStats =
      getMeasurementAvgMaxUs(endSensor, &avgEndUs, &maxEndUs);
  uint32_t errUsTypical = 0;
  if (haveStartStats && haveEndStats) {
    errUsTypical = (avgStartUs + avgEndUs) / 2;
  }

  float deltaSeconds = static_cast<float>(deltaUs) / kUsPerSecond;
  float distanceMeters = static_cast<float>(kSensorSpacingMm) / kMmPerMeter;
  float speedMps = distanceMeters / deltaSeconds;
  float scaleMph = speedMps * kMpsToMph * kHoScaleRatio;
  float deltaMs = static_cast<float>(deltaUs) / 1000.0f;
  float speedErrMphTypical = 0.0f;
  if (errUsTypical > 0 && deltaUs > errUsTypical) {
    float ratio =
        static_cast<float>(errUsTypical) / static_cast<float>(deltaUs);
    speedErrMphTypical = scaleMph * ratio;
  }
  bool outOfRange = scaleMph < kMinScaleMph || scaleMph > kMaxScaleMph;

  if (outOfRange) {
    Serial.print("\x1b[31m");
  }
  Serial.print("Transit dir=");
  Serial.print(startSensor);
  Serial.print("->");
  Serial.print(endSensor);
  Serial.print(" delta=");
  Serial.print(deltaMs, 3);
  Serial.print("ms mph=");
  Serial.print(scaleMph, 1);
  Serial.print(" +/-");
  Serial.print(speedErrMphTypical, 1);
  Serial.print(" scale mph");
  if (outOfRange) {
    Serial.print("\x1b[0m");
  }
  Serial.println();
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
  Wire.setClock(kI2cClockHz);

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

    status = sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(
        kTimingBudgetUs);
    if (status != VL53L4CX_ERROR_NONE) {
      Serial.print("TimingBudget warning on sensor ");
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

    measurementStartUs[i] = micros();
    resetTimingStats(measurementStats[i]);
    resetTimingStats(transferStats[i]);
  }

  lastStatsReportUs = micros();
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
      uint32_t sampleUs = micros();
      updateTimingStats(measurementStats[i],
                        static_cast<uint32_t>(sampleUs - measurementStartUs[i]));
      VL53L4CX_MultiRangingData_t data;
      uint32_t transferStartUs = micros();
      status = sensor.VL53L4CX_GetMultiRangingData(&data);
      uint32_t transferElapsedUs = micros() - transferStartUs;
      updateTimingStats(transferStats[i], transferElapsedUs);
      if (status == VL53L4CX_ERROR_NONE) {
        bool objectPresent =
            findNearestObjectWithin(data, kObjectMaxRangeMm, nullptr);
        if (objectPresent) {
          if (presentStreak[i] < UINT8_MAX) {
            presentStreak[i] += 1;
          }
          clearStreak[i] = 0;
        } else {
          if (clearStreak[i] < UINT8_MAX) {
            clearStreak[i] += 1;
          }
          presentStreak[i] = 0;
        }

        if (presentStreak[i] >= kDebounceSamples) {
          objectPresentState[i] = true;
        } else if (clearStreak[i] >= kDebounceSamples) {
          objectPresentState[i] = false;
        }

        bool debouncedPresent = objectPresentState[i];
        if (debouncedPresent != lastObjectPresentState[i]) {
          if (kDebugSensorState) {
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(debouncedPresent ? " blocked" : " clear");
            Serial.print(" at ");
            Serial.print(static_cast<float>(sampleUs) / 1000.0f, 3);
            Serial.println("ms");
          }
          lastObjectPresentState[i] = debouncedPresent;
        }
        if (debouncedPresent) {
          clearSamples[i] = 0;
          if (!objectBlocked[i]) {
            objectBlocked[i] = true;
            if (!waitingForSecondSensor) {
              if (transitArmed) {
                waitingForSecondSensor = true;
                transitArmed = false;
                firstSensorIndex = i;
                firstBlockUs = sampleUs;
                lastWaitLogUs = sampleUs;
                if (kDebugTransitState) {
                  Serial.print("Transit start sensor ");
                  Serial.print(i);
                  Serial.print(" at ");
                  Serial.print(static_cast<float>(sampleUs) / 1000.0f, 3);
                  Serial.println("ms");
                }
              }
            }
          }
          if (waitingForSecondSensor && i != firstSensorIndex) {
            uint32_t deltaUs = sampleUs - firstBlockUs;
            if (deltaUs >= kMinTransitUs) {
              if (kDebugTransitState) {
                Serial.print("Transit second sensor ");
                Serial.print(i);
                Serial.print(" delta=");
                Serial.print(static_cast<float>(deltaUs) / 1000.0f, 3);
                Serial.println("ms");
              }
              reportTransitSpeed(firstSensorIndex, i, deltaUs);
            } else if (kDebugTransitState) {
              Serial.print("Transit rejected delta=");
              Serial.print(static_cast<float>(deltaUs) / 1000.0f, 3);
              Serial.print("ms < min ");
              Serial.print(static_cast<float>(kMinTransitUs) / 1000.0f, 3);
              Serial.println("ms");
            }
            waitingForSecondSensor = false;
            lastWaitLogUs = 0;
          }
        } else {
          objectBlocked[i] = false;
          if (clearSamples[i] < UINT8_MAX) {
            clearSamples[i] += 1;
          }
        }

        if (clearSamples[0] >= kClearSamplesRequired &&
            clearSamples[1] >= kClearSamplesRequired) {
          if (!waitingForSecondSensor) {
            transitArmed = true;
          }
        } else if (!waitingForSecondSensor) {
          transitArmed = false;
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
      } else {
        measurementStartUs[i] = micros();
      }
    }
  }

  uint32_t nowUs = micros();
  if (waitingForSecondSensor && kDebugTransitState) {
    uint32_t elapsedUs = nowUs - firstBlockUs;
    if (elapsedUs >= kWaitLogIntervalUs &&
        (lastWaitLogUs == 0 ||
         (uint32_t)(nowUs - lastWaitLogUs) >= kWaitLogIntervalUs)) {
      uint32_t elapsedSec = elapsedUs / kWaitLogIntervalUs;
      uint32_t totalSec = kTransitTimeoutUs / kWaitLogIntervalUs;
      Serial.print("Waiting for second sensor reading (");
      Serial.print(elapsedSec);
      Serial.print("/");
      Serial.print(totalSec);
      Serial.println(" seconds elapsed)");
      lastWaitLogUs = nowUs;
    }
  }
  if (waitingForSecondSensor &&
      (uint32_t)(nowUs - firstBlockUs) >= kTransitTimeoutUs) {
    waitingForSecondSensor = false;
    lastWaitLogUs = 0;
    if (kDebugTransitState) {
      Serial.println("Transit timeout");
    }
  }
  if ((uint32_t)(nowUs - lastStatsReportUs) >= kStatsIntervalUs) {
    for (uint8_t i = 0; i < kSensorCount; ++i) {
      cacheMeasurementStats(i);
      resetTimingStats(measurementStats[i]);
      resetTimingStats(transferStats[i]);
    }
    lastStatsReportUs = nowUs;
  }

  bool anyObjectPresent = false;
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (objectPresentState[i]) {
      anyObjectPresent = true;
      break;
    }
  }
  digitalWrite(LED_BUILTIN, anyObjectPresent ? HIGH : LOW);
}
