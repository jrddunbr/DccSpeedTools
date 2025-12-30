#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>

#define XSHUT_PIN A1
#define SERIAL_BAUD 115200
constexpr uint32_t kReportIntervalMs = 250;
constexpr uint8_t kMaxTargets = VL53L4CX_MAX_RANGE_RESULTS;
constexpr uint8_t kSensorCount = 2;
constexpr uint8_t kMuxAddress = 0x70;
constexpr uint8_t kMuxPorts[kSensorCount] = {0, 1};

VL53L4CX tof0(&Wire, XSHUT_PIN);
VL53L4CX tof1(&Wire, XSHUT_PIN);
VL53L4CX *const kSensors[kSensorCount] = {&tof0, &tof1};

static uint32_t rangeSum[kSensorCount][kMaxTargets];
static uint16_t rangeCount[kSensorCount][kMaxTargets];
static uint16_t sampleCount[kSensorCount];
static uint32_t lastReportMs = 0;

static bool selectMuxChannel(uint8_t channel)
{
  if (channel > 7) {
    return false;
  }
  Wire.beginTransmission(kMuxAddress);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

static void resetAveraging(uint8_t sensorIndex)
{
  for (uint8_t i = 0; i < kMaxTargets; ++i) {
    rangeSum[sensorIndex][i] = 0;
    rangeCount[sensorIndex][i] = 0;
  }
  sampleCount[sensorIndex] = 0;
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

static void accumulateRangingData(uint8_t sensorIndex,
                                  const VL53L4CX_MultiRangingData_t &data)
{
  for (uint8_t i = 0; i < data.NumberOfObjectsFound && i < kMaxTargets; ++i) {
    rangeSum[sensorIndex][i] += data.RangeData[i].RangeMilliMeter;
    rangeCount[sensorIndex][i] += 1;
  }
  sampleCount[sensorIndex] += 1;
}

static void printAveragedRangingData(uint8_t sensorIndex)
{
  uint8_t objectsPrinted = 0;
  for (uint8_t i = 0; i < kMaxTargets; ++i) {
    if (rangeCount[sensorIndex][i] > 0) {
      objectsPrinted += 1;
    }
  }

  Serial.print("Sensor ");
  Serial.print(sensorIndex);
  Serial.print(" objects ");
  Serial.println(objectsPrinted);

  for (uint8_t i = 0; i < kMaxTargets; ++i) {
    if (rangeCount[sensorIndex][i] == 0) {
      continue;
    }
    uint32_t avg = (rangeSum[sensorIndex][i] + (rangeCount[sensorIndex][i] / 2)) /
                   rangeCount[sensorIndex][i];
    Serial.print("Obj ");
    Serial.print(i);
    Serial.print(": distance=");
    Serial.print(avg);
    Serial.println("mm");
  }
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

    resetAveraging(i);
  }

  lastReportMs = millis();
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
      VL53L4CX_MultiRangingData_t data;
      status = sensor.VL53L4CX_GetMultiRangingData(&data);
      if (status == VL53L4CX_ERROR_NONE) {
        accumulateRangingData(i, data);
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

  uint32_t now = millis();
  if ((now - lastReportMs) >= kReportIntervalMs) {
    for (uint8_t i = 0; i < kSensorCount; ++i) {
      if (sampleCount[i] > 0) {
        printAveragedRangingData(i);
        resetAveraging(i);
      }
    }
    lastReportMs = now;
  }
}
