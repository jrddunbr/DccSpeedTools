#include <Arduino.h>
#include <Wire.h>
#include <cbor.h>
#include <vl53l4cx_class.h>
#include <string.h>

#define XSHUT_PIN A1
#define SERIAL_BAUD 115200
constexpr char kApiVersion[] = "1.0.0";
constexpr size_t kCborBufferSize = 256;
constexpr size_t kCborRxBufferSize = 256;
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
constexpr float kMaxSpeedMps = (kMaxScaleMph / kHoScaleRatio) * kMphToMps;
constexpr uint32_t kMinTransitUs = static_cast<uint32_t>(
    ((static_cast<float>(kSensorSpacingMm) / kMmPerMeter) / kMaxSpeedMps) *
        kUsPerSecond +
    0.5f);
constexpr uint32_t kStatsIntervalUs = 15000000;
constexpr uint32_t kTransitTimeoutUs = 30000000;
constexpr uint32_t kWaitLogIntervalUs = 1000000;
constexpr uint32_t kApiVersionIntervalUs = 300000000;
constexpr uint8_t kDebounceSamples = 3;
constexpr bool kDebugSensorState = true;
constexpr bool kDebugTransitState = true;

VL53L4CX tof0(&Wire, XSHUT_PIN);
VL53L4CX tof1(&Wire, XSHUT_PIN);
VL53L4CX *const kSensors[kSensorCount] = {&tof0, &tof1};

static uint8_t cborBuffer[kCborBufferSize];
static uint8_t cborRxBuffer[kCborRxBufferSize];
static size_t cborRxLen = 0;

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
static uint32_t lastApiVersionReportUs = 0;
static uint16_t sensorSpacingMm = kSensorSpacingMm;
static uint32_t minTransitUs = kMinTransitUs;
static uint32_t transitTimeoutUs = kTransitTimeoutUs;
static uint16_t objectMaxRangeMm = kObjectMaxRangeMm;

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

static void beginCborMap(CborEncoder *encoder, CborEncoder *map, size_t pairs)
{
  cbor_encoder_init(encoder, cborBuffer, sizeof(cborBuffer), 0);
  cbor_encoder_create_map(encoder, map, pairs);
}

static void endCborMap(CborEncoder *encoder, CborEncoder *map)
{
  cbor_encoder_close_container(encoder, map);
  size_t len = cbor_encoder_get_buffer_size(encoder, cborBuffer);
  Serial.write(cborBuffer, len);
}

static void cborEncodeText(CborEncoder *map, const char *key, const char *value)
{
  cbor_encode_text_stringz(map, key);
  cbor_encode_text_stringz(map, value);
}

static void cborEncodeUint(CborEncoder *map, const char *key, uint64_t value)
{
  cbor_encode_text_stringz(map, key);
  cbor_encode_uint(map, value);
}

static void cborEncodeInt(CborEncoder *map, const char *key, int32_t value)
{
  cbor_encode_text_stringz(map, key);
  cbor_encode_int(map, value);
}

static void cborEncodeFloat(CborEncoder *map, const char *key, float value)
{
  cbor_encode_text_stringz(map, key);
  cbor_encode_float(map, value);
}

static void cborEncodeBool(CborEncoder *map, const char *key, bool value)
{
  cbor_encode_text_stringz(map, key);
  cbor_encode_boolean(map, value);
}

static void cborEncodeIntArray(CborEncoder *map,
                               const char *key,
                               const int32_t *values,
                               size_t count)
{
  cbor_encode_text_stringz(map, key);
  CborEncoder array;
  cbor_encoder_create_array(map, &array, count);
  for (size_t i = 0; i < count; ++i) {
    cbor_encode_int(&array, values[i]);
  }
  cbor_encoder_close_container(map, &array);
}

static void emitCborLog(const char *level,
                        const char *message,
                        int32_t sensorIndex,
                        int32_t code,
                        int32_t port,
                        uint32_t tsUs)
{
  size_t pairs = 4;
  if (sensorIndex >= 0) {
    pairs += 1;
  }
  if (code >= 0) {
    pairs += 1;
  }
  if (port >= 0) {
    pairs += 1;
  }

  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, pairs);
  cborEncodeText(&map, "t", "log");
  cborEncodeText(&map, "l", level);
  cborEncodeText(&map, "m", message);
  cborEncodeUint(&map, "ts", tsUs);
  if (sensorIndex >= 0) {
    cborEncodeInt(&map, "s", sensorIndex);
  }
  if (code >= 0) {
    cborEncodeInt(&map, "c", code);
  }
  if (port >= 0) {
    cborEncodeInt(&map, "p", port);
  }
  endCborMap(&encoder, &map);
}

static void emitCborSensorInfo(uint8_t sensorIndex,
                               const VL53L4CX_Version_t &version,
                               bool hasVersion,
                               const VL53L4CX_DeviceInfo_t &info,
                               bool hasInfo,
                               uint64_t uid,
                               bool hasUid)
{
  if (!hasVersion && !hasInfo && !hasUid) {
    return;
  }

  size_t pairs = 2;
  if (hasVersion) {
    pairs += 1;
  }
  if (hasInfo) {
    pairs += 1;
  }
  if (hasUid) {
    pairs += 1;
  }

  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, pairs);
  cborEncodeText(&map, "t", "sensor");
  cborEncodeInt(&map, "s", sensorIndex);
  if (hasVersion) {
    int32_t drv[4] = {version.major, version.minor, version.build,
                      version.revision};
    cborEncodeIntArray(&map, "drv", drv, 4);
  }
  if (hasInfo) {
    int32_t prod[3] = {info.ProductType, info.ProductRevisionMajor,
                       info.ProductRevisionMinor};
    cborEncodeIntArray(&map, "prod", prod, 3);
  }
  if (hasUid) {
    cborEncodeUint(&map, "uid", uid);
  }
  endCborMap(&encoder, &map);
}

static void emitSensorInfo(VL53L4CX &sensor, uint8_t sensorIndex)
{
  VL53L4CX_Version_t version;
  VL53L4CX_DeviceInfo_t info;
  uint64_t uid = 0;
  uint32_t tsUs = micros();

  VL53L4CX_Error status = sensor.VL53L4CX_GetVersion(&version);
  bool hasVersion = status == VL53L4CX_ERROR_NONE;
  if (!hasVersion) {
    emitCborLog("w", "driver_version_failed", sensorIndex, status, -1, tsUs);
  }

  status = sensor.VL53L4CX_GetDeviceInfo(&info);
  bool hasInfo = status == VL53L4CX_ERROR_NONE;
  if (!hasInfo) {
    emitCborLog("w", "device_info_failed", sensorIndex, status, -1, tsUs);
  }

  status = sensor.VL53L4CX_GetUID(&uid);
  bool hasUid = status == VL53L4CX_ERROR_NONE;
  if (!hasUid) {
    emitCborLog("w", "device_uid_failed", sensorIndex, status, -1, tsUs);
  }

  emitCborSensorInfo(sensorIndex, version, hasVersion, info, hasInfo, uid,
                     hasUid);
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

static void emitCborSensorState(uint8_t sensorIndex,
                                bool blocked,
                                uint32_t tsUs)
{
  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, 4);
  cborEncodeText(&map, "t", "state");
  cborEncodeInt(&map, "s", sensorIndex);
  cborEncodeText(&map, "st", blocked ? "blk" : "clr");
  cborEncodeUint(&map, "ts", tsUs);
  endCborMap(&encoder, &map);
}

static void emitCborTransitStart(uint8_t sensorIndex, uint32_t tsUs)
{
  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, 3);
  cborEncodeText(&map, "t", "start");
  cborEncodeInt(&map, "s", sensorIndex);
  cborEncodeUint(&map, "ts", tsUs);
  endCborMap(&encoder, &map);
}

static void emitCborTransitSecond(uint8_t sensorIndex,
                                  uint32_t deltaUs,
                                  uint32_t tsUs)
{
  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, 4);
  cborEncodeText(&map, "t", "finish");
  cborEncodeInt(&map, "s", sensorIndex);
  cborEncodeUint(&map, "dt", deltaUs);
  cborEncodeUint(&map, "ts", tsUs);
  endCborMap(&encoder, &map);
}

static void emitCborTransitRejected(uint32_t deltaUs, uint32_t minUs)
{
  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, 3);
  cborEncodeText(&map, "t", "reject");
  cborEncodeUint(&map, "dt", deltaUs);
  cborEncodeUint(&map, "min", minUs);
  endCborMap(&encoder, &map);
}

static void emitCborTransitResult(uint8_t startSensor,
                                  uint8_t endSensor,
                                  uint32_t deltaUs,
                                  float scaleMph,
                                  float mphErr,
                                  bool inRange)
{
  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, 7);
  cborEncodeText(&map, "t", "transit");
  cborEncodeInt(&map, "f", startSensor);
  cborEncodeInt(&map, "to", endSensor);
  cborEncodeUint(&map, "dt", deltaUs);
  cborEncodeFloat(&map, "mph", scaleMph);
  cborEncodeFloat(&map, "err", mphErr);
  cborEncodeBool(&map, "ok", inRange);
  endCborMap(&encoder, &map);
}

static void emitCborTransitTimeout(uint32_t elapsedSec)
{
  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, 2);
  cborEncodeText(&map, "t", "timeout");
  cborEncodeUint(&map, "el", elapsedSec);
  endCborMap(&encoder, &map);
}

static void emitCborTransitWait(uint32_t elapsedSec,
                                uint32_t timeoutSec,
                                uint8_t startSensor)
{
  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, 4);
  cborEncodeText(&map, "t", "wait");
  cborEncodeUint(&map, "el", elapsedSec);
  cborEncodeUint(&map, "to", timeoutSec);
  cborEncodeInt(&map, "f", startSensor);
  endCborMap(&encoder, &map);
}

static void emitCborApiVersion()
{
  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, 2);
  cborEncodeText(&map, "t", "version");
  cborEncodeText(&map, "v", kApiVersion);
  endCborMap(&encoder, &map);
}

static void emitCborConfig()
{
  CborEncoder encoder;
  CborEncoder map;
  beginCborMap(&encoder, &map, 4);
  cborEncodeText(&map, "t", "cfg");
  cborEncodeUint(&map, "to", transitTimeoutUs / 1000000UL);
  cborEncodeUint(&map, "d", sensorSpacingMm);
  cborEncodeUint(&map, "r", objectMaxRangeMm);
  endCborMap(&encoder, &map);
}

struct CborCommand {
  enum Type : uint8_t {
    kNone = 0,
    kGetVersion,
    kGetConfig,
    kSetConfig,
  };
  Type type = kNone;
  bool hasTimeout = false;
  uint32_t timeoutSec = 0;
  bool hasDistance = false;
  uint32_t distanceMm = 0;
  bool hasRange = false;
  uint32_t rangeMm = 0;
};

enum CborParseStatus : uint8_t {
  kParseOk = 0,
  kParseNeedMore,
  kParseInvalid,
};

static CborParseStatus parseCborHeader(const uint8_t *buf,
                                       size_t len,
                                       size_t *pos,
                                       uint8_t *major,
                                       uint64_t *value)
{
  size_t p = *pos;
  if (p >= len) {
    return kParseNeedMore;
  }
  uint8_t ib = buf[p++];
  *major = static_cast<uint8_t>(ib >> 5);
  uint8_t ai = static_cast<uint8_t>(ib & 0x1f);
  if (ai < 24) {
    *value = ai;
    *pos = p;
    return kParseOk;
  }

  size_t extra = 0;
  switch (ai) {
    case 24:
      extra = 1;
      break;
    case 25:
      extra = 2;
      break;
    case 26:
      extra = 4;
      break;
    case 27:
      extra = 8;
      break;
    default:
      return kParseInvalid;
  }
  if (p + extra > len) {
    return kParseNeedMore;
  }
  uint64_t val = 0;
  for (size_t i = 0; i < extra; ++i) {
    val = (val << 8) | buf[p + i];
  }
  p += extra;
  *value = val;
  *pos = p;
  return kParseOk;
}

static CborParseStatus skipCborItem(const uint8_t *buf,
                                    size_t len,
                                    size_t *pos,
                                    uint8_t depth)
{
  if (depth > 8) {
    return kParseInvalid;
  }
  uint8_t major = 0;
  uint64_t value = 0;
  CborParseStatus status =
      parseCborHeader(buf, len, pos, &major, &value);
  if (status != kParseOk) {
    return status;
  }

  switch (major) {
    case 0:
    case 1:
    case 7:
      return kParseOk;
    case 2:
    case 3:
      if (*pos + value > len) {
        return kParseNeedMore;
      }
      *pos += static_cast<size_t>(value);
      return kParseOk;
    case 4:
      for (uint64_t i = 0; i < value; ++i) {
        status = skipCborItem(buf, len, pos, depth + 1);
        if (status != kParseOk) {
          return status;
        }
      }
      return kParseOk;
    case 5:
      for (uint64_t i = 0; i < value; ++i) {
        status = skipCborItem(buf, len, pos, depth + 1);
        if (status != kParseOk) {
          return status;
        }
        status = skipCborItem(buf, len, pos, depth + 1);
        if (status != kParseOk) {
          return status;
        }
      }
      return kParseOk;
    case 6:
      return skipCborItem(buf, len, pos, depth + 1);
    default:
      return kParseInvalid;
  }
}

static CborParseStatus parseCborText(const uint8_t *buf,
                                     size_t len,
                                     size_t *pos,
                                     char *out,
                                     size_t outSize)
{
  uint8_t major = 0;
  uint64_t value = 0;
  CborParseStatus status =
      parseCborHeader(buf, len, pos, &major, &value);
  if (status != kParseOk) {
    return status;
  }
  if (major != 3) {
    return kParseInvalid;
  }
  if (*pos + value > len) {
    return kParseNeedMore;
  }
  size_t copyLen = value < (outSize - 1) ? static_cast<size_t>(value)
                                         : (outSize - 1);
  memcpy(out, buf + *pos, copyLen);
  out[copyLen] = '\0';
  *pos += static_cast<size_t>(value);
  return kParseOk;
}

static CborParseStatus parseCborUint(const uint8_t *buf,
                                     size_t len,
                                     size_t *pos,
                                     uint64_t *out)
{
  uint8_t major = 0;
  uint64_t value = 0;
  CborParseStatus status =
      parseCborHeader(buf, len, pos, &major, &value);
  if (status != kParseOk) {
    return status;
  }
  if (major != 0) {
    return kParseInvalid;
  }
  *out = value;
  return kParseOk;
}

static CborParseStatus parseCborCommand(const uint8_t *buf,
                                        size_t len,
                                        size_t *used,
                                        CborCommand *cmd)
{
  *used = 0;
  *cmd = CborCommand{};
  size_t pos = 0;
  uint8_t major = 0;
  uint64_t pairs = 0;
  CborParseStatus status =
      parseCborHeader(buf, len, &pos, &major, &pairs);
  if (status != kParseOk) {
    return status;
  }
  if (major != 5) {
    pos = 0;
    status = skipCborItem(buf, len, &pos, 0);
    if (status == kParseOk) {
      *used = pos;
    }
    return status;
  }

  for (uint64_t i = 0; i < pairs; ++i) {
    char key[4] = {};
    status = parseCborText(buf, len, &pos, key, sizeof(key));
    if (status != kParseOk) {
      return status;
    }

    if (strcmp(key, "t") == 0) {
      char value[8] = {};
      status = parseCborText(buf, len, &pos, value, sizeof(value));
      if (status != kParseOk) {
        return status;
      }
      if (strcmp(value, "getv") == 0) {
        cmd->type = CborCommand::kGetVersion;
      } else if (strcmp(value, "getc") == 0) {
        cmd->type = CborCommand::kGetConfig;
      } else if (strcmp(value, "cfg") == 0) {
        cmd->type = CborCommand::kSetConfig;
      }
    } else if (strcmp(key, "to") == 0) {
      uint64_t value = 0;
      status = parseCborUint(buf, len, &pos, &value);
      if (status != kParseOk) {
        return status;
      }
      cmd->hasTimeout = true;
      cmd->timeoutSec = value > UINT32_MAX ? UINT32_MAX
                                           : static_cast<uint32_t>(value);
    } else if (strcmp(key, "d") == 0) {
      uint64_t value = 0;
      status = parseCborUint(buf, len, &pos, &value);
      if (status != kParseOk) {
        return status;
      }
      cmd->hasDistance = true;
      cmd->distanceMm = value > UINT16_MAX ? UINT16_MAX
                                           : static_cast<uint32_t>(value);
    } else if (strcmp(key, "r") == 0) {
      uint64_t value = 0;
      status = parseCborUint(buf, len, &pos, &value);
      if (status != kParseOk) {
        return status;
      }
      cmd->hasRange = true;
      cmd->rangeMm = value > UINT16_MAX ? UINT16_MAX
                                        : static_cast<uint32_t>(value);
    } else {
      status = skipCborItem(buf, len, &pos, 0);
      if (status != kParseOk) {
        return status;
      }
    }
  }

  *used = pos;
  return kParseOk;
}

static uint32_t computeMinTransitUs(uint16_t spacingMm)
{
  if (kMaxSpeedMps <= 0.0f) {
    return 0;
  }
  float spacingMeters = static_cast<float>(spacingMm) / kMmPerMeter;
  return static_cast<uint32_t>((spacingMeters / kMaxSpeedMps) * kUsPerSecond +
                               0.5f);
}

static void applyConfigCommand(const CborCommand &cmd)
{
  if (cmd.type == CborCommand::kGetVersion) {
    emitCborApiVersion();
    return;
  }

  if (cmd.type == CborCommand::kGetConfig) {
    emitCborConfig();
    return;
  }

  if (cmd.type != CborCommand::kSetConfig) {
    return;
  }

  if (cmd.hasTimeout && cmd.timeoutSec > 0) {
    uint32_t maxSec = UINT32_MAX / 1000000UL;
    uint32_t clamped = cmd.timeoutSec > maxSec ? maxSec : cmd.timeoutSec;
    transitTimeoutUs = clamped * 1000000UL;
  }

  if (cmd.hasDistance && cmd.distanceMm > 0) {
    sensorSpacingMm = static_cast<uint16_t>(cmd.distanceMm);
    minTransitUs = computeMinTransitUs(sensorSpacingMm);
  }

  if (cmd.hasRange && cmd.rangeMm > 0) {
    objectMaxRangeMm = static_cast<uint16_t>(cmd.rangeMm);
  }
}

static void pollCborCommands()
{
  while (Serial.available() > 0) {
    int byteRead = Serial.read();
    if (byteRead < 0) {
      break;
    }
    if (cborRxLen < kCborRxBufferSize) {
      cborRxBuffer[cborRxLen++] = static_cast<uint8_t>(byteRead);
    } else {
      cborRxLen = 0;
      emitCborLog("w", "rx_overflow", -1, -1, -1, micros());
      break;
    }
  }

  while (cborRxLen > 0) {
    CborCommand cmd;
    size_t used = 0;
    CborParseStatus status =
        parseCborCommand(cborRxBuffer, cborRxLen, &used, &cmd);
    if (status == kParseNeedMore) {
      break;
    }
    if (status != kParseOk || used == 0 || used > cborRxLen) {
      memmove(cborRxBuffer, cborRxBuffer + 1, cborRxLen - 1);
      cborRxLen -= 1;
      continue;
    }
    if (used < cborRxLen) {
      memmove(cborRxBuffer, cborRxBuffer + used, cborRxLen - used);
    }
    cborRxLen -= used;
    applyConfigCommand(cmd);
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
                               uint32_t deltaUs)
{
  if (deltaUs == 0) {
    return;
  }
  uint32_t avgStartUs = 0;
  uint32_t avgEndUs = 0;
  bool haveStartStats =
      getMeasurementAvgMaxUs(startSensor, &avgStartUs, nullptr);
  bool haveEndStats =
      getMeasurementAvgMaxUs(endSensor, &avgEndUs, nullptr);
  uint32_t errUsTypical = 0;
  if (haveStartStats && haveEndStats) {
    errUsTypical = (avgStartUs + avgEndUs) / 2;
  }

  float deltaSeconds = static_cast<float>(deltaUs) / kUsPerSecond;
  float distanceMeters = static_cast<float>(sensorSpacingMm) / kMmPerMeter;
  float speedMps = distanceMeters / deltaSeconds;
  float scaleMph = speedMps * kMpsToMph * kHoScaleRatio;
  float speedErrMphTypical = 0.0f;
  if (errUsTypical > 0 && deltaUs > errUsTypical) {
    float ratio =
        static_cast<float>(errUsTypical) / static_cast<float>(deltaUs);
    speedErrMphTypical = scaleMph * ratio;
  }
  bool inRange = scaleMph >= kMinScaleMph && scaleMph <= kMaxScaleMph;
  emitCborTransitResult(startSensor, endSensor, deltaUs, scaleMph,
                        speedErrMphTypical, inRange);
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000)) {
    delay(10);
  }

  emitCborLog("i", "init", -1, -1, -1, micros());
  emitCborLog("i", "wiring: SDA=D14, SCL=D15, XSHUT=A1, VIN=3V3, GND=GND",
              -1, -1, -1, micros());
  emitCborLog("i", "qwiic_mux: address 0x70, sensors on ports 0 and 1", -1,
              -1, -1, micros());

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();
  Wire.setClock(kI2cClockHz);

  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (!selectMuxChannel(kMuxPorts[i])) {
      emitCborLog("e", "mux_select_failed", -1, -1, kMuxPorts[i],
                  micros());
      while (true) {
        delay(100);
      }
    }

    VL53L4CX &sensor = *kSensors[i];
    sensor.begin();
    sensor.VL53L4CX_Off();

    VL53L4CX_Error status = sensor.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
    if (status != VL53L4CX_ERROR_NONE) {
      emitCborLog("e", "init_sensor_failed", i, status, -1, micros());
      while (true) {
        delay(100);
      }
    }

    emitSensorInfo(sensor, i);

    status = sensor.VL53L4CX_StopMeasurement();
    if (status != VL53L4CX_ERROR_NONE) {
      emitCborLog("w", "stop_measurement_failed", i, status, -1, micros());
    }

    status = sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(
        kTimingBudgetUs);
    if (status != VL53L4CX_ERROR_NONE) {
      emitCborLog("w", "timing_budget_failed", i, status, -1, micros());
    }

    status = sensor.VL53L4CX_StartMeasurement();
    if (status != VL53L4CX_ERROR_NONE) {
      emitCborLog("e", "start_measurement_failed", i, status, -1, micros());
      while (true) {
        delay(100);
      }
    }

    measurementStartUs[i] = micros();
    resetTimingStats(measurementStats[i]);
    resetTimingStats(transferStats[i]);
  }

  lastStatsReportUs = micros();
  lastApiVersionReportUs = lastStatsReportUs;
}

void loop()
{
  pollCborCommands();
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (!selectMuxChannel(kMuxPorts[i])) {
      emitCborLog("e", "mux_select_failed", -1, -1, kMuxPorts[i],
                  micros());
      delay(100);
      continue;
    }

    VL53L4CX &sensor = *kSensors[i];
    uint8_t dataReady = 0;
    VL53L4CX_Error status = sensor.VL53L4CX_GetMeasurementDataReady(&dataReady);
    if (status != VL53L4CX_ERROR_NONE) {
      emitCborLog("e", "data_ready_failed", i, status, -1, micros());
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
            findNearestObjectWithin(data, objectMaxRangeMm, nullptr);
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
            emitCborSensorState(i, debouncedPresent, sampleUs);
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
                  emitCborTransitStart(i, sampleUs);
                }
              }
            }
          }
          if (waitingForSecondSensor && i != firstSensorIndex) {
            uint32_t deltaUs = sampleUs - firstBlockUs;
            if (deltaUs >= minTransitUs) {
              if (kDebugTransitState) {
                emitCborTransitSecond(i, deltaUs, sampleUs);
              }
              reportTransitSpeed(firstSensorIndex, i, deltaUs);
            } else if (kDebugTransitState) {
              emitCborTransitRejected(deltaUs, minTransitUs);
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
        emitCborLog("e", "get_multi_ranging_failed", i, status, -1,
                    micros());
      }

      status = sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
      if (status != VL53L4CX_ERROR_NONE) {
        emitCborLog("e", "clear_interrupt_failed", i, status, -1, micros());
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
      uint32_t totalSec = transitTimeoutUs / kWaitLogIntervalUs;
      emitCborTransitWait(elapsedSec, totalSec, firstSensorIndex);
      lastWaitLogUs = nowUs;
    }
  }
  if (waitingForSecondSensor &&
      (uint32_t)(nowUs - firstBlockUs) >= transitTimeoutUs) {
    waitingForSecondSensor = false;
    lastWaitLogUs = 0;
    if (kDebugTransitState) {
      uint32_t elapsedSec = (nowUs - firstBlockUs) / 1000000UL;
      emitCborTransitTimeout(elapsedSec);
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
  if ((uint32_t)(nowUs - lastApiVersionReportUs) >= kApiVersionIntervalUs) {
    emitCborApiVersion();
    lastApiVersionReportUs = nowUs;
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
