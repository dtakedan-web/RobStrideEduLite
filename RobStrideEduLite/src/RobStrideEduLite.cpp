/*
 * RobStrideEduLite.cpp
 * RobStride EduLite 05 用 Arduino ライブラリ実装 (ESP32 TWAI + SN65HVD230)
 */

#include "RobStrideEduLite.h"

// 共有バス状態の静的メンバ定義
bool    RobStrideEduLite::s_busInit       = false;
int8_t  RobStrideEduLite::s_rxPin         = -1;
int8_t  RobStrideEduLite::s_txPin         = -1;
uint8_t RobStrideEduLite::s_instanceCount = 0;

RobStrideEduLite::RobStrideEduLite(uint8_t motorId, uint8_t hostId)
  : _motorId(motorId), _hostId(hostId) {}

float RobStrideEduLite::constrainFloat(float x, float lo, float hi) {
  if (x > hi) return hi;
  if (x < lo) return lo;
  return x;
}

uint16_t RobStrideEduLite::floatToUint(float x, float xMin, float xMax) {
  x = constrainFloat(x, xMin, xMax);
  float span = xMax - xMin;
  return (uint16_t)((x - xMin) * 65535.0f / span);
}

float RobStrideEduLite::uintToFloat(uint16_t x, float xMin, float xMax) {
  float span = xMax - xMin;
  return ((float)x) * span / 65535.0f + xMin;
}

// 29bit 拡張ID を組み立て: bit28~24=通信タイプ, bit23~8=データエリア, bit7~0=対象ID
uint32_t RobStrideEduLite::makeId(uint8_t type, uint16_t dataArea) const {
  return ((uint32_t)(type & 0x1F) << 24) | ((uint32_t)dataArea << 8) | _motorId;
}

bool RobStrideEduLite::sendFrame(uint8_t type, uint16_t dataArea,
                                 const uint8_t *payload, uint8_t len) {
  return sendFrameTo(type, dataArea, _motorId, payload, len);
}

bool RobStrideEduLite::sendFrameTo(uint8_t type, uint16_t dataArea, uint8_t targetId,
                                   const uint8_t *payload, uint8_t len) {
  if (!_begun) return false;
  twai_message_t msg;
  memset(&msg, 0, sizeof(msg));
  msg.identifier = ((uint32_t)(type & 0x1F) << 24) |
                   ((uint32_t)dataArea << 8) | targetId;
  msg.extd             = 1;              // 拡張フレーム (29bit)
  msg.rtr              = 0;
  msg.data_length_code = len;
  if (payload && len > 0) memcpy(msg.data, payload, len);
  // 送信キューが詰まっていても 10ms 待つ
  return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

// データフィールド全0の空コマンド (有効化/停止など)
bool RobStrideEduLite::sendEmpty(uint8_t type) {
  uint8_t zeros[8] = {0};
  return sendFrame(type, _hostId, zeros, 8);
}

bool RobStrideEduLite::busInitialized() { return s_busInit; }

bool RobStrideEduLite::begin(int8_t rxPin, int8_t txPin) {
  s_instanceCount++;

  // 既に初期化済みなら共有バスを使うだけ (複数モーター対応)
  if (s_busInit) {
    if (rxPin != s_rxPin || txPin != s_txPin) {
      // 既存と異なるピン指定は警告として失敗扱い (実害はないが混乱防止)
      // 既存バスをそのまま使う
    }
    _begun = true;
    return true;
  }

  // 1Mbps / ノーマルモード / 全ID受信
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)txPin, (gpio_num_t)rxPin, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 32;          // 受信取りこぼし対策でキュー拡大 (既定5)
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    s_instanceCount--;
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    s_instanceCount--;
    return false;
  }
  // 診断用アラートを有効化
  twai_reconfigure_alerts(TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR |
                          TWAI_ALERT_BUS_OFF | TWAI_ALERT_ARB_LOST |
                          TWAI_ALERT_RX_QUEUE_FULL, nullptr);
  s_rxPin   = rxPin;
  s_txPin   = txPin;
  s_busInit = true;
  _begun    = true;
  return true;
}

void RobStrideEduLite::end() {
  if (!_begun) return;
  if (s_instanceCount > 0) s_instanceCount--;
  // 最後のインスタンスが終わる時だけバスを停止
  if (s_instanceCount == 0 && s_busInit) {
    twai_stop();
    twai_driver_uninstall();
    s_busInit = false;
    s_rxPin = s_txPin = -1;
  }
  _begun = false;
}

// ------------------------------------------------------------------
// 基本コマンド
// ------------------------------------------------------------------
bool RobStrideEduLite::enable() {
  return sendEmpty(EL05_TYPE_ENABLE);
}

bool RobStrideEduLite::disable(bool clearFault) {
  uint8_t data[8] = {0};
  data[0] = clearFault ? 1 : 0;   // Byte0=1 で故障クリア
  return sendFrame(EL05_TYPE_DISABLE, _hostId, data, 8);
}

bool RobStrideEduLite::setZeroPosition() {
  uint8_t data[8] = {0};
  data[0] = 1;                    // Byte[0]=1 が規定
  return sendFrame(EL05_TYPE_SET_ZERO, _hostId, data, 8);
}

bool RobStrideEduLite::saveParameters() {
  uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  return sendFrame(EL05_TYPE_SAVE, _hostId, data, 8);
}

bool RobStrideEduLite::setCanId(uint8_t newId) {
  // データエリア bit16~23 に新しい CAN_ID、bit15~8 にホストID
  uint16_t dataArea = ((uint16_t)newId << 8) | _hostId;
  // makeId は dataArea を bit8~23 に置く → bit16~23 = 上位バイト = newId
  bool ok = sendFrame(EL05_TYPE_SET_CAN_ID, dataArea, nullptr, 0);
  if (ok) _motorId = newId;
  return ok;
}

// ------------------------------------------------------------------
// 運転制御 (MIT) モード: 通信タイプ1
// トルクは ID のデータエリア (bit23~8) に 16bit で格納
// ------------------------------------------------------------------
bool RobStrideEduLite::setMIT(float pos, float vel, float kp, float kd, float tff) {
  uint16_t p  = floatToUint(pos, EL05_P_MIN,  EL05_P_MAX);
  uint16_t v  = floatToUint(vel, EL05_V_MIN,  EL05_V_MAX);
  uint16_t kP = floatToUint(kp,  EL05_KP_MIN, EL05_KP_MAX);
  uint16_t kD = floatToUint(kd,  EL05_KD_MIN, EL05_KD_MAX);
  uint16_t t  = floatToUint(tff, EL05_T_MIN,  EL05_T_MAX);

  uint8_t data[8];
  data[0] = (uint8_t)(p  >> 8);  data[1] = (uint8_t)(p  & 0xFF);
  data[2] = (uint8_t)(v  >> 8);  data[3] = (uint8_t)(v  & 0xFF);
  data[4] = (uint8_t)(kP >> 8);  data[5] = (uint8_t)(kP & 0xFF);
  data[6] = (uint8_t)(kD >> 8);  data[7] = (uint8_t)(kD & 0xFF);

  return sendFrame(EL05_TYPE_MIT_CONTROL, t, data, 8);
}

// 角度を degree、速度を rpm で指定する MIT 版
bool RobStrideEduLite::setMIT_deg(float posDeg, float velRpm, float kp, float kd, float tff) {
  return setMIT(posDeg * EL05_DEG2RAD, velRpm * EL05_RPM2RPS, kp, kd, tff);
}

// ------------------------------------------------------------------
// パラメータ書き込み: 通信タイプ18 (揮発性。保存には通信タイプ22)
// Byte0~1: index (リトルエンディアン), Byte2~3: 0, Byte4~7: 値 (リトルエンディアン)
// ------------------------------------------------------------------
bool RobStrideEduLite::writeParamRaw(uint16_t index, const uint8_t *value4) {
  uint8_t data[8] = {0};
  data[0] = (uint8_t)(index & 0xFF);
  data[1] = (uint8_t)(index >> 8);
  memcpy(&data[4], value4, 4);
  return sendFrame(EL05_TYPE_PARAM_WRITE, _hostId, data, 8);
}

bool RobStrideEduLite::writeParamFloat(uint16_t index, float value) {
  uint8_t b[4];
  memcpy(b, &value, 4);
  return writeParamRaw(index, b);
}

bool RobStrideEduLite::writeParamU8(uint16_t index, uint8_t value) {
  uint8_t b[4] = {value, 0, 0, 0};
  return writeParamRaw(index, b);
}

bool RobStrideEduLite::writeParamU16(uint16_t index, uint16_t value) {
  uint8_t b[4];
  memcpy(b, &value, 2);
  return writeParamRaw(index, b);
}

bool RobStrideEduLite::writeParamU32(uint16_t index, uint32_t value) {
  uint8_t b[4];
  memcpy(b, &value, 4);
  return writeParamRaw(index, b);
}

// パラメータ読み出し: 通信タイプ17 → 応答はタイプ17 で Byte4~7 に値
bool RobStrideEduLite::readParam(uint16_t index, uint8_t *out4, uint32_t timeoutMs) {
  uint8_t data[8] = {0};
  data[0] = (uint8_t)(index & 0xFF);
  data[1] = (uint8_t)(index >> 8);
  if (!sendFrame(EL05_TYPE_PARAM_READ, _hostId, data, 8)) return false;

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) == ESP_OK && rx.extd) {
      uint8_t type = (uint8_t)((rx.identifier >> 24) & 0x1F);
      uint8_t from = (uint8_t)((rx.identifier >> 8) & 0xFF);
      if (type == EL05_TYPE_PARAM_READ && from == _motorId &&
          rx.data_length_code >= 8 &&
          rx.data[0] == (uint8_t)(index & 0xFF) &&
          rx.data[1] == (uint8_t)(index >> 8)) {
        memcpy(out4, &rx.data[4], 4);
        return true;
      }
    }
  }
  return false;
}

bool RobStrideEduLite::readParamFloat(uint16_t index, float &value, uint32_t timeoutMs) {
  uint8_t b[4];
  if (!readParam(index, b, timeoutMs)) return false;
  memcpy(&value, b, 4);
  return true;
}

// ------------------------------------------------------------------
// モード切替 / 各モード指令 (内部は全てパラメータ書き込み)
// ------------------------------------------------------------------
bool RobStrideEduLite::setRunMode(EL05RunMode mode) {
  return writeParamU8(EL05_IDX_RUN_MODE, (uint8_t)mode);
}

bool RobStrideEduLite::setCurrent(float iqA) {
  return writeParamFloat(EL05_IDX_IQ_REF, constrainFloat(iqA, -EL05_IQ_MAX, EL05_IQ_MAX));
}

bool RobStrideEduLite::setVelocity(float radS) {
  return writeParamFloat(EL05_IDX_SPD_REF, constrainFloat(radS, EL05_V_MIN, EL05_V_MAX));
}

bool RobStrideEduLite::setVelocity_rpm(float rpm) {
  return setVelocity(rpm * EL05_RPM2RPS);
}

bool RobStrideEduLite::setPositionCSP(float rad) {
  return writeParamFloat(EL05_IDX_LOC_REF, constrainFloat(rad, EL05_P_MIN, EL05_P_MAX));
}

bool RobStrideEduLite::setPositionCSP_deg(float deg) {
  return setPositionCSP(deg * EL05_DEG2RAD);
}

bool RobStrideEduLite::setPositionPP(float rad) {
  return writeParamFloat(EL05_IDX_LOC_REF, constrainFloat(rad, EL05_P_MIN, EL05_P_MAX));
}

bool RobStrideEduLite::setPositionPP_deg(float deg) {
  return setPositionPP(deg * EL05_DEG2RAD);
}

bool RobStrideEduLite::setVelocityAccel(float radS2) {
  return writeParamFloat(EL05_IDX_ACC_RAD, radS2);
}

bool RobStrideEduLite::setPPProfile(float velMax, float acc) {
  if (!writeParamFloat(EL05_IDX_VEL_MAX, velMax)) return false;
  return writeParamFloat(EL05_IDX_ACC_SET, acc);
}

bool RobStrideEduLite::setPPProfile_rpm(float velMaxRpm, float accRpmS) {
  // 速度は rpm→rad/s、加速度は rpm/s→rad/s^2 (同じ換算係数)
  return setPPProfile(velMaxRpm * EL05_RPM2RPS, accRpmS * EL05_RPM2RPS);
}

bool RobStrideEduLite::setCSPSpeedLimit(float radS) {
  return writeParamFloat(EL05_IDX_LIMIT_SPD, constrainFloat(radS, 0.0f, EL05_V_MAX));
}

bool RobStrideEduLite::setCSPSpeedLimit_rpm(float rpm) {
  return setCSPSpeedLimit(rpm * EL05_RPM2RPS);
}

bool RobStrideEduLite::setCurrentLimit(float amp) {
  return writeParamFloat(EL05_IDX_LIMIT_CUR, constrainFloat(amp, 0.0f, EL05_IQ_MAX));
}

// ------------------------------------------------------------------
// 能動レポート: 通信タイプ24 (F_CMD: 00=無効, 01=有効 既定10ms周期)
// ------------------------------------------------------------------
bool RobStrideEduLite::setActiveReport(bool on) {
  uint8_t data[8] = {1, 2, 3, 4, 5, 6, (uint8_t)(on ? 1 : 0), 0};
  return sendFrame(EL05_TYPE_ACTIVE_REPORT, _hostId, data, 8);
}

// ------------------------------------------------------------------
// ミドル関数
// ------------------------------------------------------------------
bool RobStrideEduLite::requestFeedback(EL05Feedback &fb, uint32_t timeoutMs) {
  // 中立指令 (kp=kd=tff=0) で応答を誘発。速度/位置/電流モード中でも無害。
  if (!setMIT(0, 0, 0, 0, 0)) return false;
  fb = readFeedback(timeoutMs);
  return fb.valid;
}

bool RobStrideEduLite::getPosition(float &outRad, uint32_t timeoutMs) {
  EL05Feedback fb;
  if (!requestFeedback(fb, timeoutMs)) return false;
  outRad = fb.positionRad;
  return true;
}

bool RobStrideEduLite::getVelocity(float &outRadS, uint32_t timeoutMs) {
  EL05Feedback fb;
  if (!requestFeedback(fb, timeoutMs)) return false;
  outRadS = fb.velocityRps;
  return true;
}

bool RobStrideEduLite::getPosition_deg(float &outDeg, uint32_t timeoutMs) {
  float rad;
  if (!getPosition(rad, timeoutMs)) return false;
  outDeg = rad * EL05_RAD2DEG;
  return true;
}

bool RobStrideEduLite::getVelocity_rpm(float &outRpm, uint32_t timeoutMs) {
  float radS;
  if (!getVelocity(radS, timeoutMs)) return false;
  outRpm = radS * EL05_RPS2RPM;
  return true;
}

bool RobStrideEduLite::getTorque(float &outNm, uint32_t timeoutMs) {
  EL05Feedback fb;
  if (!requestFeedback(fb, timeoutMs)) return false;
  outNm = fb.torqueNm;
  return true;
}

bool RobStrideEduLite::getTemperature(float &outC, uint32_t timeoutMs) {
  EL05Feedback fb;
  if (!requestFeedback(fb, timeoutMs)) return false;
  outC = fb.tempC;
  return true;
}

bool RobStrideEduLite::getBusVoltage(float &outV, uint32_t timeoutMs) {
  return readParamFloat(EL05_IDX_VBUS, outV, timeoutMs);
}

bool RobStrideEduLite::setTorqueLimit(float nm) {
  return writeParamFloat(EL05_IDX_LIMIT_TORQUE, constrainFloat(nm, 0.0f, EL05_T_MAX));
}

bool RobStrideEduLite::clearFault() {
  return disable(true);
}

// ------------------------------------------------------------------
// バージョン番号読み出し (タイプ4 + Byte0=0x00, Byte1=0xC4)
// 応答: タイプ2 で Byte3~6 にバージョン番号 (上位から)
// ------------------------------------------------------------------
bool RobStrideEduLite::readVersion(char *version, size_t len, uint32_t timeoutMs) {
  if (!_begun || !version || len < 2) return false;
  uint8_t data[8] = {0};
  data[0] = 0x00;
  data[1] = 0xC4;
  if (!sendFrame(EL05_TYPE_DISABLE, _hostId, data, 8)) return false;

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) != ESP_OK || !rx.extd) continue;
    if (rx.data_length_code < 8) continue;
    uint8_t type = (uint8_t)((rx.identifier >> 24) & 0x1F);
    uint8_t from = (uint8_t)((rx.identifier >> 8) & 0xFF);
    if (type != EL05_TYPE_FEEDBACK || from != _motorId) continue;
    if (rx.data[0] != 0x00 || rx.data[1] != 0xC4 || rx.data[2] != 0x56) continue;
    // Byte3~6 がバージョン番号 (上位から)
    snprintf(version, len, "%u.%u.%u.%u", rx.data[3], rx.data[4], rx.data[5], rx.data[6]);
    return true;
  }
  return false;
}

// ------------------------------------------------------------------
// 故障フィードバック (通信タイプ21 = 0x15)
// ID: bit28~24=0x15, bit15~8=モーターCAN_ID, bit7~0=ホストCAN_ID
// Data: Byte0~3=故障値, Byte4~7=警告値 (ともにリトルエンディアン想定)
// ------------------------------------------------------------------
bool RobStrideEduLite::readFault(EL05Fault &out, uint32_t timeoutMs) {
  out = EL05Fault();
  if (!_begun) return false;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) != ESP_OK || !rx.extd) continue;
    if (rx.data_length_code < 8) continue;
    uint8_t type = (uint8_t)((rx.identifier >> 24) & 0x1F);
    if (type != EL05_TYPE_FAULT_FEEDBACK) continue;
    uint8_t from = (uint8_t)((rx.identifier >> 8) & 0xFF);
    if (from != _motorId) continue;
    out.valid      = true;
    out.motorId    = from;
    memcpy(&out.faultValue, &rx.data[0], 4);
    memcpy(&out.warnValue,  &rx.data[4], 4);
    out.hasFault   = (out.faultValue != 0);
    out.hasWarning = (out.warnValue  != 0);
    return true;
  }
  return false;
}

// ------------------------------------------------------------------
// フィードバック受信 (通信タイプ2 / 24 の応答)
// ID: bit28~24=タイプ, bit8~15=モーターID, bit16~21=故障, bit22~23=モード状態
// Data: Byte0~1 位置, Byte2~3 速度, Byte4~5 トルク, Byte6~7 温度(℃×10) ※ビッグエンディアン
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// 診断
// ------------------------------------------------------------------
uint8_t RobStrideEduLite::scanBus(uint8_t *foundIds, uint8_t maxFound,
                                  uint16_t perIdTimeoutMs) {
  uint8_t count = 0;
  if (!_begun) return 0;
  uint8_t zeros[8] = {0};

  // 2 周リトライ (物理層の不具合対策として多周していた名残。現在は安定しているため最小限に)
  for (uint8_t round = 0; round < 2 && count < maxFound; round++) {
    for (uint16_t id = 1; id <= 127 && count < maxFound; id++) {
      // 前 ID の応答残りを捨ててキューを空にする
      twai_message_t dump;
      while (twai_receive(&dump, 0) == ESP_OK) {}

      if (!sendFrameTo(EL05_TYPE_GET_DEVICE_ID, _hostId, (uint8_t)id, zeros, 8)) {
        continue;  // 送信失敗 (ACKなし等)
      }
      uint32_t start = millis();
      while (millis() - start < perIdTimeoutMs) {
        twai_message_t rx;
        if (twai_receive(&rx, pdMS_TO_TICKS(2)) != ESP_OK || !rx.extd) continue;
        uint8_t type = (uint8_t)((rx.identifier >> 24) & 0x1F);
        uint8_t dest = (uint8_t)(rx.identifier & 0xFF);
        if (type == EL05_TYPE_GET_DEVICE_ID && dest == 0xFE) {
          uint8_t from = (uint8_t)((rx.identifier >> 8) & 0xFF);
          bool dup = false;
          for (uint8_t i = 0; i < count; i++) if (foundIds[i] == from) { dup = true; break; }
          if (!dup) foundIds[count++] = from;
          break;
        }
      }
    }
  }
  return count;
}

bool RobStrideEduLite::getStatus(twai_status_info_t &out) {
  if (!_begun) return false;
  return twai_get_status_info(&out) == ESP_OK;
}

uint32_t RobStrideEduLite::getAlerts(uint32_t timeoutMs) {
  uint32_t alerts = 0;
  if (!_begun) return 0;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(timeoutMs));
  return alerts;
}

EL05Feedback RobStrideEduLite::readFeedback(uint32_t timeoutMs) {
  EL05Feedback fb;
  if (!_begun) return fb;

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) != ESP_OK) continue;
    if (!rx.extd || rx.data_length_code < 8) continue;

    uint8_t type = (uint8_t)((rx.identifier >> 24) & 0x1F);
    if (type != EL05_TYPE_FEEDBACK && type != EL05_TYPE_ACTIVE_REPORT) continue;

    uint8_t  from  = (uint8_t)((rx.identifier >> 8) & 0xFF);
    if (from != _motorId) continue;

    uint8_t faults = (uint8_t)((rx.identifier >> 16) & 0x3F);
    uint8_t mode   = (uint8_t)((rx.identifier >> 22) & 0x03);

    uint16_t pRaw = ((uint16_t)rx.data[0] << 8) | rx.data[1];
    uint16_t vRaw = ((uint16_t)rx.data[2] << 8) | rx.data[3];
    uint16_t tRaw = ((uint16_t)rx.data[4] << 8) | rx.data[5];
    int16_t  temp = ((int16_t)rx.data[6] << 8) | rx.data[7];

    fb.valid       = true;
    fb.motorId     = from;
    fb.mode        = mode;
    fb.faultBits   = faults;
    fb.hasFault    = (faults != 0);
    fb.positionRad = uintToFloat(pRaw, EL05_P_MIN, EL05_P_MAX);
    fb.velocityRps = uintToFloat(vRaw, EL05_V_MIN, EL05_V_MAX);
    fb.torqueNm    = uintToFloat(tRaw, EL05_T_MIN, EL05_T_MAX);
    fb.tempC       = (float)temp / 10.0f;
    return fb;
  }
  return fb;
}
