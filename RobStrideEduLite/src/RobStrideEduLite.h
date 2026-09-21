/*
 * RobStrideEduLite.h
 *
 * RobStride EduLite 05 (EL05) QDD アクチュエータ用 Arduino ライブラリ
 * ESP32 内蔵 TWAI(CAN コントローラ)+ SN65HVD230 トランシーバー構成向け
 *
 * 対応プロトコル: RobStride プライベートプロトコル (CAN 2.0B, 1Mbps, 拡張フレーム 29bit ID)
 * 出典: RobStride EL05 User Manual (Driver protocol and instructions)
 *
 * 29bit ID 構成:
 *   bit28~24 : 通信タイプ
 *   bit23~8  : データエリア2 (通常はホスト CAN_ID を bit15~8 に格納 / MIT ではトルク)
 *   bit7~0   : 対象モーター CAN_ID
 */

#ifndef ROBSTRIDE_EDULITE_H
#define ROBSTRIDE_EDULITE_H

#include <Arduino.h>
#include <driver/twai.h>

// ---- EL05 の物理レンジ (マニュアル表記) ----
#define EL05_P_MIN   (-12.57f)   // 位置 [rad]  (-4π ~ +4π)
#define EL05_P_MAX   ( 12.57f)
#define EL05_V_MIN   (-50.0f)    // 速度 [rad/s]
#define EL05_V_MAX   ( 50.0f)
#define EL05_KP_MIN  (  0.0f)    // 位置ゲイン
#define EL05_KP_MAX  (500.0f)
#define EL05_KD_MIN  (  0.0f)    // 速度ゲイン
#define EL05_KD_MAX  (  5.0f)
#define EL05_T_MIN   ( -6.0f)    // トルク [Nm]  (EL05: ±6Nm)
#define EL05_T_MAX   (  6.0f)
#define EL05_IQ_MAX  ( 11.0f)    // 電流モード Iq 指令 [A] (-11~11A)

// ---- 通信タイプ (bit28~24) ----
enum EL05CommType : uint8_t {
  EL05_TYPE_GET_DEVICE_ID   = 0x00,  // デバイスID取得
  EL05_TYPE_MIT_CONTROL     = 0x01,  // 運転制御(MIT)モード指令
  EL05_TYPE_FEEDBACK        = 0x02,  // モーターフィードバック
  EL05_TYPE_ENABLE          = 0x03,  // モーター有効化
  EL05_TYPE_DISABLE         = 0x04,  // モーター停止 (Byte0=1 で故障クリア)
  EL05_TYPE_SET_ZERO        = 0x06,  // 機械原点設定 (Byte0=1)
  EL05_TYPE_SET_CAN_ID      = 0x07,  // CAN_ID 変更
  EL05_TYPE_PARAM_READ      = 0x11,  // 単一パラメータ読み出し (17)
  EL05_TYPE_PARAM_WRITE     = 0x12,  // 単一パラメータ書き込み (18, 揮発性)
  EL05_TYPE_FAULT_FEEDBACK  = 0x15,  // 故障フィードバック (21)
  EL05_TYPE_SAVE            = 0x16,  // データ保存 (22)
  EL05_TYPE_BAUDRATE        = 0x17,  // ボーレート変更 (23, 要再起動)
  EL05_TYPE_ACTIVE_REPORT   = 0x18,  // 能動レポート設定 (24)
  EL05_TYPE_PROTOCOL        = 0x19,  // プロトコル変更 (25, 要再起動)
};

// ---- run_mode パラメータ (index 0x7005) ----
enum EL05RunMode : uint8_t {
  EL05_MODE_MIT      = 0,  // 運転制御モード (デフォルト)
  EL05_MODE_PP       = 1,  // 位置モード (PP: 台形加減速)
  EL05_MODE_VELOCITY = 2,  // 速度モード
  EL05_MODE_CURRENT  = 3,  // 電流モード
  EL05_MODE_CSP      = 5,  // 位置モード (CSP: 周期位置)
};

// ---- パラメータインデックス (抜粋) ----
#define EL05_IDX_RUN_MODE      0x7005  // uint8  制御モード
#define EL05_IDX_IQ_REF        0x7006  // float  電流モード Iq 指令 [A]
#define EL05_IDX_SPD_REF       0x700A  // float  速度モード 速度指令 [rad/s]
#define EL05_IDX_LIMIT_TORQUE  0x700B  // float  トルク制限 [Nm] (0~6)
#define EL05_IDX_LOC_REF       0x7016  // float  位置モード 角度指令 [rad]
#define EL05_IDX_LIMIT_SPD     0x7017  // float  CSP 速度制限 [rad/s]
#define EL05_IDX_LIMIT_CUR     0x7018  // float  速度/位置モード 電流制限 [A]
#define EL05_IDX_MECH_POS      0x7019  // float  現在位置 [rad] (R)
#define EL05_IDX_MECH_VEL      0x701B  // float  現在速度 [rad/s] (R)
#define EL05_IDX_VBUS          0x701C  // float  バス電圧 [V] (R)
#define EL05_IDX_LOC_KP        0x701E  // float  位置 Kp (既定 40)
#define EL05_IDX_SPD_KP        0x701F  // float  速度 Kp (既定 6)
#define EL05_IDX_SPD_KI        0x7020  // float  速度 Ki (既定 0.02)
#define EL05_IDX_ACC_RAD       0x7022  // float  速度モード加速度 [rad/s^2]
#define EL05_IDX_VEL_MAX       0x7024  // float  PP 速度 [rad/s]
#define EL05_IDX_ACC_SET       0x7025  // float  PP 加速度 [rad/s^2]
#define EL05_IDX_EPSCAN_TIME   0x7026  // uint16 能動レポート間隔 (1=10ms, +1で+5ms)
#define EL05_IDX_CAN_TIMEOUT   0x7028  // uint32 CAN タイムアウト (20000=1s)
#define EL05_IDX_ZERO_STA      0x7029  // uint8  零点フラグ (0: 0~2π, 1: -π~π)

// ---- フィードバック構造体 ----
struct EL05Feedback {
  bool     valid       = false;  // 受信できたか
  uint8_t  motorId     = 0;      // 応答モーターの CAN_ID
  uint8_t  mode        = 0;      // 0:Reset 1:Cali 2:Run
  uint8_t  faultBits   = 0;      // bit0:低電圧 1:三相電流 2:過温度 3:磁気エンコーダ 4:ロック過負荷 5:未キャリブ
  bool     hasFault    = false;
  float    positionRad = 0.0f;   // -4π ~ 4π
  float    velocityRps = 0.0f;   // -50 ~ 50 rad/s
  float    torqueNm    = 0.0f;   // -6 ~ 6 Nm
  float    tempC       = 0.0f;   // ℃
};

// ---- 故障フィードバック構造体 (通信タイプ21 = 0x15) ----
// モーターから自発的に送られる詳細な故障・警告情報
struct EL05Fault {
  bool     valid      = false;
  uint8_t  motorId    = 0;
  uint32_t faultValue = 0;   // Byte0~3: 故障ビット (0=正常)
  uint32_t warnValue  = 0;   // Byte4~7: 警告ビット
  bool     hasFault   = false;
  bool     hasWarning = false;
  // 故障ビット内訳 (faultValue)
  bool overtemperature() const { return faultValue & (1UL << 0); }  // モーター過温度 (既定135℃)
  bool driverChip()      const { return faultValue & (1UL << 1); }  // ドライバチップ故障
  bool undervoltage()    const { return faultValue & (1UL << 2); }  // 低電圧
  bool overvoltage()     const { return faultValue & (1UL << 3); }  // 過電圧
  bool overcurrentB()    const { return faultValue & (1UL << 4); }  // B相電流過電流
  bool overcurrentC()    const { return faultValue & (1UL << 5); }  // C相電流過電流
  bool encUncalibrated() const { return faultValue & (1UL << 7); }  // エンコーダ未キャリブ
  bool hwIdFault()       const { return faultValue & (1UL << 8); }  // ハード識別故障
  bool posInitFault()    const { return faultValue & (1UL << 9); }  // 位置初期化故障
  bool stallOverload()   const { return faultValue & (1UL << 14); } // ロック過負荷保護
  bool overcurrentA()    const { return faultValue & (1UL << 16); } // A相電流過電流
};

class RobStrideEduLite {
public:
  // motorId: モーターの CAN_ID (出荷時 1 または 127)。hostId: ESP32 側の仮想ホストID
  explicit RobStrideEduLite(uint8_t motorId, uint8_t hostId = 0xFD);

  // TWAI 初期化。rxPin/txPin は SN65HVD230 の RXD/CTX へ接続した GPIO。1Mbps 固定。
  // 複数インスタンスで呼んでもバス初期化は1回だけ行われる (共有バス)。
  bool begin(int8_t rxPin = 4, int8_t txPin = 5);
  void end();

  // 共有バスが初期化済みか
  static bool busInitialized();

  // ---- 基本コマンド ----
  bool enable();                     // 通信タイプ3: 有効化
  bool disable(bool clearFault = false); // 通信タイプ4: 停止 (clearFault=true で故障クリア)
  bool setZeroPosition();            // 通信タイプ6: 現在位置を機械原点に設定
  bool saveParameters();             // 通信タイプ22: パラメータを不揮発保存
  bool setCanId(uint8_t newId);      // 通信タイプ7: CAN_ID 変更 (即時有効)

  // ---- 運転制御(MIT)モード ----
  // t_ref = kd*(vel - v_actual) + kp*(pos - p_actual) + tff
  bool setMIT(float pos, float vel, float kp, float kd, float tff);

  // ---- モード切替と各モードの指令 ----
  bool setRunMode(EL05RunMode mode);           // run_mode 書き込み (停止中に行うこと)
  bool setCurrent(float iqA);                  // 電流モード: Iq 指令
  bool setVelocity(float radS);                // 速度モード: 速度指令
  bool setPositionCSP(float rad);              // 位置モード(CSP): 角度指令
  bool setPositionPP(float rad);               // 位置モード(PP): 角度指令
  bool setVelocityAccel(float radS2);          // 速度モードの加速度
  bool setPPProfile(float velMax, float acc);  // PP の速度/加速度
  bool setCSPSpeedLimit(float radS);           // CSP の速度制限
  bool setCurrentLimit(float amp);             // 速度/位置モードの電流制限

  // ---- パラメータ読み書き (通信タイプ17/18) ----
  bool writeParamFloat(uint16_t index, float value);
  bool writeParamU8(uint16_t index, uint8_t value);
  bool writeParamU16(uint16_t index, uint16_t value);
  bool writeParamU32(uint16_t index, uint32_t value);
  bool readParam(uint16_t index, uint8_t *out4, uint32_t timeoutMs = 100); // 4バイト生データ(リトルエンディアン)
  bool readParamFloat(uint16_t index, float &value, uint32_t timeoutMs = 100);

  // ---- フィードバック ----
  // 受信キューから最新のフィードバック(タイプ2/24)を取得。timeoutMs 待っても無ければ valid=false
  EL05Feedback readFeedback(uint32_t timeoutMs = 20);
  // 能動レポート (10ms 周期の自動フィードバック) ON/OFF
  bool setActiveReport(bool on);

  // ---- ミドル関数 (個別取得の簡易版) ----
  // 内部で「中立MIT指令→フィードバック受信」を行い、該当値だけ返す。
  // いずれも取得失敗時は false を返し、out は変更しない。
  // ※ MIT モード以外 (速度/位置/電流) でも中立指令は無害 (kp=kd=tff=0)
  bool getPosition(float &outRad, uint32_t timeoutMs = 50);
  bool getVelocity(float &outRadS, uint32_t timeoutMs = 50);
  bool getTorque(float &outNm, uint32_t timeoutMs = 50);
  bool getTemperature(float &outC, uint32_t timeoutMs = 50);
  // バス電圧はパラメータ読み出し (0x701C) で取得
  bool getBusVoltage(float &outV, uint32_t timeoutMs = 100);

  // ---- 制限・整備 ----
  bool setTorqueLimit(float nm);   // トルク制限 (0~6Nm)
  bool clearFault();               // 故障クリア (disable(true) の明示版)

  // ---- デバイス情報 (v0.2.4〜) ----
  // ファームウェアのバージョン番号を読み出す (タイプ4 + Byte0=0x00, Byte1=0xC4)。
  // 取得できたら version に値(例 "1.2.3.4")が入り true を返す。
  bool readVersion(char *version, size_t len, uint32_t timeoutMs = 100);

  // ---- 故障フィードバック (通信タイプ21 = 0x15) ----
  // モーターから自発的に送られる詳細な故障・警告情報を読み取る。
  // 受信があれば true を返し out に格納。タイムアウトなら valid=false。
  bool readFault(EL05Fault &out, uint32_t timeoutMs = 50);

  // ---- ユーティリティ ----
  void setMotorId(uint8_t id) { _motorId = id; }
  uint8_t motorId() const { return _motorId; }
  static float constrainFloat(float x, float lo, float hi);

  // ---- 診断 ----
  // バススキャン: 通信タイプ0(デバイスID取得)を全ID(1..127)に送信し、
  // 応答(宛先0xFE)があったモーターIDを foundIds に格納。戻り値は見つかった台数。
  uint8_t scanBus(uint8_t *foundIds, uint8_t maxFound, uint16_t perIdTimeoutMs = 15);
  // TWAI ステータス取得 (state/TEC/REC/送信失敗数など)。false=未取得
  bool getStatus(twai_status_info_t &out);
  // TWAI アラート読み出し (begin で TX_FAILED/BUS_ERROR/BUS_OFF 等を有効化済み)
  uint32_t getAlerts(uint32_t timeoutMs = 0);

private:
  uint8_t _motorId;
  uint8_t _hostId;
  bool    _begun = false;

  // TWAI バスは全インスタンスで共有 (ドライバは1つ)
  static bool    s_busInit;
  static int8_t  s_rxPin;
  static int8_t  s_txPin;
  static uint8_t s_instanceCount;

  uint32_t makeId(uint8_t type, uint16_t dataArea) const;
  bool sendFrame(uint8_t type, uint16_t dataArea, const uint8_t *payload, uint8_t len = 8);
  bool sendFrameTo(uint8_t type, uint16_t dataArea, uint8_t targetId,
                   const uint8_t *payload, uint8_t len = 8);
  bool sendEmpty(uint8_t type);
  bool writeParamRaw(uint16_t index, const uint8_t *value4);

  // 中立MIT指令(kp=kd=tff=0)を送ってフィードバックを1つ取得する内部ヘルパー
  bool requestFeedback(EL05Feedback &fb, uint32_t timeoutMs);

  static uint16_t floatToUint(float x, float xMin, float xMax);
  static float    uintToFloat(uint16_t x, float xMin, float xMax);
};

#endif // ROBSTRIDE_EDULITE_H
