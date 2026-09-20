/*
 * 11_RunMode_PositionPP.ino — 位置モード (PP) の動作サンプル
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * PP (Point to Point) モードは台形加減速で目標位置へ移動します。
 *
 * 動作: 0 → +180° → -180° → 0 … を繰り返します。
 *
 * 調整パラメータ:
 *   POS_A_RAD / POS_B_RAD … 行き先の2点 [rad]
 *   VEL_MAX   … 移動速度 [rad/s]
 *   ACC       … 加速度 [rad/s^2]
 *   WAIT_MS   … 各点での待機時間 [ms]
 *
 * 注意: 必ず無負荷・固定した状態で実行してください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float POS_A_RAD =  PI;        // 行き先A: +180°
const float POS_B_RAD = -PI;        // 行き先B: -180°
const float VEL_MAX   = 5.0f;       // 速度 5 rad/s
const float ACC       = 10.0f;      // 加速度 10 rad/s^2
const uint32_t WAIT_MS = 2000;      // 各点で 2秒待機

RobStrideEduLite motor(1);
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== 位置モード (PP) サンプル ==="));

  if (!motor.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) delay(1000);
  }

  uint8_t found[8];
  if (motor.scanBus(found, 8, 5) == 0) {
    Serial.println(F("モーターが見つかりません"));
    while (1) delay(1000);
  }
  motor.setMotorId(found[0]);
  Serial.printf("モーター検出: ID=%u\n", found[0]);

  // PP モードに切り替え
  motor.disable(true);
  delay(100);
  motor.setRunMode(EL05_MODE_PP);
  motor.setPPProfile(VEL_MAX, ACC);
  delay(50);
  motor.enable();
  ready = true;
  Serial.println(F("動作開始: +180° と -180° を往復します"));
}

void loop() {
  if (!ready) return;

  // A点へ
  motor.setPositionPP(POS_A_RAD);
  Serial.printf("-> 目標 %+.2f rad\n", POS_A_RAD);
  delay(WAIT_MS);

  // B点へ
  motor.setPositionPP(POS_B_RAD);
  Serial.printf("-> 目標 %+.2f rad\n", POS_B_RAD);
  delay(WAIT_MS);

  // 0へ戻る
  motor.setPositionPP(0.0f);
  Serial.println(F("-> 目標 0 rad"));
  delay(WAIT_MS);
}
