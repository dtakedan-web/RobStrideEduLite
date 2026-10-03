/*
 * 31_RelativeMove.ino — 相対移動 (現在位置から ±角度) のサンプル
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * 「今いる角度から ±◯° 動かす」相対移動関数の使用例です。
 *
 * 動作 (位置モード CSP):
 *   現在位置から +45° → さらに +45° → -90° (元付近へ戻る) … を繰り返します。
 *
 * 使っている関数:
 *   moveRelativeCSP(rad)       … 現在位置から rad 分だけ移動
 *   moveRelativeCSP_deg(deg)   … 現在位置から 度 分だけ移動
 *   moveRelativePP(rad) / _deg … PP モード版も同様に用意
 *
 * 通常の setPositionCSP (絶対位置指定) との違い:
 *   setPositionCSP_deg(90)  … 原点から見て 90° の位置へ (絶対)
 *   moveRelativeCSP_deg(90) … 今の位置から +90° 進む (相対)
 *
 * 注意:
 *   - 必ず無負荷・固定した状態で実行してください
 *   - 移動後の位置が ±720° (±4π rad) を超える指令は範囲外になります
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float STEP_DEG  = 45.0f;    // 1 ステップの移動量 [度]
const float LIMIT_RPM = 30.0f;    // CSP 速度制限 [rpm]
const uint32_t WAIT_MS = 2000;    // 各ステップ後の待機時間

RobStrideEduLite motor(1);
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== 相対移動サンプル ==="));

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

  // CSP モードに切り替え
  motor.disable(true);
  delay(100);
  motor.setRunMode(EL05_MODE_CSP);
  motor.setCSPSpeedLimit_rpm(LIMIT_RPM);
  delay(50);
  motor.enable();
  ready = true;

  float deg;
  if (motor.getPosition_deg(deg)) {
    Serial.printf("開始位置: %+.1f°\n", deg);
  }
  Serial.println(F("動作開始: +45° → +45° → -90° を繰り返します"));
}

void showDeg() {
  float deg;
  if (motor.getPosition_deg(deg)) {
    Serial.printf("  現在位置: %+.1f°\n", deg);
  }
}

void loop() {
  if (!ready) return;

  // 現在位置から +45°
  Serial.printf("相対移動: %+.0f°\n", STEP_DEG);
  motor.moveRelativeCSP_deg(STEP_DEG);
  delay(WAIT_MS);
  showDeg();

  // さらに +45°
  Serial.printf("相対移動: %+.0f°\n", STEP_DEG);
  motor.moveRelativeCSP_deg(STEP_DEG);
  delay(WAIT_MS);
  showDeg();

  // 現在位置から -90° (開始付近へ戻る)
  Serial.printf("相対移動: %+.0f°\n", -2.0f * STEP_DEG);
  motor.moveRelativeCSP_deg(-2.0f * STEP_DEG);
  delay(WAIT_MS);
  showDeg();

  Serial.println();
}
