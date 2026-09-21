/*
 * 21_MultiMotor_Independent.ino — 2 台のモーターを別々のモードで動かすサンプル
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * 起動時に 2 台を自動検出し、それぞれ異なるモードで独立に動かします。
 *
 * 動作:
 *   モーター1台目 … 速度モードで正転・反転を繰り返す
 *   モーター2台目 … 位置モード (CSP) で +90°/−90° を往復
 *
 * 前提:
 *   - モーター 2 台を CAN バスに並列接続
 *   - 各モーターの CAN_ID が異なること
 *   - 両方に 48V 電源を供給
 *
 * 調整パラメータ:
 *   SPEED_A     … 1台目の回転速度 [rad/s]
 *   POS_B       … 2台目の往復位置 [rad]
 *
 * 注意: 必ず無負荷・両台固定した状態で実行してください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float SPEED_A = 5.0f;         // 1台目の速度
const float POS_B   = PI / 2.0f;    // 2台目の往復 ±90°

RobStrideEduLite motorA(1);   // 速度モード用
RobStrideEduLite motorB(1);   // 位置モード用
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== 2 台独立制御サンプル ==="));

  if (!motorA.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) delay(1000);
  }
  motorB.begin(CAN_RX_PIN, CAN_TX_PIN);  // 共有バス利用

  uint8_t found[8];
  uint8_t n = motorA.scanBus(found, 8, 5);
  if (n < 2) {
    Serial.printf("モーターが %u 台しか見つかりません (2台必要)\n", n);
    while (1) delay(1000);
  }
  motorA.setMotorId(found[0]);
  motorB.setMotorId(found[1]);
  Serial.printf("検出: 1台目 ID=%u (速度), 2台目 ID=%u (位置)\n", found[0], found[1]);

  // 1台目: 速度モード
  motorA.disable(true);
  motorA.setRunMode(EL05_MODE_VELOCITY);
  motorA.setVelocityAccel(20.0f);
  motorA.setCurrentLimit(5.0f);
  motorA.setVelocity(0.0f);

  // 2台目: 位置モード (CSP)
  motorB.disable(true);
  motorB.setRunMode(EL05_MODE_CSP);
  motorB.setCSPSpeedLimit(5.0f);

  delay(100);
  motorA.enable();
  motorB.enable();
  ready = true;
  Serial.println(F("動作開始: 1台目=速度回転, 2台目=位置往復"));
}

void loop() {
  if (!ready) return;

  // ---- 1台目: 速度モードで正転・反転 ----
  motorA.setVelocity(SPEED_A);
  delay(2000);
  motorA.setVelocity(0.0f);
  delay(500);
  motorA.setVelocity(-SPEED_A);
  delay(2000);
  motorA.setVelocity(0.0f);
  delay(500);

  // ---- 2台目: 位置モードで往復 ----
  motorB.setPositionCSP(POS_B);
  delay(1500);
  motorB.setPositionCSP(-POS_B);
  delay(1500);
  motorB.setPositionCSP(0.0f);
  delay(1500);
}
