/*
 * 12_RunMode_Velocity.ino — 速度モードの動作サンプル
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * 速度モードで正転・反転を繰り返します。
 *
 * 動作: 正転 3秒 → 停止 1秒 → 反転 3秒 → 停止 1秒 … を繰り返します。
 *
 * 調整パラメータ:
 *   SPEED_RAD_S … 回転速度 [rad/s] (符号が方向、絶対値が速さ)
 *   RUN_MS      … 回転している時間 [ms]
 *   STOP_MS     … 停止している時間 [ms]
 *
 * 注意: 必ず無負荷・固定した状態で実行してください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float SPEED_RAD_S = 5.0f;    // 速度 5 rad/s
const uint32_t RUN_MS   = 3000;    // 回転 3秒
const uint32_t STOP_MS  = 1000;    // 停止 1秒

RobStrideEduLite motor(1);
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== 速度モード サンプル ==="));

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

  // 速度モードに切り替え
  motor.disable(true);
  delay(100);
  motor.setRunMode(EL05_MODE_VELOCITY);
  motor.setVelocityAccel(20.0f);   // 加速度 20 rad/s^2
  motor.setCurrentLimit(5.0f);     // 電流制限 5A
  motor.setVelocity(0.0f);
  delay(50);
  motor.enable();
  ready = true;
  Serial.println(F("動作開始: 正転と反転を繰り返します"));
}

void loop() {
  if (!ready) return;

  motor.setVelocity(SPEED_RAD_S);          // 正転
  Serial.printf("正転 %+.1f rad/s\n", SPEED_RAD_S);
  delay(RUN_MS);

  motor.setVelocity(0.0f);                 // 停止
  Serial.println(F("停止"));
  delay(STOP_MS);

  motor.setVelocity(-SPEED_RAD_S);         // 反転
  Serial.printf("反転 %+.1f rad/s\n", -SPEED_RAD_S);
  delay(RUN_MS);

  motor.setVelocity(0.0f);                 // 停止
  Serial.println(F("停止"));
  delay(STOP_MS);
}
