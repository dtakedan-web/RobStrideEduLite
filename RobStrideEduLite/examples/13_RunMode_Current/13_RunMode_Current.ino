/*
 * 13_RunMode_Current.ino — 電流モードの動作サンプル
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * 電流モードは Iq 電流 (=トルク) を直接指令します。
 *
 * 動作: 正方向トルク 2秒 → 停止 1秒 → 負方向トルク 2秒 → 停止 1秒 …
 *       無負荷だとトルクで加速して回り続けるので注意してください。
 *
 * 調整パラメータ:
 *   IQ_A   … Iq 電流指令 [A] (±11A まで。小さいほど弱いトルク)
 *   RUN_MS … トルクを出している時間 [ms]
 *
 * 重要な注意:
 *   電流モードは速度制限が無いため、無負荷では回転がどんどん上がります。
 *   必ず小さい電流 (1〜2A 程度) と短い時間から試し、
 *   回転が速くなりすぎたらすぐ電源を切れるようにしてください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float IQ_A     = 1.5f;   // Iq 電流 1.5A (まずは小さめ)
const uint32_t RUN_MS  = 2000; // トルクを出す時間 2秒
const uint32_t STOP_MS = 1000; // 休止 1秒

RobStrideEduLite motor(1);
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== 電流モード サンプル ==="));
  Serial.println(F("注意: 無負荷では回転が上がります。小さい電流から試してください。"));

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

  // 電流モードに切り替え
  motor.disable(true);
  delay(100);
  motor.setRunMode(EL05_MODE_CURRENT);
  motor.setCurrent(0.0f);
  delay(50);
  motor.enable();
  ready = true;
  Serial.println(F("動作開始: 正負のトルクを繰り返します"));
}

void loop() {
  if (!ready) return;

  motor.setCurrent(IQ_A);                  // 正方向トルク
  Serial.printf("Iq = %+.1f A\n", IQ_A);
  delay(RUN_MS);

  motor.setCurrent(0.0f);                  // トルク0
  Serial.println(F("Iq = 0"));
  delay(STOP_MS);

  motor.setCurrent(-IQ_A);                 // 負方向トルク
  Serial.printf("Iq = %+.1f A\n", -IQ_A);
  delay(RUN_MS);

  motor.setCurrent(0.0f);                  // トルク0
  Serial.println(F("Iq = 0"));
  delay(STOP_MS);
}
