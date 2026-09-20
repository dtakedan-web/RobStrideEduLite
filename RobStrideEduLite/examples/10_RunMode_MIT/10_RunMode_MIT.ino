/*
 * 10_RunMode_MIT.ino — 運転制御(MIT)モードの動作サンプル
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * 起動時にモーターを自動検出し、MIT モードで正弦波の位置制御を行います。
 *
 * 動作: 軸が 0 を中心に ±90° の範囲をゆっくり往復します。
 *
 * 調整したい場合は下の定数を書き換えてください:
 *   AMPLITUDE_RAD … 振幅 [rad]   (π/2 = 90°)
 *   FREQ_HZ       … 往復の速さ [Hz] (0.2 = 5秒で1往復)
 *   KP            … 位置ゲイン (大きいほど硬く追従)
 *   KD            … 速度ゲイン (ダンピング。振動したら上げる)
 *
 * 注意: 必ず無負荷・固定した状態で実行してください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float AMPLITUDE_RAD = PI / 2.0f;  // 振幅 ±90°
const float FREQ_HZ       = 0.2f;       // 0.2 Hz (5秒で1往復)
const float KP            = 20.0f;      // 位置ゲイン
const float KD            = 0.5f;       // 速度ゲイン
const uint32_t CTRL_MS    = 10;         // 制御周期 100Hz

RobStrideEduLite motor(1);
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== MIT モード サンプル (正弦波位置制御) ==="));

  if (!motor.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) delay(1000);
  }

  // モーター自動検出
  uint8_t found[8];
  if (motor.scanBus(found, 8, 5) == 0) {
    Serial.println(F("モーターが見つかりません"));
    while (1) delay(1000);
  }
  motor.setMotorId(found[0]);
  Serial.printf("モーター検出: ID=%u\n", found[0]);

  // MIT モードで有効化
  motor.disable(true);
  delay(100);
  motor.setRunMode(EL05_MODE_MIT);
  delay(50);
  motor.enable();
  ready = true;
  Serial.println(F("動作開始: 軸が正弦波で往復します"));
}

void loop() {
  if (!ready) return;
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < CTRL_MS) return;
  last = now;

  // 目標位置 = 振幅 × sin(2π × 周波数 × 時刻)
  float target = AMPLITUDE_RAD * sinf(2.0f * PI * FREQ_HZ * (now / 1000.0f));
  motor.setMIT(target, 0, KP, KD, 0);

  // 1秒ごとに状態表示
  static int cnt = 0;
  EL05Feedback fb = motor.readFeedback(5);
  if (fb.valid && ++cnt >= 100) {
    cnt = 0;
    Serial.printf("目標=%+.2f 位置=%+.2f rad トルク=%+.2f Nm 温度=%.1f C\n",
                  target, fb.positionRad, fb.torqueNm, fb.tempC);
  }
}
