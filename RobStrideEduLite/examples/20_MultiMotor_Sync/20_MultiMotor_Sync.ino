/*
 * 20_MultiMotor_Sync.ino — 2 台のモーターを同期して動かすサンプル
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * 起動時にバス上のモーターを自動検出し、2 台見つかったら
 * MIT モードで正弦波を同期させて往復させます。
 *
 * 動作:
 *   モーター1台目 … +90° を中心に正弦波 (位相 0)
 *   モーター2台目 … 同じ正弦波を位相 180° ずらして往復 (逆位相)
 *
 * 前提:
 *   - モーター 2 台を CAN バスに並列接続 (CANH/CANL/GND を共有)
 *   - 各モーターの CAN_ID が異なること (05_MotorSetup で事前に設定)
 *   - 両方に 48V 電源を供給
 *
 * 調整パラメータ:
 *   AMPLITUDE_RAD … 振幅 [rad]
 *   FREQ_HZ       … 往復の速さ [Hz]
 *   KP / KD       … ゲイン
 *
 * 注意: 必ず無負荷・両台固定した状態で実行してください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float AMPLITUDE_RAD = PI / 2.0f;  // ±90°
const float FREQ_HZ       = 0.2f;
const float KP            = 20.0f;
const float KD            = 0.5f;
const uint32_t CTRL_MS    = 10;         // 100Hz

// 2 台分のインスタンス (ID は起動時のスキャンで自動設定)
RobStrideEduLite motorA(1);
RobStrideEduLite motorB(1);
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== 2 台同期制御サンプル ==="));

  // バスは 1 回だけ初期化 (motorA が行う)。motorB.begin は共有バスを使うだけ。
  if (!motorA.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) delay(1000);
  }
  motorB.begin(CAN_RX_PIN, CAN_TX_PIN);  // 共有バス利用

  // 2 台検出
  uint8_t found[8];
  uint8_t n = motorA.scanBus(found, 8, 5);
  if (n < 2) {
    Serial.printf("モーターが %u 台しか見つかりません (2台必要)\n", n);
    Serial.println(F("両台の CAN_ID が異なるか、48V 供給を確認してください"));
    while (1) delay(1000);
  }
  motorA.setMotorId(found[0]);
  motorB.setMotorId(found[1]);
  Serial.printf("検出: 1台目 ID=%u, 2台目 ID=%u\n", found[0], found[1]);

  // 両台を MIT モードで有効化
  motorA.disable(true);
  motorB.disable(true);
  delay(100);
  motorA.setRunMode(EL05_MODE_MIT);
  motorB.setRunMode(EL05_MODE_MIT);
  delay(50);
  motorA.enable();
  motorB.enable();
  ready = true;
  Serial.println(F("動作開始: 2 台が逆位相で往復します"));
}

void loop() {
  if (!ready) return;
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < CTRL_MS) return;
  last = now;

  float t = now / 1000.0f;
  // 1台目: 位相 0
  float targetA = AMPLITUDE_RAD * sinf(2.0f * PI * FREQ_HZ * t);
  // 2台目: 位相 180° (逆位相)
  float targetB = AMPLITUDE_RAD * sinf(2.0f * PI * FREQ_HZ * t + PI);

  motorA.setMIT(targetA, 0, KP, KD, 0);
  motorB.setMIT(targetB, 0, KP, KD, 0);

  // 1秒ごとに状態表示
  static int cnt = 0;
  EL05Feedback fa = motorA.readFeedback(3);
  EL05Feedback fb = motorB.readFeedback(3);
  if (++cnt >= 100) {
    cnt = 0;
    if (fa.valid) Serial.printf("[A ID=%u] 位置=%+.2f 温度=%.1fC  ", fa.motorId, fa.positionRad, fa.tempC);
    if (fb.valid) Serial.printf("[B ID=%u] 位置=%+.2f 温度=%.1fC\n", fb.motorId, fb.positionRad, fb.tempC);
  }
}
