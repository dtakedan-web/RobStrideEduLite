/*
 * 02_MIT_SinePosition.ino — 運転制御(MIT)モードで正弦波位置制御
 *
 * 内容:
 *   MIT モード (run_mode=0) で目標位置を正弦波で振る (振幅 π rad, 0.2Hz)
 *   t_ref = kd*(vel - v_actual) + kp*(pos - p_actual) + tff
 *
 * 安全上の注意:
 *   - 最初は kp を小さく (例: 10〜30) して様子を見ること
 *   - 負荷を付ける前に無負荷で動作確認すること
 *   - kp を上げると保持力が増すが、振動し始めたら kd を少し上げる
 *   - モーターは絶対に手で触れられる範囲で無理な固定をしないこと
 *
 * シリアルコマンド (115200bps, 改行付き):
 *   p<値>  : 中心位置オフセット [rad]  例) p1.0
 *   a<値>  : 振幅 [rad]               例) a0.5
 *   k<値>  : kp                      例) k20
 *   d<値>  : kd                      例) d0.5
 *   s      : 停止 (disable)
 *   e      : 再有効化 (enable)
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5
#define MOTOR_ID   1

RobStrideEduLite motor(MOTOR_ID);

float amp   = PI;      // 振幅 [rad]
float phase = 0.0f;
float freq  = 0.2f;    // 正弦波周波数 [Hz]
float kp    = 20.0f;   // 位置ゲイン (まずは小さめから)
float kd    = 0.5f;    // 速度ゲイン (ダンピング)
bool  running = true;

const uint32_t CTRL_PERIOD_MS = 10;  // 100Hz 制御周期

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== EL05 MIT 正弦波位置制御 ==="));

  if (!motor.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) { delay(1000); }
  }

  motor.disable(true);   // 故障クリア付きで一度停止
  delay(100);
  motor.setRunMode(EL05_MODE_MIT);  // 明示的に MIT モードへ (既定0だが確実に)
  delay(50);
  motor.enable();
  Serial.println(F("有効化完了。制御開始します。"));
  Serial.println(F("コマンド: p<rad> a<rad> k<kp> d<kd> s=停止 e=再開"));
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = (char)Serial.read();
  String rest = Serial.readStringUntil('\n');
  float val = rest.toFloat();
  switch (c) {
    case 'p': phase = 0; amp = 0; Serial.printf("中心位置 -> %.3f rad (固定)\n", val);
              // 中心位置固定モード: amp=0 にして目標=val
              motor.setMIT(val, 0, kp, kd, 0);
              break;
    case 'a': amp = RobStrideEduLite::constrainFloat(val, 0, EL05_P_MAX); Serial.printf("振幅 -> %.3f rad\n", amp); break;
    case 'k': kp = RobStrideEduLite::constrainFloat(val, EL05_KP_MIN, EL05_KP_MAX); Serial.printf("kp -> %.2f\n", kp); break;
    case 'd': kd = RobStrideEduLite::constrainFloat(val, EL05_KD_MIN, EL05_KD_MAX); Serial.printf("kd -> %.3f\n", kd); break;
    case 's': running = false; motor.disable(); Serial.println(F("停止しました")); break;
    case 'e': running = true; motor.enable(); Serial.println(F("再開しました")); break;
  }
}

void loop() {
  handleSerial();
  if (!running) { delay(50); return; }

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < CTRL_PERIOD_MS) return;
  last = now;

  // 目標位置 = 中心0 + 正弦波 (p コマンドで amp=0 の場合は固定位置保持)
  float target = amp * sinf(2.0f * PI * freq * (now / 1000.0f));
  motor.setMIT(target, 0, kp, kd, 0);

  // フィードバックを 10 回に 1 回表示
  static int cnt = 0;
  EL05Feedback fb = motor.readFeedback(5);
  if (fb.valid && ++cnt >= 10) {
    cnt = 0;
    Serial.printf("target=%+.3f pos=%+.3f vel=%+.2f trq=%+.2f temp=%.1f%s\n",
                  target, fb.positionRad, fb.velocityRps, fb.torqueNm, fb.tempC,
                  fb.hasFault ? " [FAULT!]" : "");
    if (fb.hasFault) {
      Serial.printf("故障ビット=0x%02X → 故障クリアを送信します\n", fb.faultBits);
      motor.disable(true);
      delay(100);
      motor.enable();
    }
  }
}
