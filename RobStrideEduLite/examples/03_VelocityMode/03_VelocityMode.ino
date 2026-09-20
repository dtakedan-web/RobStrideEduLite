/*
 * 03_VelocityMode.ino — 速度モードでの回転制御
 *
 * 内容:
 *   run_mode=2 (速度モード) に切り替え、速度指令を送る
 *   シリアルから速度を変更可能
 *
 * シリアルコマンド (115200bps, 改行付き):
 *   v<値>  : 目標速度 [rad/s]  例) v5.0  (範囲 ±50rad/s)
 *   a<値>  : 加速度 [rad/s^2]  例) a20
 *   s      : 停止 (速度0 + disable)
 *   e      : 再有効化 (速度0で enable)
 *
 * 注意: 速度モードは指令が残ると回り続けます。シリアル切断や電源断に備え、
 *       停止コマンドの送り方を必ず確認してから実行してください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5
#define MOTOR_ID   1

RobStrideEduLite motor(MOTOR_ID);

bool running = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== EL05 速度モード ==="));

  if (!motor.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) { delay(1000); }
  }

  // モード切替は停止状態で行う (マニュアル注意事項)
  motor.disable(true);
  delay(100);
  motor.setRunMode(EL05_MODE_VELOCITY);
  delay(50);
  motor.setVelocityAccel(20.0f);   // 加速度 20 rad/s^2 (既定値)
  motor.setCurrentLimit(5.0f);     // 電流制限 5A (様子見のため控えめ)
  motor.setVelocity(0.0f);         // 初期速度 0
  delay(50);
  motor.enable();
  running = true;

  Serial.println(F("有効化完了 (速度0)。コマンド: v<rad/s> a<rad/s^2> s=停止 e=再開"));
}

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();
    String rest = Serial.readStringUntil('\n');
    float val = rest.toFloat();
    switch (c) {
      case 'v':
        motor.setVelocity(val);
        Serial.printf("目標速度 -> %.2f rad/s\n",
                      RobStrideEduLite::constrainFloat(val, EL05_V_MIN, EL05_V_MAX));
        break;
      case 'a':
        motor.setVelocityAccel(val);
        Serial.printf("加速度 -> %.2f rad/s^2\n", val);
        break;
      case 's':
        motor.setVelocity(0.0f);
        delay(300);               // 減速を少し待つ
        motor.disable();
        running = false;
        Serial.println(F("停止しました"));
        break;
      case 'e':
        motor.enable();
        running = true;
        Serial.println(F("再開しました (速度0 から)"));
        break;
    }
  }

  // 速度指令はパラメータ書き込みのためモーター側に保持される。
  // 現在値はパラメータ読み出し(0x7019/0x701B)で 500ms ごとに表示
  static uint32_t last = 0;
  if (running && millis() - last >= 500) {
    last = millis();
    float pos, vel;
    if (motor.readParamFloat(EL05_IDX_MECH_POS, pos, 50) &&
        motor.readParamFloat(EL05_IDX_MECH_VEL, vel, 50)) {
      Serial.printf("pos=%+.3f rad  vel=%+.2f rad/s\n", pos, vel);
    }
  }
}
