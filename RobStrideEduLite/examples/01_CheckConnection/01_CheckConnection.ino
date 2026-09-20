/*
 * 01_CheckConnection.ino — 最初の動作確認スケッチ
 *
 * 内容:
 *   1. TWAI(CAN) 初期化 (GPIO4=RX, GPIO5=TX, 1Mbps)
 *   2. モーター有効化 (通信タイプ3)
 *   3. フィードバックを読んで表示 (位置/速度/トルク/温度/故障)
 *      ※ MIT 指令を送ると必ずフィードバックが返るため、
 *        無負荷で安全な「中立指令」(kp=kd=tff=0) を定期送信して応答を得る
 *   4. バス電圧・run_mode をパラメータ読み出しで確認
 *   5. モーター停止 (通信タイプ4)
 *
 * 配線:
 *   ESP32-KEY-R2A GPIO4 -> SN65HVD230 RXD
 *   ESP32-KEY-R2A GPIO5 -> SN65HVD230 CTX
 *   3.3V -> VCC, GND -> GND (モーター電源GNDと共通)
 *   CANH/CANL -> モーター XT30 信号ピン, バス両端に120Ω
 *
 * 注意: モーター電源 48V は ESP32 側に絶対に触れさせないこと!
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5
#define MOTOR_ID   1    // モーターの CAN_ID (出荷時は 1 または 127。不明ならホストPCツールで確認)

RobStrideEduLite motor(MOTOR_ID);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println(F("=== EL05 接続確認 ==="));

  if (!motor.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗。配線とピン番号を確認してください。"));
    while (1) { delay(1000); }
  }
  Serial.println(F("TWAI 初期化 OK (1Mbps)"));

  // 有効化
  if (!motor.enable()) {
    Serial.println(F("有効化フレーム送信失敗"));
  }
  delay(100);

  // 応答確認: kp=kd=tff=0 の中立MIT指令(トルクを出さない)を投げてフィードバックを受ける
  Serial.println(F("フィードバック確認中 (5回)..."));
  int okCount = 0;
  for (int i = 0; i < 5; i++) {
    motor.setMIT(0, 0, 0, 0, 0);  // 中立指令(無負荷でも軸は動きません)
    EL05Feedback fb = motor.readFeedback(100);
    if (fb.valid) {
      okCount++;
      Serial.printf("[%d] ID=%u mode=%u fault=0x%02X pos=%.3frad vel=%.2frad/s trq=%.2fNm temp=%.1fC\n",
                    i, fb.motorId, fb.mode, fb.faultBits,
                    fb.positionRad, fb.velocityRps, fb.torqueNm, fb.tempC);
    } else {
      Serial.printf("[%d] 応答なし\n", i);
    }
    delay(100);
  }

  if (okCount == 0) {
    Serial.println(F(">>> 応答がありません。CANH/CANL の配線・120Ω終端・GND共通・モーターID・48V電源を確認してください。"));
  } else {
    Serial.println(F(">>> 通信 OK!"));

    // パラメータ読み出しテスト: バス電圧と run_mode
    float vbus = 0;
    if (motor.readParamFloat(EL05_IDX_VBUS, vbus)) {
      Serial.printf("バス電圧: %.1f V\n", vbus);
    } else {
      Serial.println(F("バス電圧の読み出しに失敗"));
    }
    uint8_t rm[4];
    if (motor.readParam(EL05_IDX_RUN_MODE, rm)) {
      Serial.printf("run_mode: %u (0=MIT 1=PP 2=速度 3=電流 5=CSP)\n", rm[0]);
    }
  }

  // 停止
  motor.disable();
  Serial.println(F("=== 確認終了。モーターは停止状態です ==="));
}

void loop() {
  // 何もしない
}
