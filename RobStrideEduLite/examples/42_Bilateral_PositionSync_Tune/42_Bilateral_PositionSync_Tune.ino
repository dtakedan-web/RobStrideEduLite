/*
 * 42_Bilateral_PositionSync_Tune.ino — 位置同期型バイラテラル (シリアル調整版)
 *
 * 2 台の位置を同期させながら、シリアルコマンドでゲインを対話的に変更できます。
 * マスター(1台目)の軸を手で回すと、スレーブ(2台目)が追従します。
 *
 * シリアルコマンド (115200bps, 改行付き):
 *   k<値>  : 同期の硬さ kp を変更   例) k5   (下げると柔らかく)
 *   d<値>  : ダンピング kd を変更   例) d1.0 (振動抑制)
 *   r<値>  : 制御周期 [ms] を変更   例) r10  (既定5ms=200Hz)
 *   s      : 停止 (両モーター disable)
 *   e      : 再開 (両モーター enable)
 *   ?      : 現在の設定値を表示
 *
 * 起動時の既定値: kp=10, kd=0.5, 周期=5ms
 *
 * 注意: 必ず無負荷・両台固定した状態で実行。発振したら kp を下げてください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

float Kp = 10.0f;
float Kd = 0.5f;
uint32_t CTRL_MS = 5;
bool running = true;

RobStrideEduLite master(1);
RobStrideEduLite slave(1);
bool ready = false;

void showSettings() {
  Serial.printf("設定: kp=%.2f kd=%.3f 周期=%lums (%.0fHz) %s\n",
                Kp, Kd, (unsigned long)CTRL_MS, 1000.0f / CTRL_MS,
                running ? "動作中" : "停止中");
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== 位置同期バイラテラル (調整版) ==="));

  if (!master.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) delay(1000);
  }
  slave.begin(CAN_RX_PIN, CAN_TX_PIN);

  uint8_t found[8];
  uint8_t n = master.scanBus(found, 8, 5);
  if (n < 2) {
    Serial.printf("モーターが %u 台しか見つかりません (2台必要)\n", n);
    Serial.println(F("05_MotorSetup で各モーターに異なるIDを設定してください"));
    while (1) delay(1000);
  }
  master.setMotorId(found[0]);
  slave.setMotorId(found[1]);
  Serial.printf("マスター ID=%u, スレーブ ID=%u\n", found[0], found[1]);

  master.disable(true);
  slave.disable(true);
  delay(100);
  master.setRunMode(EL05_MODE_MIT);
  slave.setRunMode(EL05_MODE_MIT);
  delay(50);
  master.enable();
  slave.enable();
  ready = true;

  Serial.println(F("コマンド: k<kp> d<kd> r<周期ms> s=停止 e=再開 ?=設定表示"));
  showSettings();
  Serial.println(F("マスターの軸を手で回すとスレーブが追従します"));
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = (char)Serial.read();
  String rest = Serial.readStringUntil('\n');
  float val = rest.toFloat();
  switch (c) {
    case 'k':
      Kp = RobStrideEduLite::constrainFloat(val, 0.0f, EL05_KP_MAX);
      Serial.printf("kp -> %.2f\n", Kp);
      break;
    case 'd':
      Kd = RobStrideEduLite::constrainFloat(val, 0.0f, EL05_KD_MAX);
      Serial.printf("kd -> %.3f\n", Kd);
      break;
    case 'r': {
      uint32_t ms = (uint32_t)val;
      if (ms < 2) ms = 2;
      if (ms > 100) ms = 100;
      CTRL_MS = ms;
      Serial.printf("周期 -> %lu ms (%.0f Hz)\n", (unsigned long)CTRL_MS, 1000.0f / CTRL_MS);
      break;
    }
    case 's':
      master.disable(); slave.disable();
      running = false;
      Serial.println(F("停止しました"));
      break;
    case 'e':
      master.enable(); slave.enable();
      running = true;
      Serial.println(F("再開しました"));
      break;
    case '?':
      showSettings();
      break;
  }
}

void loop() {
  handleSerial();
  if (!ready || !running) { delay(10); return; }

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < CTRL_MS) return;
  last = now;

  // 両方の最新位置を取得
  master.setMIT(0, 0, 0, 0, 0);
  EL05Feedback fm = master.readFeedback(3);
  slave.setMIT(0, 0, 0, 0, 0);
  EL05Feedback fs = slave.readFeedback(3);
  if (!fm.valid || !fs.valid) return;

  // 相互に追従
  slave.setMIT(fm.positionRad, 0, Kp, Kd, 0);
  master.setMIT(fs.positionRad, 0, Kp, Kd, 0);

  // 1秒ごとに状態表示
  static int cnt = 0;
  if (++cnt >= (int)(1000 / CTRL_MS)) {
    cnt = 0;
    Serial.printf("M=%+.2f S=%+.2f 誤差=%+.3f rad\n",
                  fm.positionRad, fs.positionRad, fm.positionRad - fs.positionRad);
  }
}
