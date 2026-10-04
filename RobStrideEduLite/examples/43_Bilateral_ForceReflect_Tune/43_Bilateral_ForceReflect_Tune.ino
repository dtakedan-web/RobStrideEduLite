/*
 * 43_Bilateral_ForceReflect_Tune.ino — 力反射型バイラテラル (シリアル調整版)
 *
 * 位置同期 + 力反射を行いながら、シリアルコマンドで各ゲインを対話的に変更できます。
 * マスター(1台目)を回すとスレーブ(2台目)が追従し、スレーブへの外力が
 * マスターの手に「手ごたえ」として返ります。
 *
 * シリアルコマンド (115200bps, 改行付き):
 *   k<値>  : 位置追従 kp       例) k15
 *   d<値>  : ダンピング kd      例) d0.5
 *   f<値>  : 力反射ゲイン       例) f0.5  (大きいほど手ごたえ強い)
 *   r<値>  : 制御周期 [ms]      例) r10
 *   s      : 停止
 *   e      : 再開
 *   ?      : 現在の設定値を表示
 *
 * 起動時の既定値: kp=15, kd=0.5, 力反射=0.3, 周期=5ms
 *
 * 注意: 必ず無負荷・両台固定した状態で実行。発振したらゲインを下げてください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

float Kp = 15.0f;
float Kd = 0.5f;
float ForceGain = 0.3f;
uint32_t CTRL_MS = 5;
bool running = true;

RobStrideEduLite master(1);
RobStrideEduLite slave(1);
bool ready = false;

void showSettings() {
  Serial.printf("設定: kp=%.2f kd=%.3f 力反射=%.2f 周期=%lums %s\n",
                Kp, Kd, ForceGain, (unsigned long)CTRL_MS,
                running ? "動作中" : "停止中");
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== 力反射バイラテラル (調整版) ==="));

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

  Serial.println(F("コマンド: k<kp> d<kd> f<力反射> r<周期ms> s=停止 e=再開 ?=設定表示"));
  showSettings();
  Serial.println(F("マスターを回すとスレーブが追従し、スレーブへの外力が手に返ります"));
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
    case 'f':
      ForceGain = RobStrideEduLite::constrainFloat(val, 0.0f, 3.0f);
      Serial.printf("力反射ゲイン -> %.2f\n", ForceGain);
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

  // 両方の最新状態を取得
  master.setMIT(0, 0, 0, 0, 0);
  EL05Feedback fm = master.readFeedback(3);
  slave.setMIT(0, 0, 0, 0, 0);
  EL05Feedback fs = slave.readFeedback(3);
  if (!fm.valid || !fs.valid) return;

  // 位置同期: スレーブをマスター位置へ追従
  slave.setMIT(fm.positionRad, 0, Kp, Kd, 0);

  // 力反射: スレーブのトルクをマスターへ逆方向オフセットとして加える
  float forceOffset = -ForceGain * fs.torqueNm;
  master.setMIT(fs.positionRad + forceOffset, 0, Kp, Kd, 0);

  // 1秒ごとに状態表示
  static int cnt = 0;
  if (++cnt >= (int)(1000 / CTRL_MS)) {
    cnt = 0;
    Serial.printf("M=%+.2f S=%+.2f スレーブトルク=%+.2f Nm\n",
                  fm.positionRad, fs.positionRad, fs.torqueNm);
  }
}
