/*
 * 04_PositionMode.ino — 位置モード (CSP / PP) での制御
 *
 * MIT モードとは別の、モーター内蔵の位置ループを使うモード。
 * - CSP (run_mode=5): 周期位置モード。速度制限付きで滑らかに追従
 * - PP  (run_mode=1): 台形加減速のポイントツーポイント移動
 *
 * シリアルコマンド (115200bps, 改行付き):
 *   c<rad>  : CSP モードで目標位置へ    例) c1.57
 *   p<rad>  : PP モードで目標位置へ     例) p3.14
 *   v<rad/s>: CSP/PP の速度制限         例) v5.0
 *   s       : 停止
 *   e       : 再有効化
 *   r       : 現在位置・速度を表示
 *
 * 注意: モード切替は必ず停止状態で行います (このスケッチが自動で行います)
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5
#define MOTOR_ID   127   // 自分のモーターIDに合わせる

RobStrideEduLite motor(MOTOR_ID);
bool running = false;

void showState() {
  float pos, vel, vbus;
  if (motor.getPosition(pos) && motor.getVelocity(vel) && motor.getBusVoltage(vbus)) {
    Serial.printf("pos=%+.3f rad  vel=%+.2f rad/s  Vbus=%.1fV\n", pos, vel, vbus);
  } else {
    Serial.println(F("取得失敗"));
  }
}

// モードを安全に切り替える (停止→モード変更→有効化)
void switchMode(EL05RunMode mode, const char *name) {
  motor.disable();
  delay(100);
  motor.setRunMode(mode);
  delay(50);
  motor.enable();
  running = true;
  Serial.printf("%s モードに切り替えました\n", name);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println(F("=== EL05 位置モード (CSP / PP) ==="));

  if (!motor.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) { delay(1000); }
  }

  motor.disable(true);   // 故障クリア
  delay(100);

  // 既定は CSP モードで起動 (速度制限 5 rad/s)
  motor.setRunMode(EL05_MODE_CSP);
  motor.setCSPSpeedLimit(5.0f);
  delay(50);
  motor.enable();
  running = true;

  Serial.println(F("CSP モードで起動。コマンド: c<rad> p<rad> v<rad/s> s=停止 e=再開 r=状態"));
  showState();
}

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();
    String rest = Serial.readStringUntil('\n');
    float val = rest.toFloat();
    switch (c) {
      case 'c':
        switchMode(EL05_MODE_CSP, "CSP");
        motor.setPositionCSP(val);
        Serial.printf("CSP 目標位置 -> %.3f rad\n",
                      RobStrideEduLite::constrainFloat(val, EL05_P_MIN, EL05_P_MAX));
        break;
      case 'p':
        switchMode(EL05_MODE_PP, "PP");
        motor.setPPProfile(5.0f, 10.0f);   // 速度5 rad/s, 加速度10 rad/s^2
        motor.setPositionPP(val);
        Serial.printf("PP 目標位置 -> %.3f rad\n",
                      RobStrideEduLite::constrainFloat(val, EL05_P_MIN, EL05_P_MAX));
        break;
      case 'v':
        motor.setCSPSpeedLimit(val);
        Serial.printf("速度制限 -> %.2f rad/s\n",
                      RobStrideEduLite::constrainFloat(val, 0, EL05_V_MAX));
        break;
      case 's':
        motor.disable();
        running = false;
        Serial.println(F("停止しました"));
        break;
      case 'e':
        motor.enable();
        running = true;
        Serial.println(F("再開しました"));
        break;
      case 'r':
        showState();
        break;
    }
  }

  // 500ms ごとに状態表示
  static uint32_t last = 0;
  if (running && millis() - last >= 500) {
    last = millis();
    showState();
  }
}
