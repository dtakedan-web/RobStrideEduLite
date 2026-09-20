/*
 * 05_MotorSetup.ino — モーター設定ユーティリティ (対話式)
 *
 * CAN_ID 変更・機械原点設定・パラメータ保存をシリアルから安全に実行する整備用スケッチ。
 * モーターは「停止状態」で実行してください。
 *
 * シリアルコマンド (115200bps, 改行付き):
 *   i        : 現在のモーター情報を表示 (位置/温度/バス電圧/run_mode)
 *   n<ID>    : CAN_ID を変更 (1~127)     例) n5  → ID を 5 に変更
 *   z        : 現在位置を機械原点(0)に設定
 *   w        : パラメータを不揮発メモリに保存 (変更を永続化)
 *   s        : モーター停止
 *
 * 重要:
 *   - CAN_ID 変更は即時有効ですが、永続化するには w (保存) が必要です
 *   - ID 変更後は MOTOR_ID 定義も新しい ID に合わせてください
 *   - ゼロ点設定はモーターが動かないことを確認してから実行
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5
#define MOTOR_ID   127   // 現在のモーターID

RobStrideEduLite motor(MOTOR_ID);

void showInfo() {
  float pos, temp, vbus;
  uint8_t rm[4];
  Serial.println(F("--- モーター情報 ---"));
  Serial.printf("CAN_ID: %u\n", motor.motorId());
  if (motor.getPosition(pos))  Serial.printf("位置:   %+.3f rad\n", pos);
  if (motor.getTemperature(temp)) Serial.printf("温度:   %.1f C\n", temp);
  if (motor.getBusVoltage(vbus))  Serial.printf("電圧:   %.1f V\n", vbus);
  if (motor.readParam(EL05_IDX_RUN_MODE, rm)) {
    Serial.printf("run_mode: %u (0=MIT 1=PP 2=速度 3=電流 5=CSP)\n", rm[0]);
  }
  Serial.println(F("--------------------"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println(F("=== EL05 モーター設定ユーティリティ ==="));

  if (!motor.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) { delay(1000); }
  }

  // 安全のため最初に停止状態にする
  motor.disable(true);
  delay(100);

  Serial.println(F("コマンド: i=情報 n<ID>=ID変更 z=原点設定 w=保存 s=停止"));
  showInfo();
}

void loop() {
  if (!Serial.available()) return;
  char c = (char)Serial.read();
  String rest = Serial.readStringUntil('\n');
  rest.trim();

  switch (c) {
    case 'i':
      showInfo();
      break;

    case 'n': {
      int newId = rest.toInt();
      if (newId < 1 || newId > 127) {
        Serial.println(F("ID は 1〜127 の範囲で指定してください"));
        break;
      }
      Serial.printf("CAN_ID を %u -> %d に変更します...\n", motor.motorId(), newId);
      if (motor.setCanId((uint8_t)newId)) {
        Serial.println(F("変更しました (即時有効)。"));
        Serial.println(F("※ 永続化するには 'w' で保存してください。次回から MOTOR_ID も変更してください。"));
      } else {
        Serial.println(F("変更に失敗しました"));
      }
      break;
    }

    case 'z':
      Serial.println(F("現在位置を機械原点(0)に設定します。モーターは停止していますか?"));
      Serial.println(F("実行するにはもう一度 'z' を送ってください"));
      // 2度押し確認
      {
        uint32_t t = millis();
        bool confirmed = false;
        while (millis() - t < 5000) {   // 5秒以内に再送で確定
          if (Serial.available() && (char)Serial.read() == 'z') { confirmed = true; break; }
        }
        if (confirmed) {
          if (motor.setZeroPosition()) Serial.println(F("原点を設定しました"));
          else Serial.println(F("原点設定に失敗しました"));
        } else {
          Serial.println(F("キャンセルしました"));
        }
      }
      break;

    case 'w':
      if (motor.saveParameters()) Serial.println(F("パラメータを保存しました (不揮発)"));
      else Serial.println(F("保存に失敗しました"));
      break;

    case 's':
      motor.disable();
      Serial.println(F("停止しました"));
      break;

    default:
      Serial.println(F("不明なコマンド。i=情報 n<ID>=ID変更 z=原点設定 w=保存 s=停止"));
      break;
  }
}
