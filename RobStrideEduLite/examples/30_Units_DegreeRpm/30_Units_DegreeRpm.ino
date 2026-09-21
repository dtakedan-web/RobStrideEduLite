/*
 * 30_Units_DegreeRpm.ino — degree / rpm 単位のミドル関数サンプル
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * 角度を「度(degree)」、速度を「rpm(回転/分)」で直感的に指定できる
 * ミドル関数の使用例です。
 *
 * 動作 (2 つのモードを交互に実行):
 *   [速度モード] +60rpm で 3秒 → -60rpm で 3秒 (1分あたり60回転 = 1秒で1回転)
 *   [位置モード] +90° → -90° → 0° を rpm 指定の速度で往復
 *
 * 使っている単位変換関数:
 *   setVelocity_rpm(rpm)      … 速度モードの速度を rpm で指定
 *   setPositionCSP_deg(deg)   … 位置モード(CSP)の角度を度で指定
 *   setCSPSpeedLimit_rpm(rpm) … CSP の速度制限を rpm で指定
 *   setMIT_deg(deg, rpm, …)   … MIT の目標位置を度・目標速度を rpm で指定
 *   getPosition_deg(deg)      … 現在位置を度で取得
 *   getVelocity_rpm(rpm)      … 現在速度を rpm で取得
 *
 * 参考: rad と度、rad/s と rpm の換算
 *   度  = rad × (180/π)   例) π rad = 180°
 *   rpm = rad/s × 60/(2π) 例) 1 rad/s ≈ 9.55 rpm
 *
 * 注意: 必ず無負荷・固定した状態で実行してください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ (度 / rpm で指定) ----
const float SPEED_RPM  = 60.0f;   // 速度 60 rpm (= 1秒で1回転)
const float POS_DEG    = 90.0f;   // 位置 ±90°
const float CSP_LIMIT_RPM = 30.0f;  // CSP 速度制限 30 rpm

RobStrideEduLite motor(1);
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== degree / rpm 単位サンプル ==="));

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
  motor.disable(true);
  delay(100);
  ready = true;
}

// 現在の位置(度)と速度(rpm)を表示
void showState() {
  float deg, rpm;
  if (motor.getPosition_deg(deg) && motor.getVelocity_rpm(rpm)) {
    Serial.printf("  現在: 位置=%+.1f° 速度=%+.1f rpm\n", deg, rpm);
  }
}

void loop() {
  if (!ready) return;

  // ---- 1. 速度モード (rpm 指定) ----
  Serial.println(F("--- 速度モード (rpm 指定) ---"));
  motor.disable();
  delay(100);
  motor.setRunMode(EL05_MODE_VELOCITY);
  motor.setVelocityAccel(30.0f);
  motor.setCurrentLimit(5.0f);
  delay(50);
  motor.enable();

  Serial.printf("正転 %+.0f rpm\n", SPEED_RPM);
  motor.setVelocity_rpm(SPEED_RPM);          // rpm で指定
  delay(1500); showState();
  delay(1500);

  Serial.printf("反転 %+.0f rpm\n", -SPEED_RPM);
  motor.setVelocity_rpm(-SPEED_RPM);         // 反転
  delay(1500); showState();
  delay(1500);

  motor.setVelocity_rpm(0);                  // 停止
  delay(1000);

  // ---- 2. 位置モード (度 指定) ----
  Serial.println(F("--- 位置モード CSP (度 指定) ---"));
  motor.disable();
  delay(100);
  motor.setRunMode(EL05_MODE_CSP);
  motor.setCSPSpeedLimit_rpm(CSP_LIMIT_RPM); // 速度制限を rpm で指定
  delay(50);
  motor.enable();

  Serial.printf("-> %+.0f°\n", POS_DEG);
  motor.setPositionCSP_deg(POS_DEG);         // 度で指定
  delay(2000); showState();

  Serial.printf("-> %+.0f°\n", -POS_DEG);
  motor.setPositionCSP_deg(-POS_DEG);
  delay(2000); showState();

  Serial.println(F("-> 0°"));
  motor.setPositionCSP_deg(0);
  delay(2000); showState();

  Serial.println();
}
