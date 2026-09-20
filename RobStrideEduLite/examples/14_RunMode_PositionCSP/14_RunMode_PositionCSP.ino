/*
 * 14_RunMode_PositionCSP.ino — 位置モード (CSP) の動作サンプル
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * CSP (Cyclic Sync Position) モードは速度制限付きで滑らかに目標位置へ追従します。
 *
 * 動作: 0 → +90° → -90° → 0 … を滑らかに繰り返します。
 *       PP モードより追従が滑らかで、連続した位置指令に向きます。
 *
 * 調整パラメータ:
 *   POS_A_RAD / POS_B_RAD … 行き先の2点 [rad]
 *   SPEED_LIMIT … 速度制限 [rad/s]
 *   WAIT_MS     … 各点での待機時間 [ms]
 *
 * 注意: 必ず無負荷・固定した状態で実行してください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float POS_A_RAD   =  PI / 2.0f;  // +90°
const float POS_B_RAD   = -PI / 2.0f;  // -90°
const float SPEED_LIMIT =  5.0f;       // 速度制限 5 rad/s
const uint32_t WAIT_MS  = 2000;        // 各点で 2秒待機

RobStrideEduLite motor(1);
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== 位置モード (CSP) サンプル ==="));

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

  // CSP モードに切り替え
  motor.disable(true);
  delay(100);
  motor.setRunMode(EL05_MODE_CSP);
  motor.setCSPSpeedLimit(SPEED_LIMIT);
  delay(50);
  motor.enable();
  ready = true;
  Serial.println(F("動作開始: +90° と -90° を滑らかに往復します"));
}

void loop() {
  if (!ready) return;

  motor.setPositionCSP(POS_A_RAD);
  Serial.printf("-> 目標 %+.2f rad\n", POS_A_RAD);
  delay(WAIT_MS);

  motor.setPositionCSP(POS_B_RAD);
  Serial.printf("-> 目標 %+.2f rad\n", POS_B_RAD);
  delay(WAIT_MS);

  motor.setPositionCSP(0.0f);
  Serial.println(F("-> 目標 0 rad"));
  delay(WAIT_MS);
}
