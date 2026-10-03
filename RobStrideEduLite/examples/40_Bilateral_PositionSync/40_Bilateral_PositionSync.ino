/*
 * 40_Bilateral_PositionSync.ino — バイラテラル制御 (位置対位置型)
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * 2 台のモーターの位置を相互に同期させます。
 *
 * 動作:
 *   マスター(1台目)の軸を手で回すと、スレーブ(2台目)が同じ角度に追従します。
 *   逆にスレーブを回そうとすると、マスターにも影響が伝わります(双方向)。
 *
 * 原理:
 *   マスター位置 → スレーブの目標位置  (マスターに追従)
 *   スレーブ位置 → マスターの目標位置  (スレーブに追従)
 *   を制御周期ごとに交互に行い、両者を「仮想的な硬い軸」で繋いだように見せます。
 *
 * 前提:
 *   - モーター 2 台を CAN バスに並列接続、異なる CAN_ID
 *   - 両方に 48V 供給
 *   - 両方とも無負荷または軽い負荷で、軸が手で回せる状態
 *
 * 調整パラメータ:
 *   KP_SYNC … 同期の硬さ (大きいほど強く追従、振動したら下げる)
 *   KD_SYNC … ダンピング (振動抑制)
 *
 * 注意: 同期ゲインが高すぎると振動・発振します。まず小さめから。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float KP_SYNC = 10.0f;    // 同期の硬さ (まず小さめから)
const float KD_SYNC = 0.5f;     // ダンピング
const uint32_t CTRL_MS = 5;     // 制御周期 200Hz (同期は速いほど滑らか)

RobStrideEduLite master(1);   // 1台目 = マスター
RobStrideEduLite slave(1);    // 2台目 = スレーブ (ID はスキャンで自動設定)
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== バイラテラル制御 (位置同期型) ==="));

  if (!master.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) delay(1000);
  }
  slave.begin(CAN_RX_PIN, CAN_TX_PIN);   // 共有バス

  // 2 台検出
  uint8_t found[8];
  uint8_t n = master.scanBus(found, 8, 5);
  if (n < 2) {
    Serial.printf("モーターが %u 台しか見つかりません (2台必要)\n", n);
    Serial.println(F("両台の CAN_ID を 05_MotorSetup で異なる値に設定してください"));
    while (1) delay(1000);
  }
  master.setMotorId(found[0]);
  slave.setMotorId(found[1]);
  Serial.printf("マスター ID=%u, スレーブ ID=%u\n", found[0], found[1]);

  // 両方を MIT モードで有効化
  master.disable(true);
  slave.disable(true);
  delay(100);
  master.setRunMode(EL05_MODE_MIT);
  slave.setRunMode(EL05_MODE_MIT);
  delay(50);
  master.enable();
  slave.enable();
  ready = true;
  Serial.println(F("動作開始: マスターの軸を手で回すとスレーブが追従します"));
  Serial.printf("同期ゲイン: kp=%.1f kd=%.2f\n", KP_SYNC, KD_SYNC);
}

void loop() {
  if (!ready) return;
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < CTRL_MS) return;
  last = now;

  // 両方の現在位置を取得 (フィードバックは MIT 送信の応答で返る)
  // まず中立指令を送って最新のフィードバックを得る
  master.setMIT(0, 0, 0, 0, 0);
  EL05Feedback fm = master.readFeedback(3);

  slave.setMIT(0, 0, 0, 0, 0);
  EL05Feedback fs = slave.readFeedback(3);

  if (!fm.valid || !fs.valid) return;   // 両方取れた時だけ制御

  // 位置誤差
  float err = fm.positionRad - fs.positionRad;

  // 相互に誤差を縮める方向へトルクを掛ける
  // スレーブはマスター位置へ、マスターはスレーブ位置へ (双方向で「硬い軸」感)
  slave.setMIT(fm.positionRad, 0, KP_SYNC, KD_SYNC, 0);
  master.setMIT(fs.positionRad, 0, KP_SYNC, KD_SYNC, 0);

  // 1秒ごとに状態表示
  static int cnt = 0;
  if (++cnt >= 200) {
    cnt = 0;
    Serial.printf("マスター=%+.2f rad  スレーブ=%+.2f rad  誤差=%+.3f rad\n",
                  fm.positionRad, fs.positionRad, err);
  }
}
