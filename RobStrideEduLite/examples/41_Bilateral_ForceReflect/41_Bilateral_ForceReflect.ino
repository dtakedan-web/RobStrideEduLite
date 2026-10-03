/*
 * 41_Bilateral_ForceReflect.ino — バイラテラル制御 (力反射型)
 *
 * 書き込むだけで動きます (シリアルコマンド不要)。
 * スレーブが受けた力(トルク)をマスターに伝え、「手ごたえ」を返します。
 *
 * 動作:
 *   マスター(1台目)の軸を手で回すと、スレーブ(2台目)が追従します。
 *   スレーブの軸を手で押さえる(外力を加える)と、その抵抗がマスターの
 *   手に「硬さ・手ごたえ」として伝わります。
 *
 * 原理 (力反射型):
 *   位置同期:  マスター位置 → スレーブ目標位置 (スレーブが追従)
 *   力反射:    スレーブのトルク → マスターへ逆方向トルクとして加算
 *             (スレーブが外力を受けると、マスターにその力が返る)
 *
 * 前提:
 *   - モーター 2 台を CAN バスに並列接続、異なる CAN_ID
 *   - 両方に 48V 供給
 *   - 両方とも軸が手で回せる状態
 *
 * 調整パラメータ:
 *   KP_POS / KD_POS   … 位置追従のゲイン
 *   FORCE_GAIN        … 力反射の強さ (大きいほど手ごたえが強い)
 *
 * 注意:
 *   - 力反射のゲインが高すぎると振動・発振します。まず小さめから。
 *   - 強い力が掛かったらすぐ電源を切れるようにしてください。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ---- 調整パラメータ ----
const float KP_POS     = 15.0f;   // 位置追従ゲイン
const float KD_POS     = 0.5f;    // ダンピング
const float FORCE_GAIN = 0.3f;    // 力反射の強さ (0〜1 程度から)
const uint32_t CTRL_MS = 5;       // 制御周期 200Hz

RobStrideEduLite master(1);   // 1台目 = マスター (手で持つ側)
RobStrideEduLite slave(1);    // 2台目 = スレーブ (対象に触れる側)
bool ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== バイラテラル制御 (力反射型) ==="));

  if (!master.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) delay(1000);
  }
  slave.begin(CAN_RX_PIN, CAN_TX_PIN);

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

  master.disable(true);
  slave.disable(true);
  delay(100);
  master.setRunMode(EL05_MODE_MIT);
  slave.setRunMode(EL05_MODE_MIT);
  delay(50);
  master.enable();
  slave.enable();
  ready = true;
  Serial.println(F("動作開始: マスターを回すとスレーブが追従し、スレーブへの外力がマスターに返ります"));
  Serial.printf("位置 kp=%.1f kd=%.2f  力反射ゲイン=%.2f\n", KP_POS, KD_POS, FORCE_GAIN);
}

void loop() {
  if (!ready) return;
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < CTRL_MS) return;
  last = now;

  // 両方の最新状態を取得 (中立指令で応答を誘発)
  master.setMIT(0, 0, 0, 0, 0);
  EL05Feedback fm = master.readFeedback(3);
  slave.setMIT(0, 0, 0, 0, 0);
  EL05Feedback fs = slave.readFeedback(3);
  if (!fm.valid || !fs.valid) return;

  // ---- 位置同期: スレーブをマスター位置へ追従させる ----
  slave.setMIT(fm.positionRad, 0, KP_POS, KD_POS, 0);

  // ---- 力反射: スレーブのトルクをマスターへ逆方向に加える ----
  // スレーブが外力で押されるとトルクが発生する。それをマスターに返すと
  // マスターの手に「スレーブが何かに触れている」感触が伝わる。
  // マスターの目標は「スレーブ位置 + 力に応じたオフセット」。
  float forceOffset = -FORCE_GAIN * fs.torqueNm;
  master.setMIT(fs.positionRad + forceOffset, 0, KP_POS, KD_POS, 0);

  // 1秒ごとに状態表示
  static int cnt = 0;
  if (++cnt >= 200) {
    cnt = 0;
    Serial.printf("マスター=%+.2f rad  スレーブ=%+.2f rad  スレーブトルク=%+.2f Nm\n",
                  fm.positionRad, fs.positionRad, fs.torqueNm);
  }
}
