/*
 * 00_Diagnose.ino — 「応答なし」の原因切り分け用診断スケッチ
 *
 * 内容:
 *   1. TWAI 初期化
 *   2. 全 CAN_ID (1..127) にデバイスID取得(通信タイプ0)を送信してスキャン
 *      → モーターの実際の ID が分かる
 *   3. TWAI の送信結果・エラーカウンタ(TEC/REC)・アラートを表示
 *      → 送信自体が失敗しているか(=配線問題)を判定できる
 *
 * 結果の見方:
 *   [A] 「送信失敗」が全IDで発生 / TEC が増加し続ける
 *       → バスに応答ノードが無い。CANH/CANL 配線・終端抵抗・
 *         トランシーバー電源・モーター48V電源・GND共通を確認。
 *         GPIO4/GPIO5 の RX/TX 取り違えの可能性もある(下記の
 *         CAN_RX_PIN/CAN_TX_PIN を入れ替えて再試行)。
 *   [B] 送信は成功するがモーターが見つからない
 *       → ボーレート不一致またはモーター側の問題。
 *         モーターが 500K 等に変更されていないか、48V 供給と
 *         起動待ち(数秒)を確認。
 *   [C] ID が見つかった
 *       → その ID を他のスケッチの MOTOR_ID に設定する。
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4   // SN65HVD230 RXD 側
#define CAN_TX_PIN 5   // SN65HVD230 CTX 側
                       // ※配線に自信がなければ 4 と 5 を入れ替えても試す

RobStrideEduLite motor(1);  // スキャンには影響しない

void printStatus() {
  twai_status_info_t st;
  if (motor.getStatus(st)) {
    Serial.printf("TWAI state=%d (0=停止 1=動作 2=BUS-OFF)  TEC=%lu REC=%lu  TX失敗=%lu RX取りこぼし=%lu\n",
                  (int)st.state,
                  (unsigned long)st.tx_error_counter,
                  (unsigned long)st.rx_error_counter,
                  (unsigned long)st.tx_failed_count,
                  (unsigned long)st.rx_missed_count);
  }
  uint32_t al = motor.getAlerts(0);
  if (al) {
    Serial.printf("アラート: 0x%04lX ", (unsigned long)al);
    if (al & TWAI_ALERT_TX_FAILED)     Serial.print("[TX失敗]");
    if (al & TWAI_ALERT_BUS_ERROR)     Serial.print("[バスエラー]");
    if (al & TWAI_ALERT_BUS_OFF)       Serial.print("[BUS-OFF]");
    if (al & TWAI_ALERT_ARB_LOST)      Serial.print("[調停敗北]");
    if (al & TWAI_ALERT_RX_QUEUE_FULL) Serial.print("[RXキュー満杯]");
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println(F("=== EL05 診断スキャン ==="));
  Serial.printf("RX=GPIO%d TX=GPIO%d, 1Mbps, 拡張フレーム\n", CAN_RX_PIN, CAN_TX_PIN);

  if (!motor.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) { delay(1000); }
  }
  Serial.println(F("TWAI 初期化 OK"));

  Serial.println(F("ID 1..127 をスキャン中 (約2〜3秒)..."));
  uint8_t found[16];
  uint8_t n = motor.scanBus(found, 16, 5);   // 各ID 5ms 待機 (2周で約3秒)

  Serial.println(F("--- 結果 ---"));
  if (n == 0) {
    Serial.println(F("モーターは見つかりませんでした。"));
  } else {
    Serial.printf("%d 台検出: ", n);
    for (uint8_t i = 0; i < n; i++) Serial.printf("%u ", found[i]);
    Serial.println();
    Serial.println(F(">>> 上記の ID を他スケッチの MOTOR_ID に設定してください。"));
  }
  printStatus();
  Serial.println(F("=== 診断終了 ==="));
}

void loop() {
  // 何もしない (再スキャンしたい場合はリセットボタン)
}
