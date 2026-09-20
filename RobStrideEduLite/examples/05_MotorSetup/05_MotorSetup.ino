/*
 * 05_MotorSetup.ino — モーター設定ユーティリティ (対話式・ID自動検出)
 *
 * 接続するモーターが 1 台だけという前提の整備用スケッチ。
 * 起動時にバススキャンを実行し、モーターを自動検出してから操作する。
 * CAN_ID の事前知識は不要。
 *
 * 起動シーケンス:
 *   1. バススキャン (00_Diagnose と同じ)
 *   2. モーターが 1 台見つかる → その ID で操作モードへ
 *      見つからない          → 'r' で再スキャン
 *      複数見つかる          → 手動で ID を選択
 *
 * 操作モードのコマンド (115200bps, 改行付き):
 *   i        : モーター情報を表示 (位置/温度/バス電圧/run_mode/バージョン)
 *   n<ID>    : CAN_ID を変更 (1~127)     例) n5
 *   z        : 現在位置を機械原点(0)に設定 (2度押し確認)
 *   f        : 故障・警告情報を読み出す
 *   s        : モーター停止
 *   r        : バスを再スキャンしてモーターを選び直す
 *
 * 重要:
 *   - CAN_ID 変更と機械原点設定は、どちらも実行した時点で不揮発メモリに
 *     保存され、電源を切っても保持される (実測で確認済み)
 *   - ID 変更後はこのスケッチが自動で新しい ID を追跡する (再スキャン不要)
 *   - ゼロ点設定はモーターが動かないことを確認してから実行
 *   - なお制御パラメータ (Kp/制限値など, タイプ18書き込み) を永続化する場合の
 *     保存コマンド (タイプ22) は本スケッチでは扱わない。必要なら README 参照
 */

#include <RobStrideEduLite.h>

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

RobStrideEduLite motor(1);   // 初期値はダミー。スキャン後に正しいIDへ差し替わる
bool motorReady = false;     // モーターが選択済みか

// ---------------------------------------------------------------
// バススキャンしてモーターを選択
// 戻り値: true = 1台に確定, false = 未確定
// ---------------------------------------------------------------
bool scanAndSelect() {
  Serial.println(F("バスをスキャン中 (約2〜3秒)..."));
  uint8_t found[16];
  uint8_t n = motor.scanBus(found, 16, 5);   // 各ID 5ms 待機 (2周で約3秒)

  if (n == 0) {
    Serial.println(F("モーターが見つかりませんでした。配線と48V電源を確認し、'r' で再スキャンしてください。"));
    return false;
  }
  if (n == 1) {
    motor.setMotorId(found[0]);
    Serial.printf("モーターを検出: CAN_ID = %u\n", found[0]);
    return true;
  }
  // 複数台: 手動選択
  Serial.printf("%d 台見つかりました: ", n);
  for (uint8_t i = 0; i < n; i++) Serial.printf("%u ", found[i]);
  Serial.println();
  Serial.println(F("このスケッチは1台前提です。操作する ID を 'u<ID>' で指定してください (例: u127)"));
  Serial.println(F("または余分なモーターを外して 'r' で再スキャン。"));
  return false;
}

// ---------------------------------------------------------------
// モーター情報表示
// ---------------------------------------------------------------
void showInfo() {
  if (!motorReady) { Serial.println(F("先にモーターを検出してください ('r' で再スキャン)")); return; }
  float pos, temp, vbus;
  uint8_t rm[4];
  char ver[24];
  Serial.println(F("--- モーター情報 ---"));
  Serial.printf("CAN_ID: %u\n", motor.motorId());
  if (motor.getPosition(pos))        Serial.printf("位置:   %+.3f rad\n", pos);
  else                               Serial.println(F("位置:   取得失敗"));
  if (motor.getTemperature(temp))    Serial.printf("温度:   %.1f C\n", temp);
  if (motor.getBusVoltage(vbus))     Serial.printf("電圧:   %.1f V\n", vbus);
  if (motor.readParam(EL05_IDX_RUN_MODE, rm)) {
    Serial.printf("run_mode: %u (0=MIT 1=PP 2=速度 3=電流 5=CSP)\n", rm[0]);
  }
  if (motor.readVersion(ver, sizeof(ver))) {
    Serial.printf("ファーム: %s\n", ver);
  }
  Serial.println(F("--------------------"));
}

// 故障・警告情報の表示
void showFault() {
  if (!motorReady) { Serial.println(F("先にモーターを検出してください")); return; }
  EL05Fault flt;
  // 故障フィードバックは自発送信。短めにポーリングして拾う
  Serial.println(F("故障情報を待機中 (500ms)..."));
  if (motor.readFault(flt, 500) && flt.valid) {
    Serial.printf("CAN_ID %u の故障情報:\n", flt.motorId);
    if (!flt.hasFault && !flt.hasWarning) {
      Serial.println(F("  正常 (故障・警告なし)"));
      return;
    }
    if (flt.hasFault) {
      Serial.printf("  故障値: 0x%08lX\n", (unsigned long)flt.faultValue);
      if (flt.overtemperature())  Serial.println(F("    - モーター過温度"));
      if (flt.driverChip())       Serial.println(F("    - ドライバチップ故障"));
      if (flt.undervoltage())     Serial.println(F("    - 低電圧"));
      if (flt.overvoltage())      Serial.println(F("    - 過電圧"));
      if (flt.overcurrentA())     Serial.println(F("    - A相過電流"));
      if (flt.overcurrentB())     Serial.println(F("    - B相過電流"));
      if (flt.overcurrentC())     Serial.println(F("    - C相過電流"));
      if (flt.encUncalibrated())  Serial.println(F("    - エンコーダ未キャリブ"));
      if (flt.hwIdFault())        Serial.println(F("    - ハード識別故障"));
      if (flt.posInitFault())     Serial.println(F("    - 位置初期化故障"));
      if (flt.stallOverload())    Serial.println(F("    - ロック過負荷保護"));
    }
    if (flt.hasWarning) {
      Serial.printf("  警告値: 0x%08lX\n", (unsigned long)flt.warnValue);
    }
  } else {
    Serial.println(F("  故障フィードバックを受信できませんでした (故障が無い場合は送られません)"));
  }
}

void printHelp() {
  Serial.println(F("コマンド: i=情報 n<ID>=ID変更 z=原点設定 f=故障情報 s=停止 r=再スキャン"));
}

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println(F("=== EL05 モーター設定ユーティリティ (ID自動検出) ==="));

  if (!motor.begin(CAN_RX_PIN, CAN_TX_PIN)) {
    Serial.println(F("TWAI 初期化失敗"));
    while (1) { delay(1000); }
  }

  motorReady = scanAndSelect();
  if (motorReady) {
    // 安全のため停止状態にしてから情報表示
    motor.disable(true);
    delay(100);
    showInfo();
    printHelp();
  }
}

// ---------------------------------------------------------------
void loop() {
  if (!Serial.available()) return;
  char c = (char)Serial.read();
  String rest = Serial.readStringUntil('\n');
  rest.trim();

  // 再スキャンはいつでも可
  if (c == 'r') {
    motorReady = scanAndSelect();
    if (motorReady) { motor.disable(true); delay(100); showInfo(); printHelp(); }
    return;
  }

  // 複数台時の手動選択: u<ID>
  if (c == 'u') {
    int id = rest.toInt();
    if (id < 1 || id > 127) { Serial.println(F("ID は 1〜127")); return; }
    motor.setMotorId((uint8_t)id);
    motorReady = true;
    Serial.printf("ID %u を選択しました\n", id);
    motor.disable(true); delay(100);
    showInfo(); printHelp();
    return;
  }

  // 以降はモーター確定後のみ
  if (!motorReady) {
    Serial.println(F("モーターが未検出です。'r' で再スキャンしてください。"));
    return;
  }

  switch (c) {
    case 'i':
      showInfo();
      break;

    case 'f':
      showFault();
      break;

    case 'n': {
      int newId = rest.toInt();
      if (newId < 1 || newId > 127) {
        Serial.println(F("ID は 1〜127 の範囲で指定してください"));
        break;
      }
      Serial.printf("CAN_ID を %u -> %d に変更します...\n", motor.motorId(), newId);
      if (motor.setCanId((uint8_t)newId)) {
        Serial.println(F("変更しました (不揮発メモリに保存され、電源を切っても保持されます)。"));
        Serial.println(F("このスケッチは新しいIDを自動追跡します。"));
        showInfo();
      } else {
        Serial.println(F("変更に失敗しました"));
      }
      break;
    }

    case 'z':
      Serial.println(F("現在位置を機械原点(0)に設定します。モーターは停止していますか?"));
      Serial.println(F("実行するには5秒以内にもう一度 'z' を送ってください"));
      {
        uint32_t t = millis();
        bool confirmed = false;
        while (millis() - t < 5000) {
          if (Serial.available() && (char)Serial.read() == 'z') { confirmed = true; break; }
        }
        if (confirmed) {
          if (motor.setZeroPosition()) {
            Serial.println(F("原点を設定しました (不揮発メモリに保存されます)。"));
          } else {
            Serial.println(F("原点設定に失敗しました"));
          }
        } else {
          Serial.println(F("キャンセルしました"));
        }
      }
      break;

    case 's':
      motor.disable();
      Serial.println(F("停止しました"));
      break;

    default:
      printHelp();
      break;
  }
}
