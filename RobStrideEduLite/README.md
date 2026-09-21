# RobStrideEduLite — RobStride EduLite 05 (EL05) Arduino ライブラリ

ESP32 (ESP32-KEY-R2A 等) + SN65HVD230 CAN トランシーバーで
RobStride EduLite 05 QDD アクチュエータを制御するための Arduino ライブラリです。

- 通信: CAN 2.0B / **1Mbps** / **拡張フレーム (29bit ID)** / RobStride プライベートプロトコル
- 依存: ESP32 Arduino コア内蔵の TWAI ドライバのみ (追加インストール不要)
- 対応モード: 運転制御 (MIT) / 位置 (CSP・PP) / 速度 / 電流

> 本ライブラリは RobStride EL05 User Manual の「Driver protocol and instructions」章に基づいて実装しています。
> 実機での検証はこれからです。動作がおかしい場合はマニュアルと照合してください。

## 配線

| ESP32-KEY-R2A | SN65HVD230 | EduLite 05 (XT30) |
|---|---|---|
| GPIO4 (CAN RX) | RXD | |
| GPIO5 (CAN TX) | CTX (D) | |
| 3.3V | VCC | |
| GND | GND | **電源 GND と共通接続** |
| | CANH | CANH (信号ピン) |
| | CANL | CANL (信号ピン) |
| | Rs(8ピン) → GND | (高速モード) |

- CAN バスの**両端に 120Ω の終端抵抗**を入れてください。
- モーター電源は **48V 別系統** (動作範囲 24〜60V)。ESP32 系に 48V が触れないよう十分注意。
- SN65HVD230 は **3.3V 版**です (5V 版と混同しないこと)。

## 使い方 (最小例)

```cpp
#include <RobStrideEduLite.h>

RobStrideEduLite motor(1);          // CAN_ID=1

void setup() {
  motor.begin(4, 5);                // RX=GPIO4, TX=GPIO5 (1Mbps)
  motor.enable();                   // 有効化 (通信タイプ3)
}

void loop() {
  motor.setMIT(1.0f, 0, 20.0f, 0.5f, 0);   // 目標1rad, kp=20, kd=0.5
  EL05Feedback fb = motor.readFeedback(20);
  if (fb.valid) {
    Serial.printf("pos=%.3f temp=%.1f\n", fb.positionRad, fb.tempC);
  }
  delay(10);                        // 100Hz 制御
}
```

### 複数台 (2 台) の場合

インスタンスを分けます。CAN バスの初期化は自動で 1 回だけ行われます (v0.3.1〜)。

```cpp
RobStrideEduLite motorA(1);   // CAN_ID=1
RobStrideEduLite motorB(2);   // CAN_ID=2

void setup() {
  motorA.begin(4, 5);   // ここでバス初期化
  motorB.begin(4, 5);   // 共有バスを使うだけ (再初期化しない)
  motorA.enable();
  motorB.enable();
}
```

## 主要 API

| メソッド | 内容 |
|---|---|
| `begin(rxPin, txPin)` | TWAI 初期化 (1Mbps 固定) |
| `enable() / disable(clearFault)` | モーター有効化 / 停止 (true で故障クリア) |
| `setMIT(pos, vel, kp, kd, tff)` | 運転制御モード指令 |
| `setRunMode(mode)` | モード切替 (MIT/PP/速度/電流/CSP)。**停止中に実行** |
| `setVelocity(radS)` | 速度モード指令 (±50 rad/s) |
| `setPositionCSP(rad) / setPositionPP(rad)` | 位置モード指令 (±4π rad) |
| `setCurrent(iqA)` | 電流モード指令 (±11 A) |
| `setVelocityAccel / setPPProfile / setCSPSpeedLimit / setCurrentLimit` | 各種制限・加減速 |
| `readFeedback(timeoutMs)` | フィードバック受信 (位置/速度/トルク/温度/故障) |
| `setActiveReport(on)` | 10ms 周期の自動フィードバック ON/OFF |
| `writeParamFloat / readParamFloat` 等 | 任意パラメータ読み書き (index はヘッダ参照) |
| `setZeroPosition()` | 現在位置を機械原点に設定 |
| `saveParameters()` | パラメータを不揮発保存 |
| `setCanId(newId)` | CAN_ID 変更 |

### ミドル関数 (v0.2.0〜)

個別に値を取りたいときの簡易版。取得失敗時は `false` を返します。

| メソッド | 内容 |
|---|---|
| `getPosition(rad)` | 現在位置 [rad] |
| `getVelocity(radS)` | 現在速度 [rad/s] |
| `getTorque(nm)` | 現在トルク [Nm] |
| `getTemperature(c)` | 現在温度 [℃] |
| `getBusVoltage(v)` | バス電圧 [V] |
| `setTorqueLimit(nm)` | トルク制限 (0〜6 Nm) |
| `clearFault()` | 故障クリア |

### 単位変換ミドル関数 (v0.3.2〜)

角度を **度(degree)**、速度を **rpm** で指定・取得できる版です。
既存の rad 版と同じ動作で、内部で換算します。

| メソッド | 内容 |
|---|---|
| `setMIT_deg(deg, rpm, kp, kd, tff)` | MIT の目標位置を度・速度を rpm で指定 |
| `setVelocity_rpm(rpm)` | 速度モードの速度を rpm で指定 |
| `setPositionCSP_deg(deg)` | 位置モード(CSP)の角度を度で指定 |
| `setPositionPP_deg(deg)` | 位置モード(PP)の角度を度で指定 |
| `setPPProfile_rpm(velRpm, accRpmS)` | PP の速度[rpm]/加速度[rpm/s] |
| `setCSPSpeedLimit_rpm(rpm)` | CSP の速度制限を rpm で指定 |
| `getPosition_deg(deg)` | 現在位置を度で取得 |
| `getVelocity_rpm(rpm)` | 現在速度を rpmで取得 |

換算定数も使えます: `EL05_RAD2DEG` / `EL05_DEG2RAD` / `EL05_RPS2RPM` / `EL05_RPM2RPS`

参考: `度 = rad × 180/π`、`rpm = rad/s × 60/(2π)` (1 rad/s ≈ 9.55 rpm)

### 診断 (v0.1.1〜)

| メソッド | 内容 |
|---|---|
| `scanBus(foundIds, max, timeoutMs)` | 全 ID をスキャンして接続中のモーターを検出 |
| `getStatus(info)` | TWAI ステータス (TEC/REC/エラー数) |
| `getAlerts(timeoutMs)` | アラート (TX失敗/バスエラー等) |

### デバイス情報 (v0.2.4〜)

| メソッド | 内容 |
|---|---|
| `readVersion(buf, len)` | ファームウェアのバージョン番号を読み出す |
| `readFault(fault, timeoutMs)` | 故障・警告情報 (通信タイプ21) を読み出す。`EL05Fault` に詳細ビットを格納 |

### 意図的に実装していないもの

以下は誤操作で通信不能になるリスクがあるため実装していません。

- ボーレート変更 (タイプ23) — 変更後は ESP32 側も合わせる必要がある
- プロトコル変更 (タイプ25) — 変更後はプライベートプロトコルで通信できなくなる

## レンジ (EL05)

| 量 | 範囲 |
|---|---|
| 位置 | ±4π rad (±12.57) |
| 速度 | ±50 rad/s |
| トルク | ±6 Nm (MIT) |
| Kp | 0 〜 500 |
| Kd | 0 〜 5 |
| Iq 指令 | ±11 A |

## 永続性について(実測に基づく)

| 操作 | 永続性 |
|---|---|
| CAN_ID 変更 (`setCanId` / タイプ7) | **即座に不揮発へ保存**。電源を切っても保持 |
| 機械原点設定 (`setZeroPosition` / タイプ6) | **即座に不揮発へ保存**。電源を切っても保持 |
| 制御パラメータ書き込み (`writeParamXxx` / タイプ18) | 揮発。永続化には `saveParameters()`(タイプ22)が必要 |

つまり **ID 変更や原点設定に `saveParameters()` は不要**です。`saveParameters()` は
Kp/制限値などの制御パラメータをタイプ18で変更した場合の保存用です。

## 例スケッチ

### コマンド対話式 (設定・診断用)

1. **00_Diagnose** — 診断スキャン。接続中モーターの ID 検出とバス状態表示 (配線確認用)
2. **01_CheckConnection** — 最初に必ず実行。通信・フィードバック・バス電圧の確認のみ (軸は動きません)
3. **02_MIT_SinePosition** — MIT モードで正弦波位置制御。シリアルで kp/kd/振幅を変更可能
4. **03_VelocityMode** — 速度モード。シリアルで速度・加速度を変更可能
5. **04_PositionMode** — 位置モード (CSP / PP)。目標位置への移動と速度制限
6. **05_MotorSetup** — 設定ユーティリティ。起動時にモーター ID を自動検出し、CAN_ID 変更・原点設定・故障情報表示を対話式で実行 (モーター1台前提)

### 動作サンプル (run_mode 別・書き込むだけで動く)

シリアルコマンド不要。起動時にモーターを自動検出し、そのまま動作します。
調整はスケッチ冒頭の定数を書き換えます。**必ず無負荷・固定した状態で実行してください。**

10. **10_RunMode_MIT** — 運転制御(MIT)モード。正弦波で軸が往復
11. **11_RunMode_PositionPP** — 位置モード (PP)。台形加減速で2点を往復
12. **12_RunMode_Velocity** — 速度モード。正転・反転を繰り返す
13. **13_RunMode_Current** — 電流モード。Iq 電流(トルク)を正負に指令 ※回転が上がるので注意
14. **14_RunMode_PositionCSP** — 位置モード (CSP)。速度制限付きで滑らかに2点を往復

### 複数台制御 (2 台・書き込むだけで動く)

CAN バスに 2 台を並列接続し、各モーターに異なる CAN_ID を設定してください
(ID は 05_MotorSetup で変更できます)。起動時に 2 台を自動検出します。
ライブラリは複数インスタンスで CAN バスを共有します (v0.3.1〜)。

20. **20_MultiMotor_Sync** — 2 台を MIT モードで同期。逆位相の正弦波で往復
21. **21_MultiMotor_Independent** — 2 台を独立制御。1台目=速度回転 / 2台目=位置往復

### 単位変換 (degree / rpm)

30. **30_Units_DegreeRpm** — 角度を度・速度を rpm で指定するミドル関数の使用例。速度モードと位置モードで動作

## 安全上の注意

- **必ず無負荷・固定された状態で最初のテスト**を行い、kp は小さい値 (10〜30) から上げてください。
- MIT モードで kd だけを与えるダンピング運転ではモーターが**発電**します。電源の過電圧保護に注意してください (マニュアル記載)。
- モード切替は必ず停止状態で行ってください。
- 故障 (過温度 135℃、ロック過負荷、低電圧など) はフィードバック ID の fault ビットに出ます。`disable(true)` でクリアできます。
- トルク制限・保護温度などの工場パラメータは変更しないでください (マニュアル警告)。
