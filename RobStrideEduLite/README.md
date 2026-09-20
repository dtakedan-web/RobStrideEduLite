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

## レンジ (EL05)

| 量 | 範囲 |
|---|---|
| 位置 | ±4π rad (±12.57) |
| 速度 | ±50 rad/s |
| トルク | ±6 Nm (MIT) |
| Kp | 0 〜 500 |
| Kd | 0 〜 5 |
| Iq 指令 | ±11 A |

## 例スケッチ

1. **01_CheckConnection** — 最初に必ず実行。通信・フィードバック・バス電圧の確認のみ (軸は動きません)
2. **02_MIT_SinePosition** — MIT モードで正弦波位置制御。シリアルで kp/kd/振幅を変更可能
3. **03_VelocityMode** — 速度モード。シリアルで速度・加速度を変更可能

## 安全上の注意

- **必ず無負荷・固定された状態で最初のテスト**を行い、kp は小さい値 (10〜30) から上げてください。
- MIT モードで kd だけを与えるダンピング運転ではモーターが**発電**します。電源の過電圧保護に注意してください (マニュアル記載)。
- モード切替は必ず停止状態で行ってください。
- 故障 (過温度 135℃、ロック過負荷、低電圧など) はフィードバック ID の fault ビットに出ます。`disable(true)` でクリアできます。
- トルク制限・保護温度などの工場パラメータは変更しないでください (マニュアル警告)。
