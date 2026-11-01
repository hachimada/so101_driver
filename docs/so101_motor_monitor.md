# モーター状態の見方

スライダーGUIの「Motor status」で、Feetech公式Python SDKから取得した情報を確認できます。
制御開始前も表示します。関節の行を選ぶと、そのモーターの詳細を表示します。

## 一覧の項目

| 表示 | 意味 |
|---|---|
| ID | サーボのモーターID（1〜6） |
| Joint | スライダーと共通の関節名 |
| Torque | サーボの出力状態。OFF / ON / Damping（ダンピング） |
| Motion | サーボが報告する動作状態。Stopped（停止中） / Moving（移動中） |
| Speed (rpm) | モーターの回転速度 |
| Current (mA) | モーターの相電流 |
| Duty (%) | デューティー比。PWMのON時間 ÷ 1周期の時間 × 100 |
| Voltage (V) | サーボが報告する電圧 |
| Temp (°C) | サーボ内部の温度 |
| Status | OK（正常）、異常の内容、Communication failed（通信失敗）、Stale（更新停止）、No data（未取得） |

電流・デューティー比・速度の符号はサーボ側の方向を表します。
デューティー比の100%は、PWMの1周期全体がONである状態です。
「Stopped」はサーボ内の目標に対する動作状態なので、手で動かしていても表示される場合があります。

電流フィードバックに対応しない設定の機体は、電流を「N/A」と表示します。
速度の換算に必要な設定が未取得の場合は、rpmを「—」と表示します。

## 関節ごとの詳細

選択した関節について、次の情報を確認できます。設定値の表示は読み取り専用です。

- 現在角度、現在位置とサーボ内の目標位置のtick値
- 電流・電圧・速度の生値、出力上限、異常ビット
- モデル、ファームウェア、動作モード
- 位置範囲、温度・電圧・電流の保護設定
- 位置PIDなどの設定値

サーボ内の目標位置は、そのサーボに送信した位置目標です。
指令と状態表示の更新時刻は異なるため、目標変更直後は表示が遅れる場合があります。

## 異常・取得失敗の表示

電圧（Voltage）、エンコーダー（Encoder）、過熱（Overheat）、過電流（Overcurrent）、過負荷（Overload）の異常は、赤字で表示します。
通信失敗時や更新停止時は、数値を「—」にします。取得が復旧すると表示も戻ります。
画面には位置取得失敗・状態取得失敗・指令拒否の累計回数も表示します。

状態表示が復旧しても、停止した制御は自動再開しません。
原因を解消し、GUIの「Start control」から再開してください。

## 更新頻度

`~/.ros/so101_driver/config/follower.yaml`の`motor_state_rate_hz`で、1軸あたりの更新頻度を指定します。
既定は2 Hzで、0〜10 Hzを指定できます。`0.0`にすると状態監視を無効にします。
モデル情報や保護設定は約10秒ごとに更新します。

```yaml
/follower/so101_follower:
  ros__parameters:
    motor_state_rate_hz: 2.0
```

## ROSトピック

| トピック | 型 | 内容 |
|---|---|---|
| `/follower/motor_states` | diagnostic_msgs/msg/DiagnosticArray | 1回につき1軸の状態 |

`DiagnosticStatus.name`は`/follower/so101_follower/motor/<joint_name>`、
`hardware_id`はデバイスIDとモーターIDです。
主な`values`のキーは次のとおりです。

| キー | 内容 |
|---|---|
| `sample_valid` | 状態取得の成否 |
| `current_ma` / `voltage_v` / `temperature_c` | 電流（mA）・電圧（V）・温度（℃） |
| `position_ticks` / `speed_raw` | 位置・速度の生値 |
| `packet_error` / `servo_status` | 応答パケット・サーボの異常ビット |
| `rate_hz` / `read_failures` | 更新頻度・その軸の状態取得失敗回数 |

## 単位の換算と出典

電流は符号変換後の値に6.5 mA、電圧は0.1 V、デューティー比は0.1%を掛けて表示します。
速度はサーボの相位設定に応じ、1単位を0.0146 rpmまたは0.732 rpmとして換算します。

仕様は[Feetech公式STSメモリ表](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/)と
[STS3215のフィードバック説明](https://www.feetechrc.com/20210430-56680.html)を参照してください。
使用するSDKは[Feetech公式SDKの取得と出典](feetech_sdk_source.md)に記載しています。
