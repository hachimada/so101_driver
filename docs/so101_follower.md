# フォロワー制御インターフェース

ROSトピックで6軸の位置を指令し、サービスで制御の開始・停止とトルク解除を行います。
[スライダーGUI](so101_gui.md)と[テレオペ](so101_teleop.md)も、このインターフェースを使用します。

## 起動

[機体の設定](so101_setup.md)と[キャリブレーション](so101_calibration.md)を済ませてから実行します。

```bash
ros2 launch so101_driver follower_control.launch.py allow_writes:=true
```

`allow_writes:=true`で制御を許可します。省略時は状態取得のみです。
実際の制御は、[開始サービス](#制御の開始停止)を呼んでから有効になります。

1台のアームのUSBポートは、同時に1つのプロセスだけが使えます。
同じアームをほかのノードや`so101_calibrate`が使用中の場合は、`is in use by another process`と表示して起動を中止します。
スライダーGUIとテレオペもこのノードを起動するため、同じアームで同時には起動できません。

## ROSインターフェース

| 名前 | 型 | ノードのQoS | 内容 |
|---|---|---|---|
| `/follower/joint_command` | sensor_msgs/msg/JointState | 購読：Reliable、KeepLast 1 | 6軸の位置目標、rad |
| `/follower/joint_states` | sensor_msgs/msg/JointState | 配信：**BestEffort**、KeepLast 5 | 6軸の実測角度、rad |
| `/follower/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | 配信：Reliable、KeepLast 10 | 制御状態と異常 |
| `/follower/motor_states` | diagnostic_msgs/msg/DiagnosticArray | 配信：Reliable、KeepLast 10 | 各モーターの状態。[項目の説明](so101_motor_monitor.md#rosトピック) |
| `/follower/enable_control` | std_srvs/srv/SetBool | — | trueで開始、falseで保持・指令受付停止 |
| `/follower/release_torque` | std_srvs/srv/Trigger | — | 全軸のトルク解除 |

`/follower/joint_states`はBestEffortで配信します。購読側もBestEffortにしてください。
既定のReliableで購読すると、何も受信できません。`ros2 topic echo`では`--qos-reliability best_effort`を付けます。

### 位置指令の形式

`/follower/joint_command`には次の条件で[JointState](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg)を送信します。

| フィールド | 指定方法 |
|---|---|
| `name` | `shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`、`gripper`を各1回。順序は任意 |
| `position` | `name`に対応する6個の有限値。グリッパーもrad |
| `header.stamp` | 送信時のROS時刻。指令ごとに更新 |
| `velocity` / `effort` / `header.frame_id` | 空 |

全6軸を毎回送信します。
時刻が0、未来、制御開始前、前回以下、または有効期限を過ぎた指令は拒否します。
送信元と実機ノードは時刻を合わせ、`use_sim_time=false`で使用してください。

購読QoSはReliable / Volatile / KeepLast 1です。送信側もReliableにします。
到達完了の応答は返さないため、現在位置は`/follower/joint_states`で確認します。

### 制御の開始・停止

`/follower/enable_control`に`data: true`を送ると制御を開始し、`data: false`を送ると停止します。
コマンドの例は[ターミナルから操作する](#ターミナルから操作する)を参照してください。

開始する前に、6軸の目標を継続して送信するプログラムを用意してください。
開始時は現在位置を保持目標にします。
開始前の指令は使わないため、開始後も新しい時刻を付けた指令を送り続けてください。
有効な指令が`command_timeout_s`以内に届かないと停止します。

現在位置はサーボの可動範囲内にある必要があります。
全軸のトルクがOFF、または全軸がONで静止・保持している場合に開始できます。
起動直後に一部の軸だけがONの場合や保持状態を確認できない場合は、理由を返して開始を拒否します。

停止すると、現在位置への保持指令を送り、指令受付を停止します。トルクはONのままです。
再開には`data: true`を送ります。再開時も現在位置を初期目標にします。
この起動コマンドでは、Ctrl+Cなどでノードを終了した場合も保持を試みます。
`release_torque_on_shutdown:=true`を追加すると、終了時に全6軸のトルクを解除します。
[テレオペ](so101_teleop.md#終了)では終了時のトルク解除が自動で有効になります。

### トルク解除

腕を支えて実行します。

```bash
ros2 service call /follower/release_torque std_srvs/srv/Trigger '{}'
```

指令受付を停止し、全6軸のトルクOFFを確認して成功を返します。
解除に失敗した場合は理由を返します。接続状態を確認して再試行してください。
このサービスも`allow_writes:=true`で起動した場合に使用できます。

## ターミナルから操作する

ターミナル1で[起動コマンド](#起動)を実行した状態で、以下を操作します。
ターミナル2で位置指令を送信し、ターミナル3で開始・停止します。

### 1. 現在角度を確認する

ターミナル2で、現在の関節名と角度を1回取得します。

```bash
ros2 topic echo /follower/joint_states sensor_msgs/msg/JointState \
  --once --qos-reliability best_effort
```

`position`は`name`と同じ順序の角度（rad）です。
1軸だけ動かしたい場合は、残り5軸の目標値に現在角度を指定します。

### 2. 目標角度を送信する

ターミナル2で次を実行し、送信したままにします。
以下は`shoulder_pan`を0.1 rad（約5.7°）、ほかの5軸を0 radにする例です。
`position`の6個の値は、実機のキャリブレーション範囲内で使用する目標角度に置き換えてください。

```bash
ros2 topic pub /follower/joint_command sensor_msgs/msg/JointState \
  '{header: auto,
    name: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper],
    position: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}' \
  --rate 30 --qos-reliability reliable
```

`header: auto`で送信ごとに現在時刻を設定し、`--rate 30`で毎秒30回送信します。
制御開始までは指令が拒否されますが、次の手順で開始すると受け付けます。
目標到達後も送信を続けます。送信を止めると、既定では0.5秒後に制御が停止します。

### 3. 制御を開始する

ターミナル3で実行します。

```bash
ros2 service call /follower/enable_control std_srvs/srv/SetBool '{data: true}'
```

`success: true`を確認します。フォロワーが送信中の目標へ移動します。
続けて現在角度を表示できます。表示だけを終了するにはCtrl+Cを押します。

```bash
ros2 topic echo /follower/joint_states sensor_msgs/msg/JointState \
  --field position --qos-reliability best_effort
```

### 4. 停止する・目標を変更する

ターミナル3で角度の表示を終了し、停止サービスを実行します。

```bash
ros2 service call /follower/enable_control std_srvs/srv/SetBool '{data: false}'
```

停止後、ターミナル2の送信をCtrl+Cで終了します。
別の目標へ動かす場合は、手順2の`position`を変更して再送信し、手順3で制御を開始します。
トルクも切る場合は、腕を支えて[トルク解除](#トルク解除)のコマンドを実行してください。

## 位置指令と動作設定

最新の目標位置をサーボへ送り、移動中も新しい目標へ更新します。
移動速度・加減速と位置への追従はサーボ側で処理します。
これらの設定はGUI・ホーム移動・テレオペに共通です。

`~/.ros/so101_driver/config/follower.yaml`を編集し、ノードを再起動して反映します。

| 設定 | 既定値 | 内容 |
|---|---|---|
| `publish_rate_hz` | 30.0 | 状態取得と指令更新の頻度（5〜100 Hz） |
| `command_timeout_s` | 0.5 | 指令の有効期間と、開始直後の指令待ち期限（秒、0より大きい値） |
| `max_velocity_rad_s` | 1.57 | 各関節の移動速度（rad/s、0より大きい値。1.57は約90°/s）。大きくすると速く移動 |
| `servo_acceleration_raw` | 20 | 加減速の設定（整数1〜254）。大きくすると速度変化が急になる |
| `gripper_torque_limit_pct` | 20.0 | グリッパーの出力上限（0〜100%、0.1%単位） |
| `motor_state_rate_hz` | 2.0 | 1軸あたりの状態監視頻度（0〜10 Hz）。0で無効 |

範囲外の値を指定すると、ノードは起動時にエラーを表示して終了します。

GUI・テレオペは、関節状態と診断情報が0.5秒以上更新されないと制御を停止します。
このため`publish_rate_hz`は5 Hz以上とし、取得に1回失敗しても停止しない頻度にしています。

位置目標の範囲は、キャリブレーションで観測した範囲と、記録したサーボの可動範囲の共通部分です。
範囲外の位置目標は拒否します。URDFの可動範囲は制御範囲に使用しません。

速度は関節ごとにサーボの単位（tick/秒）へ換算し、グリッパーを含む全6軸を`max_velocity_rad_s`で動かします。
グリッパーの角度は、キャリブレーションで記録した閉・開の位置に-0.17〜1.75 radを割り当てた値です。
既定値の1.57 rad/sでは、閉から開まで約1.2秒で動く速度になります（加減速の時間を除く）。

加減速はサーボ角度を基準とします。`servo_acceleration_raw`の1単位は8.7度/秒²で、既定値20は174度/秒²に相当します。
グリッパーでは、関節角度での加減速がキャリブレーションの開閉幅によってこの値と異なります。

`gripper_torque_limit_pct`を上げると強く把持できる一方、発熱や過負荷が増える場合があります。
下げると負担を抑えられますが、物を保持できなくなる場合があります。
100%はサーボ出力の上限、0%は出力なしを意味し、把持力をN単位で指定する設定ではありません。
制御開始のたびに適用します。腕5軸の出力上限と過負荷保護の設定は変更しません。

設定値の仕様は[Feetech公式STSメモリ表](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/)を参照してください。

## 通信異常・指令途絶

期限内に有効な指令が届かない場合や、位置取得・目標送信に失敗した場合は、保持を試みて制御を停止します。
通信が復旧すると状態配信は再開しますが、制御の再開には開始サービスが必要です。

`Hold NOT confirmed`が表示された場合は保持指令の成功を確認できていません。
接続状態を確認し、停止サービス（`data: false`）で保持を再試行できます。
通信断や一部の軸への送信失敗では、全軸の保持が成立しない場合があります。

`/follower/diagnostics`で`allow_writes`（制御許可）、`control_active`（制御中）、
`control_status`（状態）、`failed_samples`（取得失敗数）、`rejected_commands`（指令拒否数）を確認できます。
