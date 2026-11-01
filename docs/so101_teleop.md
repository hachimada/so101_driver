# テレオペ

フォロワーをリーダーの姿勢に合わせてから、リーダーの動きへ追従させます。

## 起動

両機の[設定](so101_setup.md)と[キャリブレーション](so101_calibration.md)を済ませてから実行します。

```bash
ros2 launch so101_driver teleop.launch.py
```

1. 両機の状態を取得すると、自動でフォロワーがリーダーの姿勢へ移動します。
2. 「Aligning」と表示されている間は、リーダーの姿勢を保ちます。
3. 「Aligned; teleoperation running」と表示されたら、リーダーを動かして操作します。

位置合わせ中にリーダーを動かすと、目標もその姿勢へ更新します。
一致は両機のキャリブレーション後の角度で判定するため、基準姿勢と関節方向を揃えてください。

USBカメラの画像も配信する場合は、`cameras:=true`を付けます。設定は[カメラ](so101_cameras.md)を参照してください。

## 手動で開始・停止する

自動開始を無効にする場合は、起動コマンドに`autostart:=false`を追加します。

開始:

```bash
ros2 service call /so101_teleop/enable std_srvs/srv/SetBool '{data: true}'
```

停止:

```bash
ros2 service call /so101_teleop/enable std_srvs/srv/SetBool '{data: false}'
```

開始サービスは位置合わせを行い、姿勢の一致を確認してから成功応答を返します。
再開始時も位置合わせから始まります。

停止サービスではフォロワーの現在位置を保持します。

## 終了

フォロワーの腕を支えて、起動したターミナルでCtrl+Cを押します。
全6軸のトルクをOFFにし、読み戻しで確認してから接続を閉じます。
「Torque released on all 6 joints」と表示されれば完了です。

通信異常などでOFFを確認できない場合は、終了時にエラーを表示します。
その場合、トルクが残っている可能性があるため、接続状態を確認してください。

## 位置合わせと関節倍率の設定

`~/.ros/so101_driver/config/teleop.yaml`を編集し、テレオペを再起動して反映します。

```yaml
/so101_teleop:
  ros__parameters:
    joint_scales: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    alignment_tolerance_rad: 0.05
    alignment_settle_time_s: 0.2
    alignment_timeout_s: 30.0
```

| 設定 | 既定値 | 意味 |
|---|---|---|
| `alignment_tolerance_rad` | 0.05 rad（約2.9°） | 各関節の目標と実測角度の許容差（0より大きい値）。小さくすると一致条件が厳しくなる |
| `alignment_settle_time_s` | 0.2秒 | 全6軸が許容差内に連続して収まる必要がある時間（0より大きく、`alignment_timeout_s`より短い値） |
| `alignment_timeout_s` | 30秒 | 位置合わせの制限時間（0より大きい値）。低速で動かす場合は長く設定 |
| `joint_scales` | 全軸1.0 | 各関節のリーダー角度に掛ける倍率（6個の有限値） |

範囲外の値を指定すると、テレオペは起動時にエラーを表示して終了します。

関節倍率の順序は`shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`、`gripper`です。
-1.0で方向を反転し、0.0で目標を0 radに固定します。
値はすべて`-1.0`のように小数点付きで書いてください。`[-1, 1.0, ...]`のように整数と小数が混ざると、YAMLを読み込めず起動に失敗します。
位置合わせ・追従ともに「目標角度 = 関節倍率 × リーダー現在角度」を使います。

移動速度・加減速は`~/.ros/so101_driver/config/follower.yaml`で設定します。
[フォロワーの動作設定](so101_follower.md#位置指令と動作設定)を参照してください。
状態の配信頻度は、`leader.yaml`・`follower.yaml`の`publish_rate_hz`（既定30 Hz、5〜100 Hz）で設定します。

## ROSインターフェース

### リーダーノード（`/leader/so101_leader`）

テレオペの起動コマンドで一緒に起動します。モーターへの書き込みは行いません。

| 名前 | 型 | ノードのQoS | 内容 |
|---|---|---|---|
| `/leader/joint_states` | sensor_msgs/msg/JointState | 配信：**BestEffort**、KeepLast 5 | 6軸の角度、rad。グリッパーは閉〜開の角度に収めて配信 |
| `/leader/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | 配信：Reliable、KeepLast 10 | 読み取りの状態と、取得失敗数（`failed_samples`・`consecutive_failures`） |

`/leader/joint_states`を購読する場合は、購読側もBestEffortにしてください。

設定は`~/.ros/so101_driver/config/leader.yaml`で行い、ノードを再起動して反映します。

| 設定 | 既定値 | 内容 |
|---|---|---|
| `device_id` | なし | USBシリアル番号。[デバイスIDの確認](so101_setup.md#デバイスidの確認)を参照 |
| `calibration_profile` | `calibration/leader.json` | キャリブレーション結果。相対パスは`~/.ros/so101_driver`が基準 |
| `publish_rate_hz` | 30.0 | 角度の読み取りと配信の頻度（5〜100 Hz） |

### テレオペノード（`/so101_teleop`）

| 名前 | 型 | 内容 |
|---|---|---|
| `/leader/joint_states` | sensor_msgs/msg/JointState | リーダーの角度を購読（BestEffort） |
| `/so101_teleop/enable` | std_srvs/srv/SetBool | trueで位置合わせから開始、falseで停止 |

フォロワーへの指令と開始・停止には、[フォロワー制御インターフェース](so101_follower.md#rosインターフェース)を使います。

## 開始できない・途中で停止した場合

| 状況 | 確認する内容 |
|---|---|
| 状態取得を待っている | ターミナルに表示された機体の接続・設定 |
| 位置合わせできない | リーダーの姿勢がフォロワーの可動範囲に収まるか |
| 位置合わせがタイムアウト | 表示された関節の角度差、移動速度、制限時間 |
| 通信異常・状態取得の途絶 | 接続とモーターの異常表示 |

フォロワーへ送る目標はキャリブレーションに基づく指令範囲へ制限します。
範囲の端でも許容差内に合わせられない場合は、位置合わせを中止します。

リーダーまたはフォロワーの状態が0.5秒以上取得できない場合も停止します。
原因を解消してから開始サービスで再開してください。自動では再開しません。
