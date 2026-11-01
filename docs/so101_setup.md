# 機体の設定

パッケージの導入は[READMEのインストール](../README.md#インストール)を参照してください。

## モーターIDと通信速度の事前設定

このドライバーは、サーボの設定（EEPROM）を書き換えません。
組み立て時に、各サーボのモーターIDと通信速度（1 Mbps）を設定しておいてください。
モーターIDと関節の対応は次のとおりです。LeRobotのSO-101と同じ割り当てです。

| モーターID | 関節 |
|---:|---|
| 1 | `shoulder_pan` |
| 2 | `shoulder_lift` |
| 3 | `elbow_flex` |
| 4 | `wrist_flex` |
| 5 | `wrist_roll` |
| 6 | `gripper` |

LeRobotの`lerobot-setup-motors`で設定できます（LeRobot 0.6.1で確認）。
`--robot.port`・`--teleop.port`は接続しているポートに置き換えてください。

```bash
lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0
lerobot-setup-motors --teleop.type=so101_leader --teleop.port=/dev/ttyACM1
```

案内に従い、サーボを1個ずつ接続してEnterを押します。グリッパー（ID 6）から順に設定します。

## デバイスIDの確認

対象アームをUSBで接続して実行します。

```bash
ros2 run so101_driver so101_list_devices
```

最初に、設定ファイルの場所と接続中の機器を表示します。
一覧の`CURRENT CONFIG`列は、そのIDを`device_id`に書き込み済みの役割（`leader`・`follower`）です。

1. 案内が出たら、対象アームのUSBケーブルを1本抜き、Enterを押します。
2. 表示された`DEVICE ID`と案内を確認し、ケーブルを接続し直します。
3. リーダーとフォロワーそれぞれで確認します。

IDがまだ設定ファイルに書かれていない場合は、未作成の[設定ファイル](#設定ファイル)をテンプレートから作成し、IDを書き込むファイルを表示します。
表示されたファイルを開き、取り外したアームの`device_id`にIDを書き込んでください。
作成済みのファイルは上書きしません。

IDはUSBシリアル番号なので、USBの抜き差しでポート番号が変わっても設定を変更する必要はありません。
サーボのモーターID（1〜6）とは別の識別子です。

機器を特定できない場合は、対象のケーブルだけを抜いて再確認してください。
シリアル番号がない機器や、同じ番号の機器が複数ある場合はIDで特定できません。
`q`またはCtrl+Cで終了します。

## 設定ファイル

機体ごとの設定とキャリブレーション結果は、`~/.ros/so101_driver`に保存します。
環境変数`ROS_HOME`を設定している場合は`$ROS_HOME/so101_driver`です。以下の`~/.ros`も読み替えてください。
リポジトリの外に置くため、クローン先や実行するディレクトリに関係なく同じファイルを使います。

以下のパスは`~/.ros/so101_driver`を基準にしています。

| ファイル | 内容 |
|---|---|
| `config/leader.yaml` | リーダーのデバイスIDとキャリブレーション |
| `config/follower.yaml` | フォロワーのデバイスID、キャリブレーション、動作設定 |
| `config/teleop.yaml` | テレオペの初期位置合わせと関節倍率 |
| `config/cameras.yaml` | USBカメラの一覧。カメラを使う場合に`so101_list_cameras`が作成。[カメラ](so101_cameras.md)を参照 |
| `calibration/leader.json`・`calibration/follower.json` | キャリブレーション結果。[キャリブレーション](so101_calibration.md)で作成 |

`leader.yaml`・`follower.yaml`の`device_id`を確認したIDへ変更します。
数値だけのIDも、引用符で囲んで指定してください。

```yaml
device_id: "your-device-serial"
```

`calibration_profile`は、それぞれ`calibration/leader.json`・`calibration/follower.json`を参照する設定になっています。

`config/*.yaml`は`so101_list_devices`が作成します。
手動で作成する場合は、インストールしたテンプレートをコピーします。

```bash
mkdir -p ~/.ros/so101_driver/config
share="$(ros2 pkg prefix --share so101_driver)"
cp -n "$share/config/leader.template.yaml" ~/.ros/so101_driver/config/leader.yaml
cp -n "$share/config/follower.template.yaml" ~/.ros/so101_driver/config/follower.yaml
cp -n "$share/config/teleop.template.yaml" ~/.ros/so101_driver/config/teleop.yaml
```

## 主な設定項目

| 項目 | 用途 | 詳細 |
|---|---|---|
| `device_id` | 接続するアームのUSBシリアル番号 | このガイドの「デバイスIDの確認」 |
| `calibration_profile` | 使用するキャリブレーションJSON | [キャリブレーション](so101_calibration.md#保存先と適用) |
| `publish_rate_hz` | 状態の配信頻度（リーダー・フォロワー） | [動作設定](so101_follower.md#位置指令と動作設定)、[リーダーノード](so101_teleop.md#リーダーノードleaderso101_leader) |
| `command_timeout_s` | フォロワーの指令の有効期間 | [動作設定](so101_follower.md#位置指令と動作設定) |
| `max_velocity_rad_s` / `servo_acceleration_raw` | フォロワーの移動速度・加減速 | [動作設定](so101_follower.md#位置指令と動作設定) |
| `gripper_torque_limit_pct` | グリッパーの出力上限 | [動作設定](so101_follower.md#位置指令と動作設定) |
| `motor_state_rate_hz` | モーター状態の更新頻度 | [状態表示](so101_motor_monitor.md#更新頻度) |
| `alignment_*` / `joint_scales` | テレオペの位置合わせ・関節倍率 | [テレオペ設定](so101_teleop.md#位置合わせと関節倍率の設定) |

設定変更はノードの再起動で反映します。再ビルドは不要です。
各設定の有効な値は、詳細のページに記載しています。範囲外の値では、ノードは起動時にエラーを表示して終了します。
数値は`30`と`30.0`のどちらの書き方でも指定できます。ただし`joint_scales`のようなリストでは、すべての値を小数点付きで書いてください。
`calibration_profile`の相対パスは`~/.ros/so101_driver`を基準に解決します。絶対パスも指定できます。
YAML中の`$HOME`や`$PWD`は展開されません。

## 起動時の設定ファイル指定

| 機能 | 起動ファイル | 設定ファイルの引数 |
|---|---|---|
| ROS制御インターフェース | `follower_control.launch.py` | `params_file` |
| スライダーGUI | `follower_slider_control.launch.py` | `params_file` |
| テレオペ | `teleop.launch.py` | `leader_params_file`、`follower_params_file`、`teleop_params_file` |

引数を省略すると、`~/.ros/so101_driver/config/`の対応するYAMLを使用します。
そのファイルがなければテンプレートを参照します。実機を使う前に、上記の手順でデバイスIDを設定してください。
使用した設定ファイルは、起動時にターミナルへ表示します。

別の設定ファイルを使う場合は、引数に絶対パスを指定します。

```bash
ros2 launch so101_driver follower_slider_control.launch.py params_file:=/path/to/follower.yaml
```

起動コマンドは[READMEの起動](../README.md#起動)を参照してください。
