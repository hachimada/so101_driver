# so101_driver

SO-ARM101のキャリブレーションと制御を行うROS2パッケージです。テレオペでは、USBカメラの画像も一緒に配信できます。

| 機能 | 内容 |
|---|---|
| リーダー／フォロワーのキャリブレーション | CLIの案内と見本モデルを使い、基準姿勢・可動範囲を記録 |
| フォロワー制御インターフェース | ROSトピックによる6軸の位置指令と、制御開始・停止サービス |
| スライダーGUI | 6軸の操作と、電流・電圧・温度などの状態確認 |
| テレオペ | リーダーの動きをフォロワーへ反映 |
| カメラ（任意） | テレオペと同時に、USBカメラの画像をROSトピックで配信（usb_camを使用） |

## 要件

- Ubuntu 22.04 / ROS2 Humble / Python 3.10
- Git・colcon・rosdepが利用できるROS2開発環境
- SO-ARM101（STS3215 × 6、モーターID 1〜6、通信速度1 Mbps）とUSBシリアル接続
- モーターIDと通信速度は事前に設定しておく必要があります。[モーターIDと通信速度の事前設定](docs/so101_setup.md#モーターidと通信速度の事前設定)を参照してください
- カメラを使う場合：UVC対応のUSBカメラと`ros-humble-usb-cam`（so101_driverの依存には含まない）

## 安全上の注意

- 制御を許可して起動すると、実機が動きます。起動前に、アームの届く範囲に人や物がないことを確認してください。
- トルクを解除すると、腕は自重で落ちます。トルクの解除や終了の操作は、腕を支えてから行ってください。
- サーボの電源を切ったときもトルクが抜けます。ソフトウェアで止められない場合は、腕を支えてから電源を切ってください。

## クイックスタート

### インストール

ROS2ワークスペースのルートで実行します。
`<repository-url>`は、このリポジトリのクローンURLに置き換えてください。

```bash
git clone --recurse-submodules "<repository-url>" src/so101_driver
rosdep install --from-paths src/so101_driver --ignore-src --rosdistro humble -r -y
colcon build --packages-select so101_driver
source install/setup.bash
```

以降のコマンドは、`install/setup.bash`を読み込んだターミナルであれば、どのディレクトリからでも実行できます。

### 機体の設定

`so101_list_devices`で各アームのデバイスIDを確認します。
設定ファイル（`~/.ros/so101_driver/config/`）がなければテンプレートから作成するので、表示されたファイルの`device_id`にIDを書き込んでください。
手順は[機体の設定](docs/so101_setup.md)を参照してください。

### キャリブレーション

```bash
ros2 run so101_driver so101_calibrate --role leader
ros2 run so101_driver so101_calibrate --role follower
```

見本の姿勢に合わせてグリッパーを閉じ、Enterで開始します。
全関節を動かし、Enterで終了・保存します。

### 起動

いずれも実機が動きます。初回は各ガイドで開始・停止の手順を確認してください。

スライダーGUI:

```bash
ros2 launch so101_driver follower_slider_control.launch.py
```

終了時は腕を支え、「Release torque and quit」を押します。

テレオペ:

```bash
ros2 launch so101_driver teleop.launch.py
```

フォロワーがリーダーの姿勢へ移動してから、リーダーの動きに追従します。
終了時はフォロワーの腕を支えてCtrl+Cを押します。全6軸のトルクをOFFにして終了します。

USBカメラの画像も配信する場合は、`cameras:=true`を付けます。
事前に`so101_list_cameras`でカメラのパスを確認し、`cameras.yaml`に書き込んでください（[カメラ](docs/so101_cameras.md)）。

```bash
ros2 launch so101_driver teleop.launch.py cameras:=true
```

外部プログラムからROSトピックとサービスで制御する場合:

```bash
ros2 launch so101_driver follower_control.launch.py allow_writes:=true
```

## ROSインターフェース

このパッケージのノードが提供するトピックとサービスです。QoSや項目の詳細は、各行のリンク先を参照してください。

| 名前 | 型 | ノード側 | 内容 | 詳細 |
|---|---|---|---|---|
| `/follower/joint_command` | sensor_msgs/msg/JointState | 購読 | フォロワー6軸の位置目標（rad） | [フォロワー](docs/so101_follower.md#rosインターフェース) |
| `/follower/joint_states` | sensor_msgs/msg/JointState | 配信（BestEffort） | フォロワー6軸の実測角度（rad） | [フォロワー](docs/so101_follower.md#rosインターフェース) |
| `/follower/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | 配信 | フォロワーの制御状態と異常 | [フォロワー](docs/so101_follower.md#rosインターフェース) |
| `/follower/motor_states` | diagnostic_msgs/msg/DiagnosticArray | 配信 | 各モーターの電流・電圧・温度など | [モーター状態](docs/so101_motor_monitor.md#rosトピック) |
| `/follower/enable_control` | std_srvs/srv/SetBool | サービス | trueで制御開始、falseで保持・指令受付停止 | [フォロワー](docs/so101_follower.md#rosインターフェース) |
| `/follower/release_torque` | std_srvs/srv/Trigger | サービス | フォロワー全軸のトルク解除 | [フォロワー](docs/so101_follower.md#rosインターフェース) |
| `/leader/joint_states` | sensor_msgs/msg/JointState | 配信（BestEffort） | リーダー6軸の角度（rad） | [テレオペ](docs/so101_teleop.md#rosインターフェース) |
| `/leader/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | 配信 | リーダーの読み取り状態 | [テレオペ](docs/so101_teleop.md#rosインターフェース) |
| `/so101_teleop/enable` | std_srvs/srv/SetBool | サービス | テレオペの開始・停止 | [テレオペ](docs/so101_teleop.md#rosインターフェース) |
| `/cameras/<name>/image_raw` | sensor_msgs/msg/Image | 配信 | カメラの画像 | [カメラ](docs/so101_cameras.md#トピック) |
| `/cameras/<name>/image_raw/compressed` | sensor_msgs/msg/CompressedImage | 配信 | JPEGで圧縮したカメラの画像 | [カメラ](docs/so101_cameras.md#トピック) |
| `/cameras/<name>/camera_info` | sensor_msgs/msg/CameraInfo | 配信 | カメラの解像度と校正値 | [カメラ](docs/so101_cameras.md#トピック) |

起動するlaunchによって、使えるトピックとサービスが異なります。

| トピック・サービス | 起動するlaunch |
|---|---|
| `/follower/...` | `follower_control.launch.py`、`follower_slider_control.launch.py`、`teleop.launch.py` |
| `/leader/...`、`/so101_teleop/...` | `teleop.launch.py` |
| `/cameras/...` | `teleop.launch.py cameras:=true`、`cameras.launch.py` |

BestEffortで配信する`joint_states`は、購読側もBestEffortにしてください。
カメラの画像は、購読側をReliableにしてください。

## ドキュメント

| ガイド | 内容 |
|---|---|
| [機体の設定](docs/so101_setup.md) | デバイスIDの確認、設定ファイルの作成と保存場所 |
| [キャリブレーション](docs/so101_calibration.md) | 基準姿勢・可動範囲の記録、保存、適用 |
| [フォロワー制御インターフェース](docs/so101_follower.md) | ROS API、制御の開始・停止、動作設定 |
| [スライダーGUI](docs/so101_gui.md) | 関節の操作、ホーム位置の登録・復帰 |
| [モーター状態の見方](docs/so101_motor_monitor.md) | 表示項目、単位、異常の確認 |
| [テレオペ](docs/so101_teleop.md) | 初期位置合わせ、追従、開始・停止 |
| [カメラ](docs/so101_cameras.md) | USBカメラの設定、テレオペと同時の配信、記録 |

出典とライセンス:

- [Feetech公式Python SDK](docs/feetech_sdk_source.md)
- [ロボットモデル](description/README.md)
