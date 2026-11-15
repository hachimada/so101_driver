# so101_driver

[English](README.md) | **日本語**

Feetech公式SDKを使い、SO-ARM101（SO-101）のキャリブレーションと制御を行うROS 2パッケージです。
テレオペでは、USBカメラの画像もROSトピックで配信できます。

## 機能

| 機能 | 内容 |
|---|---|
| キャリブレーション | ターミナルの案内とRVizの見本モデルを使い、リーダー・フォロワーの基準姿勢と可動範囲を記録 |
| フォロワー制御インターフェース | ROSトピックによる6軸の位置指令と、制御開始・停止のサービス |
| スライダーGUI | 各関節の操作と、電流・電圧・温度などのモーター状態の確認 |
| テレオペ | フォロワーをリーダーの姿勢に合わせてから、リーダーの動きに追従 |
| カメラ（任意） | テレオペと同時に、USBカメラの画像を配信（[usb_cam](https://index.ros.org/p/usb_cam/)を使用） |

## 安全上の注意

> [!WARNING]
> - 制御を許可して起動すると、実機が動きます。起動前に、アームの届く範囲に人や物がないことを確認してください。
> - トルクを解除すると、腕は自重で落ちます。トルクの解除や終了の操作は、腕を支えてから行ってください。
> - サーボの電源を切ったときもトルクが抜けます。ソフトウェアで止められない場合は、腕を支えてから電源を切ってください。

## 要件

- Ubuntu 22.04、ROS 2 Humble、Python 3.10
- Git・colcon・rosdepが利用できるROS 2開発環境
- USBシリアルで接続したSO-ARM101（STS3215 × 6、モーターID 1〜6、通信速度1 Mbps）。
  ドライバーはモーターIDと通信速度を変更しないため、事前に設定しておいてください。
  [モーターIDと通信速度の設定](docs/ja/so101_setup.md#モーターidと通信速度の設定)を参照してください。
- カメラを使う場合：UVC対応のUSBカメラと`ros-humble-usb-cam`（so101_driverの依存には含まない）

## インストール

ROS 2ワークスペースのルートで実行します。

```bash
git clone --recurse-submodules https://github.com/hachimada/so101_driver.git src/so101_driver
rosdep install --from-paths src/so101_driver --ignore-src --rosdistro humble -r -y
colcon build --packages-select so101_driver
source install/setup.bash
```

以降のコマンドは、`install/setup.bash`を読み込んだターミナルであれば、どのディレクトリからでも実行できます。

## クイックスタート

### 1. デバイスIDの設定

`so101_list_devices`を実行し、案内に従って各アームのUSBを抜くと、デバイスID（USBシリアル番号）を確認できます。
IDがまだどの設定ファイルにも書かれていない場合は、`~/.ros/so101_driver/config/`に未作成の設定ファイルをテンプレートから作成します。
表示されたファイルの`device_id`にIDを書き込んでください。
詳しくは[機体の設定](docs/ja/so101_setup.md)を参照してください。

```bash
ros2 run so101_driver so101_list_devices
```

### 2. キャリブレーション

```bash
ros2 run so101_driver so101_calibrate --role leader
ros2 run so101_driver so101_calibrate --role follower
```

見本の姿勢に合わせてグリッパーを閉じ、Enterで開始します。
全関節を動かし、Enterで終了・保存します。
[キャリブレーション](docs/ja/so101_calibration.md)を参照してください。

### 3. 起動

いずれも実機が動きます。初回は各ガイドで開始・停止の手順を確認してください。

スライダーGUI（[ガイド](docs/ja/so101_gui.md)）:

```bash
ros2 launch so101_driver follower_slider_control.launch.py
```

終了時は腕を支え、**Release torque and quit**を押します。

テレオペ（[ガイド](docs/ja/so101_teleop.md)）:

```bash
ros2 launch so101_driver teleop.launch.py
```

フォロワーがリーダーの姿勢へ移動してから、リーダーの動きに追従します。
終了時はフォロワーの腕を支えてCtrl+Cを押します。全6軸のトルクを解除してからドライバーが終了します。

USBカメラの画像も配信する場合は、`cameras:=true`を付けます。
事前に`so101_list_cameras`でカメラのパスを確認し、`cameras.yaml`に書き込んでください（[カメラ](docs/ja/so101_cameras.md)）。

```bash
ros2 launch so101_driver teleop.launch.py cameras:=true
```

外部プログラムからROSトピックとサービスで制御する場合（[ガイド](docs/ja/so101_follower.md)）:

```bash
ros2 launch so101_driver follower_control.launch.py allow_writes:=true
```

## 使い方

### コマンドラインツール

| コマンド | 内容 |
|---|---|
| `ros2 run so101_driver so101_list_devices` | USBを抜いてアームのデバイスIDを確認し、未作成の設定ファイルを作成。[機体の設定](docs/ja/so101_setup.md#デバイスidの確認)を参照 |
| `ros2 run so101_driver so101_calibrate` | リーダーまたはフォロワーのキャリブレーション。[キャリブレーション](docs/ja/so101_calibration.md)を参照 |
| `ros2 run so101_driver so101_list_cameras` | USBを抜いてカメラのUSBポートのパスを確認し、`cameras.yaml`を作成。[カメラ](docs/ja/so101_cameras.md#カメラのパスの確認)を参照 |

### launchファイル

| launchファイル | 起動するもの | 引数（既定値） |
|---|---|---|
| `follower_control.launch.py` | フォロワーノード | `params_file`、`allow_writes`（`false`）、`release_torque_on_shutdown`（`false`） |
| `follower_slider_control.launch.py` | フォロワーノードとスライダーGUI | `params_file`、`allow_writes`（`true`） |
| `teleop.launch.py` | リーダー・フォロワー・テレオペのノード。`cameras:=true`でカメラも起動 | `leader_params_file`、`follower_params_file`、`teleop_params_file`、`allow_writes`（`true`）、`autostart`（`true`）、`cameras`（`false`）、`cameras_file` |
| `cameras.launch.py` | カメラごとのusb_camノード | `cameras_file` |

- `*params_file`引数の既定値は、`~/.ros/so101_driver/config/`の対応するYAMLです。そのファイルがなければ、インストールしたテンプレートを使います。[起動時の設定ファイル指定](docs/ja/so101_setup.md#起動時の設定ファイル指定)を参照してください。
- `cameras_file`の既定値は`~/.ros/so101_driver/config/cameras.yaml`です。
- `teleop.launch.py`は、終了時に必ずフォロワーのトルクを解除します（既定の`allow_writes:=true`の場合）。
- アームのノードやスライダーGUIのいずれかが終了すると、launch全体が終了します。カメラのノードが停止しても終了しません。

### ノード

| ノード | 実行ファイル | 起動するlaunch |
|---|---|---|
| `/follower/so101_follower` | `follower_node` | `follower_control.launch.py`、`follower_slider_control.launch.py`、`teleop.launch.py` |
| `/leader/so101_leader` | `leader_node` | `teleop.launch.py` |
| `/so101_teleop` | `teleop_node` | `teleop.launch.py` |
| `/so101_slider_control` | `slider_gui` | `follower_slider_control.launch.py` |
| `/cameras/<name>/usb_cam` | `usb_cam_node_exe`（usb_cam） | `cameras.launch.py`、`teleop.launch.py cameras:=true` |

## ROSインターフェース

このパッケージのノードが提供するトピックとサービスです。QoSや項目の詳細は、各行のリンク先を参照してください。

| 名前 | 型 | ノード側 | 内容 | 詳細 |
|---|---|---|---|---|
| `/follower/joint_command` | sensor_msgs/msg/JointState | 購読 | フォロワー6軸の位置目標（rad） | [フォロワー](docs/ja/so101_follower.md#rosインターフェース) |
| `/follower/joint_states` | sensor_msgs/msg/JointState | 配信（BestEffort） | フォロワー6軸の実測角度（rad） | [フォロワー](docs/ja/so101_follower.md#rosインターフェース) |
| `/follower/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | 配信 | フォロワーの制御状態と異常 | [フォロワー](docs/ja/so101_follower.md#診断情報) |
| `/follower/motor_states` | diagnostic_msgs/msg/DiagnosticArray | 配信 | 各モーターの電流・電圧・温度など | [モーター状態](docs/ja/so101_motor_monitor.md#rosトピック) |
| `/follower/enable_control` | std_srvs/srv/SetBool | サービス | `true`で制御開始、`false`で保持・指令受付停止 | [フォロワー](docs/ja/so101_follower.md#rosインターフェース) |
| `/follower/release_torque` | std_srvs/srv/Trigger | サービス | フォロワー全軸のトルク解除 | [フォロワー](docs/ja/so101_follower.md#rosインターフェース) |
| `/leader/joint_states` | sensor_msgs/msg/JointState | 配信（BestEffort） | リーダー6軸の角度（rad） | [テレオペ](docs/ja/so101_teleop.md#rosインターフェース) |
| `/leader/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | 配信 | リーダーの読み取り状態 | [テレオペ](docs/ja/so101_teleop.md#rosインターフェース) |
| `/so101_teleop/enable` | std_srvs/srv/SetBool | サービス | テレオペの開始・停止 | [テレオペ](docs/ja/so101_teleop.md#rosインターフェース) |
| `/cameras/<name>/image_raw` | sensor_msgs/msg/Image | 配信 | カメラの画像 | [カメラ](docs/ja/so101_cameras.md#トピック) |
| `/cameras/<name>/image_raw/compressed` | sensor_msgs/msg/CompressedImage | 配信 | JPEGで圧縮したカメラの画像 | [カメラ](docs/ja/so101_cameras.md#トピック) |
| `/cameras/<name>/camera_info` | sensor_msgs/msg/CameraInfo | 配信 | カメラの解像度と校正値 | [カメラ](docs/ja/so101_cameras.md#トピック) |

起動するlaunchによって、使えるトピックとサービスが異なります。

| トピック・サービス | 起動するlaunch |
|---|---|
| `/follower/...` | `follower_control.launch.py`、`follower_slider_control.launch.py`、`teleop.launch.py` |
| `/leader/...`、`/so101_teleop/...` | `teleop.launch.py` |
| `/cameras/...` | `teleop.launch.py cameras:=true`、`cameras.launch.py` |

BestEffortで配信する`joint_states`は、購読側もBestEffortにしてください。
カメラの画像は、購読側をReliableにしてください。

## 設定

機体ごとの設定とキャリブレーション結果は、リポジトリの外の`~/.ros/so101_driver`に保存します（`ROS_HOME`を設定している場合は`$ROS_HOME/so101_driver`）。

| ファイル | 内容 |
|---|---|
| `config/leader.yaml` | リーダーのデバイスIDとキャリブレーションファイル |
| `config/follower.yaml` | フォロワーのデバイスID、キャリブレーションファイル、動作設定 |
| `config/teleop.yaml` | テレオペの位置合わせと関節倍率 |
| `config/cameras.yaml` | USBカメラの一覧（カメラを使う場合のみ） |
| `calibration/leader.json`、`calibration/follower.json` | キャリブレーション結果 |

設定の変更はノードの再起動で反映します。再ビルドは不要です。
各設定項目は[機体の設定](docs/ja/so101_setup.md#設定ファイル)を参照してください。

## ドキュメント

| ガイド | 内容 |
|---|---|
| [機体の設定](docs/ja/so101_setup.md) | モーターID・デバイスIDの設定、設定ファイルの作成と保存場所 |
| [キャリブレーション](docs/ja/so101_calibration.md) | 基準姿勢・可動範囲の記録、保存、適用 |
| [フォロワー制御インターフェース](docs/ja/so101_follower.md) | ROS API、制御の開始・停止、動作設定 |
| [スライダーGUI](docs/ja/so101_gui.md) | 関節の操作、ホーム位置の登録・復帰 |
| [モーター状態の見方](docs/ja/so101_motor_monitor.md) | 表示項目、単位、異常の確認 |
| [テレオペ](docs/ja/so101_teleop.md) | 初期位置合わせ、追従、開始・停止 |
| [カメラ](docs/ja/so101_cameras.md) | USBカメラの設定、テレオペと同時の配信、記録 |
| [Feetech SDK](docs/ja/feetech_sdk_source.md) | 同梱するFeetech公式SDKの出典とインストール |

## 開発

リポジトリのディレクトリで、[Ruff](https://docs.astral.sh/ruff/)によるlintと整形を行います（設定は`ruff.toml`）。

```bash
ruff check .
ruff format .
```

テストはワークスペースのルートで実行します。

```bash
colcon test --packages-select so101_driver
colcon test-result --verbose
```

## ライセンス

このパッケージは[Apache License 2.0](LICENSE)で公開しています。
次のものは、それぞれのライセンスに従います。

| 対象 | ライセンス | 出典 |
|---|---|---|
| Feetech公式Python SDK（サブモジュール） | MIT | [Feetech SDK](docs/ja/feetech_sdk_source.md) |
| ロボットモデル（URDF・メッシュ） | Apache-2.0 | [ロボットモデル](description/README.ja.md) |
