# カメラ

テレオペと一緒に、USBカメラ（UVC対応）の画像をROSトピックで配信します。
カメラの制御には、ROSの標準的なカメラドライバー[usb_cam](https://index.ros.org/p/usb_cam/)を使います。

## 導入

usb_camはso101_driverの依存に含まれていません。カメラを使う場合にインストールします。

```bash
sudo apt install ros-humble-usb-cam
```

## カメラのパスの確認

カメラは、差し込んだUSBポートで決まるパス（`/dev/v4l/by-path/…`）で指定します。
同じ型番のカメラを複数使っても、ポートが違えば区別できます。
アームの[デバイスIDの確認](so101_setup.md#デバイスidの確認)と同じく、USBを抜いてどのカメラかを確かめます。

使うカメラをすべてUSBで接続してから実行します。

```bash
ros2 run so101_driver so101_list_cameras
```

1. 接続中のカメラの一覧を表示します。`NAME IN CONFIG`列は、`cameras.yaml`に設定済みのカメラの名前です。
2. 案内が出たら、確認するカメラのUSBケーブルを1本抜き、Enterを押します。
3. 抜いたカメラのパスと、設定ファイルに書き込む行を表示します。`cameras.yaml`がなければテンプレートから作成します。
4. ケーブルを同じUSBポートに挿し直します。

カメラごとに繰り返します。
`q`またはCtrl+Cで終了します。

パスはUSBポートで決まるため、カメラを別のポートに差し替えると変わります。
差し替えた場合は、もう一度確認して`cameras.yaml`を書き換えてください。

カメラが対応する画素形式・解像度・フレームレートは、`v4l2-ctl`（`v4l-utils`パッケージ）で確認できます。

```bash
v4l2-ctl --list-formats-ext -d /dev/v4l/by-path/…-video-index0
```

## 設定

`so101_list_cameras`が作成した`~/.ros/so101_driver/config/cameras.yaml`に、使うカメラを書き込みます。
2台のカメラを使う場合の例です。

```yaml
cameras:
  - name: front
    device: /dev/v4l/by-path/pci-0000:80:14.0-usb-0:9:1.0-video-index0
    width: 640
    height: 480
    fps: 30
    parameters:
      pixel_format: mjpeg2rgb
  - name: wrist
    device: /dev/v4l/by-path/pci-0000:80:14.0-usb-0:10:1.0-video-index0
    parameters:
      pixel_format: mjpeg2rgb
```

| 項目 | 既定値 | 内容 |
|---|---|---|
| `name` | なし（必須） | カメラの名前。英字で始まり、英数字と`_`だけを使う。トピック名に使う |
| `device` | なし（必須） | `so101_list_cameras`で表示されたUSBポートのパス（`/dev/v4l/by-path/…`） |
| `width`・`height` | 640・480 | 画像の幅と高さ（ピクセル） |
| `fps` | 30 | フレームレート |
| `parameters` | なし | usb_camのパラメータ。書いた値をそのまま渡す |

`pixel_format`は`mjpeg2rgb`を指定してください。
usb_camの既定（`yuyv`）では、圧縮画像のトピック（`image_raw/compressed`）がグレースケールになります。
カメラがMJPEGに対応していない場合は、`yuyv2rgb`を指定します。
MJPEGのほうがUSBの転送量が少ないため、複数のカメラを同時に使う場合にも向いています。

この設定ファイルは、カメラの一覧として`cameras.launch.py`が読み込みます。ROSのパラメータファイルではありません。

起動時に次の内容を確認し、当てはまる場合は理由を表示して終了します。

| 確認する内容 | 表示 |
|---|---|
| `device`がUSBポートのパス（`/dev/v4l/by-path/…`）か | `… set device to the USB port path …` |
| `device`のパスが存在するか | `… not found. Connect the camera to the same USB port …` |
| `device`が映像を取得できるデバイスか | `… does not capture video …` |
| 2台のカメラが同じデバイスを指していないか | `Cameras 'front' and 'wrist' both use …` |
| 名前・数値の形式 | 誤りのある項目 |

## 起動

テレオペと一緒に起動する場合は、`cameras:=true`を付けます。

```bash
ros2 launch so101_driver teleop.launch.py cameras:=true
```

カメラの設定に誤りがある場合は、アームのノードを起動する前に終了します。
起動後にカメラが停止しても、テレオペは継続します。ターミナルのエラー表示を確認してください。

カメラだけを起動することもできます。

```bash
ros2 launch so101_driver cameras.launch.py
```

どちらも`cameras_file:=/path/to/cameras.yaml`で、別の設定ファイルを指定できます。

## 起動中・終了時のメッセージ

| 表示 | 原因と対処 |
|---|---|
| `Unable to start stream`でカメラが終了する | カメラが映像の送信を開始できなかった。ほかのプログラムが同じカメラを使っていないかを確認し、起動し直す |
| 終了時に`process has died [… exit code -11 …]` | usb_cam 0.8.1の終了処理の不具合。`pixel_format`が`mjpeg2rgb`・`yuyv2rgb`のとき、終了するたびに表示される。配信中の画像には影響しない |

## トピック

カメラごとに、`/cameras/<name>`の下へ配信します。

| トピック | 型 | 内容 |
|---|---|---|
| `/cameras/<name>/image_raw` | sensor_msgs/msg/Image | 画像（`mjpeg2rgb`・`yuyv2rgb`では`rgb8`） |
| `/cameras/<name>/image_raw/compressed` | sensor_msgs/msg/CompressedImage | JPEGで圧縮した画像。記録に向く |
| `/cameras/<name>/camera_info` | sensor_msgs/msg/CameraInfo | 解像度と校正値 |

圧縮画像のトピックは、`compressed_image_transport`がインストールされている場合に配信します。
画像の`header.frame_id`は`<name>_optical_frame`です。

画像は1枚あたりのデータが大きいため、購読側はReliableにしてください。
`ros2 topic hz`などBestEffortで購読するツールでは、受信の欠落により実際より低い頻度が表示される場合があります。

## カメラの校正値

usb_camは、`~/.ros/camera_info/<name>.yaml`があれば校正値として読み込み、`camera_info`で配信します。
ファイルがない場合は、起動時に`Unable to open camera calibration file`と表示し、校正値なしで配信します。画像の配信には影響しません。

## 記録の例

アームの関節角度とカメラの圧縮画像を、rosbagに記録します。

```bash
ros2 bag record /leader/joint_states /follower/joint_states /cameras/front/image_raw/compressed
```
