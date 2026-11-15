# キャリブレーション

[English](../so101_calibration.md) | **日本語**

`so101_calibrate`で、リーダー・フォロワーの基準姿勢、関節の可動範囲、グリッパーの閉・開位置を記録します。
結果はJSONに保存し、関節角度の変換と制御目標の範囲制限に使用します。
ソフトウェア上の記録のみで、サーボ内の原点補正や可動範囲は変更しません。

## 始める前に

- 対象アームの[デバイスIDを確認](so101_setup.md#デバイスidの確認)しておきます。
- ドライバーのlaunch（制御インターフェース・スライダーGUI・テレオペ）と、ほかの`so101_calibrate`を終了しておきます。リーダー・フォロワー・キャリブレーションのノードが起動中の場合は、開始せずに終了します。
- キャリブレーションは1台ずつ行います。

## 起動

```bash
ros2 run so101_driver so101_calibrate --role leader --device-id "your-leader-serial"
ros2 run so101_driver so101_calibrate --role follower --device-id "your-follower-serial"
```

`--device-id`を省略すると、[設定ファイル](so101_setup.md#設定ファイル)（`~/.ros/so101_driver/config/`の`leader.yaml`または`follower.yaml`）の`device_id`を使用します。

## 手順

1. RVizの青い見本と同じ姿勢にし、グリッパーを閉じてEnterを押します。
2. 全関節とグリッパーを動かして可動範囲を記録します。関節の順序は自由で、同時に動かしても構いません。
3. Enterを押すと記録を終了し、保存します。

記録中は、ターミナルに各関節の最小値・最大値をtick単位で表示します。
記録不足の関節がある場合は、動かす関節を表示するので、記録を再開して追加できます。
`q`またはCtrl+Cで中止します。

`wrist_roll`以外の各関節には、10回以上の有効な取得と20 tick以上の移動が必要です。
範囲が4000 tickを超える関節は、エンコーダーの0と4095の境界をまたいだ可能性があるため受け付けません。

### RVizの表示

開始前は青い見本だけを表示します。
Enterで基準姿勢を記録すると、その隣にグレーの現在姿勢が表示されます。
青い見本は固定され、グレーが実機の動きに追従します。マウスで視点を変更できます。
通信異常時はグレーのモデルが最後の姿勢のまま残ることがあるため、ターミナルの状態表示も確認してください。

### グリッパーとwrist_roll

グリッパーは開始時の位置を「閉」、そこから最も離れた記録位置を「開」として扱います。
`wrist_roll`は全周（0〜4095 tick）を可動範囲として保存します。

## トルクが有効な場合

トルクが有効な軸があると、そのままでは開始できません。
解除してキャリブレーションする場合は`--disable-torque`を付けます。

```bash
ros2 run so101_driver so101_calibrate --role follower \
  --device-id "your-follower-serial" --disable-torque
```

案内に従って腕を支え、Enterを押すとトルクを解除して進みます。
`q`で解除前に中止できます。終了・中止後も、解除したトルクはOFFのままです。
リーダーでも同じオプションを使えます。

## 保存先と適用

| 対象 | 既定の保存先（`~/.ros/so101_driver`内） |
|---|---|
| リーダー | `calibration/leader.json` |
| フォロワー | `calibration/follower.json` |

既存ファイルがある場合は、起動直後にターミナルで上書きを確認します。
`y`または`yes`で続行し、Enterまたは`n`で終了します。
既存の内容は、新しい結果の保存が完了するまで変更しません。

テンプレートから作成した設定YAMLは、上記の保存先を参照します。
保存先を変更した場合は、対象YAMLの`calibration_profile`も変更してください。

```yaml
calibration_profile: calibration/follower.json
```

`calibration_profile`の相対パスは`~/.ros/so101_driver`を基準に解決します。
一方、`--output`に指定する相対パスは実行ディレクトリを基準にします。

## オプション

| オプション | 内容 |
|---|---|
| `--role leader\|follower` | 対象の役割。既定は`leader` |
| `--device-id` | USBシリアル番号。省略時は役割別YAMLの値 |
| `--output` | 保存先 |
| `--no-rviz` | 見本の表示（RVizとrobot_state_publisher）を起動せずに実行 |
| `--disable-torque` | 確認後に、有効なトルクを解除 |

## トラブルシューティング

| 表示・状況 | 確認する内容 |
|---|---|
| `Stop the running driver nodes and other calibration sessions first.` | ドライバーのノードと、ほかの`so101_calibrate`を終了する |
| `... is in use by another process` | 同じアームのUSBポートを、ほかのプロセスが使用している |
| `Torque is enabled on some ... joints` | `--disable-torque`を付けて実行し直す |
| `Some servos are not in position mode` / `Found a servo that is not an STS3215` | サーボの設定や型番がSO-101と一致しない。設定は変更していない |
| `Not enough data recorded` | 表示された関節をさらに動かし、記録を再開する |

### キャリブレーション結果を使用できない場合

リーダー用・フォロワー用のファイルを取り違えると、起動時にエラー（`Calibration role mismatch`）になります。
サーボの原点補正・可動範囲・動作モードが記録時と異なる場合も、不一致（`Hardware changed since calibration`）を通知して起動を停止します。
対象機体と設定を確認し、サーボ設定を変更した場合はキャリブレーションし直してください。
