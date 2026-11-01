# キャリブレーション

リーダー・フォロワーの基準姿勢、関節の可動範囲、グリッパーの閉・開位置を記録します。
結果はJSONに保存し、関節角度の変換と制御範囲に使用します。サーボ内の原点補正や可動範囲は変更しません。

## 起動

[デバイスIDを確認](so101_setup.md#デバイスidの確認)し、対象アームを指定します。

```bash
ros2 run so101_driver so101_calibrate --role leader --device-id "your-leader-serial"
ros2 run so101_driver so101_calibrate --role follower --device-id "your-follower-serial"
```

対象のコマンドを1つ実行してください。
制御ノードや別の`so101_calibrate`が起動中の場合は、開始せずに終了します。
`--device-id`を省略すると、[設定ファイル](so101_setup.md#設定ファイル)（`~/.ros/so101_driver/config/`の`leader.yaml`または`follower.yaml`）の値を使用します。

## 操作

1. RVizの青い見本と同じ姿勢にし、グリッパーを閉じてEnterを押します。
2. 全関節とグリッパーを動かして可動範囲を記録します。関節の順序は自由で、同時に動かしても構いません。
3. Enterを押すと記録を終了し、保存します。

ターミナルには各関節の最小値・最大値をtick単位で表示します。
記録不足の関節がある場合は、その関節を動かして追加記録できます。
`q`またはCtrl+Cで中止します。

開始前は青い見本だけを表示します。Enterで基準姿勢を記録すると、左にグレーの現在姿勢が表示されます。
青い見本は固定され、グレーが実機の動きに追従します。マウスで視点を変更できます。
通信異常時は最後の姿勢が残ることがあるため、ターミナルの状態表示も確認してください。

グリッパーは開始時の姿勢を「閉」、そこから最も離れた記録位置を「開」として扱います。
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
`y`または`yes`で続行し、Enterまたは`n`で終了します。既存の内容は保存を完了するまで変更しません。

テンプレートから作成した設定YAMLは、上記の保存先を参照します。
保存先を変更した場合は、対象YAMLの`calibration_profile`も変更してください。

```yaml
calibration_profile: calibration/follower.json
```

`calibration_profile`の相対パスは`~/.ros/so101_driver`を基準に解決します。
一方、CLIの`--output`に指定する相対パスは実行ディレクトリを基準にします。

## オプション

| オプション | 内容 |
|---|---|
| `--role leader\|follower` | 対象の役割。既定はleader |
| `--device-id` | USBシリアル番号。省略時は役割別YAMLの値 |
| `--output` | 保存先を指定 |
| `--no-rviz` | 見本の表示（RVizとrobot_state_publisher）を起動せずに実行 |
| `--disable-torque` | 操作確認後にトルクを解除 |

## 結果を使用できない場合

リーダー用・フォロワー用のファイルを取り違えると起動時にエラーになります。
サーボの原点補正や可動範囲などが記録時と異なる場合も、不一致を通知して起動を停止します。
対象機体と設定を確認し、サーボ設定を変更した場合はキャリブレーションし直してください。
