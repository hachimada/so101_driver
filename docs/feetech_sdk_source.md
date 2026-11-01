# Feetech公式Python SDK

モーター通信にはFeetech公式の[FTServo_Python](https://github.com/ftservo/FTServo_Python)を使用します。

| 項目 | 内容 |
|---|---|
| 提供元 | Feetech / FTServo |
| 配置先 | `vendor/FTServo_Python`（Gitサブモジュール） |
| インストール先 | `so101_driver._vendor.scservo_sdk`（ドライバーのパッケージ内） |
| 使用コミット | `cbcfa64674d592f7e5028ae72af42580f60500b4` |
| ライセンス | MIT（`vendor/FTServo_Python/LICENSE`。`share/so101_driver/licenses/FTServo_Python/`にもインストール） |

## 取得

[READMEのインストール](../README.md#インストール)では、SDKも一緒に取得します。
クローン済みでSDKが未取得の場合は、リポジトリのディレクトリで次を実行してください。
SDKの取得を求めるビルドエラーが出た場合も、このコマンドで取得してから再ビルドしてください。

```bash
git submodule update --init --recursive
```

SDKはパッケージと一緒にビルドし、`so101_driver`パッケージの中にインストールします。
LeRobotが使うpipの`feetech-servo-sdk`もトップレベルの`scservo_sdk`を提供しますが、インストール先が異なるため、同じPython環境で併用できます。

### 以前のバージョンからの更新

以前のバージョンでは、SDKをトップレベルの`scservo_sdk`としてインストールしていました。
その頃にビルドしたワークスペースでは、再ビルドしても古い`scservo_sdk`がビルド結果に残ります。
ワークスペースのディレクトリで次を実行し、ビルド結果を削除してから再ビルドしてください。

```bash
rm -rf build/so101_driver install/so101_driver
colcon build --packages-select so101_driver
```

## APIとサンプル

[公式SDKのリポジトリ](https://github.com/ftservo/FTServo_Python)で、上の表の使用コミットのソースとサンプルを参照してください。
サブモジュールのSDKのコードは変更していません。
