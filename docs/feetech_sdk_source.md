# Feetech SDK

**English** | [日本語](ja/feetech_sdk_source.md)

Motor communication uses the official Feetech [FTServo_Python](https://github.com/ftservo/FTServo_Python) SDK.

| Item | Value |
|---|---|
| Provider | Feetech / FTServo |
| Location | `vendor/FTServo_Python` (Git submodule) |
| Installed as | `so101_driver._vendor.scservo_sdk` (inside the driver package) |
| Commit | `cbcfa64674d592f7e5028ae72af42580f60500b4` |
| License | MIT (`vendor/FTServo_Python/LICENSE`, also installed to `share/so101_driver/licenses/FTServo_Python/`) |

## Get the SDK

[Installation in the README](../README.md#installation) fetches the SDK together with the package.
If you already cloned the repository without the SDK, run the following in the repository directory.
Run it as well if the build fails with an error asking for the SDK, then build again.

```bash
git submodule update --init --recursive
```

The SDK is built with the package and installed inside the `so101_driver` package.
LeRobot's pip package `feetech-servo-sdk` also provides a top-level `scservo_sdk`, but because the install locations differ, both can be used in the same Python environment.

### Upgrade from an earlier version

Earlier versions installed the SDK as a top-level `scservo_sdk`.
In a workspace built with such a version, the old `scservo_sdk` remains in the build output even after rebuilding.
Run the following in the workspace directory to remove the build output, then rebuild:

```bash
rm -rf build/so101_driver install/so101_driver
colcon build --packages-select so101_driver
```

## API and examples

See the source and examples at the commit in the table above in the [official SDK repository](https://github.com/ftservo/FTServo_Python).
The SDK code in the submodule is not modified.
