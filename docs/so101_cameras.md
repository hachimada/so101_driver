# Cameras

**English** | [日本語](ja/so101_cameras.md)

Alongside teleoperation, the package can publish images from USB cameras (UVC) on ROS topics.
The cameras are driven by [usb_cam](https://index.ros.org/p/usb_cam/), the standard ROS camera driver.

## Install

usb_cam is not a dependency of so101_driver. Install it when you use cameras:

```bash
sudo apt install ros-humble-usb-cam
```

## Find the camera path

Each camera is selected by a path that depends on the USB port it is plugged into (`/dev/v4l/by-path/…`).
Cameras of the same model can be told apart as long as they use different ports.
As with the arm's [device ID](so101_setup.md#find-the-device-id), you identify a camera by unplugging it.

Connect all the cameras you will use, then run:

```bash
ros2 run so101_driver so101_list_cameras
```

1. The connected cameras are listed. The `NAME IN CONFIG` column shows the camera name already set in `cameras.yaml`.
2. When prompted, unplug the USB cable of one camera and press Enter.
3. The path of the unplugged camera and the lines to add to the settings file are shown. If `cameras.yaml` does not exist, it is created from the template.
4. Plug the cable back into the same USB port.

Repeat for each camera.
Press `q` or Ctrl+C to quit.

The path depends on the USB port, so it changes if you move a camera to another port.
In that case, run the tool again and update `cameras.yaml`.

To see the pixel formats, resolutions and frame rates a camera supports, use `v4l2-ctl` (from the `v4l-utils` package):

```bash
v4l2-ctl --list-formats-ext -d /dev/v4l/by-path/…-video-index0
```

## Configure

List the cameras in `~/.ros/so101_driver/config/cameras.yaml`, created by `so101_list_cameras`.
An example with two cameras:

```yaml
cameras:
  - name: front
    device: /dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0
    width: 640
    height: 480
    fps: 30
    parameters:
      pixel_format: mjpeg2rgb
  - name: wrist
    device: /dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0
    parameters:
      pixel_format: mjpeg2rgb
```

| Key | Default | Description |
|---|---|---|
| `name` | None (required) | Camera name. Starts with a letter and uses only letters, digits and `_`. Used in the topic names |
| `device` | None (required) | USB port path shown by `so101_list_cameras` (`/dev/v4l/by-path/…`) |
| `width`, `height` | 640, 480 | Image width and height in pixels |
| `fps` | 30 | Frame rate |
| `parameters` | None | usb_cam parameters, passed as they are |

Set `pixel_format` to `mjpeg2rgb`.
With the usb_cam default (`yuyv`), the compressed image topic (`image_raw/compressed`) becomes grayscale.
If the camera does not support MJPEG, use `yuyv2rgb`.
MJPEG also uses less USB bandwidth, which helps when several cameras run at once.

This file is a camera list read by `cameras.launch.py`, not a ROS parameter file.

At startup, the launch file checks the following and exits with the reason if one fails:

| Check | Message |
|---|---|
| `device` is a USB port path (`/dev/v4l/by-path/…`) | `… set device to the USB port path …` |
| The `device` path exists | `… not found. Connect the camera to the same USB port …` |
| `device` is a device that captures video | `… does not capture video …` |
| No two cameras use the same device | `Cameras 'front' and 'wrist' both use …` |
| Names are unique and valid, there are no unknown keys, and the numbers are positive | The key that is wrong |
| usb_cam is installed, the file exists and lists at least one camera | `usb_cam is not installed …`, `… not found …`, `No cameras are listed …` |

## Launch

To start the cameras with teleoperation, add `cameras:=true`:

```bash
ros2 launch so101_driver teleop.launch.py cameras:=true
```

If the camera settings are wrong, the launch exits before starting the arm nodes.
If a camera stops later, teleoperation keeps running. Check the error in the terminal.

You can also start only the cameras:

```bash
ros2 launch so101_driver cameras.launch.py
```

Both accept `cameras_file:=/path/to/cameras.yaml` to use another settings file.

## Messages while running and at shutdown

| Message | Cause and action |
|---|---|
| The camera exits with `Unable to start stream` | The camera could not start streaming. Check that no other program is using the same camera, and launch again |
| `process has died [… exit code -11 …]` at shutdown | A bug in the shutdown of usb_cam 0.8.1. It appears at every shutdown when `pixel_format` is `mjpeg2rgb` or `yuyv2rgb`, and does not affect the published images |

## Topics

Each camera publishes under `/cameras/<name>`.

| Topic | Type | Description |
|---|---|---|
| `/cameras/<name>/image_raw` | sensor_msgs/msg/Image | Image (`rgb8` with `mjpeg2rgb` and `yuyv2rgb`) |
| `/cameras/<name>/image_raw/compressed` | sensor_msgs/msg/CompressedImage | JPEG-compressed image, suited to recording |
| `/cameras/<name>/camera_info` | sensor_msgs/msg/CameraInfo | Resolution and calibration |

The compressed image topic is published when `compressed_image_transport` is installed.
The `header.frame_id` of the images is `<name>_optical_frame`.

Each image carries a lot of data, so subscribe with Reliable QoS.
Tools that subscribe with BestEffort, such as `ros2 topic hz`, can drop messages and show a lower rate than the actual one.

## Camera calibration

If `~/.ros/camera_info/<name>.yaml` exists, usb_cam loads it as the camera calibration and publishes it on `camera_info`.
Without it, usb_cam prints `Unable to open camera calibration file` at startup and publishes without calibration. Image publishing is not affected.

## Recording example

Record the arm joint angles and the compressed camera images to a rosbag:

```bash
ros2 bag record /leader/joint_states /follower/joint_states /cameras/front/image_raw/compressed
```
