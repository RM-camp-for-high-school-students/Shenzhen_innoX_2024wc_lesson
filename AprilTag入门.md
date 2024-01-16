# AprilTag入门

首先，为了使用摄像头，建议在原生Ubuntu系统上跑程序。

## 安装

```shell
sudo apt-get install ros-noetic-apriltag-ros
sudo apt-get install ros-noetic-usb-cam
sudo apt-get install ros-noetic-camera-calibration
```

## 检查摄像头

检查usb是否连接成功：

```shell
lsusb
```

检查摄像头是否识别成功：

```shell
ls -l /dev/video*
```

运行测试程序：

```shell
roslaunch usb_cam usb_cam-test.launch
```

运行标定程序，其中size为标定纸的大小，square为格子的边长（单位米）：

```shell
rosrun camera_calibration cameraclibrator.py --size 7x5 --square 0.042 image:=/usb_cam/image_raw camera:=/usb_cam
```

上下左右移动摄像头，当calibrate变亮之后点击它，等待一会后save然后commit。标定文件默认保存在`~/.ros/camera_info/head_camera.yaml`。