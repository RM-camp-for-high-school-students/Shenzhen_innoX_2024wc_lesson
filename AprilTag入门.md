# AprilTag入门

## 安装

```shell
sudo apt install ros-noetic-apriltag-ros
sudo apt install ros-noetic-camera-calibration
sudo apt install ros-noetic-usb-cam
```

## 编译WSL内核以安装驱动

以下折腾全都是使用WSL情况下的坑，如果你使用的是其它类型的Ubuntu，可以跳过WSL相关部分。

首先，安装依赖：

```shell
sudo apt install build-essential flex bison libssl-dev libelf-dev libncurses-dev autoconf libudev-dev libtool dwarves
```

确认内核版本：

```shell
uname -r
5.15.133.1-microsoft-standard-WSL2
```

在GitHub上找到对应的Tag，例如：

- [microsoft/WSL2-Linux-Kernel at linux-msft-wsl-5.15.133.1 (github.com)](https://github.com/microsoft/WSL2-Linux-Kernel/tree/linux-msft-wsl-5.15.133.1)

然后clone下来：

```shell
git clone --depth 1 --branch linux-msft-wsl-5.15.133.1 https://github.com/microsoft/WSL2-Linux-Kernel.git
```

这种clone方式只会下载当前的代码，可以大幅减少下载的流量和时间消耗。

复制当前的配置文件并使用menuconfig来配置：

```shell
cd WSL2-Linux-Kernel
make menuconfig KCONFIG_CONFIG=Microsoft/config-wsl
```

以下内容都要选上：

> *「Device Drivers」→「Multimedia support」→「Filter media drivers」enable.
> *「Device Drivers」→「Multimedia support」→「Media device types」→「Cameras and video grabbers」enable.
> *「Device Drivers」→「Multimedia support」→「Media core support」→「Video4Linux options」→「V4L2 sub-device userspace API」enable.
> *「Device Drivers」→「Multimedia support」→「Media core support」→「Media drivers」→「Media USB Adapters」→「USB Video Class (UVC)」and「UVC input events device support」enable.
> *「Device Drivers」→「Multimedia support」→「Media core support」→「Media drivers」→「Media USB Adapters」→「GSPCA based webcams」enable. 以及里面所有的子项

进行编译：

```shell
sudo make -j $(nproc) KCONFIG_CONFIG=Microsoft/config-wsl
```

将内核文件复制出来：

```shell
cp arch/x86/boot/bzImage /mnt/c/Users/<user>/usbip-bzImage
```

在Windows的用户目录下，新建`.wslconfig`文件，在其中写入：

```
[wsl2]
kernel=c:\\users\\<user>\\usbip-bzImage
```

重启wsl后，可以用`uname -a`根据内核编译时间判定是否替换成功。

## WSL连接USB设备

由于WSL无法直接读取视频设备，这里需要一些额外手段，在这里下载USBIPB：

- [Releases · dorssel/usbipd-win (github.com)](https://github.com/dorssel/usbipd-win/releases)

在WSL中执行以下命令：

```shell
sudo apt install linux-tools-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20
```

在Windows中列出所有的设备：

```powershell
usbipd list
```

根据busid共享设备（我的设备上是2-2，根据自己的设备来）：

```powershell
usbipd bind -b 2-2
```

将设备附加到WSL上：

```powershell
usbipd attach --wsl --busid 2-2
```

为其它用户赋予摄像头的使用权：

```shell
sudo chgrp video /dev/video*
sudo chmod g+rw /dev/video*
```

在WSL中，可以使用`lsusb`查看可用的设备，然后，使用`ls -l /dev/video*`确认是否成功。

参考文档：

- [连接 USB 设备 | Microsoft Learn](https://learn.microsoft.com/zh-cn/windows/wsl/connect-usb)
- [Xuanhoang214/WSL2-USB-CAMERA: Building your own USB/IP enabled WSL 2 kernel 5.10.102.1 (github.com)](https://github.com/Xuanhoang214/WSL2-USB-CAMERA)
- [WSL support · dorssel/usbipd-win Wiki (github.com)](https://github.com/dorssel/usbipd-win/wiki/WSL-support)

## 标定摄像头

```shell
roslaunch usb_cam usb_cam-test.launch
```
