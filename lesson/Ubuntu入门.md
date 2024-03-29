# Ubuntu 20.04 双系统安装

## 镜像地址

[Ubuntu 20.04.6 LTS (Focal Fossa)](http://releases.ubuntu.com/20.04/)

## 安装前的准备

准备一个至少8G的空U盘，使用UltraISO（或其它类似的软件）将下载的镜像制作为启动盘。

使用系统分区软件（如DiskGenius）在硬盘上划分出至少50G的空闲空间。

保持U盘插入的前提下，重启电脑进入BIOS，确认刚刚制作的U盘启动盘是第一个启动顺序，并关闭会影响启动的一些选项。这一步不同的电脑操作方式不同，如果遇到问题最好根据自己的电脑型号上网搜索解决方案。

## 安装Ubuntu

再重启电脑，应该进入了Ubuntu的引导，此时选择Install Ubuntu。语言先选择英文。不要连接网络。选择最小安装。安装类型选择Something else（重要）。

然后就是很重要的分区环节，选中你之前划分出的空闲空间，按照以下方式划分：

- 500MB，主分区，空间开始，用于EFI

- 等同于内存大小的空间，逻辑分区，空间开始，用于swap area

- 剩余的所有空间，逻辑分区，空间开始，用于Etx4，挂载到`/`

然后将Device for boot loader installation选择为你一开始划分的EFI分区，检查无误后继续，后续内容按照提示进行即可。

## 配置Ubuntu

安装完毕后，首先在软件与更新的设置中，更新源（或者`sudo apt-get update`）。然后在Additional Driver里面将显卡驱动安装好。

之后使用软件更新器更新软件（或者`sudo apt-get upgrade`），重启电脑。

可以修改`/etc/default/grub`文件，将主要使用的系统的启动项放到默认选项，并缩短一些自动等待的时间。修改完毕后使用`sudo update-grub`更新配置。

## 一些踩过的坑

以前安装双系统的时候时间会出问题，但是这次我没有出问题，可能是内核更新了？

由于我使用的是几年前的安装镜像，在upgrade之后出现了黑屏问题，可以用进阶选项指定启动的核心版本，这种问题一般是没有安装显卡驱动导致的，所以建议先安装驱动再upgrade。我一开始没有WIFI选项，是因为内核版本过低没有驱动程序，升级内核一般就可以连接WIFI了。

## 参考博客

[Ubuntu20.04+Win10双系统安装 - 皓月当空dy - 博客园](https://www.cnblogs.com/Moon-404/p/13872470.html)

没错是我自己的博客，只不过年代有点久远了。

## 常用指令

- ls：列出目录下所有文件

- cd：进入指定目录

- mkdir：新建文件夹

- rm：删除

- cp：复制

- mv：移动

- sudo：以root权限运行后续指令

- 
