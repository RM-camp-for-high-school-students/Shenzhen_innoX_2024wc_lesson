# WSL安装

第一步，启用WSL：

```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

第二步，启用虚拟机功能：

```powershell
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

现在需要重启计算机。

第三步，下载并安装内核包：

- https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi

第四步，设置WSL2为默认版本：

```powershell
wsl --set-default-version 2
```

第五步，使用`wsl -l -o`检查一共有哪些WSL发行版可供使用，会显示以下内容：

```
NAME                                   FRIENDLY NAME
Ubuntu                                 Ubuntu
Debian                                 Debian GNU/Linux
kali-linux                             Kali Linux Rolling
Ubuntu-18.04                           Ubuntu 18.04 LTS
Ubuntu-20.04                           Ubuntu 20.04 LTS
Ubuntu-22.04                           Ubuntu 22.04 LTS
OracleLinux_7_9                        Oracle Linux 7.9
OracleLinux_8_7                        Oracle Linux 8.7
OracleLinux_9_1                        Oracle Linux 9.1
openSUSE-Leap-15.5                     openSUSE Leap 15.5
SUSE-Linux-Enterprise-Server-15-SP4    SUSE Linux Enterprise Server 15 SP4
SUSE-Linux-Enterprise-15-SP5           SUSE Linux Enterprise 15 SP5
openSUSE-Tumbleweed                    openSUSE Tumbleweed
```

我们需要20.04版本，之后使用`wsl --install Ubuntu-20.04`安装即可。

安装完成后，可以使用`wsl -l -v`检查版本信息，应该是类似于这样：

```
  NAME            STATE           VERSION
* Ubuntu-20.04    Stopped         2
```

这只是提取了官方文档中的关键部分，完整文档参见：

- [安装 WSL | Microsoft Learn](https://learn.microsoft.com/zh-cn/windows/wsl/install)

- [旧版 WSL 的手动安装步骤 | Microsoft Learn](https://learn.microsoft.com/zh-cn/windows/wsl/install-manual)
