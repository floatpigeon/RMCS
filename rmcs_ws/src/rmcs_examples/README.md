# 指南

## 相机使用

linux下使用相机非常方便，工业相机直接通过USB接到电脑上就能识别了

不过要注意使用USB3的接口和线材，否则跑不满帧率

使用WSL稍微麻烦一点，因为WSL无法直接使用USB硬件

解决步骤：

一、window方面的准备，从命令行安装usbipd

``` 
winget install usbipd
```

执行`usbipd list`，可以找到接入的所有设备，注意有一栏STATE，如果想要在WSL中使用的设备是`Not shared`，需要执行以下命令

```
sudo usbipd bind --busid x-x

#  x-x替换成你需要在WSL中使用的设备的BUSID，例如
sudo usbipd bind --busid 2-5
```


以上完成后，确保执行`usbipd list`后指定的设备STATE为Shared

然后执行
```
usbipd attach --wsl --busid x-x

# 同样的，x-x换成BUSID
usbipd attach --wsl --busid 2-5
```

二、以上完成后在WSL或者容器里面执行`lsusb`，可以找到想要的设备，但是这个时候去启动调用相机的代码，可能出现还是找不到相机的问题，这大概是因为没有权限。

修改用户组：将用户添加到 plugdev 或 dialout 用户组
```
sudo usermod -aG plugdev xxx
# 将xxx换成你的用户名
```

完成后需要重启WSL，退出到windows，执行`wsl --shutdown`，再次进入就生效了

尝试一下执行你的代码，看看还有没有找不到相机的报错，并且尝试一下插拔一下相机再次走一遍步骤一

如果还有，那就需要修改udev规则了

首先执行`lsusb`，查看厂商ID和产品ID
```
Bus 002 Device 004: ID 2bdf:0001 Hikrobot MV-CS016-10UC

# 一般以这种形式出现：厂商ID:产品ID，上述内容中就是2bdf:0001
```

新建或者修改一个udev规则文件，例如，你可以创建一个名为`/etc/udev/rules.d/99-usb-permissions.rules`的文件

添加一行内容
```
SUBSYSTEM=="usb", ATTR{idVendor}=="<你的供应商ID>", ATTR{idProduct}=="<你的产品ID>", MODE="0666"
```

最后，重新加载 udev 规则并触发
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重启WSL，应该就能解决问题了