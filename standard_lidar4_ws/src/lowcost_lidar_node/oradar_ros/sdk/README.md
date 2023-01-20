
# MS500 SDK基本介绍
MS500 SDK是专门为Oradar MS500激光雷达产品设计的软件开发套件。提供易于使用的C/C++风格的API。通过MS500 SDK，用户可以快速连接Oradar MS500激光雷达并接收激光雷达点云数据。

# 运行环境
- Linux
- Windows7、10

# 运行要求
- C++11编译器
- CMake，版本号为2.8.3或更高

# 编译方法

Linux下编译：
```
cd sdk
mkdir build
cd build
cmake ..
make
```
Windows下编译：

按住shift键，鼠标右键，打开powershell
```
cd sdk
mkdir build
cd build
cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ..
mingw32-make -j8
```

生成`ord_sdk.a`库文件和`scan_frame_test`可执行文件


# SDK 主要函数API说明
|函数名称 | 功能介绍|
|---------|---------------|
|open| 打开雷达设备 |
|close| 关闭雷达设备 |
|isOpened| 查询是否已经处于open状态 |
|trackConnect           | 连接设备 |
|enableMeasure   | 设备开启 |
|disableMeasure          | 设备关闭 |
|enabelDataStream  | 使能雷达点云数据传输 |
|disableDataStream      | 关闭雷达点云数据传输 |
|getScanBlockData      | 获取一个block点云数据 |
|getScanFrameData          | 获取一圈点云数据(一圈有6个block) |
|setLidarIPAddress    | 设置雷达IP地址 |
|getLidarIPAddress              | 获取雷达IP地址 |
|setLidarNetPort              | 设置雷达端口号 |
|setScanSpeed | 设置雷达扫描频率 设置范围[15、20、25、30]。单位为Hz|
|getScanSpeed | 获取雷达扫描频率 |
|setTailFilterLevel | 设置点云滤波等级, 设置范围为[0、1、2、3、4、5]。0表示关闭滤波功能 |
|getTailFilterLevel | 获取点云滤波等级 |
|setTimestamp | 设置雷达时间戳 |
|getTimestamp | 获取雷达时间戳 |
|applyConfigs | 参数配置信息保存（当对雷达参数进行设置后，调用该接口才能将配置信息保存到flash中，重启后得到保持） |
|deviceReboot | 设备复位 |
|getFirmwareVersion | 获取固件版本号 |
|getHardwareVersion | 获取硬件版本号 |
|getSDKVersion | 获取SDK版本号 |
|getMotorSpeed | 获取电机实时转速，单位RPM |
|getInternalTemperature | 获取雷达内部的温度信息，保留小数点后两位。单位为摄氏度|
|setTimeout | 设置API超时时间，单位为毫秒。默认3000毫秒 |


# 示例使用说明
scan_frame_test是一个简单从MS500激光雷达获取一圈数据的示例程序，连接雷达成功会一直打印出一圈的点云个数，值为1512。

Linux下：

将MS500激光雷达设备通过网线，连接到Ubuntu系统，在Ubuntu系统下打开终端，把对应网卡IP设置为`192.168.1.10`。然后执行SDK Sample,输入如下命令：

```
cd sdk/build
./scan_frame_test                 #获取一圈数据测试程序
```
Windows下:

将MS500激光雷达设备通过网线，连接到Windows系统，在网络设置中把对应网卡IP设置为`192.168.1.10`。直接双击`scan_frame_test.exe`可执行文件即可。

