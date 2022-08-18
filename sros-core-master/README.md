# README

SROS(Standard-Robots Operating System)

## How to start

### 开发环境配置

* 操作系统: Ubuntu_16.04 （开发电脑可以用docker编译，不一定要求Ubuntu_16.04）
* 编程语言: c++ 11
* 编译工具: cmake(>= 2.8) + gcc(>= 4.8)
* IDE: 推荐Clion
* 工作路径: `~/workspace/`

### 交叉编译SROS

请参参见：https://standard-robots.yuque.com/sn973i/ads13d/bl37md

### 贡献指南 （必读）

https://standard-robots.yuque.com/sn973i/ads13d
./cfg/sros-resources/README.md

### 目录结构

```shell script
.
├── .gitlab/                              # gitlab相关配置
│   └── merge_request_templates/          # gitlab合并时的提交模板
│     └── 新功能合并请求模板.md              # gitlab新功能合并请求模板
├── cfg/
│   ├── resources/                        # 这个是自动生成的，给Matrix使用的
│   ├── sros-resources/                   # 配置自仓库，具体参见：http://git.standard-robots.com/SROS/sros-resources.git
│   ├── .gitignore                        # GIT忽略路径
│   ├── action_table.csv                  # Matrix获取的动作表（待废弃，从sros-resources中拉取）
│   ├── argparse.py                       # python命令行参数解析库（为了兼容才放到这的，待废弃）
│   ├── config.csv                        # 生成后的配置文件，严禁修改
│   ├── db_schema.sql                     # 数据库原始建表
│   ├── db_schema_readonly_data.sql       # 数据库只读数据建表，升级SROS时重新建议遍
│   ├── db_schema_upgrade_*.sql           # 数据库结构升级脚本
│   ├── db_upgrade_config_table.py        # 对比升级数据库配置脚本
│   ├── generate_db.sh                    # 身材数据库脚本
│   └── main.db3                          # generate_db.sh生成的测试结果
├── cmake/                                # 放一些Cmake文件
│   ├── toradex-toolchain.cmake           # 设置交叉编译链的CMake
│   └── ....
├── core/                                 # 放一些公共的数据结构，公共的工具类
│   ├── db/                               # 全局数据库实例
│   ├── device/                           # 一些设备接口
│   ├── hardware/                         # 具体的硬件驱动，基于device
│   ├── log/                              # glog的一些配置
│   ├── map/                              # 地图相关
│   ├── mission/                          # 任务串的一些数据结构
│   ├── modbus/                           # modbus相关的公共类
│   ├── monitor/                          # 数据监控的一些基础类
│   ├── msg/                              # 模块之间消息通信的消息定义
│   ├── proto/                            # 对外网络通信的protobuf消息定义
│   ├── rack/                             # 货架数据类
│   ├── schedule/                         # 调度（定时让任务、触发任务）的一些技术数据类
│   ├── task/                             # 任务的基础类
│   ├── temp/                             # ???
│   ├── tf/                               # tf相关类
│   ├── usart/                            # 串口通信相关类
│   ├── util/                             # 一些小工具
│   └── ....                              # 一些没法分类的数据结构
├── doc/                                  # 【废弃】所有文档搬迁至语雀：https://standard-robots.yuque.com/sn973i
├── git-hooks                             # git的钩子
├── modbus/                               # SROS所有模块，添加模块方法见：https://standard-robots.yuque.com/sn973i/ads13d/xrof2m
│   ├── action_controller/                # 动作控制器模块
│   ├── camera/                           # 相机模块
│   ├── communication/                    # 通信模块
│   ├── device/                           # 设备模块
│   ├── extension_action_controller/      # 扩展动作控制器模块
│   ├── hmi/                              # 人机交互模块
│   ├── huawei_communication/             # 华为测试线模块
│   ├── laser/                            # 雷达模块
│   ├── location/                         # 定位模块
│   ├── main/                             # 主模块，里面有几个主要的状态机，还有些没法归类的功能
│   ├── mission/                          # 任务模块
│   ├── modbus/                           # 对外modbus通信模块
│   ├── monitor/                          # 系统监控模块
│   ├── msg_recod/                        # 消息记录模块
│   ├── navigation/                       # 导航模块，里面处理路径规划、避障等
│   ├── network/                          # 对外的网络模块
│   ├── obstacle/                         # 障碍模块
│   ├── pipe/                             # 管道模块，与HTTP后台通信
│   ├── posefilter/                       # 位姿融合模块
│   ├── rack_query/                       # 货架腿识别模块
│   ├── registers/                        # 寄存器模块
│   ├── schedule/                         # 调度模块
│   ├── security/                         # 安全模块
│   ├── slam/                             # slam模块
│   ├── station_recognition/              # 站点识别模块
│   ├── timer/                            # 时钟模块
│   ├── touch_screen/                     # 触摸屏v1模块
│   ├── upgrade/                          # 升级模块
│   ├── usart/                            # 【废弃】串口通信模块，后被modbus替代
│   ├── vision/                           # 视觉模块
│   ├── vsc/                              # vsc模块，包括安全电源、电源等
│   └── CMakeLists.txt                    # cmake文件
├── rootfs/                               # 系统相关
├── src/                                  # 和src通信的接口
├── test/                                 # 测试相关
│   ├── simulate/                         # 仿真
│   ├── tools/                            # 测试工具
│   ├── util/                             # C++单元测试
│   └── ....              
├── thirty-party/                         # 第三方库
├── tools/                                # 一些工具类
├── update/                               # 打包脚本
├── web/                                  # web相关
├── .clang-format                         # C++格式化配置文件
├── .gitignore                            # GIT忽略路径
├── .gitlab-ci.yml                        # gitlab CI 配置文件，可参考仓库：http://git.standard-robots.com/SROS/sros-docker.git
├── .gitmodules                           # git跟随子模块文件
├── CMakeLists.txt                        # 顶层CMake文件
├── cpplint.py                            # C++代码静态检测工具
├── main.cpp                              # 程序入库文件
├── package.csv                           # SROS依赖关系，可参考文档：https://standard-robots.yuque.com/sn973i/ads13d/xbaq3m
├── README.md                             # 自述文件（本文件）
├── version.sh                            # 生成的版本文件
└── version.sh.in                         # 生成的cmake版本输入
```

### modbus

#### 参考资料

- [modbus官网](http://www.modbus.org/)
- [simply modebus](http://www.simplymodbus.ca/index.html)
- [MODBUS Application Protocol Specification](http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)

### glog

#### Verbose Logging

- 对于一些用于追踪问题的日志，但是在开发过程中不想打印出来，只有在具体追踪某一问题的时候才打印出来的需求可以用VLOG来实现。
- vlog会将低于等于当前等级的日志都打印出来。
- 由于其他的库也用到了glog，导致只要vlog等级大于0，就会持续输出其他库中的日志，这不是我们想要的.为了解决这个问题，我们将我们的
  vlog等级设置为负数，来规避这样的问题。具体做法：sros启动的时候，加载数据库中的vlog等级，默认为-10。在我们的代码中，
  vlog等级处于大于-10并且小于0的等级，若通过界面配置vlog等级使其大于-10时，就可以输出我们想要的数据了。
- vlog的宏请参见log_helper.hpp中的定义
- 在chip设置中“debug.vlog_v”能够动态设置vlog的等级.
- 开启VLOG打印INFO级输出：GLOG_v=-4 ./sros
- 开启只打印hmi_module.h、hmi_module.cpp的INFO级VLOG日志输出：GLOG_vmodule=hmi_module=-4 ./sros

#### 参考资料

[How To Use Google Logging Library](http://rpg.ifi.uzh.ch/docs/glog.html#verbose)

## 常用指令

### 解压升级包

```shell script
cat SROS-v4.12.3-1ffabc7.update | openssl des3 -md md5 -d -k SROS2016-update | tar xzvf - -C ./ 
```

### 解压数据库
```shell
cat sros.db_export|openssl des3 -md md5 -d -k SROS-DB-EXPORT | tar xzvf - -C ./ 
```