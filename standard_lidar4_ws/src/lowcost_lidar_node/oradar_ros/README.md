# ORADAR ROS package 2.0.4
ORADAR ROS包用于连接Oradar MS500激光雷达，此ROS包支持indigo，kinetic，melodic等ROS版本。

# 使用方法： 
1. 在系统中安装ROS环境，具体安装方法参考下面链接：  
    安装链接：http://wiki.ros.org/kinetic/Installation/Ubuntu  
    搭建ROS工程链接：http://wiki.ros.org/cn/ROS/Tutorials/InstallingandConfiguringROSEnvironment  
  
2. 将源码复制到ros工作目录下的src目录
```
   mkdir -p ~/lidar_ros_ws/src  
   cp -ar oradar_ros ~/lidar_ros_ws/src/
```

3. 编译工程
```  
  cd ~/lidar_ros_ws/
  catkin_make
```
4. 设置环境变量
```
  source devel/setup.bash
```

5. 配置Ubuntu系统IP  
    与雷达连接的网卡IP： 默认雷达IP为192.168.1.100, Ubuntu系统IP可配置为192.168.1.10（不能和雷达IP一样）
  
6. 配置雷达参数  
    打开oradar_ros/launch/ms500_scan.launch 进行参数配置  
    或者oradar_ros/launch/ms500_pointcloud.launch 进行参数配置  
    参数说明：
    1）device_model雷达设备型号，default=MS500
    2）frame_id 雷达id，default=laser_frame  
    3）scan_topic 雷达的topic名称，default=scan  
    4）angle_min 最小角度，单位度，取值范围 [-135,135],default=-135，即-135度  
    5）angle_max 最大角度，单位度，取值范围 [-135,135],default=135，即135度   
    6）range_min 最小距离，单位米，default=0.05   
    7）range_max 最大距离，单位米，default=30.0  
    8）inverted 是否设置倒装，取值范围true，false。default=false  
    9）motor_speed 雷达转速，单位Hz，取值范围为10,15,20,25,30。default=15Hz  
    10）lidar_ip 所要连接的雷达IP地址，默认为192.168.1.100  
    11）lidar_port 所要连接的雷达端口号，默认为2007  
    12）filter_size 雷达滤波等级，取值范围0,1,2,3,4,5。default=1
  13) motor_dir 雷达电机旋转方向，取值范围0(逆时针)，1(顺时针)。default=0

7. 启动Oradar ROS节点  
   发布LaserScan消息  
    1.roslaunch oradar_ros ms500_scan.launch  
    2.roslaunch oradar_ros ms500_scan_view.launch (使用rviz显示)  
   发布PointCloud消息   
    1.roslaunch oradar_ros ms500_pointcloud.launch  
    2.roslaunch oradar_ros ms500_pointcloud_view.launch (使用rviz显示） 

