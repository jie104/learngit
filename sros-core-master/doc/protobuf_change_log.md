# protobuf change log


## v4.6.0 -> v4.7.0
### 雷达点不再主动上传，当客户端需要上传时，需要主动设置开启雷达点自动上传
```shell
@@ -78,14 +78,17 @@ enum CommandType {
 
     CMD_SPEED_SLOW_DOWN = 0x26; // [废弃] 减速命令
 
-    CMD_CALIBRATION = 0x27;
+    CMD_CALIBRATION = 0x27; // [废弃]
 
     CMD_TRIGGER_EMERGENCY = 0x28; // 触发急停
 
+    CMD_ENABLE_AUTO_UPLOAD_LASER_POINT = 0x2A; // 设置主动上传雷达点
+    CMD_DISABLE_AUTO_UPLOAD_LASER_POINT = 0x2B; // 设置不主动上传雷达点
+
     CMD_ENTER_POWER_SAVE_MODE = 0x31; // 进入低功耗模式
     CMD_EXIT_POWER_SAVE_MODE = 0x32; // 退出低功耗模式
 
-    CMD_STOP_CHARGE = 0x33; // 停止充电
+    CMD_STOP_CHARGE = 0x33; // 停止充电 （若在充电时，需要先停止充电才能断开agv与充电桩充电接口的连接，不然可能会出火花）
 
     CMD_INPUT_ACTION_VALUE = 0x41; // 设置ActionTask的输入值
 
```

### 不再发送NOTIFY_MOVE_PATH_SENT类型的通知
```shell
@@ -163,7 +166,7 @@ message Notification {
     enum NotifyType {
         NOTIFY_MOVE_TASK_FINISHED = 0;
         NOTIFY_ACTION_TASK_FINISHED = 1;
-        NOTIFY_MOVE_PATH_SENT = 2;
+        NOTIFY_MOVE_PATH_SENT = 2; // [废弃]
         NOTIFY_CALIBRATION_FINISHED = 3;
         NOTIFY_UPDATE_FINISHED = 4;
         NOTIFY_MISSION_LIST_CHANGED = 5;
```

### 添加更严格的错误检测
```shell
@@ -326,22 +329,27 @@ message ResponseResult {
         RESULT_CODE_NO_LOCATING = 11; // 车辆不处于定位状态
         /* end of deprecated */
 
-        RESULT_CODE_USER_OR_PASSWORD_INVALID = 101;
+        RESULT_CODE_USER_OR_PASSWORD_INVALID = 101; // 登录失败，账号或密码错误
 
         RESULT_CODE_UNDEFINED = 200; //undefined command
 
-        RESULT_CODE_CONTROL_MUTEX_IS_LOCKED = 300;  // 当前处于独占模式，不允许对agv进行“写”操作
+        RESULT_CODE_CONTROL_MUTEX_IS_LOCKED = 300; // 当前处于独占模式，不允许对agv进行“写”操作
 
         RESULT_CODE_LOCATION_RUNNING = 10001; // 定位已启动
         RESULT_CODE_LOCATION_SLAM_STATE_ERROR = 10002; // 启动定位时SLAM状态异常
+        RESULT_CODE_LOCATION_STATION_NOT_EXIST = 10003; // 通过站点定位，但是该站点不存在
 
         RESULT_CODE_LOCATION_NO_RUNNING = 20001; // 关闭定位时定位未启动
         RESULT_CODE_LOCATION_PATH_RUNNING = 20002; // 关闭定位时正在运动
         RESULT_CODE_LOCATION_MOVEMENT_RUNNING = 20003; // 关闭定位时正在执行移动任务
         RESULT_CODE_LOCATION_MISSON_RUNNING = 20004; // 关闭定位时mission正在运行
 
+        RESULT_CODE_CANCEL_EMERGENCY_CAN_NOT_RECOVER = 30001; // 由于有急停不可解除，导致解除急停失败，可能的原因是：急停开关没有解除等
+
         RESULT_CODE_CONTINUE_IN_EMERGENCY = 40001; // 继续时车辆处于急停状态
 
+        RESULT_CODE_SYNC_TIME_SYSTEM_NOT_IDLE = 50001; // 设置系统时间的时候，系统处于繁忙状态
+
         RESULT_CODE_MANUAL_CONTROL_NOT_ON = 70001; // 小车不处于手动控制状态
         RESULT_CODE_START_MANUAL_CONTROL_IN_EMERGENCY = 70002; // 小车不处于急停状态下启动手动控制
         RESULT_CODE_START_MANUAL_CONTROL_IN_BREAK_SW = 70003; // 小车不处于解抱闸状态下启动手动控制
@@ -365,7 +373,7 @@ message ResponseResult {
         RESULT_CODE_SET_PGV_INFO_ID_IS_REPEAT = 170002; // 保存当前pgv时，pgv的id重复.同一张地图中不能同时存在两张id一样的pgv
 
         RESULT_CODE_LOCK_CONTROL_MUTEX_MOVEMENT_RUNNING = 180001; // 获取独占模式时移动任务正在运行
-        RESULT_CODE_LOCK_CONTROL_MUTEX_ACTION_RUNNING  = 180002; // 获取独占模式时动作任务正在运行
+        RESULT_CODE_LOCK_CONTROL_MUTEX_ACTION_RUNNING = 180002; // 获取独占模式时动作任务正在运行
         RESULT_CODE_LOCK_CONTROL_MUTEX_MISSION_RUNNING = 180003; // 获取独占模式时mission任务正在运行
         RESULT_CODE_LOCK_CONTROL_MUTEX_NONE_NICK_NAME = 180004; // 获取独占模式时没有带上锁人的绰号
         RESULT_CODE_LOCK_CONTROL_MUTEX_NONE_IP_ADDRESS = 180005; // 获取独占模式时没有带上锁人的ip
@@ -380,14 +388,32 @@ message ResponseResult {
         RESULT_CODE_MOVEMENT_PRE_TASK_RUNNING = 320003; // previous task is running
         RESULT_CODE_MOVEMENT_INVALID_CMD_PARAM = 320004; // invalid param
         RESULT_CODE_MOVEMENT_IN_MANUAL_CONTROL = 320005; // 处于手动控制状态，不能启动移动任务
+        RESULT_CODE_MOVEMENT_FOLLOW_PATH_START_POSE_OFFSET = 320010; // 用户发送移动路径时，起始点偏离了agv的真实位置，阈值由main.manual_path_start_pose_check_threshold配置
+        RESULT_CODE_MOVEMENT_FOLLOW_PATH_EXIST_ARC = 320011; // 用户发送移动路径时，路径中存在圆弧路径，圆弧路径已经被废弃
+        RESULT_CODE_MOVEMENT_FOLLOW_PATH_START_ANGLE_OFFSET = 320012; // 用户发送路径时，起点的角度与当前src角度相差过大，阈值由main.manual_path_start_pose_check_threshold配置
+        RESULT_CODE_MOVEMENT_FOLLOW_PATH_POSE_NOT_CONTINUOUS = 320013; // 用户发送移动路径，路径不连续，阈值值由main.manual_path_pose_continuous_check_threshold 配置
+        RESULT_CODE_MOVEMENT_FOLLOW_PATH_ANGLE_NOT_CONTINUOUS = 320014; // 用户发送移动路径，角度不连续，阈值值由main.manual_path_angle_continuous_check_threshold 配置
+        RESULT_CODE_MOVEMENT_IN_POWER_SAVE_MODE = 320020; // 处于低功耗模式，不能启动移动任务
+        RESULT_CODE_MOVEMENT_BREAK_SWITCH_ON = 320021; // 处于解抱闸状态（电源开关处于第二档），不能启动移动任务
+        RESULT_CODE_MOVEMENT_LASER_ERROR = 320022; // 雷达错误，不能启动移动任务
+        RESULT_CODE_MOVEMENT_VSC_ERROR = 320023; // VSC错误，不能启动移动任务
+        RESULT_CODE_MOVEMENT_SRC_ERROR = 320024; // SRC错误，不能启动移动任务
+        RESULT_CODE_MOVEMENT_MOTOR1_ERROR = 320025; // 电机1错误，不能启动移动任务
+        RESULT_CODE_MOVEMENT_MOTOR2_ERROR = 320026; // 电机2错误，不能启动移动任务
 
         RESULT_CODE_ACTION_IN_EMERGENCY = 340001; // 车辆处于急停状态
         RESULT_CODE_ACTION_PRE_TASK_RUNNING = 340002; // previous task is running
+        RESULT_CODE_ACTION_IN_POWER_SAVE_MODE = 340003; // 处于低功耗模式，不能启动动作任务
+        RESULT_CODE_ACTION_RESPONSE_TIMEOUT = 340010; // 回复超时
+        RESULT_CODE_ACTION_RESPONSE_INCORRECT = 340011; // 回复出错
+        RESULT_CODE_ACTION_SYSTEM_BUSY = 340012; // 系统繁忙
+        RESULT_CODE_ACTION_ID_NOT_SUPPORT = 340013; // action id 不支持
+        RESULT_CODE_ACTION_PARAM_0_NOT_SUPPORT = 340014; // action param 0 不支持
+        RESULT_CODE_ACTION_PARAM_1_NOT_SUPPORT = 340015; // action param 1 不支持
+        RESULT_CODE_ACTION_EAC_DISABLED = 340016; // eac模块没有开启
 
         RESULT_CODE_MISSION_IN_EMERGENCY = 660001; // 车辆处于急停状态
         RESULT_CODE_MISSION_NO_LOCATING = 660002; // 车辆不处于定位状态
-        RESULT_CODE_MISSION_PENDING = 660003; // 任务已经在队列中
-        RESULT_CODE_MISSION_RUNNING = 660004; // 任务正在执行
         RESULT_CODE_MISSION_ID_NOT_EXIST = 660005; //任务Id不存在
         RESULT_CODE_MISSION_ENQUEUE_TIMEOUT = 660006; // 加入队列超时
     }
```

### 命令参数中添加boolean类型参数
```shell
@@ -408,6 +434,8 @@ message Command {
 
     Pose pose = 6;
 
+    bool param_boolean = 7;
+
     string param_str1 = 8;
 
     MovementTask movement_task = 9;
@@ -421,6 +449,7 @@ message Command {
     string locker_nickname = 14; // 独占人nickname
 }
```
应用场景如：定位时是否采取强制定位的参数由param_boolean传递

### 给task添加IN_CANCEL和WAIT_FOR_ACK状态
- IN_CANCEL -- 任务正在取消中
- WAIT_FOR_ACK -- 任务执行结构已经结束，此时已经给task所有者发送Notification，并正在等待task所有者确认Notification的ACK。
```shell
@@ -541,6 +571,8 @@ message MovementTask {
         MT_RUNNING = 3; // 任务正在执行
         MT_PAUSED = 4;
         MT_FINISHED = 5;
+        MT_IN_CANCEL = 6; // 正在取消中
+        MT_TASK_WAIT_FOR_ACK = 7; // 等待ack,当收到网络发送过来的消息的时候，需要等待完ack后才算结束
     }
 
     TaskState state = 2;
@@ -602,11 +636,14 @@ message ActionTask {
         AT_RUNNING = 3; // 任务正在执行
         AT_PAUSED = 4;
         AT_FINISHED = 5;
+        AT_IN_CANCEL = 6; // 正在取消中
+        AT_TASK_WAIT_FOR_ACK = 7; // 等待ack,当收到网络发送过来的消息的时候，需要等待完ack后才算结束
     }
     TaskState state = 2;
 
     int32 no = 3;
 
``` 

### movement task结束时，若agv到达的位置和movement task的目标站点偏差过大，任务导航就失败
- 是否开启本项错误检测由“main.enable_movement_task_arrive_check”配置
- 检测阈值由“nav.station_loc_threshold”配置
```shell
@@ -588,6 +621,7 @@ message MovementTask {
         FAILED_CODE_NET_NAV_FINDE_PATH_NO_WAY = 0x08; // 路网导航时，没有路径到达目标位置
         FAILED_CODE_FREE_NAV_DST_POSE_UNWALKABLE = 0x0A; // 自由导航失败，目标站点不可到达
         FAILED_CODE_FREE_NAV_NO_WAY = 0x0B; // 自由导航失败，没找到合适的路径
+        FAILED_CODE_AGV_NOT_ARRIVED_DEST = 0x10; // agv没有导航到达目标位置，检测阈值由“nav.station_loc_threshold”配置
     }
 
     FailedCode failed_code = 15; // 失败码
```

### 上传是否有货物的状态
### 上传急停触发源
### 添加一个字段，指示当前agv是否能运行movement task
```shell
@@ -794,6 +831,36 @@ message SystemState {
     }
     ControlMutexLockState control_mutex_lock_state = 21; // 独占锁是否被锁上
     ControlMutexInfo control_mutex_info = 22; // 独占模式信息
+
+    uint32 multi_load_state = 23; // 每个bit用于表示机构的其中一个货位是否存在货物（0为无货，1为有货）
+
+    enum EmergencySource {
+        EMERGENCY_SRC_NONE = 0x00;
+        EMERGENCY_SRC_BUTTON_1 = 0x11; // 紧急按钮1触发
+        EMERGENCY_SRC_BUTTON_2 = 0x12; // 紧急按钮2触发
+        EMERGENCY_SRC_BUTTON_3 = 0x13; // 紧急按钮3触发
+        EMERGENCY_SRC_BUTTON_4 = 0x14; // 紧急按钮4触发
+        EMERGENCY_SRC_EDGE_1 = 0x21; // 边缘碰撞检测开关1触发
+        EMERGENCY_SRC_EDGE_2 = 0x22; // 边缘碰撞检测开关2触发
+        EMERGENCY_SRC_EDGE_3 = 0x23; // 边缘碰撞检测开关3触发
+        EMERGENCY_SRC_EDGE_4 = 0x24; // 边缘碰撞检测开关4触发
+        EMERGENCY_SRC_SOFTWARE_1 = 0x31; // 软件触发急停1
+        EMERGENCY_SRC_SOFTWARE_2 = 0x32; // 软件触发急停2
+        EMERGENCY_SRC_SOFTWARE_3 = 0x33; // 软件触发急停3
+        EMERGENCY_SRC_SOFTWARE_4 = 0x34; // 软件触发急停4
+    };
+
+    EmergencySource emergency_source = 24; // 急停触发源
+
+    // 标记是否可以开始新移动任务
+    enum NewMovementTaskState {
+        NEW_MOVEMENT_TASK_STATE_NONE = 0x00;
+        // 系统空闲、没有急停、没有解抱闸、不是低电量模式、定位成功、非手动控制模式、
+        // 没有移动任务或是移动任务已经结束、雷达、VSC、MOTOR1、MOTOR2、SRC都正常
+        NEW_MOVEMENT_TASK_STATE_READY = 0x01;
+        NEW_MOVEMENT_TASK_STATE_USELESS = 0x02;
+    }
+    NewMovementTaskState new_movement_task_state = 25;
 }
 
 message HardwareState {
```

### 添加电池剩余容量、标称容量、循环次数
```shell
@@ -827,12 +894,15 @@ message HardwareState {
     uint32 battery_voltage = 18; // 电池当前电压，单位mV
     int32 battery_current = 19; // 电池当前电流，单位mA
     int32 battery_temperature = 14; // 电池温度，单位℃
-    uint32 battery_remain_time = 15; // 电池预计剩余工作时间，单位min
+    int32 battery_remain_capacity = 26; // 电池剩余容量（mAh)
+    int32 battery_nominal_capacity = 27; // 电池标称容量（mAh)
+    int32 battery_use_cycles = 28; // 电池循环次数
+    uint32 battery_remain_time = 29; // 电池预计剩余工作时间，单位min
 
     enum PowerState {
         POWER_NA = 0;
         POWER_NORMAL = 1;
-        POWER_SAVE_MODE = 2;
+        POWER_SAVE_MODE = 2; // 低功耗模式
     }
     PowerState power_state = 24;
 
```

### 上传信标的匹配状态
```shell
@@ -917,11 +987,27 @@ message Device {
     string version_no = 6; // 版本号
 }
 
+message LmkMatchInfo {
+    int32 x = 1; // 单位mm
+    int32 y = 2;
+    int32 yaw = 3; //
+    enum LmkMatchType {
+        NONE = 0;
+        TYPE_FLAT = 1;
+        TYPE_CYLINDER = 2;
+    }
+    LmkMatchType lmk_type = 4;
+    bool is_matched = 5;
+}
+
 message LaserPoints {
     uint32 req_seq = 1;
 
     repeated int32 xs = 2 [packed = true];
     repeated int32 ys = 3 [packed = true];
+    repeated int32 reliabilitys = 4 [packed = true]; // [保留] 每个雷达点的置信度
+
+    repeated LmkMatchInfo lmks = 5;
 }
```