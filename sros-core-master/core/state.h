/**
 * @file state.h
 *
 * @author lhx
 * @date 15-12-27.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_STATE_H_
#define CORE_STATE_H_

#include <math.h>
#include <stdint.h>
#include <boost/thread/shared_mutex.hpp>
#include <string>
#include <utility>
#include <vector>
#include "control_mutex.hpp"
#include "core/music_id.h"
#include "core/util/read_write_lock_container.hpp"
#include "device/device_manager.h"
#include "error_code.h"  // NOTE: 本文件找不到？ 解决办法：在./update目录下执行./one.sh resources
#include "session_manager.h"
#include "util/utils.h"
#include "core/map/mark/FeatureMark.hpp"
#include "pose.h"

namespace sros {
namespace core {
static const double MM_TO_M = 0.001;
static const double DEG_TO_RAD = M_PI / 180.0;

enum CommandType {
    CMD_START_LOCATION = 0x01,
    CMD_STOP_LOCATION = 0x02,

    CMD_SRC_PAUSE = 0x03,
    CMD_SRC_CONTINUE = 0x04,
    CMD_SRC_STOP = 0x05,

    CMD_START_MANUAL_CONTROL = 0x06,
    CMD_STOP_MANUAL_CONTROL = 0x07,

    CMD_SRC_RESET = 0x08,  // 重启SRC

    CMD_SRC_SPEED_LEVEL = 0x09,

    CMD_SET_CUR_MAP = 0x0A,  // 设置当前使用的map名称

    CMD_SET_CUR_STATION = 0x0B,  // 设置当前所在站点
    CMD_MAP_SWITCHING = 0x37,    // 地图切换

    CMD_ENABLE_OBSTACLE_AVOID = 0x0C,   // 启用避障
    CMD_DISABLE_OBSTACLE_AVOID = 0x0D,  // 停用避障

    CMD_STOP_SROS = 0x0E,  // 正常停止sros各模块

    CMD_RESET_SROS = 0x0F,  // 重新启动sros各模块(包括src)

    CMD_SET_SRC_SPEED = 0x10,  // 手动设置SRC运动速度 连续运动时发送间隔不能小于100ms

    CMD_TASK_ACTION = 0x11,  // 设置ActionTask @废弃@

    CMD_CANCEL_EMERGENCY = 0x12,  // 解除急停状态

    CMD_ENABLE_AUTO_CHARGE = 0x13,  // 启动自动充电

    CMD_NEW_MAP_START = 0x14,   // 开始绘制新地图
    CMD_NEW_MAP_STOP = 0x15,    // 结束绘制新地图（保存）
    CMD_NEW_MAP_CANCEL = 0x16,  // 取消绘制新地图（不保存）

    CMD_SET_LOCATION_INITIAL_POSE = 0x17,  // 设置定位初始位姿

    CMD_SYNC_TIME = 0x18,

    CMD_COMMON_CANCEL = 0x19,  // 通用的任务取消指令

    CMD_LOCK_CONTROL_MUTEX = 0x1A,  // 获取独占许可，获取成功后，其他用户将不能继续控制sros，只能监控sros
    CMD_UNLOCK_CONTROL_MUTEX = 0x1B,        // 释放独占许可
    CMD_FORCE_UNLOCK_CONTROL_MUTEX = 0x1C,  // 强制释放别人的独占许可(需要admin及其以上的权限）

    CMD_NEW_MOVEMENT_TASK = 0x20,
    CMD_CANCEL_MOVEMENT_TASK = 0x21,
    CMD_SET_CHECKPOINT = 0x35,       // 设置移动任务的关卡，用于交通管制，车辆会停止关卡前
    CMD_PATH_REPLACE = 0x36,  // 路径替换（华为专用）

    CMD_NEW_ACTION_TASK = 0x22,
    CMD_CANCEL_ACTION_TASK = 0x23,

    CMD_SET_GPIO_OUTPUT = 0x24,  // 设置GPIO的输出状态

    CMD_SET_SPEAKER_VOLUME = 0x25,
    CMD_SET_HMI_STATE = 0x29,      // 设置HMI状态，并保持一段时间
    CMD_SET_HMI_CUSTOM_STATE = 0x38,    //设置自定义HMI状态

    CMD_SPEED_SLOW_DOWN = 0x26,

    CMD_CALIBRATION = 0x27,
    CMD_TRIGGER_EMERGENCY = 0x28,  // 触发急停状态

    CMD_RESET_FAULT = 0x2C,  // 尝试复位故障

    CMD_ENTER_POWER_SAVE_MODE = 0x31,  // 进入低功耗模式
    CMD_EXIT_POWER_SAVE_MODE = 0x32,   // 退出低功耗模式

    CMD_STOP_CHARGE = 0x33,  // 停止充电

    CMD_INPUT_ACTION_VALUE = 0x41,  // 设置ActionTask的输入值

    CMD_START_MISSION = 0x42,  // 开始mission
    CMD_CANCEL_MISSION = 0x43,
    CMD_CONTINUE_MISSION = 0x45,
    CMD_REORDER_MISSION = 0x44,

    CMD_ADD_PGV_INFO = 0x50,  // 记录当前pgv信息
    CMD_DEL_PGV_INFO = 0x51,  // 删除当前pgv信息
};

enum ActionID {
    ACTION_ID_COMMON_SRC = 0,   // 通用src发送指令
    ACTION_ID_JACKING = 1,      // 顶升机构
    ACTION_ID_JACK_ROTATE = 4,  // 旋转顶升机构

    ACTION_ID_PGV_RECTIFY = 7,  // 下视PGV矫正
    ACTION_ID_CALIBRATION = 8,    //标定动作

    ACTION_ID_GET_RFID_BY_SRC = 11,  //SRC动作获取RFID

    ACTION_ID_SET_GPIO_OUTPUT = 65,
    ACTION_ID_SET_GPIO_OUTPUT_DELAY = 66,
    ACTION_ID_GET_SCAN_CODE = 70,  // 常州光宝在用

    ACTION_ID_WAIT_TIMEOUT = 129,
    ACTION_ID_WAIT_IO_SIGNAL = 130,
    ACTION_ID_WAIT_COMMAND = 131,
    ACTION_ID_GET_RFID_BY_INNER = 132,  // INNER动作获取rfid的值

    ACTION_ID_GET_SVC_INFO = 133,  // 获取svc扫码信息
    ACTION_ID_OVERTIME_IO_SIGNAL = 134,  // 放行超时

    ACTION_ID_TRACTOR_DOCKING = 135,    //二维码牵引对接
    ACTION_ID_POSTURE_CORRECT = 136,    //位姿矫正动作
    ACTION_ID_MATCH_SVC_INFO = 137,    // 单机匹配料架号(OPPO项目新增需求)

    ACTION_ID_LOAD_SEARCH_GOOGS = 161,   //取货前寻货动作
    ACTION_ID_LOAD_MOVE_DOCKING = 162,   //取货前移动对接动作
    ACTION_ID_LOAD_PICKUP_GOODS = 163,   //载货动作
    ACTION_ID_LOAD_MOVE_BACK    = 164,   //载货退出动作
    ACTION_ID_GET_TOP_DECARDS    = 165,   //解堆叠动作
    ACTION_ID_FORK_LIFT         = 166,   //叉臂升降动作
    ACTION_ID_UNLOAD_DETECT     = 171,   //卸货检测动作
    ACTION_ID_UNLOAD_PICKDOWN_GOODS = 172,  //卸货动作
    ACTION_ID_UNLOAD_MOVE_BACK = 173,       //卸货退出动作

    ACTION_ID_HOOK_SWITCH = 180,    //挂钩脱钩动作
    ACTION_ID_ENABLE_BACK_OBA = 181, //使能后避障
    ACTION_ID_READ_TAG = 188,    //标签读取动作
    ACTION_ID_DEBUG = 189,       //调试动作

    ACTION_ID_BELT_CAR_BRIDGE = 206,  //皮带车搭桥

    ACTION_ID_COMMON_EAC = 255,   //通用eac发送指令
};

enum ResultState {
    RESPONSE_NONE = 0,
    RESPONSE_PROCESSING = 1,  // processing
    RESPONSE_OK = 2,          // excute command success
    RESPONSE_FAILED = 4,      // fail to excute command
};

// 弃用,改用SystemState
enum RobotState {
    STATE_INITIALING,
    STATE_IDLE,
    STATE_TASK_NAVIGATING,  // 执行自动导航任务
    STATE_TASK_NEW_MAP,     // 绘制新地图
    STATE_TASK_RUN_PATH,    // 执行路径
    STATE_ERROR,
};

enum LocalNavigationState {
    STATE_LOCAL_NAV_ZERO = 0,  // 无
    STATE_LOCAL_NAV_IDLE = 1,
    STATE_LOCAL_NAV_RUNNING = 2,
    STATE_LOCAL_NAV_PAUSE = 3,
};
// 用于添加局部路径规划后的分类
enum NavigationType {
    NAV_TYPE_NONE,  // 无
    NAV_TYPE_NET,   // 路网导航；
    NAV_TYPE_MAP,   // 自由导航；
};

enum NavigationMode {
    NAV_MODE_PATH,        //固定路网模式
    NAV_MODE_LOCAL_PLAN,  //  局部避障模式
};

// 障碍的方位
enum ObstacleDirection {
    OBSTACLE_DIRECTION_UNKNOWN = 0,  // 如：TIM320报的故障只知道有障碍，但不知道障碍的方向

    OBSTACLE_DIRECTION_FORWARD = 1,  // 前方
    OBSTACLE_DIRECTION_BACKWARD,     // 后方
    OBSTACLE_DIRECTION_LEFT,
    OBSTACLE_DIRECTION_RIGHT,
    OBSTACLE_DIRECTION_LEFT_FORWARD,
    OBSTACLE_DIRECTION_RIGHT_FORWARD,
    OBSTACLE_DIRECTION_LEFT_BACKWARD,
    OBSTACLE_DIRECTION_RIGHT_BACKWARD,
};

enum SystemState {
    SYS_STATE_ZERO = 0,
    SYS_STATE_INITIALING = 0x01,  // 系统正在初始化
    SYS_STATE_IDLE = 0x02,        // 系统空闲
    SYS_STATE_ERROR = 0x03,       // 系统出错

    SYS_STATE_START_LOCATING = 0x04,  // 正在启动定位

    SYS_STATE_TASK_NAV_INITIALING = 0x05,           // 导航正在初始化
    SYS_STATE_TASK_NAV_FINDING_PATH = 0x06,         // 导航正在寻路
    SYS_STATE_TASK_NAV_WAITING_FINISH = 0x07,       // 正在等待到达目标位置
    SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW = 0x08,  // 检测到障碍,减速
    SYS_STATE_TASK_NAV_REFINDING_PATH = 0x09,       // 导航正在重新寻路
    SYS_STATE_TASK_NAV_PAUSED = 0x0A,               // 遇到障碍暂停运动
    SYS_STATE_TASK_NAV_NO_WAY = 0x0B,               // 无法抵达目标位置
    SYS_STATE_TASK_NAV_PATH_ERROR = 0x14,           //导航过程中路径出错：例偏离路径

    SYS_STATE_TASK_NEWMAP_DRAWING = 0x0C,  // 正在绘制新地图
    SYS_STATE_TASK_NEWMAP_SAVING = 0x0D,   // 正在保存新地图

    SYS_STATE_TASK_PATH_NAV_INITIALING = 0x0E,           // 正在初始化执行固定路径
    SYS_STATE_TASK_PATH_WAITING_FINISH = 0x0F,           // 正在等待固定路径执行结束
    SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW = 0x10,      // 检测到障碍,减速前进
    SYS_STATE_TASK_PATH_WAITING_CHECKPOINT = 0x17,       // 遇到交通管制，暂停等待 [废弃]
    SYS_STATE_TASK_PATH_WAITING_CHECKPOINT_SLOW = 0x18,  // [保留]遇到交通管制，减速运行 [废弃]
    SYS_STATE_TASK_PATH_PAUSED = 0x11,                   // 遇到障碍暂停运动

    SYS_STATE_TASK_NAV_NO_STATION = 0x12,  // 无法检测到目标站点
    SYS_STATE_TASK_MANUAL_PAUSED = 0x13,
    SYS_STATE_TASK_MANUAL_PATH_ERROR = 0x15,  //导航过程中路径出错：例偏离路径
    SYS_STATE_HARDWARE_ERROR = 0x16,
};

enum LocationState {
    LOCATION_STATE_NONE = 0x01,
    LOCATION_STATE_INITIALING = 0x02,  // 初始化中
    LOCATION_STATE_RUNNING = 0x03,     // 定位正常
    LOCATION_STATE_RELOCATING = 0x04,  // 重定位
    LOCATION_STATE_ERROR = 0x05,       // 出错,定位时为环境变化过大, 绘图时为未检测到landmark
};

enum BatteryState {
    BATTERY_NA = 0x00,  // 充电状态不可用

    BATTERY_CHARGING = 0x02,
    BATTERY_NO_CHARGING = 0x03,
};

// 电源供电模式
enum PowerState {
    POWER_NA = 0x00,  // 状态不可用

    POWER_NORMAL = 0x01,     // 正常模式
    POWER_SAVE_MODE = 0x02,  // power save mode
};

// 解抱闸开关状态
enum BreakSwitchState {
    BREAK_SW_NA = 0x00,

    BREAK_SW_OFF = 0x01,  // 解抱闸开关处于关闭状态，此时抱闸生效
    BREAK_SW_ON = 0x02,   // 解抱闸开关处于打开状态，此时抱闸失效，可自由推动
};

enum SLAM_COMMAND_TYPE {
    COMMAND_START_LOCATION_AUTO,    // 使用sift自动定位
    COMMAND_START_LOCATION_MANUAL,  // 使用初始位姿手动定位
    COMMAND_STOP_LOCATION,          // 停止定位指令,进入IDLE状态

    COMMAND_START_DRAW_MAP,   // 绘制地图
    COMMAND_STOP_DRAW_MAP,    // 保存地图,进入IDLE状态
    COMMAND_CANCEL_DRAW_MAP,  // 不保存地图,进入IDLE状态

    COMMAND_LOAD_MAP,       // 加载地图
    COMMAND_INIT_COMPLETE,  // 下位机初始化完成

    COMMAND_START_LOCAL_LOCATION,  // 切换到局部定位模式
    COMMAND_STOP_LOCAL_LOCATION,   // 结束局部定位模式，恢复为正常SLAM定位

    COMMAND_START_LOCAL_REAL_TIME_SLAM_LOCATION,
    COMMAND_STOP_LOCAL_REAL_TIME_SLAM_LOCATION,
    COMMAND_START_RELOCATION,  //重定位
};

enum SLAM_STATE_CODE {
    STATE_SLAM_IDLE,                  // SLAM模块系统默认状态
    STATE_SLAM_WAITING_INITIAL_POSE,  // 等待获取初始位姿
    STATE_SLAM_SIFT_LOCATING,         // sift匹配中
    STATE_SLAM_LOCATING,              // 定位中

    STATE_SLAM_SAVING_MAP,  // 保存地图中
    STATE_SLAM_DRAWING,     // 绘图中

    STATE_SLAM_LOCATING_AMCL,  // AMCL位姿搜索中

    STATE_SLAM_ERROR,  //异常状态

    STATE_COMMAND_NOT_EXECUTE,        // slam系统无法执行当前指令
    STATE_MAP_SAVING,                 // 地图保存中
    STATE_MAP_SAVE_SUCCESSFULLY,      // 保存地图成功
    STATE_POSE_RECEIVE_REQUEST,       // 请输入初始位姿
    STATE_POSE_RECEIVE_SUCCESSFULLY,  // 成功接收位置信息
    STATE_RECOVER_POSE_SUCCESSFULLY,  // 定位修复完成,请确认修复是否正确,不正确,请重新输入初始位姿
    STATE_SCAN_NOT_PUBLISHED,         // 激光雷达没有发布
    STATE_SCAN_PUBLISHED,             //  激光雷达已经发布
};

enum NavigationCommand {
    COMMAND_NAV_NAVIGATE_TO,  // 自动导航到目标点

    COMMAND_NAV_CANCEL,

    COMMAND_NAV_FINISH,  // 到达目标点

    COMMAND_NAV_CONVERT_MAP,  // 转换地图

    COMMAND_NAV_MANUAL_PATH,  // 手动路径

    COMMAND_NAV_SINGLE_APTH_REPLACE,  // 路径替换

    COMMAND_NAV_GET_STATION_POSE,  // 获取站点对应的Pose [废弃] 现在可以在Mapmanage中直接获取

    COMMAND_NAV_FIND_FEATURE,         // 检测站点精确位置
    COMMAND_NAV_FIND_FEATURE_RESULT,  // 返回检测到的站点位置

    COMMAND_NAV_PAUSE,     // 用户发送暂停
    COMMAND_NAV_CONTINUE,  // 用户发送继续
};

enum NavigationState {
    STATE_NAV_ZERO = 0,
    STATE_NAV_IDLE = 1,

    STATE_NAV_INITIALIZING = 2,
    STATE_NAV_PATH_FINDING = 3,
    STATE_NAV_WAITING_FOR_FINISH = 4,
    STATE_NAV_WAITING_FOR_FINISH_SLOW = 5,  // (低速)
    STATE_NAV_PATH_REFINDING = 6,
    STATE_NAV_PATH_REFINDING_PAUSED = 7,  // 重新规划路径时发现无路可走
    STATE_NAV_PATH_PAUSED = 8,            // 等待障碍物移开

    STATE_NAV_MANUAL_WAITING_FOR_START = 9,          // 等待障碍检测结果来决定是否开始运动
    STATE_NAV_MANUAL_WAITING_FOR_START_PAUSED = 10,  // 起点附近发现障碍,无法开始运动
    STATE_NAV_MANUAL_WAITING_FOR_FINISH = 11,        // 等待手动路径执行结束
    STATE_NAV_MANUAL_WAITING_FOR_FINISH_SLOW = 12,   // 等待手动路径执行结束(低速)
    STATE_NAV_MANUAL_WAITING_FOR_CHECKPOINT = 22,    // 遇到关卡，等待暂停 [废弃]
    STATE_NAV_MANUAL_PAUSED = 13,                    // 手动路径等待障碍物消失

    STATE_NAV_CONVERTING_MAP = 14,  // 正在转换GrayMap到Navigation

    STATE_NAV_LOCATING_STATION = 15,  // 正在检测目标站点的位置

    STATE_NAV_LOCATING_STATION_FAILED = 16,  // 检测目标点失败,

    STATE_NAV_NOWAY = 17,  // 无路可达目标点

    STATE_NAV_ERROR = 18,   // 异常状态
    STATE_NAV_PAUSED = 19,  // 被暂停
    STATE_NAV_CONTINUE = 20,

    STATE_NAV_PATH_ERROR = 21,  // 执行路径过程中异常错误
};

enum ActionState {
    STATE_ACTION_NA = 0x00,  // 动作状态不可用

    STATE_ACTION_IDLE = 0x01,     // 空闲或动作执行完毕
    STATE_ACTION_RUNNING = 0x02,  // 动作正在执行

    STATE_ACTION_ERROR = 0x10,  // 异常状态
};

enum EmergencyState {
    STATE_EMERGENCY_NA = 0x00,  // 紧急状态不可用

    STATE_EMERGENCY_NONE = 0x01,         // 不处于急停状态
    STATE_EMERGENCY_TRIGER = 0x02,       // 急停状态触发
    STATE_EMERGENCY_RECOVERABLE = 0x03,  // 可恢复急停状态
};

enum EmergencySource {
    EMERGENCY_SRC_NONE = 0x00,
    EMERGENCY_SRC_BUTTON_1 = 0x11,    // 紧急按钮1触发
    EMERGENCY_SRC_BUTTON_2 = 0x12,    // 紧急按钮2触发
    EMERGENCY_SRC_BUTTON_3 = 0x13,    // 紧急按钮3触发
    EMERGENCY_SRC_BUTTON_4 = 0x14,    // 紧急按钮4触发
    EMERGENCY_SRC_EDGE_1 = 0x21,      // 边缘碰撞检测开关1触发
    EMERGENCY_SRC_EDGE_2 = 0x22,      // 边缘碰撞检测开关2触发
    EMERGENCY_SRC_EDGE_3 = 0x23,      // 边缘碰撞检测开关3触发
    EMERGENCY_SRC_EDGE_4 = 0x24,      // 边缘碰撞检测开关4触发
    EMERGENCY_SRC_SOFTWARE_1 = 0x31,  // 软件触发急停1
    EMERGENCY_SRC_SOFTWARE_2 = 0x32,  // 软件触发急停2
    EMERGENCY_SRC_SOFTWARE_3 = 0x33,  // 软件触发急停3
    EMERGENCY_SRC_SOFTWARE_4 = 0x34,  // 软件触发急停4

    EMERGENCY_SRC_BATTERY_DOOR = 0x40, // 电池舱门被打开了
    EMERGENCY_SRC_SAFETY_LIDAR = 0x41, // 区域避障雷达触发
    
    EMERGENCY_SRC_INTERNAL_FAULT = 0x50, //src内部自定义故障
    EMERGENCY_SRC_EXTERNAL_FAULT = 0x51, //src外部自定义故障
    EMERGENCY_SRC_NXP_ESTOP = 0x52, //硬件NXP触发(NXP_ESTOP)
    EMERGENCY_SRC_ST_ESTOP = 0x53, //硬件STM32触发(ST_ESTOP)
    EMERGENCY_SRC_TK1_ESTOP = 0x54, //硬件TK1触发(TK1_ESTOP)
    EMERGENCY_SRC_ESTOP_OUT = 0x55, //硬件急停锁存电路触发(ESTOP_OUT)
};

enum ConfigCommand {
    //    CFG_COMMAND_SET_CURRENT_MAP,
    //    CFG_COMMAND_UPDATE_SRC_PARAMETER,
    CFG_COMMAND_START_LOCATION,
};

enum ObstacleAvoidPolicy {
    OBSTACLE_AVOID_WAIT = 0x01,    // 暂停运动直至障碍消失
    OBSTACLE_AVOID_REPLAN = 0x02,  // 重新规划路径绕过障碍
    OBSTACLE_AVOID_NONE = 0x10,    // 不处理
};

enum OperationState {
    OPERATION_AUTO = 0x01,    // 自动控制模式
    OPERATION_MANUAL = 0x02,  // 手动控制模式
};

enum ManualBtnState {
    BTN_AUTO = 0,    //  按钮打到自动
    BTN_MANUAL = 1,  // 按钮打到手动
};

// 车体载荷状态
enum LoadState {
    LOAD_NONE = 0x00,
    LOAD_FREE = 0x01,  // 没有载荷
    LOAD_FULL = 0x02,  // 满载荷
};

enum DstStationType {
    DS_DEFAULT = 0,
    DS_NO_ROTATE = 1,
};

enum ObaEnableState {
    OBA_NONE = 0,
    OBA_ENABLED = 1,
    OBA_DISABLED = 2,
};

enum FreshState {
    FRESH_NONE = 0,
    FRESH_NO = 1,
    FRESH_YES = 2,
};

enum HardwareErrorCode {
    ERR_SRC_DOWN = 0x0001,
    ERR_MOTOR1_DOWN = 0x0002,
    ERR_MOTOR2_DOWN = 0x0004,
    ERR_MOTOR3_DOWN = 0x0008,
    ERR_MOTOR4_DOWN = 0x0010,
    ERR_IMU_DOWN = 0x0020,
    ERR_PGV1_DOWN = 0x0040,
    ERR_PGV2_DOWN = 0x0080,
    ERR_VSC_DOWN = 0x0100,
    ERR_BATTERY_DOWN = 0x0200,
    ERR_NETWORK_DOWN = 0x0400,
    ERR_LIDAR_DOWN = 0x0800,
    ERR_SPEAKER_DOWN = 0x1000,
    ERR_D435_DOWN = 0x2000,
    ERR_SVC100_DOWN = 0x4000,
    ERR_LC100_DOWN = 0x6000
};

enum HState {
    H_STATE_ZERO = 0,
    H_STATE_INITIALING = 1,
    H_STATE_OK = 2,
    H_STATE_ERROR = 3,
};

// 动作执行机构，动作执行结构不仅包括SRC的动作执行机构，还包括扩展动作执行机构
enum ActionControllerType {
    ACTION_CONTROLLER_TYPE_NONE = 0,                // 没有结构
    ACTION_CONTROLLER_TYPE_SRC_DOUBLE_DRUM = 3,     // src 双滚筒模组
    ACTION_CONTROLLER_TYPE_SRC_PUTTER_JACKING = 4,  // src 推杆顶升模组
    ACTION_CONTROLLER_TYPE_SRC_JACKING_ROTATE = 6,  // src 顶升旋转
};

// 表示调度模式
enum FleetMode {
    FLEET_MODE_NONE = 0,
    FLEET_MODE_OFFLINE = 1,  // 单机模式
    FLEET_MODE_ONLINE = 2,   // 调度模式
};

// 动作执行机构类型
enum ActionUnit {
    ACTION_UNIT_SRC = 1,
    ACTION_UNIT_EAC = 2,
};

//modbus显示硬件设备状态
enum HardwareDevice {
    HD_LASER_NAV = 0x01,        // 激光导航雷达
    HD_CAMERA_D435 = 0x02,      // D435深度相机
    HD_CAMERA_O3D = 0x04,       // O3D303探货相机
    HD_UST_LIDAR_LEFT = 0x08,   // 左侧避障雷达
    HD_UST_LIDAR_BACK = 0x10,   // 后侧避障雷达
    HD_UST_LIDAR_RIGHT = 0x20,  // 右侧避障雷达
};

// sros 触发急停源
enum SROS_TRIGGER_EMERGENCY_SRC {
    SROS_EMERGENCY_LASER_TIMEOUT = 0x01,
    SROS_EMERGENCY_SRC_TIMEOUT = 0x02,
    SROS_EMERGENCY_ODO_TIMEOUT = 0x04,
    SROS_EMERGENCY_UPSVC100_SCAN = 0x08,
};

typedef uint16_t StationNo_t;    // 站点编号类型
typedef uint16_t StationType_t;  // 站点类别类型
typedef std::pair<StationNo_t, StationType_t> StationPair_t;

typedef unsigned char BatteryPercentage_t;
typedef uint8_t Progress_t;  // 当前进度百分比数据(0~100)

const char NO_MAP[] = "NO_MAP";  // 当没有地图的时候显示NO_MAP

class DMCodeOffset {
 public:
    DMCodeOffset() = default;
    DMCodeOffset(const std::string &no, double x, double y, double yaw) : no(no), x(x), y(y), yaw(yaw) {
        // NOTE: 此处算法要和vision_module 中保持一致
        int64_t code_int_64 = 0;
        for (auto &char_s : no) {
            if (char_s >= '0' && char_s <= '9') {
                code_int_64 = code_int_64 * 10 + char_s - 48;
            }
        }
        int64_t rest_int = 1e9;
        id = code_int_64 % rest_int;
    }

    friend std::ostream &operator<<(std::ostream &os, const DMCodeOffset &offset) {
        os << "DMCodeOffset: no: " << offset.no << ", " << offset.id << "(" << offset.x << ", " << offset.y << ", "
           << offset.yaw << ")";
        return os;
    }

    std::string no;
    int id = 0;        // 传给src时，id给的是id，所以此处要模拟一下，
    double x = 0.0;    // m
    double y = 0.0;    // m
    double yaw = 0.0;  // 弧度
};

class State {
 public:
    State();
    ~State() {}

    bool isEmergency() const {
        return emergency_state == STATE_EMERGENCY_TRIGER || emergency_state == STATE_EMERGENCY_RECOVERABLE;
    }                                                                                               // 是否是急停
    bool isEmergencyRecoverable() const { return emergency_state == STATE_EMERGENCY_RECOVERABLE; }  // 是否是急停可恢复
    bool isBreakSwitchON() const { return break_sw_state == BREAK_SW_ON; }

    bool isChargeState() const { return battery_state == BATTERY_CHARGING; }

    bool isPowerSaveMode() const { return power_state == POWER_SAVE_MODE; }

    bool isSlowdownForObstacle() const {
        return sys_state == SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW ||
               sys_state == SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW;
    }  // 遇到障碍减速
    bool isPausedForObstacle() const {
        return sys_state == SYS_STATE_TASK_NAV_PAUSED || sys_state == SYS_STATE_TASK_PATH_PAUSED;
    }  // 遇到障碍暂停运动
    bool isLocationError() const {
        return location_state == LOCATION_STATE_ERROR || location_state == LOCATION_STATE_NONE;
    }                                                                                  // 定位错误
    bool isLocateSucceed() const { return location_state == LOCATION_STATE_RUNNING; }  // 定位正常
    bool isSystemIDLE() const { return sys_state == SYS_STATE_IDLE || isNavNoWay(); }  // 是否是系统空闲
    bool isNavNoWay() const {
        return (sys_state == SYS_STATE_TASK_NAV_NO_WAY) || (sys_state == SYS_STATE_TASK_NAV_NO_STATION);
    }                                                                             // 无法导航到站点
    bool isManualControl() const { return (operation_state == OPERATION_MANUAL || manual_btn_state == BTN_MANUAL); }  // 当前处于手动控制模式
    bool isLaserError() const {
        return !device::DeviceManager::getInstance()->isDeviceOK(device::DEVICE_LIDAR);
    }  // 雷达故障
    bool isSrcError() const {
        return !device::DeviceManager::getInstance()->isDeviceOK(device::DEVICE_SRC);
    }  // SRC 故障
    bool isMotor1Error() const { return !device::DeviceManager::getInstance()->isDeviceOK(device::DEVICE_MOTOR_1); }

    bool isMotor2Error() const { return !device::DeviceManager::getInstance()->isDeviceOK(device::DEVICE_MOTOR_2); }

    bool isMotor3Error() const { return !device::DeviceManager::getInstance()->isDeviceOK(device::DEVICE_MOTOR_3); }

    bool isMotor4Error() const { return !device::DeviceManager::getInstance()->isDeviceOK(device::DEVICE_MOTOR_4); }

    bool isMotorError() const { return isMotor1Error() || isMotor2Error() || isMotor3Error() || isMotor4Error(); }

    bool isVscError() const { return !device::DeviceManager::getInstance()->isDeviceOK(device::DEVICE_VSC); }

    bool isDrawMap() const {
        return sys_state == SYS_STATE_TASK_NEWMAP_DRAWING || sys_state == SYS_STATE_TASK_NEWMAP_SAVING;
    }  // 是否在绘图
    // 返回当前导航状态是否需要避障
    bool isNeedAvoidObaNavState() const {
        if (nav_state == STATE_NAV_WAITING_FOR_FINISH || nav_state == STATE_NAV_WAITING_FOR_FINISH_SLOW ||
            nav_state == STATE_NAV_PATH_PAUSED || nav_state == STATE_NAV_PATH_REFINDING_PAUSED ||
            nav_state == STATE_NAV_MANUAL_WAITING_FOR_START || nav_state == STATE_NAV_MANUAL_WAITING_FOR_START_PAUSED ||
            nav_state == STATE_NAV_MANUAL_WAITING_FOR_FINISH || nav_state == STATE_NAV_MANUAL_WAITING_FOR_FINISH_SLOW ||
            nav_state == STATE_NAV_MANUAL_PAUSED || nav_state == STATE_NAV_MANUAL_WAITING_FOR_CHECKPOINT) {
            return true;
        }
        return false;
    }

    bool is_kernel_release_4_14() const {
        return kernel_release.compare(0, 4, "4.14") == 0;
    }

    std::string getCurMapName() const;

    void setCurMapName(const std::string &map_name);

    bool needDebugInfo() const { return need_debug_info_; }
    void setNeedDebugInfo(bool enable);

    // 检测启动移动任务的条件
    void checkStartMovementTaskCondition() const;
    // 检测启动动作任务的条件
    void checkStartActionTaskCondition(const int& _actionId,const int& _param0,const int& _param1) const;

    void setError(uint32_t error_code) { laster_error_code = error_code; }  // 设置错误码

    /**
     * 获取当前动作执行机构
     * @return 动作执行结构对应的ID，将其与ActionControllerType去对比，返回值的取值范围会包含ActionControllerType
     */
    int getActionControllerType() const;

    /**
     * @def 当调度模式更新时，需要更新调度模式底层实现 —— 独占模式
     * 实现内容：
     * 1. 当车辆处于非调度模式时，需要将当前fms独占强制解除
     * 2. 当车辆处于调度模式时，需要解除当前独占用户，并由fms独占
     * 3. 当车辆处于调度模式时，且fms未联机，此时需要sros自己独占，防止被别的用户抢占
     * 调用时机：
     * 本函数是幂等函数，理论上是可以多次调用
     * 当fms登录变更、调度模式切换都需要调用此函数
     */
    void updateFleetState();

    void playMusic(hmi::MusicID music_id);

    std::string kernel_release;

    core::RobotState main_state;
    core::SLAM_STATE_CODE slam_state;
    core::NavigationState nav_state;

    core::SystemState sys_state;
    core::LocationState location_state;
    core::HState hardware_state;

    StationNo_t station_no;  // 当前所在站点

    Progress_t progress;  // 配合sys_state表示当前正在处理的任务进度百分比

    ActionState action_state;

    BatteryPercentage_t battery_percentage;  // 电池剩余电量，单位%
    int16_t battery_current;                 // 电池电流，单位mA
    uint16_t battery_voltage;                // 电池电压，单位mV
    int16_t battery_temperature;             // 电池温度，单位℃
    uint32_t battery_remain_capacity = 0;    // 电池剩余容量（mAh)
    uint32_t battery_nominal_capacity = 0;   // 电池标称容量（mAh)
    uint16_t battery_use_cycles = 0;         // 电池循环次数
    uint32_t battery_remain_time = 0;        // 电池预计剩余工作时间，单位min
    BatteryState battery_state;              // 电池状态（充电）
    PowerState power_state;                  // 供电状态

    bool auto_charging;     //用于判断自动充电是否异常断开

    BreakSwitchState break_sw_state;  // 解抱闸开关状态

    EmergencyState emergency_state;    // 急停状态
    EmergencySource emergency_source;  // 急停触发源

    OperationState operation_state=OPERATION_AUTO;  // 控制状态：自动/手动

    ManualBtnState manual_btn_state = BTN_AUTO;  // 手动按钮状态

    ObaEnableState oba_state; /* status of oba-sensor */
    FreshState fresh_state;
    LoadState load_state;  // 载荷状态
    int cur_goal_id = -1;  //当前叉车货物的编号
    bool is_fork_pose_adjust = false;  //是否进行位姿调整
    double dst_pose_yaw;               //目标位姿的raw值

    uint16_t io_1353 = 0;             //传感器状态
    float fork_height_encoder = 0;    //叉车叉臂高度
    uint32_t fork_updown_state = 0;   //升降到位状态
    int32_t steering_angle = 0;      //轮子转向角
    Pose goods_pose;

    bool sync_rotate = false;  // 记录货架是否同步旋转

    uint16_t gpio_input;
    uint16_t gpio_output;

    uint8_t speed_level = 0;
    uint32_t hardware_error_code = 0;              // 系统的上一次错误 【废弃】
    uint32_t laster_error_code = ERROR_CODE_NONE;  // 系统的上一次错误
    uint8_t cur_volume;

    ReadWriteLockContainer<DMCodeOffset> cur_down_camera_offset;  // 当前下视摄像头偏差
    ReadWriteLockContainer<DMCodeOffset> station_camera_offset;  // 站点对应的视摄像头偏差
    ReadWriteLockContainer<DMCodeOffset> cur_up_camera_offset;  // 当前上视摄像头偏差

    uint32_t cpu_usage = 0;     // cpu使用率 (0.1%)
    uint32_t memory_usage = 0;  // memory使用率 (0.1%)

    int32_t cpu_temperature = 0;    // cpu温度（0.1℃）
    int32_t board_temperature = 0;  // 主板温度（0.1℃）

    // 需要vsc >= v3.7.1 && 安装了传感器硬件
    int32_t box_temperature_max = 0; // 本体最大温度，单位0.1℃
    int32_t box_temperature_min = 0; // 本体最小温度，单位0.1℃
    int32_t box_humidity_max = 0; // 本体最大湿度，单位0.1%RH
    int32_t box_humidity_min = 0; // 本体最小湿度，单位0.1%RH
    bool fan_switch_state = false; // 风扇状态，true -- 开启； false -- 关闭

    bool screen_turned_up_ = false; // 触摸屏被翻转起来了

    SessionManager modbus_session_manager;   // modbus-TCP的网络连接管理
    SessionManager network_session_manager;  // 网络模块的网络连接管理

    ControlMutex control_mutex;  // 独占模式相关信息

    ActionUnit action_unit = ACTION_UNIT_SRC;
    uint32_t multi_load_state = 0;  // 每个bit用于表示机构的其中一个货位是否存在货物（0为无货，1为有货）

    // 标记是可以开始新移动任务,为true时需要同时满足以下条件：
    // 系统空闲、没有急停、没有解抱闸、不是低电量模式、定位成功、非手动控制模式、没有移动任务或是移动任务已经结束、雷达、VSC、MOTOR1、MOTOR2、SRC都正常
    bool ready_for_new_movement_task = false;

    bool need_avoid_obstacle_prediction = false;  // 标记是否需要上传避障预测信息

    bool in_manual_obstacle_paused = false;     // 标记手动控制条件下，是否发生避障

    ObstacleDirection obstacle_direction = OBSTACLE_DIRECTION_UNKNOWN;  // 避障时，最近的障碍点相对于车的放行

    int rotate_value = 0;

    int checkpoint_no = 0;

    // FleetMode expect_fleet_state = FLEET_MODE_NONE; //
    // 期望的调度模式，当车辆进入某个现场后，若无fms则需要设置单机模式，若有需要设置调度模式
    FleetMode fleet_mode = FLEET_MODE_NONE;  // 当前的调度模式，当车辆的期望调度模式为调度模式时，允许临时切换到单机模式

    ReadWriteLockContainer<std::string> ip_addr;  // 车辆对外的IP

    bool is_map_switching = false; // 标记当前是否处于地图切换

    uint32_t sros_emergency_src;  //sros触发急停源


    bool enable_station_feature_recognize = false;  //标记目标站点是否特征信息识别
    sros::map::FeatureMark feature_mark;            //目标站点绑定的特征信息

  // NOTE(pengjiali): 看下后面设备禁止移动任务的情况多不多，若多的话应该直接放到设备中去。
    bool eac_prohibit_movement_task = false; // EAC禁止执行移动任务

    bool is_send_forklift_common_poses = false; //卸货检测时发送调试检测位姿

    bool is_loading_check_pallet_signal_ = false; //载货状态下检测栈板到位开关
    
    bool is_check_up_svc100_scan_state = false;  //检测上视扫码
    bool is_forbid_send_up_svc100_to_src = false; //上视扫码信息禁止发送src
    bool is_forbid_send_down_svc100_to_src = false; //下视扫码信息禁止发送src

    bool photoelectric_switch_on_action = false; //用于只有162动作才打开的场景

    bool enable_back_main_laser_oba = true;    //用于叉车取放货关闭后雷达和主雷达避障

 private:
    mutable boost::shared_mutex cur_map_name_mutex_;  // 当前地图名的锁
    std::string cur_map_name_ = NO_MAP;  // 当前系统使用的地图, 由于存在多个线程读写，所以此处需要加锁
    bool need_debug_info_ = false;  // 标记是否需要调试信息
};

class DeviceSRCState {
 public:
    DeviceSRCState() {}

    ~DeviceSRCState() {}

    uint8_t src_state;                    // src-sdk中SRCState类型
    uint8_t src_state_error_reason;       // 出错原因
    bool is_waitting_checkpoint = false;  // 当前是否处于等待关卡

    uint8_t cur_path_no = 0;
    uint8_t cur_checkpoint = 0;    // 当前关卡的位置（0）表示没有设置关卡，也不允许再设置关卡
    int16_t cur_remain_time;       // s
    uint16_t cur_remain_distance;  // cm
    uint16_t cur_total_distance;   // cm
    int32_t cur_v;                 // mm/s
    int32_t cur_w;                 // 0.001 rad/s

    uint8_t movement_state;  // 运动状态（前进、后退、转弯等）

    uint32_t total_power_cycle = 0;   // 上电次数
    uint32_t total_poweron_time = 0;  // 总开机时间，单位s
    uint32_t total_mileage = 0;       // 总运动里程，单位m
    uint32_t total_device_count = 0;   // src上挂着的设备总数(src_proto_2新增)

    uint32_t cpu_usage = 0;     // 0.1%(src_proto_2新增)
    uint32_t memory_usage = 0;  // 0.1%(src_proto_2新增)

    // 电机1
    uint8_t m1_status;        // 状态
    uint32_t m1_status_code;  // 状态码
    uint32_t m1_mileage;      // 运动里程

    // 电机2
    uint8_t m2_status;        // 状态
    uint32_t m2_status_code;  // 状态码
    uint32_t m2_mileage;      // 运动里程

    // 电机3
    uint8_t m3_status;        // 状态
    uint32_t m3_status_code;  // 状态码
    uint32_t m3_mileage;      // 运动里程

    // 电机4
    uint8_t m4_status;        // 状态
    uint32_t m4_status_code;  // 状态码
    uint32_t m4_mileage;      // 运动里程
    // IMU
    uint8_t imu_status;  // 状态

    uint8_t pgv1_status;
    uint8_t pgv2_status;

    // 手动控制器
    uint8_t manual_controller_status;  // 状态

    uint8_t device4_status;
    uint8_t device5_status;
    uint8_t device6_status;
    uint8_t device7_status;

    uint64_t reserved_field;
};

}  // namespace core
}  // namespace sros

extern sros::core::State g_state;

extern sros::core::DeviceSRCState g_src_state;

#endif  // CORE_STATE_H_
