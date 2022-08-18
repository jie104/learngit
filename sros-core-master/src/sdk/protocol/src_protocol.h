/**
 * @file src_protocol.h
 *
 * @author lhx
 * @date 16-1-20.
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_SRC_SDK_PROTOCOL_SRC_PROTOCOL_H_
#define SROS_SRC_SDK_PROTOCOL_SRC_PROTOCOL_H_

#define COMMAND_FIELD_SIZE 1
#define SRC_STATE_FIELD_SIZE 1
#define SRC_PARAMETER_FIELD_SIZE 32
#define SRC_SIGNAL_FIELD_SIZE 1
#define SRC_GIT_VERSION_STR_SIZE 8

#include <stdint.h>
#include "core/task/task.h"

typedef enum MSG_TYPE {
    MSG_COMMAND = 0x01,
    MSG_STATE = 0x02,
    MSG_POSE = 0x04,
    MSG_PATH = 0x05,
    MSG_VELOCITY = 0x06,
    MSG_PARAMETER = 0x07,
    MSG_SIGNAL = 0x08,
    MSG_INFO = 0x09,
    MSG_ACTION_STATE = 0x0A,
    MSG_MONITOR_STATE = 0x0B,
    MSG_OPT_POSE = 0x0C,  // 用于上传src优化后的机器人位姿。
    MSG_SECURITY = 0x0D, // 安全相关的状态
    MSG_SROS_STATE = 0x0E, // sros相关的状态

    MSG_IAP = 0xab,
    MSG_WWDG = 0xFF,  // 窗口看门狗

} MSG_TYPE_t;

// 指令
typedef enum SRCCommand {
    COMMAND_RUN = 0x01,
    COMMAND_STOP = 0x02,
    COMMAND_PAUSE = 0x03,
    COMMAND_CONTINUE = 0x04,

    COMMAND_PATH_MODE = 0x05,      // 进入执行路径模式
    COMMAND_VELOCITY_MODE = 0x06,  // 进入执行速度模式

    COMMAND_SET_CHECKPOINT = 0x08,  // 设置交通管制点, param0 关卡位置，也就是需要执行完路径的no

    COMMAND_CPU_RESET = 0x0B,  // 重启CPU

    COMMAND_SET_SPEED_LEVEL = 0x0C,  // 设置运行速度级别, level范围为1~100(表示最大速度的1%~100%)

    COMMAND_EMERGENCY_PAUSE = 0x0D,   // 急停触发，紧急暂停
    COMMAND_EMERGENCY_CANCEL = 0x0E,  // 解除紧急暂停状态，解除时需重新初始化驱动器

    COMMAND_CALIB_ODO = 0x0F,  // 标定ODO

    COMMAND_CLEAR_PATH = 0x10,  // 清除当前的路径,为接受新路径做准备

    COMMAND_SET_VELOCITY = 0x11,  // 设置手动控制模式下的运动速度

    COMMAND_RESET_FAULT = 0x13,  // 复位故障
    COMMAND_EMERGENCY_RECOVER = 0x14,  // 发命令让安全单元解除急停
    COMMAND_TRIGGER_EMERGENCY = 0x15,  // 触发急停

    COMMAND_CONNECT = 0x21,      // 请求建立连接
    COMMAND_QUERY_STATE = 0x22,  // 主动查询当前状态
    COMMAND_QUERY_INFO = 0x23,   // 主动查询硬件/版本等信息

    COMMAND_LAUNCH = 0x24, // 给src初始化完后发送这条指令，告诉src可以正常运行了（src_v2添加）

    COMMAND_SET_IO_OUPUT = 0x31,  // 设置io输出寄存器

    COMMAND_NAV_STATE = 0x32,
    COMMAND_SET_LED = 0x33,
    COMMAND_SET_BRAKE = 0x34, //设置抱闸状态

    COMMAND_SET_NAVALGORITHM = 0x40,
    COMMAND_EXECUTE_ACTION = 0x41,  // 启动执行动作任务，如果当前任务未完成，忽略此命令
    COMMAND_CANCEL_ACTION = 0x42,  // 取消当前正在执行的动作任务，立即停止动作执行，置result为FAILED

    COMMAND_QUERY_PGV_VALUE = 0x50,  // 查询PGV偏差值

    COMMAND_UPGRADE = 0x58,  // 0x41

    COMMAND_UPGRADE_TEST = 0x60,

    COMMAND_UPGRADE_REQUEST = 0x61,
    /* 0xA0 ~ 0xFF free of use */
} COMMAND_t;

typedef enum SRCEmergencyPauseReason {
    EMERGENCY_PAUSE_EMERGENCY_STOP = 0x10,
    EMERGENCY_PAUSE_BREAK_SWITCH = 0x20,
    EMERGENCY_PAUSE_LOW_POWER = 0x30,
} SRC_EMERGENCY_PAUSE_REASON_t;

// SRC命令执行结果
typedef enum SRCCommandResult {
    SRC_COMMAND_EXECUTE_OK = 0x10,
    SRC_COMMAND_EXECUTE_FAILED = 0x20,
} SRC_COMMAND_RESULT_t;

// SRC命令执行结果
typedef enum SRCNavType {
    SRC_LOCAL_PLANNER = 0x01,  // 发送主动绕障。
    SRC_PATH_FOLLOW = 0x02,    // 固定路网跟随。
} SRC_NAV_TYPE_t;

typedef struct SRCCommandMsg {
    COMMAND_t command;  // 命令内容
    int32_t param_0;
    int32_t param_1;
    int32_t param_2;
    int32_t param_3;

    uint32_t seq_no;  // 序列号
} SRC_COMMAND_MSG_t;

// SRC状态反馈
typedef enum {
    STATE_NONE = 0x01,

    STATE_PATH_WAITING = 0x02,
    STATE_PATH_RUNNING = 0x03,
    STATE_PATH_WAITING_CHECKPOINT = 0x10,  // [保留]遇到交通管制，暂停等待关卡解除， 放到tate_.reserved_3上传
    STATE_PATH_WAITING_CHECKPOINT_SLOW = 0x11,  // [保留]遇到交通管制，减速运行
    STATE_PATH_PAUSED = 0x04,
    STATE_PATH_OUT = 0x05,  // TODO(x): 使用ERROR_REASON中ERROR_PATH_OUT代替

    STATE_VELOCITY_WAITING = 0x06,
    STATE_VELOCITY_RUNNING = 0x07,
    STATE_VELOCITY_PAUSED = 0x08,

    STATE_NORMAL_ERROR = 0x0B,  // 一般错误,可恢复(接收到STOP命令后恢复到正常状态)
    // 例如接受到的路径错误: (1)最后一条路径编号为零,
    // (2)普通路径结束类型不为NORMAL
    // (3)最后一条路径类型不为LAST, (4)有路径类型不对

    STATE_FATAL_ERROR = 0x0C,  // 致命错误,不可恢复,需要重启
} SRC_STATE_t;

typedef enum SRCErrorReason {
    EROOR_NONE = 0x00,
    ERROR_PATH_OUT = 0x01,  // 偏离预定路径
    ERROR_POSE_TIMEOUT = 0x02,

    ERROR_L14_HEIGHT_TIMEOUT = 0x10,
    ERROR_L14_POSE_TIMEOUT = 0x11,

    // 继续添加
} SRC_ERROR_REASON_t;

typedef enum {
    SIGNAL_NONE = 0x01, // 空信号

    SIGNAL_PATH_FINISHED = 0x02, // 执行到路径的终点
    SIGNAL_PATH_ABORTED = 0x03, // 路径执行提前终止

    SIGNAL_PATH_PAUSED = 0x04, // 执行Path接收到暂停指令后已经暂停
    SIGNAL_UPGRADE_SUCCESS = 0x05, //升级测试/请求成功
    SIGNAL_UPGRADE_FAILED = 0x06,    //升级测试/请求失败
    SIGNAL_PATH_OUT=0x07,    // 路径偏离过大
    SIGNAL_POSE_TIMEOUT=0x08,  // 接受slam位置超时
    SIGNAL_MOTOR_ERROR=0x09,   // 电机有错误吗
    SIGNAL_MOTOR_SLIP=0x0a,   // 驱动轮打滑 取消了
    SIGNAL_MOTOR_TIMEOUT=0x0b,//  驱动器心跳超时。

    SIGNAL_COMMAND_RUN =0x0c, // 接收到command_run 指令。
    SIGNAL_COMMAND_CONTINUE =0x0d, // 接收到command_run 指令。
    SIGNAL_POSE_RECOVERY =0x0e, // 接收到command_run 指令。
    SIGNAL_SEND_ODO_TIMEOUT = 0x0f,// 发送odo超时。
    SIGNAL_COUPLING_BREAK = 0x10,  //联轴器断裂
} SRC_SIGNAL_t;

typedef enum SRCMovementState {
    MOVEMENT_STILL = 0x01,        // 静止
    MOVEMENT_GO_FORWARD = 0x02,   // 前进
    MOVEMENT_GO_BACKWARD = 0x03,  // 后退
    MOVEMENT_TURN_LEFT = 0x04,    // 左转
    MOVEMENT_TURN_RIGHT = 0x05,   // 右转
} SRC_MOVEMENT_t;

typedef unsigned char uint8_t;

// 路径结束类型
typedef enum {
    OVER_NORMAL = 0x01,
    OVER_FIRST = 0x02,
    OVER_LAST = 0x03,
} PATH_OVER_t;

// 路径类型
typedef enum {
    TYPE_LINE = 0x01,
    TYPE_CIRCLE = 0x02,
    TYPE_BEZIER = 0x03,
    TYPE_ROTATE = 0x04,
} PATH_TYPE_t;

// 运动方向
typedef enum {
    DIRECTION_FORWARD = 0x01,   // 运动方向与车正方向方向一致
    DIRECTION_BACKWARD = 0x02,  // 运动方向与车正方向方向相反
} PATH_DIRECTION_t;

// 路径结构体，大小应该为 48 bytes
#pragma pack(push, 4)
typedef struct {
    int32_t sx;      // 起点x坐标，单位mm
    int32_t sy;      // 起点y坐标，单位mm
    int32_t ex;      // 终点x坐标，单位mm
    int32_t ey;      // 终点y坐标，单位mm
    int32_t cx;      // 圆弧圆心x坐标 / 控制点c x坐标，单位mm
    int32_t cy;      // 圆弧圆心y坐标 / 控制点c y坐标，单位mm
    int32_t dx;      // 控制点d x坐标，单位mm
    int32_t dy;      // 控制点d y坐标，单位mm
    int32_t radius;  // 圆弧半径(顺圆为正，逆圆为负，单位mm)
    int32_t angle;   // 原地旋转角度(逆时针位正，顺时针为负，单位是(1/10000)rad)

    int16_t max_v;  // 路径上的最大线速度，单位mm/s
    int16_t mav_w;  // 路径上最大角速度，单位(1/1000)rad/s

    uint8_t no;         // 编号，从1开始
    uint8_t type;       // 路径类型 LINE/ROTATE/CIRCLE/BEZIER
    uint8_t over;       // 结束类型 FIRST/NORMAL/LAST
    uint8_t direction;  // 运动方向 FORWARD(1)/BACKWARD(2)/LEFT(3)/RIGHT(4)
} PATH_t;
#pragma pack(pop)


enum SRC_SYSTEM_STATE {
    SYS_INVALID = 0,
    SYS_INITING = 1,
    SYS_IDLE = 2,
    SYS_BUSY = 3,
    SYS_ERROR = 4,
};

enum SRC_AC_TYPE {
    AC_TASK_TYPE_NULL = 0x00,  // 无任务类型
    AC_TASK_TYPE_SRL_SRD = 0x01, // SRL/SRD 背负牵引类任务
    AC_TASK_TYPE_SRT = 0x02, //SRT辊筒传输类任务
    AC_TASK_TYPE_SRF = 0x03, //SRF叉车类任务
};

typedef struct SRCState {
    SRC_STATE_t src_state = STATE_NONE;  // SRC状态

    // 路网移动任务状态
    sros::core::TaskNo_t mc_task_no = 0;
    SRC_SYSTEM_STATE src_system_state = SYS_INVALID; // src 系统状态（src_v2添加）
    int32_t src_system_error_code;  // src 系统状态错误码（src_v2添加）
    sros::core::TaskState mc_task_state = sros::core::TASK_NA; // 运动控制状态（src_v2添加）
    sros::core::TaskResult mc_result = sros::core::TASK_RESULT_NA; // 运动控制执行结果（src_v2添加）
    int32_t mc_result_code = 0; // 运动控制执行结果的值（src_v2添加）

    // 动作移动任务状态（7,1,0）
    sros::core::TaskNo_t ac_mc_task_no = 0;
    sros::core::TaskState ac_mc_task_state = sros::core::TASK_NA;
    sros::core::TaskResult ac_mc_result = sros::core::TASK_RESULT_NA;
    int32_t ac_mc_result_code = 0;
    
    // 动作任务状态（区别于7,1,0）
    sros::core::TaskNo_t ac_task_no = 0;
    sros::core::TaskState ac_task_state = sros::core::TASK_NA; // 动作控制状态（src_v2添加）
    sros::core::TaskResult ac_result = sros::core::TASK_RESULT_NA; // 动作控制执行结果（src_v2添加）
    int32_t ac_result_code = 0; // 运动控制执行结果的值（src_v2添加）

    int16_t v_x = 0;  // 当前x运动速度（单位 mm/s）
    int16_t v_y = 0;  // 当前y运动速度（单位 mm/s）
    int16_t w = 0;    // 当前运动角速度（单位 (1/1000)rad/s）

    uint8_t path_no = 0;                // 当前正在执行的path编号（0表示没有执行任何路径）
    int16_t path_remain_time = 0;       // 全部路径执行结束剩余时间，小于0等价于无穷（单位s）
    uint16_t path_remain_distance = 0;  // 全部路径执行结束剩余距离（单位cm）
    uint16_t path_total_distance = 0;   // 全部路径距离（单位cm）

    // 运动状态：静止、前进、后退、左转、右转
    // 主要用于外部灯光指示及避障区域设置
    SRC_MOVEMENT_t movement_state = MOVEMENT_STILL;

    uint16_t gpio_input = 0;   // IO输入端口状态
    uint16_t gpio_output = 0;  // IO输出端口状态

    SRC_ERROR_REASON_t error_reason = EROOR_NONE;  // SRC状态为ERROR时，具体的ERROR原因
    enum SRC_AC_TYPE ac_type;

    uint8_t reserved_0 = 0;  // 上传LOAD_STATE
    uint8_t reserved_1 = 0;  // 上传电量(gulf)
    uint8_t reserved_2 = 0;  // 上传关卡的编号（0表示没有关卡）
    uint8_t reserved_3 =
        0;  // 上传是否正在等待关卡，由于老的src用这个字段穿过PGV的值，为做兼容，sros判断src大于某个版本时才认为其值是交通管制
    uint16_t reserved_4 = 0;  // 上传货叉实际高度(gulf)，单位cm
    uint16_t reserved_5 = 0;  // 上传货叉目标高度(gulf)，单位cm
} SRCState_t;

typedef enum SRCActionResult {
    SRC_ACTION_RESULT_RUNNING = 0x00,
    SRC_ACTION_RESULT_OK = 0x01,
    SRC_ACTION_RESULT_FAILED = 0x02,
    SRC_ACTION_RESULT_CANCELED = 0x03,  // 动作被取消
    SRC_ACTION_RESULT_PAUSED = 0x04,
} SRC_ACTION_RESULT_t;

typedef struct SRCActionState {
    uint32_t action_no;      // 动作编号
    uint32_t action_result;  // 参见枚举SRCActionResult
    uint32_t action_result_value;
    uint16_t action_reserved_0;
    uint16_t action_reserved_1;
} SRC_ACTION_STATE_t;

typedef struct SRCMonitorState {
    uint16_t value_0;
    uint16_t value_1;

    uint16_t value_2;
    uint16_t value_3;

    uint32_t value_4;
    uint32_t value_5;
} SRC_MONITOR_STATE_t;

// SRC上连接的设备的工作状态
// 与 sros/core/device/device.h 中定义相同
typedef enum SRCDeviceState {
    DEVICE_NONE = 0x00,  // 状态不可用

    // 0x01 ~ 0x0F
    DEVICE_OK = 0x01,  // 工作正常

    // 0x10 ~ 0x1F
    DEVICE_INITIALIZING = 0x10,  // 初始化中

    // 0x20 ~ 0x2F
    DEVICE_OFF = 0x40,  // 设备正常关闭

    // 0x30 ~ 0x7F
    // RESERVED

    // 0x80 ~ 0xFF
    DEVICE_ERROR = 0x80,              // 其他ERROR
    DEVICE_ERROR_OPEN_FAILED = 0x81,  // 设备打开失败
    DEVICE_ERROR_TIMEOUT = 0x82,      // 通信超时
    DEVICE_ERROR_INITIAL = 0x93,      // 初始化出错
} SRC_DEVICE_STATE_t;

// 设备状态统一使用SRC_DEVICE_STATE_t枚举定义的值
typedef struct SRCHardwareState {
    uint32_t total_power_cycle;   // 上电次数
    uint32_t total_poweron_time;  // 总开机时间，单位s
    uint32_t total_mileage;       // 总运动里程，单位m

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

    uint8_t device0_status;  // IMU
    uint8_t device1_status;  // PGV1(下视)
    uint8_t device2_status;  // PGV2(上视）
    uint8_t device3_status;  // 手动控制器
    uint8_t device4_status;
    uint8_t device5_status;
    uint8_t device6_status;
    uint8_t device7_status;

    uint64_t reserved;
} SRC_HARDWARE_STATE_t;

// SRTOS寄存器地址
enum SRTOS_ADDR {
    // FIXME(pengjiali): srtos 地址是错的需要最终更新一下
    // 配置
    SRTOS_ADDR_SYSTEM_CONFIG = 0x0101,
    SRTOS_ADDR_MC_CONFIG = 0x0001,
    SRTOS_ADDR_AC_CONFIG = 0x0201,

    SRTOS_ADDR_CPU_USAGE = 0x1000,
    SRTOS_ADDR_MOTOR_1_ID = 0x1018,

    SRTOS_ADDR_CONTROL_TYPE = 0x1450,  //手动/自动 控制模式

    // 系统状态寄存器0x2000
    SRTOS_ADDR_HW_VERSION = 0x2000,
    SRTOS_ADDR_HW_SERIAL_NO = SRTOS_ADDR_HW_VERSION + 1,
    SRTOS_ADDR_KERNEL_VERSION = SRTOS_ADDR_HW_SERIAL_NO + 4,
    SRTOS_ADDR_FM_VERSION = SRTOS_ADDR_KERNEL_VERSION + 1,
    SRTOS_ADDR_FM_COMMIT_ID = SRTOS_ADDR_FM_VERSION + 1,
    SRTOS_ADDR_FM_COMPILE_DATE = SRTOS_ADDR_FM_COMMIT_ID + 1,

    SRTOS_ADDR_REQ_SEQ = 0x2012,
    SRTOS_ADDR_MONITOR_INFO = 0x2022, // 看门狗

    SRTOS_ADDR_TASK_MC_ID = 0x2400,
    SRTOS_ADDR_MC_MODE = 0x2408,
    SRTOS_ADDR_VELOCITY_TIMESTAMP = 0x2419,
    SRTOS_ADDR_POSE_TIMESTAMP = 0x241D,
    SRTOS_ADDR_OPT_POSE_TIMESTAMP = 0x2424,
    SRTOS_ADDR_SET_PATHS = 0x242B,

    SRTOS_ADDR_SVC_DOWN_STATUS = 0x2800, // svc100的偏差
    SRTOS_ADDR_SVC_UP_STATUS = 0x2808, // svc100的偏差
    SRTOS_ADDR_SVC_LEFT_STATUS = 0x2810,
    SRTOS_ADDR_SVC_RIGHT_STATUS = 0x2818,
    SRTOS_ADDR_PGV_OFFSET_X_POSITION = 0x2820,
    SRTOS_ADDR_SHELF_X_POSITION = 0x2824,
    SRTOS_ADDR_TASK_AC_NO = 0x2890,
    SRTOS_ADDR_CUR_ACTION_STATE = 0x289B,
    SRTOS_ADDR_ACTION_DEBUG_STATE = 0x28D0,

    SRTOS_ADDR_CMD = 0x3000,
};


#endif  // SROS_SRC_SDK_PROTOCOL_SRC_PROTOCOL_H_
