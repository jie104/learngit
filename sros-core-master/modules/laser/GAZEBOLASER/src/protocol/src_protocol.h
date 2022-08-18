//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_SRC_PROTOCOL_H
#define SRC_SDK_NETWORK_PROTOCOL_SRC_PROTOCOL_H

#define COMMAND_FIELD_SIZE      1
#define SRC_STATE_FIELD_SIZE    1
#define SONAR_STATE_FIELD_SIZE  1
#define SRC_PARAMETER_FIELD_SIZE 32
#define SRC_SIGNAL_FIELD_SIZE   1
#define SRC_USART_DATA_SIZE 64

#define SRC_PROTOCOL_HEADER_SIZE 3
#define MAX_PROTOCOL_MSG_SIZE 1498
namespace network {

typedef enum MSG_TYPE {
    MSG_COMMAND = 0x01,
    MSG_STATE,
    MSG_VELOCITY,
    MSG_POSE,
    MSG_PATH,
    MSG_GAZEBO_LASER_SCAN,
    MSG_PARAMETER,
    MSG_SIGNAL,
    MSG_INFO,
    MSG_LASER_SCAN_STAMPED,
    MSG_POSE_STAMPED,
    MSG_USART_DATA,
    PF_LASER_SCAN_STAMPED,
} MSG_TYPE_t;

// 指令
typedef enum {
    COMMAND_RUN = 0x01,
    COMMAND_STOP,
    COMMAND_PAUSE,
    COMMAND_CONTINUE,

    COMMAND_PATH_MODE,      // 进入执行路径模式
    COMMAND_VELOCITY_MODE,  // 进入执行速度模式

    COMMAND_MOTOR_ENABLE,   // 电机使能
    COMMAND_MOTOR_DISABLE,  // 电机失能（手动推拉）

    COMMAND_SONAR_ENABLE,
    COMMAND_SONAR_DISABLE,

    COMMAND_CPU_RESET, // 重启CPU

    COMMAND_SET_SPPED_LEVEL, // 设置运行速度级别, level范围为1~100(表示最大速度的1%~100%)

    COMMAND_EMERGENCY_PAUSE, // 触发紧急暂停,进入状态
    COMMAND_EMERGENCY_CANCEL, // 解除紧急暂停状态(解除时需检查IO信号是否仍然有效)

    COMMAND_CALIB_ODO,//进入ODO标定模式

    COMMAND_CLEAR_PATH, // 清除当前的路径,为接受新路径做准备
} COMMAND_t;


// SRC状态反馈
typedef enum {
    STATE_NONE = 0x01,

    STATE_PATH_WAITING,
    STATE_PATH_RUNNING,
    STATE_PATH_PAUSED,
    STATE_PATH_OUT, // 偏离预定路径

    STATE_VELOCITY_WAITING,
    STATE_VELOCITY_RUNNING,
    STATE_VELOCITY_PAUSED,

    STATE_SONAR_PAUSED, // 遇到障碍暂停

    STATE_MOTOR_DISABLED, // 电机失能

    STATE_NORMAL_ERROR,       // 一般错误,可恢复(接收到STOP命令后恢复到正常状态)
    // 例如接受到的路径错误: (1)最后一条路径编号为零,
    // (2)普通路径结束类型不为NORMAL
    // (3)最后一条路径类型不为LAST, (4)有路径类型不对

    STATE_FATAL_ERROR, // 致命错误,不可恢复,需要重启

    STATE_EMERGENCY_PAUSED, // 紧急停车,通过IO信号触发
} SRC_STATE_t;

typedef enum {
    SIGNAL_NONE = 0x01, // 空信号

    SIGNAL_PATH_FINISHED, // 执行到路径的终点
    SIGNAL_PATH_ABORTED, // 路径执行提前终止

    SIGNAL_PATH_PAUSED, // 执行Path接收到暂停指令后已经暂停

    SIGNAL_EMERGENCY_PAUSED, // 紧急停车,通过IO信号触发
    SIGNAL_EMERGENCY_RESUME, //状态恢复
} SRC_SIGNAL_t;

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
    DIRECTION_FORWARD = 0x01, // 运动方向与车正方向方向一致
    DIRECTION_BACKWARD = 0x02, // 运动方向与车正方向方向相反
} PATH_DIRECTION_t;

// 路径结构体，大小应该为 44 bytes
#pragma pack(push, 4)
typedef struct {
    float sx;       // 起点x坐标
    float sy;       // 起点y坐标
    float ex;       // 终点x坐标
    float ey;       // 终点y坐标
    float cx;       // 圆弧圆心x坐标 / 控制点c x坐标
    float cy;       // 圆弧圆心y坐标 / 控制点c y坐标
    float dx;       // 控制点d x坐标
    float dy;       // 控制点d y坐标
    float radius;   // 圆弧半径(顺圆为正，逆圆为负，单位m)
    float angle;    // 原地旋转角度(逆时针位正，顺时针为负，单位是rad)
    uint8_t no;    // 编号，从1开始
    uint8_t type;  // 路径类型 LINE/ROTATE/CIRCLE/BEZIER
    uint8_t over;  // 结束类型 FIRST/NORMAL/LAST
    uint8_t direction; // 运动方向 FORWARD/BACKWARD
} PATH_t;
}
#pragma pack(pop)

#endif //SRC_SDK_NETWORK_PROTOCOL_SRC_PROTOCOL_H
