/**
 * @file sr_modbus_address.h
 *
 * @author pengjiali
 * @date 18-10-24.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_MODBUS_SR_MODBUS_ADDRESS_H_
#define CORE_MODBUS_SR_MODBUS_ADDRESS_H_

#include <stdint.h>

namespace sros {

/**
 * ## 地址段分配
 * .
 * ├── [0, 999]/            # SROS专用地址
 * │   └── [300, 400]/      # 扩展动作控制器专用
 * │   └── [400, 500]/      # 安全PLC/手控盒专用
 * │── [5000, 5999]  /      # src 寄存器
 * │   └── [5500, 5599]/    # src 动作状态寄存器 src v1
 * │   └── [5600, 5799]/    # src 动作状态寄存器 src v2(SRTOS)
 * └── [9000, 9999]  /      # 调度
 */
// NOTE：所有地址段偏移地址大于等于9000的都被占用了，如触摸屏里面用大于9000偏移的地址来链接触摸屏
// Discrete Inputs Address (离散量输入状态)
// 基地址 10001(十进制)
const uint16_t DIA_EMERGENCY_STATE = 0x0000;              // 急停状态      NOTE:以后这种开关类型考虑一个寄存器实现
const uint16_t DIA_EMERGENCY_RECOVERABLE_STATE = 0x0001;  // 急停可恢复
const uint16_t DIA_BREAK_SWITCH_STATE = 0x0002;           // 抱闸开关状态
const uint16_t DIA_CHARGE_STATE = 0x0003;                 // 充电状态
const uint16_t DIA_POWER_SAVE_MODE = 0x0004;              // 是否处于低功耗模式
const uint16_t DIA_SLOWDOWN_FOR_OBSTACLE_STATE = 0x0005;  // 遇到障碍减速
const uint16_t DIA_PAUSED_FOR_OBSTACLE_STATE = 0x0006;    // 遇到障碍暂停
const uint16_t DIA_READY_FOR_NEW_MOVEMENT_TASK = 0x0008;  // 是否可以运行新移动任务
// const uint16_t DIA_SAVE = 5; // 保留
const uint16_t DIA_DI_0_STATE = 0x0010;              // DPIO input
const uint16_t DIA_DI_1_STATE = DIA_DI_0_STATE + 1;  // DPIO input
const uint16_t DIA_DI_2_STATE = DIA_DI_0_STATE + 2;  // DPIO input
const uint16_t DIA_DI_3_STATE = DIA_DI_0_STATE + 3;  // DPIO input
const uint16_t DIA_DI_4_STATE = DIA_DI_0_STATE + 4;  // DPIO input
const uint16_t DIA_DI_5_STATE = DIA_DI_0_STATE + 5;  // DPIO input
const uint16_t DIA_DI_6_STATE = DIA_DI_0_STATE + 6;  // DPIO input
const uint16_t DIA_DI_7_STATE = DIA_DI_0_STATE + 7;  // DPIO input

const uint16_t DIA_DO_0_STATE = 0x0020;              // DPIO output
const uint16_t DIA_DO_1_STATE = DIA_DO_0_STATE + 1;  // DPIO output
const uint16_t DIA_DO_2_STATE = DIA_DO_0_STATE + 2;  // DPIO output
const uint16_t DIA_DO_3_STATE = DIA_DO_0_STATE + 3;  // DPIO output
const uint16_t DIA_DO_4_STATE = DIA_DO_0_STATE + 4;  // DPIO output
const uint16_t DIA_DO_5_STATE = DIA_DO_0_STATE + 5;  // DPIO output
const uint16_t DIA_DO_6_STATE = DIA_DO_0_STATE + 6;  // DPIO output
const uint16_t DIA_DO_7_STATE = DIA_DO_0_STATE + 7;  // DPIO output

const uint16_t DIA_WAITING_LET_GO = 0x0030;  // 是否正在等待放行

const uint16_t DIA_FLEET_MODE = 0x0032;  // 是否正在调度模式

// Discrete Output Coils (线圈状态)
// 基地址 00001(十进制)
const uint16_t DOC_PAUSE = 0x0000;     // 暂停运动
const uint16_t DOC_CONTINUE = 0x0001;  // 继续运动
const uint16_t DOC_STOP = 0x0002;      // 停止运动（取消移动任务）

const uint16_t DOC_STOP_LOCATION = 0x0004;  // 停止定位

const uint16_t DOC_TRIGGER_EMERGENCY = 0x0006;     // 触发急停
const uint16_t DOC_CANCEL_EMERGENCY = 0x0007;      // 解除急停
const uint16_t DOC_ENABLE_CHARGE = 0x0008;         // 启动充电
const uint16_t DOC_DISENABLE_CHARGE = 0x0009;      // 停止充电
const uint16_t DOC_ENABLE_POWER_MODE = 0x000A;     // 进入低功耗模式
const uint16_t DOC_DISENABLE_POWER_MODE = 0x000B;  // 退出低功耗模式

const uint16_t DOC_RESTART_SYSTEM = 0x000D;           // 重启系统
const uint16_t DOC_ENABLE_MANUAL_CONTROL = 0x00E;     // 启动手动控制
const uint16_t DOC_DISENABLE_MANUAL_CONTROL = 0x00F;  // 停止手动控制

const uint16_t DOC_GO_FORWARD = 0x0010;  // 以指定速度向前移动
const uint16_t DOC_GO_BACK = 0x0011;     // 以指定速度向后移动
const uint16_t DOC_GO_LEFT = 0x0012;     // 以指定速度向左旋转
const uint16_t DOC_GO_RIGHT = 0x0013;    // 以指定速度向右旋转

const uint16_t DOC_DPIO_OUTPUT_0 = 0x0020;                 // dpio 接口
const uint16_t DOC_DPIO_OUTPUT_1 = DOC_DPIO_OUTPUT_0 + 1;  // DPIO output
const uint16_t DOC_DPIO_OUTPUT_2 = DOC_DPIO_OUTPUT_0 + 2;  // DPIO output
const uint16_t DOC_DPIO_OUTPUT_3 = DOC_DPIO_OUTPUT_0 + 3;  // DPIO output
const uint16_t DOC_DPIO_OUTPUT_4 = DOC_DPIO_OUTPUT_0 + 4;  // DPIO output
const uint16_t DOC_DPIO_OUTPUT_5 = DOC_DPIO_OUTPUT_0 + 5;  // DPIO output
const uint16_t DOC_DPIO_OUTPUT_6 = DOC_DPIO_OUTPUT_0 + 6;  // DPIO output
const uint16_t DOC_DPIO_OUTPUT_7 = DOC_DPIO_OUTPUT_0 + 7;  // DPIO output

const uint16_t DOC_LET_GO = 0x0030;  // 放行

const uint16_t DOC_FLEET_MODE = 0x0032;  // 1进入调度模式/0退出调度模式

// mission
const uint16_t DOC_PAUSE_MISSION = 0x0060;     // 暂停任务
const uint16_t DOC_CONTINUE_MISSION = 0x0061;  // 继续任务
const uint16_t DOC_CANCEL_MISSION = 0x0062;    // 取消任务

// Input Registers Address
// 基地址 30001(十进制)
const uint16_t IRA_SYS_STATE = 0x0000;                 // 系统状态 (enum/uint16)
const uint16_t IRA_LOCATION_STATE = 0x0001;            // 定位状态 (enum/uint16)
const uint16_t IRA_POSE = 0x0002;                      // 位姿 (int32_t) * 3 单位mm/s 单位mm/s 单位(1/1000)rad/s
const uint16_t IRA_RELIABILITY_STATE = 0x0008;         // 位姿置信度 (uint16_t)
const uint16_t IRA_MOVEMENT_STATE_ABANDONED = 0x0009;  // [废弃] 运动状态 (enum/uint16)
const uint16_t IRA_MOVEMENT_TASK_STATE = 0x000A;       // [废弃] 导航任务状态 (enum/uint16)
const uint16_t IRA_ACTION_TASK_STATE = 0x000B;         // [废弃] 动作任务状态 (enum/uint16)
const uint16_t IRA_ACTION_RESULT_ABANDONED = 0x000C;   // [废弃] 动作任务结果值 (uint32_t)
const uint16_t IRA_STATION_NO = 0x000E;                // 当前站点编号 (uint16_t)
const uint16_t IRA_MANUAL_CONTROL_STATE = 0x000F;      // 手动控制状态 (enum/uint16) OperationState
const uint16_t IRA_SPEED_FOR_X_STATE = 0x0010;         // x方向线速度  (int16) 单位mm/s NOTE: 16位足够了
const uint16_t IRA_SPEED_FOR_Y_STATE = 0x0011;         // y方向线速度  (int16) 单位mm/s
const uint16_t IRA_SPEED_FOR_YAW_STATE = 0x0012;       // 角速度  (int16) 单位(1/1000)rad/s

const uint16_t IRA_GPIO_INPUT = 0x0014;   // DI状态 (uint16_t)
const uint16_t IRA_GPIO_OUTPUT = 0x0015;  // DO状态 (uint16_t)

const uint16_t IRA_HARDWARE_ERROR_CODE = 0x0018;  // 硬件错误码 (uint32_t)
const uint16_t IRA_SYSTEM_LAST_ERROR = 0x001A;    // 系统错误码 (uint32_t)

const uint16_t IRA_BATTERY_VOLTAGE = 0x0020;                // 电池电压 (uint16) 单位mV
const uint16_t IRA_BATTERY_ELECTRICITY = 0x0021;            // 电池电流 (int16) 单位mA (负数为充电电流）
const uint16_t IRA_BATTERY_TEMPERATURE = 0x0022;            // 电池温度 (int16) 单位0.1℃
const uint16_t IRA_BATTERY_ESTIMATED_USABLE_TIME = 0x0023;  // 预计可使用时间 (uint16) 单位hour
const uint16_t IRA_BATTERY_PERCENTAGE = 0x0024;             // 当前电量百分比 (uint16) [1, 100]
const uint16_t IRA_BATTERY_STATE = 0x0025;                  // 当前电池的状态 (enum/uint16) BatteryState
const uint16_t IRA_BATTERY_USE_CYCLES = 0x0026;             // 电池循环次数 (enum/uint16)
const uint16_t IRA_BATTERY_NOMINAL_CAPACITY = 0x0027;       // 电池标称容量 (enum/uint16) 单位mA

const uint16_t IRA_TOTAL_MILEAGE = 0x0028;  // 总里程 单位m (uint32_t)
const uint16_t IRA_POWERON_TIME = 0x002A;   // 总开机时间 单位s (uint32_t)
const uint16_t IRA_POWER_CYCLE = 0x002C;    // 开机总次数 (uint32_t)
const uint16_t IRA_SYSTEM_TIME = 0x002E;    // 系统时间 Linux时间戳 (uint32_t)

const uint16_t IRA_IP_1 = 0x0030;                  // 对外通信IP地址1 (uint16_t)
const uint16_t IRA_IP_2 = 0x0031;                  // 对外通信IP地址2 (uint16_t)
const uint16_t IRA_IP_3 = 0x0032;                  // 对外通信IP地址3 (uint16_t)
const uint16_t IRA_IP_4 = 0x0033;                  // 对外通信IP地址4 (uint16_t)
const uint16_t IRA_SYSTEM_MAJOR_VERSION = 0x0034;  // 系统版本 (uint16_t)
const uint16_t IRA_SYSTEM_MINOR_VERSION = 0x0035;  // 系统次版本 (uint16_t)
const uint16_t IRA_SYSTEM_PATCH_VERSION = 0x0036;  // 系统修订版本 (uint16_t)

const uint16_t IRA_DOWN_PGV_ID = 0x0038;   // 下视PGV扫描到的二维码ID (int32_t)
const uint16_t IRA_DOWN_PGV_X = 0x003A;    // 下视PGV扫描到的二维码x (int32_t)
const uint16_t IRA_DOWN_PGV_Y = 0x003C;    // 下视PGV扫描到的二维码y (int32_t)
const uint16_t IRA_DOWN_PGV_YAW = 0x003E;  // 下视PGV扫描到的二维码yaw (int32_t)

const uint16_t IRA_CUR_MAP = 0x0040;          // 当前地图名的前两个字节  (uint16_t)
const uint16_t IRA_CUR_MOVEMENT_NO = 0x0041;  // [废弃] 当前移动任务no (int32_t)
const uint16_t IRA_CUR_ACTION_NO = 0x0043;    // [废弃] 当前动作任务no (int32_t)
const uint16_t IRA_VOLUME = 0x0045;           // 系统音量 (uint16_t)

const uint16_t IRA_HARDWARE_ERROR_CODE_1 = 0x0050;  // 硬件错误码1 (uint32_t) 前5个错故障码
const uint16_t IRA_HARDWARE_ERROR_CODE_2 = 0x0052;  // 硬件错误码2 (uint32_t) 下面几个为代码自动设置，所以可能未用到
const uint16_t IRA_HARDWARE_ERROR_CODE_3 = 0x0054;  // 硬件错误码3 (uint32_t)
const uint16_t IRA_HARDWARE_ERROR_CODE_4 = 0x0056;  // 硬件错误码4 (uint32_t)
const uint16_t IRA_HARDWARE_ERROR_CODE_5 = 0x0058;  // 硬件错误码5 (uint32_t)

// mission状态
const uint16_t IRA_RUNNING_MISSION = 0x0060;     // 当前正在运行的mission id (uint32_t)
const uint16_t IRA_MISSION_STATE = 0x0062;       // Mission运行状态(uint16_t)
const uint16_t IRA_MISSION_RESULT = 0x0063;      // Mission执行结果(uint16_t)
const uint16_t IRA_MISSION_ERROR_CODE = 0x0064;  // Mission错误码(uint32_t)

// movement 状态
const uint16_t IRA_MOVEMENT_STATE = 0x0070;         // 运动状态 (enum/uint16)
const uint16_t IRA_MOVEMENT_NO = 0x0071;            // 当前移动任务no (int32_t)
const uint16_t IRA_MOVEMENT_DST_STATION = 0x0073;   // 当前移动任务目标站点 (uint16_t)
const uint16_t IRA_MOVEMENT_CUR_PATH = 0x0074;      // 当前路径编号，移动任务运行过程中有效 (uint16_t)
const uint16_t IRA_MOVEMENT_RESULT = 0x0079;        // 移动任务结果 (enum/uint16)
const uint16_t IRA_MOVEMENT_RESULT_VALUE = 0x007A;  // 移动任务结果值 (uint32_t)

// action 状态
const uint16_t IRA_ACTION_STATE = 0x0080;         // 动作cc任务状态 (enum/uint16)
const uint16_t IRA_ACTION_NO = 0x0081;            // 当前动作任务no (int32_t)
const uint16_t IRA_ACTION_ID = 0x0083;            // 当前动作任务ID (int32_t)
const uint16_t IRA_ACTION_PARAM_0 = 0x0085;       // 当前动作任务参数0 (int32_t)
const uint16_t IRA_ACTION_PARAM_1 = 0x0087;       // 当前动作任务参数1 (int32_t)
const uint16_t IRA_ACTION_RESULT = 0x0089;        // 动作任务结果 (enum/uint16)
const uint16_t IRA_ACTION_RESULT_VALUE = 0x008A;  // 动作任务结果值 (uint32_t)

//硬件设备状态显示，每一位代表一种设备，该位置1表示正常，置0表示异常
/*
 * 00000001: 激光导航雷达
 * 00000010: D435深度相机
 * 00000100: O3D303探货相机
 * 00001000: 左侧避障雷达
 * 00010000: 后侧避障雷达
 * 00100000: 右侧避障雷达
*/
const uint16_t IRA_DEVICE_STATE = 0x008C;         // 设备状态

// 自定义寄存器
// src 自定义寄存器 [100, 199]
const uint16_t IRA_TIE_AN_JACK_HEIGHT = 0x0096;  // 获取铁安顶升机构高度 （int16_t)


//src v1动作调试信息
const uint16_t SRC_ACTION_DEBUG_INFO_1 = 0x157b; // SRC 动作调试地址0x5000
const uint16_t SRC_ACTION_DEBUG_INFO_2 = 0x157c; // SRC 动作调试地址0x5001
const uint16_t SRC_ACTION_DEBUG_INFO_3 = 0x157d; // SRC 动作调试地址0x5002
const uint16_t SRC_ACTION_DEBUG_INFO_4 = 0x157e; // SRC 动作调试地址0x5003

//src v2(SRTOS) 动作控制调试
const uint16_t SRTOS_ACTION_DEBUG_INFO = 0x15df; //SRTOS 动作控制调试数据起地址

// Holding Registers Address
// 基地址 40001(十进制)
const uint16_t HRA_START_LOCATION_POSE = 0x0000;  // 通过位姿定位 (int32_t) * 3 单位mm/s 单位mm/s 单位(1/1000)rad/s
const uint16_t HRA_START_LOCATION_STATION = 0x0006;  // 通过站点定位 (uint16_t)
const uint16_t HRA_MOVE_TO_POSE = 0x0008;     // 移动到位姿 (int32_t) * 3 单位mm/s 单位mm/s 单位(1/1000)rad/s
const uint16_t HRA_MOVE_TO_STATION = 0x000E;  // 移动到站点 (uint16_t)
const uint16_t HRA_SET_AVOID_POLICY = 0x000F;  // 设置壁障策略  (enum/uint16)
const uint16_t HRA_SET_ACTION = 0x0010;        // 执行动作 (uint16_t) × 3

const uint16_t HRA_MANUAL_CONTROL_V_X = 0x0015;    // 手动控制模式下设置Vx线速度  (int16) 单位mm/s
const uint16_t HRA_MANUAL_CONTROL_V_Y = 0x0016;    // 手动控制模式下设置Vy线速度 保留 (int16) 单位mm/s
const uint16_t HRA_MANUAL_CONTROL_V_YAW = 0x0017;  // 手动控制模式下设置w角速度  (int16) 单位(1/1000)rad/s

const uint16_t HRA_SET_SPEED_LEVEL = 0x0019;  // 设置速度级别 (uint16_t) [1, 100]
const uint16_t HRA_SET_CUR_STATION = 0x001A;  // 设置当前站点 (uint16_t)
const uint16_t HRA_VOLUME = 0x001B;           // 设置音量 (uint16_t) [1, 100]

const uint16_t HRA_SET_CUR_MAP =
    0x001C;  // 设置当前地图的前两个字节，若存在多个地图前两个字节相同则按照字符排序选最前一个(uint16_t)
const uint16_t HRA_SET_GPIO_OUT_VALUE = 0x001D;  // 设置GPIO输出值 (uint16_t)
const uint16_t HRA_SET_GPIO_OUT_MASK = 0x001E;  // 设置GPIO输出掩码 (uint16_t) FIXME(pengjiali):4.9.0 现在src没有做掩码

const uint16_t HRA_PROXY_IO_0 = 0x0020;  // 作为存储中介供外部读写操作寄存器，
const uint16_t HRA_PROXY_IO_1 = 0x0021;
const uint16_t HRA_PROXY_IO_2 = 0x0022;
const uint16_t HRA_PROXY_IO_3 = 0x0023;
const uint16_t HRA_PROXY_IO_4 = 0x0024;
const uint16_t HRA_PROXY_IO_5 = 0x0025;
const uint16_t HRA_PROXY_IO_6 = 0x0026;
const uint16_t HRA_PROXY_IO_7 = 0x0027;

const uint16_t HRA_START_LOCATION_POSE_FORCE =
    0x0030;  // 通过位姿强制定位 (int32_t) * 3 单位mm/s 单位mm/s 单位(1/1000)rad/s
const uint16_t HRA_MOVE_TO_POSE_WITH_NO = 0x0038;  // 移动到位姿 (int32_t) * 3 单位mm/s 单位mm/s 单位(1/1000)rad/s
const uint16_t HRA_MOVE_TO_STATION_WITH_NO = 0x0041;  // 移动到站点 (uint16_t)
const uint16_t HRA_SET_ACTION_WITH_NO = 0x0045;       // 执行动作 (int32_t) × 3

// mission
const uint16_t HRA_START_MISSION = 0x0060;  // 执行mission任务, (int32_t) * 1

}  // namespace sros

#endif  // CORE_MODBUS_SR_MODBUS_ADDRESS_H_
