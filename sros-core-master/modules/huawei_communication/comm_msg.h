//
// describe: 
// Created by pengjiali on 18-11-6.
//

#ifndef SROS_COMM_MSG_H
#define SROS_COMM_MSG_H

#include <memory>
#include <algorithm>    // std::move (ranges)
#include <iterator>
#include <iostream>
#include <glog/logging.h>

namespace huawei {
// 用户级错误
const uint16_t ERROR_EMERGENCY_TRIGGERED = 0x0001; // 急停触发
const uint16_t ERROR_BREAK_SWITCH_TRIGGERED = 0x0002; // 抱闸
const uint16_t ERROR_LOCATION_NOT_RUNNING = 0x0003; // 定位未启动
const uint16_t ERROR_LOW_BATTERY = 0x0004; // 低电量
const uint16_t ERROR_LAST_ACTION = 0x0005; // 上次执行动作失败
// 开发级错误
const uint16_t ERROR_SYSTEM_BUSY = 0x0101; // 系统繁忙
const uint16_t ERROR_SYSTEM = 0x0102; // 系统错误，1.在系统出错的情况下收到动作命令 <br/> 2.在查状态的时候没有出错，然后发命令的时候出错了
const uint16_t ERROR_LAST_MOVEMENT = 0x0103; // 上次移动站点失败
const uint16_t ERROR_REQUEST_CMD = 0x0104; // 命令字未定义
const uint16_t ERROR_REQUEST_DATA = 0x0105; // 命令的数据域出错
const uint16_t ERROR_FAILED_EXEC = 0x0106; // 命令执行失败

const uint16_t ERROR_ERROR_NO_GOODS_GET_UNLOAD_CMD = 0x0108; // 链板线上没货收到了下货指令
const uint16_t ERROR_ERROR_HAS_GOODS_GET_LOAD_CMD = 0x0109; // 链板线上有货收到了上货指令

// 需要人为干预的错误
const uint16_t FAILED_SENSOR_ERROR = 0x0201; // 两边的传感器检测到有东西
const uint16_t FAILED_NAV_NO_WAY = 0x0202; // 无法导航到站点
const uint16_t FAILED_LACATION_ERROR = 0x0203; // 定位错误

// 告警
const uint16_t WARNING_PAUSED_FOR_OBSTACAL = 0x0301; // 前方有障碍导致暂停等待
const uint16_t WARNING_LOW_BATTERY = 0x0302; // 电量低 ,电量低于30%

enum LINE_HAS_GOODS { // 链板线是否有货
    NO_GOODS = 0x00, // 没货
    FIRST_LINE_HAS_GOODS = 0x01, // 第一排有货
    SECOND_LINE_HAS_GOODS = 0x02, // 第二排有货
    BOTH_LINE_HAS_GOODS = 0x03, // 两条线都有或

    GOODS_INFO_ERROR = 0xFF, // 是否后货的传感器信息有误
};

enum IO_INSTRUCT {
    FRIST_LINE_ROTATE_REVERSE = 0x00, // 第一排反转
    FRIST_LINE_ROTATE_FORWARD = 0x01, // 第一排正转
    SECOND_LINE_ROTATE_REVERSE = 0x02, // 第一排反转
    SECOND_LINE_ROTATE_FORWARD = 0x03, // 第二排正转
};

class CommMsg {
public:
    enum class Field { // 协议字段
        SOF = 0, // 起始符 1
        T,  // 设备类型域 1
        A,  // 标识位 1
        D,  // 方向字 1
        L,  // 数据长度域 1
        C,  // 命令字 1
        DATA, //  数据域 0<N<=255
        // 由于DATA是变成，下面两个字段的index不正确
                CS, // 累加和校验码 1
        EOF_F // 结束符 1
    };

    enum class CMD {
        INITIALIZE = 0x00, // 初始化
        QUERY_STATE = 0x01, // 查询状态
        QUERY_POSITION = 0x02, // 位置查询
        PARAM_SETTING = 0x03, // 参数设置
        IO_SETTING = 0x04, // IO设置
        MOVE_TO_POSITION = 0x05, // 定点移动
        LOAD_MATERIAL = 0x06, // 接料
        UNLOAD_MATERIAL = 0x07, // 送料
        MOVE_TO_ORIGIN = 0x08, // 回到原点
        RESERVED = 0x09, // 预留
        QUERY_ALARM = 0x0A, // 查询警报
    };

    void init() { is_error_ = false; }

    void generateRawData();

    bool decodeRawData();

    const std::vector<uint8_t>& getData() const { return data_; }
    void setErrorCode(uint16_t mask);
    bool isError() const { return is_error_; } // 返回是否错误
    void setData(const std::vector<uint8_t> &data); // 设置正常的数据

    int getLeftLength() { return raw_data_[(size_t)Field ::L] + 1; } // 获取剩余的长度，
    std::vector<uint8_t> raw_data_; // 原始完整数据帧

    uint8_t seq_; // 标示
    uint8_t command_; // 指令

    const int header_length_ = 5; // 头的长度
private:
    bool is_error_ = false; // 是否错误

    std::vector<uint8_t> data_; // 数据段
};

}


#endif //SROS_COMM_MSG_H
