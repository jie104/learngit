//
// Created by john on 18-9-27.
//

#ifndef SROS_HUAWEI_COMMUNICATION_MODULE_H
#define SROS_HUAWEI_COMMUNICATION_MODULE_H

#include "core/pose.h"
#include "core/module.h"
#include "core/msg/str_msg.hpp"

#include "core/usart/connection.hpp"
#include "simple_server.h"
#include "modules/communication/communication_module.h"
#include "comm_msg.h"
#include "core/hardware/LC100.h"

namespace huawei {

enum LINE_SENSOR { // 链板线上传感器的状态
    FRIST_LINE_LEFT_SENSOR = 0x01,
    FRIST_LINE_MIDDLE_SENSOR = 0x02,
    FRIST_LINE_RIGHT_SENSOR = 0x04,
    SECOND_LINE_LEFT_SENSOR = 0x08,
    SECOND_LINE_MIDDLE_SENSOR = 0x10,
    SECOND_LINE_RIGHT_SENSOR = 0x20,
};

// TODO:该类是为华为 测试线 做的通信，需要统一将名字改为测试线，因为华为还有其他的对接项目
class HuaweiCommModule: public sros::CommunicationModule {
public:
    HuaweiCommModule();
    virtual ~HuaweiCommModule() = default;

    virtual bool subClassRunPrepare() override;

private:
    void recvDataMsgCallback(SimpleSession_Ptr session_ptr); // 其他线程调用该线程
    void onNewMsg(sros::core::base_msg_ptr msg);

    virtual void onMoveTaskFinishedNotify(sros::core::NotificationMsg_ptr msg) override;
    virtual void onActionTaskFinishedNotify(sros::core::NotificationMsg_ptr msg) override;

    bool isSystemError() const;

    void runServer();
    void onTimer_200ms(sros::core::base_msg_ptr m);


    void updateGoodsInfo(); // 更新链板线上获取的情况

    void updateLEDState(); // 更新led灯的状态

    bool sendCanMsg(uint32_t id, const std::vector<uint8_t> &data);

    bool enabled_ = false; // 是否是能本模块

    std::shared_ptr<SimpleServer> server_ptr_;

    int low_battery_threshold_; // 低电量阈值

    bool is_action_run_ = false; // 标记是否在上货或者是在下货
    bool is_pgv_run_ = false; // 标记pgv是否在运动, 不包括pgv是否run2
    bool is_agv_run_ = false; // 标记agv是否在运动

    bool is_location_error_ = false; // 标记定位出错，此时需要人工干预，取消定位，并且重新定位

    LINE_HAS_GOODS line_goods_info_ = NO_GOODS; // 传感器上货物的信息

    // TODO: 此处可能会隐含一个问题，就是chip发送的任务时累加的，可能会和此处固定的action冲突，
    // 导致出错，现在下层的active_no都为无符，所以不能用负的no
    const sros::core::TaskNo_t movement_task_no_ = -888; // 移动动作的movement
    const sros::core::TaskNo_t task_no_ = 99999; // taskNo
    const sros::core::TaskNo_t pgv_adjust_task_no_ = 99998; // pgv校准的动作

    sros::device::LC100_ptr lc100_a_; // 电气图中左边的lc100，LED1控制三色灯，LED2控制“运行、故障、低电量”，LED3控制电池指示灯
    sros::device::LC100_ptr lc100_b_; // 电气图中右边的lc100，LED1控制左转向，LED2控制“是否有障碍”，LED3控制右转向灯
};

}


#endif //SROS_HUAWEI_COMMUNICATION_MODULE_H
