//
// Created by john on 19-5-14.
//

#ifndef SROS_EXTENSION_ACTION_CONTROLLER_H
#define SROS_EXTENSION_ACTION_CONTROLLER_H

#include "core/core.h"

#include <memory>
#include <queue>

#include "core/device/device.h"
#include "core/device/virtual_device.hpp"
#include "core/modbus/modbus_action_controller.h"
#include "core/modbus/modbus_synchronizer.h"
#include "core/msg/command_msg.hpp"
#include "core/task/action_task.h"
#include "core/task/task_manager.h"
#include "core/task/task.h"
#include "core/util/async_condition_variable.hpp"
#include "src/sdk/protocol/src_protocol.h"
#include "core/modbus/modbus_action_controller.h"

namespace sros {
namespace eac {

enum ErrorCode {
    ERROR_CODE_NONE,
    ERROR_CODE_RESPONSE_TIMEOUT,   // 回复超时
    ERROR_CODE_RESPONSE_INORRECT,  // 回复错误
};

constexpr int DELAY_DETECTION_TIME = 2 * 1000;  // 延迟检测时间

class ExtensionActionController : public core::Module {
 public:
    ExtensionActionController() : Module("eac"), driver_(30300, 40300) {}

    virtual ~ExtensionActionController() {}

    virtual void run() override;

 private:
    void onDebugCmdMsg(core::base_msg_ptr msg);

    void onActionCmdMsg(core::base_msg_ptr msg);
    void onEacActionCmdMsg(core::base_msg_ptr msg);
    void onEmergencyStateMsg(core::base_msg_ptr msg);

    void onTimer_100ms(sros::core::base_msg_ptr msg);

    void onTimer_1s(sros::core::base_msg_ptr msg);

    void updateActionTaskState();

    /**
     * 更新Sros子任务状态
     */
    void updateSrosTaskState();
    void onSRCActionState(sros::core::TaskStateStruct src_action_state);

    void responseCommand(const sros::core::CommandMsg_ptr &msg);
    void responseEacActionResult(int action_no, sros::core::TaskResult act_task_state, int result_code);
    
    // 回复EAC结果状态
    void responseEacActionResult(const sros::modbus::ActionStatus& _status);

    void updateMusic(int eac_type);

    sros::modbus::SystemState eac_device_system_state_ = sros::modbus::SYSTEM_STATE_NONE;  // eac设备的系统状态
    sros::modbus::ModbusActionController driver_;                                          // 动作控制器驱动

    modbus::ModbusSynchronizer synchronizer_;
    bool eac_disable_heat_beat_ = false;  // 部分动作控制器无法提供心跳，心跳用能正常读写寄存器替代

    bool strict_model_ = true;  // 是否是严格模式,非严格模式下解决客户常见的问题，不符合协议最开始的设定
    int action_execute_time_ms_ = 0;  // 程序已经开始了多久了
    int action_finished_time_ms_ = -1;  // 程序已经结束多久了, 开始时为-1，表示还没有结束，当不为-1是表示已经开始结束了

    sros::device::VirtualDevice_ptr eac_device_;

    sros::core::ActionTask_ptr sros_sub_task_;  // sros子任务,多个线程访问，注意要加原子操作

    sros::core::ActionTask_ptr cur_eac_task_ = nullptr;  //当前运行的eac任务
};

}  // namespace eac
}  // namespace sros

#endif  // SROS_EXTENSION_ACTION_CONTROLLER_H
