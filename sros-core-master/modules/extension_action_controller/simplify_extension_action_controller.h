//
// Created by caoyan on 6/22/21.
//

#ifndef SROS_SIMPLIFY_EXTENSION_ACTION_CONTROLLER_H
#define SROS_SIMPLIFY_EXTENSION_ACTION_CONTROLLER_H

#include "core/core.h"

#include <memory>
#include <queue>

#include "core/device/device.h"
#include "core/device/virtual_device.hpp"
#include "simplify_modbus_action_controller.h"
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

class SimplifyExtensionActionController : public core::Module {
 public:
    SimplifyExtensionActionController() : Module("eac"), driver_(30300, 40300, 30350, 40350) {}

    virtual ~SimplifyExtensionActionController() {}

    virtual void run() override;

 private:
    //监听动作任务
    void onDebugCmdMsg(core::base_msg_ptr msg);
    void onActionCmdMsg(core::base_msg_ptr msg);
    void onEacActionCmdMsg(core::base_msg_ptr msg);
    void onEmergencyStateMsg(core::base_msg_ptr msg);

    //定时任务
    void onTimer_50ms(sros::core::base_msg_ptr msg);
    void onTimer_1s(sros::core::base_msg_ptr msg);

    //状态更新
    void updateHeatBeat();
    void updateLoadState();

    void updateEacActionTaskState();
    void updateSrosActionTaskState();

    void monitorControlState();

    void onSRCActionState(sros::core::TaskStateStruct src_action_state);

    //回复
    void responseCommand(const sros::core::CommandMsg_ptr &msg);
    void responseEacActionResult(int action_no, sros::core::TaskResult act_task_state, int result_code);

    // 回复EAC结果状态
    void responseEacActionResult(const sros::modbus::ActionStatus& _status);

 private:
    SimplifyModbusActionController driver_;                                                // 动作控制器驱动

    sros::modbus::ModbusSynchronizer synchronizer_;

    bool seacp_enable_extend_ = false;
    bool eac_disable_heat_beat_ = false;  // 部分动作控制器无法提供心跳，心跳用能正常读写寄存器替代
    int eac_heat_beat_timeout_ = 0;

    int eac_register_state_keep_time_ = 0;

    int action_execute_time_ms_ = 0;  // 程序已经开始了多久了
    int action_finished_time_ms_ = -1;  // 程序已经结束多久了, 开始时为-1，表示还没有结束，当不为-1是表示已经开始结束了

    sros::device::VirtualDevice_ptr eac_device_;

    sros::core::ActionTask_ptr sros_sub_task_;  // sros子任务,多个线程访问，注意要加原子操作

    sros::core::ActionTask_ptr cur_eac_task_ = nullptr;  //当前运行的eac任务
};

}  // namespace eac
}  // namespace sros

#endif  // SROS_SIMPLIFY_EXTENSION_ACTION_CONTROLLER_H
