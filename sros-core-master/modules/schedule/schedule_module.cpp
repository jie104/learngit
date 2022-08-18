//
// Created by huangwuxian on 20-3-9.
//

#include "schedule_module.h"
#include "core/util/time.h"
#include "core/msg/common_msg.hpp"
#include "core/msg/command_msg.hpp"
#include "core/logger.h"

namespace sros {

ScheduleModule::ScheduleModule(const std::string &module_name) : Module(module_name),
register_admin_(RegisterAdmin::getInstance()){

}

ScheduleModule::~ScheduleModule() {

}

void ScheduleModule::run() {
    LOGGER(INFO, SCHEDULE) << "ScheduleModule: run";

    subscribeTopic("TIMER_5S", CALLBACK(&ScheduleModule::onTimer5S));
    subscribeTopic("TOPIC_UPDATE_SCHEDULE", CALLBACK(&ScheduleModule::onUpdateSchedule));
    dispatch();
}

void ScheduleModule::onTimer5S(sros::core::base_msg_ptr m) {
    for (auto schedule : ScheduleAdmin::getInstance()->getAllSchedules()) {
        if (!schedule->isEnable()) {
            continue;
        }
        if (schedule->isTimerSchedule()) {
            TimerSchedulePtr timer_schedule = dynamic_pointer_cast<TimerSchedule>(schedule);
            if (!timer_schedule) {
                continue;
            }
            processTimerSchedule(timer_schedule);
        } else if (schedule->isModbusIOSchedule()) {
            RegisterSchedulePtr register_schedule = dynamic_pointer_cast<RegisterSchedule>(schedule);
            if (!register_schedule) {
                continue;
            }
            processRegisterSchedule(register_schedule);
        }
    }
}

void ScheduleModule::onUpdateSchedule(sros::core::base_msg_ptr m) {
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);
    LOGGER(INFO, SCHEDULE) << "ScheduleModule::onUpdateSchedule(): " << msg->str_0_;
    ScheduleAdmin::getInstance()->loadSchedules();
}

void ScheduleModule::processTimerSchedule(TimerSchedulePtr timer_schedule) {
    if (!timer_schedule) {
        return;
    }

    uint64_t cur_time = util::get_timestamp_in_ms();
    if (!timer_schedule->isTimeSatisfied(cur_time)) {
        return;
    }

    startTask(timer_schedule->getTaskID());
    timer_schedule->updateCurrentRepeateTime();
}

void ScheduleModule::processRegisterSchedule(RegisterSchedulePtr register_schedule) {
    if (!register_schedule) {
        return;
    }

    ModbusAddrType register_type = register_schedule->getRegisterType();
    uint16_t absolute_addr = register_admin_->absoluteAddr(register_schedule->getRegisterAddr(), register_type);

    uint32_t cur_reg_dat = 0;
    if (!register_admin_->doReadRegister(absolute_addr, cur_reg_dat, register_type)) {
        return;
    }

//    LOG(INFO) << "register: " << cur_reg_dat << "@predict: " << register_schedule->getRegisterValue();

    // 第一次读取到寄存器的值时,不进行判断,只有出现跳变才会触发任务执行
    if (register_schedule->getRegisterLastValue() < 0) {
        register_schedule->setRegisterLastValue(cur_reg_dat);
        return;
    }

    if (!register_schedule->isRegisterValueSatisfied(cur_reg_dat)) {
        return;
    }

    LOGGER(INFO, SCHEDULE) << "Register value match!!! register addr: " << absolute_addr <<
    " predict: " << register_schedule->getRegisterValue() << "current read value: " << cur_reg_dat;

    startTask(register_schedule->getTaskID());
}

void ScheduleModule::startTask(uint64_t task_id) {
    LOGGER(INFO, SCHEDULE) << "Schedule module start task, task id: " << task_id;

    auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
    d_msg->req_seq = 0;
    d_msg->session_id = 0;
    d_msg->command = sros::core::CMD_START_MISSION;
    d_msg->mission_no = sros::core::util::get_timestamp_in_ms();
    d_msg->mission_id = task_id;
    d_msg->mission_cur_step_id = "";
    d_msg->mission_avoid_policy = sros::core::ObstacleAvoidPolicy::OBSTACLE_AVOID_WAIT;
    d_msg->user_name = "";

    return sendMsg(d_msg);
}

}
