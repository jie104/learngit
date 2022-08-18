//
// Created by huangwuxian on 20-3-9.
//

#ifndef SCHEDULE_MODULE_H
#define SCHEDULE_MODULE_H

#include "core/core.h"
#include "core/schedule/schedule_admin.h"
#include "core/modbus/register_admin.h"

namespace sros {

using namespace core;
using namespace std;

class ScheduleModule : public sros::core::Module {
public:
    explicit ScheduleModule(const std::string &module_name = "ScheduleModule");
    ~ScheduleModule();

    virtual void run();

private:
    void onTimer5S(sros::core::base_msg_ptr m);
    void onUpdateSchedule(sros::core::base_msg_ptr m);

    /**
     * 处理定时任务
     * @param timer_schedule
     */
    void processTimerSchedule(TimerSchedulePtr timer_schedule);

    /**
     * 处理寄存器触发任务
     * @param register_schedule
     */
    void processRegisterSchedule(RegisterSchedulePtr register_schedule);

    void startTask(uint64_t task_id);

private:
    RegisterAdmin* register_admin_;
};

}

#endif //SCHEDULE_MODULE_H
