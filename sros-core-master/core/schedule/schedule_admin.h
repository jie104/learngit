//
// Created by huangwuxian on 20-3-9.
//

#ifndef SROS_SCHEDULEADMIN_H
#define SROS_SCHEDULEADMIN_H

#include <vector>

#include "../db/db.h"
#include "abstrace_schedule.h"
#include "timer_schedule.h"
#include "register_schedule.h"

namespace sros {
namespace core {

class ScheduleAdmin {
public:
    ~ScheduleAdmin();

    static ScheduleAdmin* getInstance();

    bool loadSchedules();
    std::vector<AbstractSchedulePtr> getAllSchedules() const { return schedules_; }

private:
    explicit ScheduleAdmin();

private:
    SQLite::Database *db_;
    std::vector<AbstractSchedulePtr> schedules_;
};

}
}




#endif //SROS_SCHEDULEADMIN_H
