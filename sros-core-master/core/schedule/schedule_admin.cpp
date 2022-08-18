//
// Created by huangwuxian on 20-3-9.
//

#include "schedule_admin.h"

namespace sros {
namespace core {

ScheduleAdmin::ScheduleAdmin() : db_(&g_db) {
    loadSchedules();
}

ScheduleAdmin::~ScheduleAdmin() {

}

ScheduleAdmin* ScheduleAdmin::getInstance() {
    static ScheduleAdmin admin;
    return &admin;
}

bool ScheduleAdmin::loadSchedules() {
    schedules_.clear();
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "SELECT * FROM " + SCHEDULE_TABLE_NAME + ";";
        LOG(INFO) <<  sql;

        SQLite::Statement query(*db_, sql);
        while (query.executeStep()) {
            auto schedule_type = (ScheduleType)query.getColumn(4).getUInt();
            AbstractSchedulePtr instance = nullptr;
            switch (schedule_type) {
                case ScheduleType::SCHEDULE_TIMER: {
                    instance = std::make_shared<TimerSchedule>();
                    break;
                }
                case ScheduleType::SCHEDULE_MODBUS: {
                    instance = std::make_shared<RegisterSchedule>();
                    break;
                }
                default: {
                    break;
                }
            }
            if (!instance) {
                continue;
            }

            if (!instance->fromQuery(query)) {
                LOG(ERROR) << "ScheduleAdmin::loadSchedules(): load schedule error";
            }
            schedules_.push_back(instance);
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
        return false;
    }

    return true;
}

}
}
