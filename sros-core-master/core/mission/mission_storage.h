//
// Created by john on 18-8-28.
//

#ifndef SROS_MISSION_STORAGE_H
#define SROS_MISSION_STORAGE_H

#include <string>
#include <vector>

#include <thirty-party/SQLiteCpp/include/SQLiteCpp/Database.h>
#include "core/util/time.h"
#include "mission_info.h"
#include "mission_instance.h"

namespace sros {
namespace core {

using MissionInfoList = std::vector<MissionInfo>;
using MissionInfosCache_t = std::map<std::string, MissionInfo>;

class MissionStorage {
public:
    MissionStorage();

    bool getMission(int id, MissionInfo &info);

    list<MissionInstancePtr> getFinishTasks();
    list<MissionInstancePtr> getUnfinishedTasks();

    // NOTE: missions's add/remove/change transmission from the http server

    // mission历史记录
    void createMission(uint32_t mission_id, uint64_t mission_no, std::string user_name, std::string cur_step_id,
                       MissionState state = MissionState::PENDING, MissionResult result = MissionResult::MISSION_RESULT_NA,
                       uint64_t error_code = 0, uint64_t start_time = sros::core::util::get_timestamp_in_ms(), uint64_t end_time = 0);
    void updateMissionState(uint64_t mission_no, MissionState state);
    void updateMissionCurrentStep (uint64_t mission_no, const std::string &cur_step_id);
    void setMissionResult (uint64_t mission_no, MissionResult result, uint64_t error_code);

private:
    MissionInstancePtr missionInstanceFromQuery(SQLite::Statement &query);

private:

    SQLite::Database *db_;

    const std::string MISSION_TABLE_NAME = "mission";
    const std::string MISSION_RECORD_TABLE_NAME = "mission_record";
};

}
}


#endif //SROS_MISSION_STORAGE_H
