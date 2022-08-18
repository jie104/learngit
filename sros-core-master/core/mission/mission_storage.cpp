//
// Created by john on 18-8-28.
//

#include "mission_storage.h"

#include "core/db/db.h"
#include "mission_info.h"

namespace sros {
namespace core {

MissionStorage::MissionStorage()
        : db_(&g_db) {

}

bool MissionStorage::getMission(int id, MissionInfo &info) {
    using nlohmann::json;

    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "SELECT * FROM " + MISSION_TABLE_NAME + " WHERE id = " + std::to_string(id) + ";";
        LOG(INFO) <<  sql;

        SQLite::Statement query(*db_, sql);
        if (query.executeStep()) {
            info.id_ = query.getColumn(0).getInt();
            info.name_ = query.getColumn(1).getString();
            info.create_time_ = query.getColumn(2).getInt();
            info.last_modified_time_ = query.getColumn(3).getInt();
            info.body_ = json::parse(query.getColumn(4).getString());
            info.map_name_ = query.getColumn(7).getString();
        } else {
            return false;
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
        return false;
    }

    return true;
}

void MissionStorage::createMission(uint32_t mission_id, uint64_t mission_no, std::string user_name, std::string cur_step_id,
        MissionState state, MissionResult result,
        uint64_t error_code, uint64_t start_time, uint64_t end_time) {
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "INSERT INTO " + MISSION_RECORD_TABLE_NAME +
                " (mission_id, mission_no, user_name, cur_step_id, state, result, error_code, start_time, end_time) VALUES (" +
                std::to_string(mission_id) + ", " + std::to_string(mission_no) + ", '" + user_name  + "', '" +
                cur_step_id + "', " + std::to_string((int) state) + ", " + std::to_string(int(result)) + ", " +
                std::to_string(error_code) + ", " + std::to_string(start_time) + ", " +
                std::to_string(end_time) + ");";
        LOG(INFO) <<  sql;
        db_->exec(sql);
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
    }
}

void MissionStorage::updateMissionState(uint64_t mission_no, MissionState state) {
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "UPDATE " + MISSION_RECORD_TABLE_NAME + " SET state=" + std::to_string(int(state)) +
                          " WHERE mission_no=" + std::to_string(mission_no) + ";";
        if (state == MissionState::FINISHED) {
            sql = "UPDATE " + MISSION_RECORD_TABLE_NAME + " SET state=" + std::to_string(int(state)) +
                        ", end_time=" +
                        std::to_string(sros::core::util::get_timestamp_in_ms()) +
                        " WHERE mission_no=" + std::to_string(mission_no) +";";
        }
        LOG(INFO) <<  sql;
        db_->exec(sql);
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
    }
}

void MissionStorage::updateMissionCurrentStep(uint64_t mission_no, const std::string &cur_step_id) {
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "UPDATE " + MISSION_RECORD_TABLE_NAME + " SET cur_step_id='" + cur_step_id + "'" +
                          " WHERE mission_no=" + std::to_string(mission_no) +";";

        LOG(INFO) <<  sql;
        db_->exec(sql);
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
    }
}

void MissionStorage::setMissionResult(uint64_t mission_no, MissionResult result, uint64_t error_code) {
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "UPDATE " + MISSION_RECORD_TABLE_NAME + " SET result=" +
              std::to_string((int)result) + ", error_code=" + std::to_string(error_code) +
              " WHERE mission_no=" + std::to_string(mission_no) +";";

        LOG(INFO) <<  sql;
        db_->exec(sql);
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
    }
}

list<MissionInstancePtr> MissionStorage::getFinishTasks() {
    list<MissionInstancePtr> mission_list;
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "SELECT * FROM " + MISSION_RECORD_TABLE_NAME +
                          " WHERE state=" + std::to_string((int)MissionState::FINISHED) + " ORDER BY id DESC LIMIT 10;";

        LOG(INFO) <<  sql;

        SQLite::Statement query(*db_, sql);
        while (query.executeStep()) {
            auto instance = missionInstanceFromQuery(query);
            if (!instance) {
                continue;
            }
            mission_list.push_front(instance);
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
    }

    LOG(INFO) << "Load finish task from database " << mission_list.size();
    return mission_list;
}

list<MissionInstancePtr> MissionStorage::getUnfinishedTasks() {
    list<MissionInstancePtr> mission_list;
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "SELECT * FROM " + MISSION_RECORD_TABLE_NAME +
                          " WHERE state!=" + std::to_string((int)MissionState::FINISHED) + ";";

        LOG(INFO) <<  sql;

        SQLite::Statement query(*db_, sql);
        while (query.executeStep()) {
            auto instance = missionInstanceFromQuery(query);
            if (!instance) {
                continue;
            }
            mission_list.push_back(instance);
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
    }

    LOG(INFO) << "Load unfinished task from database " << mission_list.size();
    return mission_list;
}

MissionInstancePtr MissionStorage::missionInstanceFromQuery(SQLite::Statement &query) {
    uint32_t mission_id = query.getColumn(1).getUInt64();
    auto mission_instance = make_shared<MissionInstance>();
    if (!getMission(mission_id, mission_instance->mission_info_)) {
        LOG(ERROR) << "Mission id not exist " << mission_id;
        return nullptr;
    }

    mission_instance->no_ = query.getColumn(2).getUInt64();
    mission_instance->user_name_ = query.getColumn(3).getString();
    mission_instance->cur_step_id_ = query.getColumn(4).getString();
    mission_instance->state_ = (MissionState)(query.getColumn(5).getInt());
    mission_instance->result_ = (MissionResult)(query.getColumn(6).getInt());
    mission_instance->err_code_ = query.getColumn(7).getUInt64();
    mission_instance->start_timestamp_ = query.getColumn(8).getUInt64();
    mission_instance->finish_timestamp_ = query.getColumn(9).getUInt64();

    mission_instance->seq_ = 0;
    mission_instance->session_id_ = 0;

    return mission_instance;
}

}
}

