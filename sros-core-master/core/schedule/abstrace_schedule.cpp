//
// Created by huangwuxian on 20-3-9.
//

#include "abstrace_schedule.h"

namespace sros {
namespace core {

AbstractSchedule::AbstractSchedule(sros::core::ScheduleType eType) : type_(eType)
, uuid_(0)
, name_("")
, desc_("")
, timestamp_(0)
, is_enable_(false)
, task_id_(0) {

}

AbstractSchedule::~AbstractSchedule() {

}

bool AbstractSchedule::fromQuery(SQLite::Statement &query) {
    uuid_ = query.getColumn(0).getUInt64();
    name_ = query.getColumn(1).getString();
    desc_ = query.getColumn(2).getString();
    timestamp_ = query.getColumn(3).getUInt64();
    type_ = (ScheduleType)query.getColumn(4).getUInt();
    is_enable_ = (query.getColumn(5).getUInt() > 0);
    task_id_ = query.getColumn(6).getUInt64();

    std::string dat = query.getColumn(7).getString();
    return loadDetailsFromJson(nlohmann::json::parse(dat));
}

bool AbstractSchedule::loadDetailsFromJson(const nlohmann::json &dat) {
//    if (dat.is_null() || !dat.is_object()) {
//        LOG(ERROR) << "AbstractSchedule::fromJson(): Invalid json dat " << dat;
//        return false;
//    }
//
//    uuid_ = dat.at("id").get<uint64_t>();
//    name_ = dat.at("name").get<std::string>();
//    desc_ = dat.at("desc").get<std::string>();
//    type_ = (ScheduleType) dat.at("type").get<int>();
//    is_enable_ = (dat.at("is_enable").get<int>() > 0);
//    task_id_ = dat.at("task_id").get<uint64_t>();
//    timestamp_ = dat.at("timestamp").get<uint64_t>();
    return true;
}

}
}
