//
// Created by huangwuxian on 20-3-9.
//

#ifndef SROS_ABSTRACE_SCHEDULE_H
#define SROS_ABSTRACE_SCHEDULE_H

#include <thirty-party/SQLiteCpp/include/SQLiteCpp/SQLiteCpp.h>
#include <glog/logging.h>

#include "constant.h"
#include "core/util/json.h"

namespace sros {
namespace core {

class AbstractSchedule {
public:
    explicit AbstractSchedule(ScheduleType eType);
    virtual ~AbstractSchedule();

    inline bool isTimerSchedule () const { return type_ == ScheduleType::SCHEDULE_TIMER; }
    inline bool isModbusIOSchedule () const { return type_ == ScheduleType::SCHEDULE_MODBUS; }

    bool fromQuery(SQLite::Statement &query);

    inline uint64_t getUUID() const { return uuid_; }
    inline uint64_t getTaskID() const { return task_id_; }
    inline uint64_t getTimestamp() const { return timestamp_; }
    inline ScheduleType scheduleType() const { return type_; }

    inline bool isEnable() const { return  is_enable_; }

protected:
    /**
     * 从json中解析业务逻辑数据,子类重写该实现
     * @param dat
     * @return
     */
    virtual bool loadDetailsFromJson(const nlohmann::json &dat);

private:
    uint64_t uuid_; // 任务id
    std::string name_; // 任务名称
    std::string desc_; // 描述信息

    bool is_enable_; // 是否使能

    uint64_t task_id_; // 触发的任务id

    ScheduleType type_; // 计划任务类型,包括定时触发任务和IO(modbus触发任务)
    uint64_t timestamp_; // 任务修改的时间戳,会影响当日定时任务

};

typedef std::shared_ptr<AbstractSchedule> AbstractSchedulePtr;
}
}





#endif //SROS_ABSTRACE_SCHEDULE_H
