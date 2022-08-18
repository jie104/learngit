/*
    告警管理记录一些重要的故障信息，方便查看
*/
#ifndef ALARM_RECORD_
#define ALARM_RECORD_
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include "core/db/db.h"
#include "thirty-party/SQLiteCpp/include/SQLiteCpp/Database.h"
#include "thirty-party/SQLiteCpp/sqlite3/sqlite3.h"
#include "core/logger.h"
#include "core/task/action_task.h"
#include "core/task/movement_task.h"
#include "core/util/time.h"
#include "core/fault_code.h"

namespace sros {
namespace core {

using std::string;
using std::vector;
using std::to_string;
using std::dynamic_pointer_cast;

struct alarmInfo
{
    int level;
    int type;
    int error_code;
    int task_no = -1;
    std::string details;
    std::string describe;
    std::string how_to_fix;
    std::string occurr_time;
    std::string time_stamp_int;
};


class AlarmRecord{
public:
    static AlarmRecord &getInstance() {
        static AlarmRecord alarmRecord;
        return alarmRecord;
    }

    void addMovementTaskFailedAlarmInfo(const core::Task_ptr &task) {
        if(task == nullptr) {
            return;
        }
        core::MovementTask_ptr movement_task = dynamic_pointer_cast<core::MovementTask>(task);
        alarmInfo alarm_info;
        alarm_info.level = 3;
        alarm_info.type = 0; //0表示移动任务
        alarm_info.error_code = movement_task->getFailedCode();
        alarm_info.task_no = movement_task->getTaskNo();  
        std::vector<string>  item = getAlarmConfigInfos(error_table, alarm_info.error_code);
        if(item.size() == 5) {
            alarm_info.describe = item[2];
            alarm_info.how_to_fix = item[3];
        } else {
            alarm_info.describe = "其他原因导致任务失败";
            alarm_info.how_to_fix = "再试或联系厂商";
        }
        switch(movement_task->getMovementType()) {
            case core::MOVE_FOLLOW_PATH:
                alarm_info.details = "移动到路径";
                break;
            case core::MOVE_TO_STATION:
                alarm_info.details = "站点"
                        + to_string(movement_task->getCurStartStation())
                        + " ——> 站点"
                        + to_string(movement_task->getCurDstStation());
                break;
            case core::MOVE_TO_POSE:
                alarm_info.details = "移动到位姿";
                break;
            case core::MOVE_MIX2:
                alarm_info.details = "扫码移动";
                break;
            default:
                alarm_info.details = "移动类型未知";
                break;
        }
        insertAlarmInfoToTable(alarm_info);

    }

    void addActionTaskFailedAlarmInfo(const core::Task_ptr &task) {
        if(task == nullptr) {
            return;
        }
        core::ActionTask_ptr action_task = dynamic_pointer_cast<core::ActionTask>(task);

        alarmInfo alarm_info;
        alarm_info.level = 3;
        alarm_info.type = 1;
        alarm_info.error_code = action_task->getActionResultValue();
        alarm_info.task_no = action_task->getTaskNo();  
        alarm_info.details = "动作指令: (" + to_string(action_task->getActionID()) + "," 
                            + to_string(action_task->getActionParam()) + "," 
                            + to_string(action_task->getActionParam1()) + ")";
        vector<string> item = getAlarmConfigInfos(error_table, alarm_info.error_code);
        if(item.size() == 5) {
            alarm_info.describe = item[2];
            alarm_info.how_to_fix = item[3];
        } else {
            alarm_info.describe = "其他原因导致任务失败";
            alarm_info.how_to_fix = "再试或联系厂商";
        }
        insertAlarmInfoToTable(alarm_info);
        return;
    }

    void addEmergencyAlarmInfo(uint32_t emergency_source, string sourc_str) {
        int fault_code = getEmergencyFaultCode(emergency_source);
        if(fault_code == 0) {
            LOG(ERROR) << "emergency trigger but can't parse the source: " << emergency_source << "|" << sourc_str;
            return;
        }
        alarmInfo alarm_info;
        alarm_info.type = 3; //3表示急停触发
        alarm_info.details = sourc_str;
        alarm_info.error_code = fault_code;
        vector<string> item = getAlarmConfigInfos(fault_table, fault_code);
        if(item.size() == 9) {
            alarm_info.level = std::stoi(item[2]);
            alarm_info.describe = item[4];
            alarm_info.how_to_fix = item[5];
        } else {
            alarm_info.level = 3;
            alarm_info.describe = "其他原因导致急停故障";
            alarm_info.how_to_fix = "解除急停";
        }
        insertAlarmInfoToTable(alarm_info);
        return;
    }

    void addDivceFaultAlarmInfo(uint32_t fault_code) {
        alarmInfo alarm_info;
        auto device_list = sros::device::DeviceManager::getInstance()->getDeviceList();
        for(auto it : *device_list) {
            auto device_ptr = it.second;
            if(device_ptr->getUnicornFaultCode() == fault_code) {
                alarm_info.type = 2; //2表示设备错误
                alarm_info.error_code = fault_code;
                std::vector<string> item = getAlarmConfigInfos(fault_table, alarm_info.error_code);
                alarm_info.details = "设备名:" + device_ptr->getName();;
                if(item.size() == 9) {
                    alarm_info.level = std::stoi(item[2]);
                    alarm_info.describe = item[4];
                    alarm_info.how_to_fix = item[5];
                } else {
                    alarm_info.level = 3;
                    alarm_info.describe = "其他原因导致设备出错";
                    alarm_info.how_to_fix = "再试或联系厂商";
                } 
                insertAlarmInfoToTable(alarm_info);
                break;
            }
        }
        return;
    }

    void checkAlarmTableSize() {
        int table_size = 0;
        string sql = "select count(*) from error_log";
        try {
            std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
            SQLite::Statement query(g_db, sql);
            if(query.executeStep()) {
                int num = query.getColumnCount();
                if(num == 0) {
                    LOG(INFO) << "get table error_log size failed";
                    return;
                }
                table_size = query.getColumn(0).getInt();
            }
            if(table_size > max_table_size) {
                uint32_t delete_size = table_size - max_table_size / 2;
                sql = "delete from error_log where id in (select id from error_log order by id limit 0," + to_string(delete_size) + ")";
                db_ptr->exec(sql.c_str());
            }
        } catch (SQLite::Exception &e) {
            LOG(ERROR) << "check size error: " << e.what();
        }
    }

private:
    AlarmRecord(): db_ptr(&g_db){}


private:
    vector<string> getAlarmConfigInfos(string table_name, const uint32_t key) {
        std::vector<std::string> item;
        if(cache_.find(key) != cache_.end()) {
            return cache_[key];
        }
        LOG(INFO) << "cache_ miss in get key:" << key <<" from " << table_name;
        const std::string sql = "SELECT * from " + table_name + " where id=" + std::to_string(key);
        try {
            std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
            SQLite::Statement query(g_db, sql);
            if(query.executeStep()) {
                int num = query.getColumnCount();
                for(int i = 0; i < num; ++i) {
                    item.push_back(query.getColumn(i).getString());
                }
            }
        } catch (SQLite::Exception &e) {
            LOG(ERROR) << "read error: " << e.what();
            item.clear();
        }
        cache_[key] = item;
        return item;
    }

    void insertAlarmInfoToTable(const alarmInfo &alarm_info) {
        std::string task_no_value, task_no_data;
        if(alarm_info.task_no != not_task_flag) {
            task_no_data = " task_no,";
            task_no_value = std::to_string(alarm_info.task_no) + ",";
        }

        char time_format[80] = {0};
        struct tm nowTime;
        uint64_t time_ms = util::get_timestamp_in_ms();
        time_t time_s = time_ms / 1000;
        localtime_r(&time_s, &nowTime);
        strftime(time_format, sizeof(time_format), "%Y-%m-%d %H:%M:%S", &nowTime);

        std::string query = "insert into error_log (level, type, error_code," + task_no_data + " details, describe, how_to_fix, occurr_time, time_stamp_int) values(" + 
                             std::to_string(alarm_info.level) + "," + std::to_string(alarm_info.type) + "," + std::to_string(alarm_info.error_code) + "," + task_no_value
                             + "'" + alarm_info.details + "'" + "," 
                             + "'" + alarm_info.describe + "'" + "," 
                             + "'" + alarm_info.how_to_fix + "'" + "," 
                             + "'" + time_format + "'" + ","  // totest
                             + "'" + to_string(time_ms) + "'" + ")";
        try {
            std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
            db_ptr->exec(query.c_str());
        } catch (SQLite::Exception &e) {
            LOG(ERROR) << "insert error: " << query;
        }
        return;
    }


int getEmergencyFaultCode(const int emergency_source) {
    for(int i = 0; i < 20; ++i) {
        if(((1 << i) & emergency_source) != 0) {
            switch (1 << i)
            {
            case 0x1:
            case 0x2:
            case 0x4:
            case 0x8:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_BUTTON;
            case 0x10:
            case 0x20:
            case 0x40:
            case 0x80:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_EDGE;
            case 0x100:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_BATTERY_DOOR_OPEN;
            case 0x200:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_SRC_INTERNAL_FAULT;
            case 0x400:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_SRC_EXTERNAL_FAULT;
            case 0x1000:
            case 0x2000:
            case 0x4000:
            case 0x8000:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_SOFTWARE;
            case 0x10000:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_NXP_ESTOP;
            case 0x20000:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_ST_ESTOP;
            case 0x40000:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_TK1_ESTOP;
            case 0x80000:
                return core::FAULT_CODE_EMERGENCY_TRIGGER_ESTOP_OUT;
            default:
                break;
            }
        }
    }
    return 0;
}


private:
    std::unordered_map<int, std::vector<std::string>> cache_;
    SQLite::Database *db_ptr;
    const std::string error_table = "error_code";
    const std::string fault_table = "fault_code";
    const uint32_t max_table_size = 10000;
    int not_task_flag = -1;

};
}// end of core
}//end of sros

#endif
