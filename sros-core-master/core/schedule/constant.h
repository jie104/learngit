//
// Created by huangwuxian on 20-3-9.
//

#ifndef SROS_SCHEDULE_CONSTANT_H
#define SROS_SCHEDULE_CONSTANT_H

#include <string>

namespace sros {
namespace core {

using namespace std;

const int TIME_5_SECOND = 5 * 1000;
const string SCHEDULE_TABLE_NAME = "schedule";

enum class ScheduleType {
    SCHEDULE_NONE = 0,
    SCHEDULE_TIMER,
    SCHEDULE_MODBUS
};

enum class FinishCondition {
    NONE = 0,
    LIMIT_NUMBER = 10, // 不要改变枚举值,与前端对应
    STOP_TIME = 20,
    FOREVER = 30
};

}
}

#endif //SROS_SCHEDULE_CONSTANT_H
