//
// Created by huangwuxian on 20-3-9.
//

#include "timer_schedule.h"
#include "core/util/time.h"

namespace sros {
namespace core {

TimerSchedule::TimerSchedule() : AbstractSchedule(ScheduleType::SCHEDULE_TIMER)
, finish_condition_(FinishCondition::NONE)
, is_repeat_(false)
, start_time_(0)
, end_time_(0)
, repeat_number_(0)
, cur_execute_time_(0)
, interval_(0){

}

TimerSchedule::~TimerSchedule() {

}

bool TimerSchedule::loadDetailsFromJson(const nlohmann::json &dat) {
    if (dat.is_null() || !dat.is_object()) {
        LOG(ERROR) << "TimerSchedule::loadDetailsFromJson(): Invalid dat info";
        return false;
    }

    finish_condition_ = (FinishCondition) dat.at("finishCondition").get<int>();
    is_repeat_ = (dat.at("isRepeated").get<int>() > 0);
    start_time_ = dat.at("startTime").get<uint64_t>();
    end_time_ = dat.at("endTime").get<uint64_t>();
    interval_ = dat.at("interval").get<uint64_t>();
    repeat_number_ = dat.at("repeatNumber").get<uint32_t>();
    return true;
}

bool TimerSchedule::isTimeSatisfied(uint64_t cur_time) {
    uint64_t start_time = getStartTime();
    uint64_t end_time = getEndTime();
    if (!isRepeated()) {
        int64_t diff = cur_time - start_time;
        return (diff >= 0 && diff < TIME_5_SECOND);
    }

    switch (finish_condition_) {
        case FinishCondition::STOP_TIME: {
            // 由于获取的当前时间戳不一定为5s整数倍,因此需要处理最后时间点的问题,例如
            // 从9:00开始每隔10分钟执行一次,截止时间为9:30,时间戳可能是9:29:28 -> 9:30:03
            // 此时在9:30:03应该需要执行任务
            if (cur_time < start_time || (cur_time > end_time && (cur_time - end_time) >= TIME_5_SECOND)) {
                return false;
            }
            break;
        }
        case FinishCondition::FOREVER: {
            if (cur_time < start_time) {
                return false;
            }
            break;
        }
        case FinishCondition::LIMIT_NUMBER: {
            if (cur_time < start_time || getCurrentRepeatTime() >= getRepeatNumber()) {
                return false;
            }
            break;
        }
        default: {
            return false;
        }
    }
    int64_t diff = cur_time - start_time;
    return (diff % getInterval()) < TIME_5_SECOND;
}

uint32_t TimerSchedule::getDaytimeInMs(uint64_t timestamp) {
    time_t time_second = (time_t) (round(timestamp / 1000));
    tm *ptmTime = localtime(&time_second);
    return (ptmTime->tm_hour * 3600 + ptmTime->tm_min * 60 + ptmTime->tm_sec) * 1000;
}

}
}
