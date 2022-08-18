//
// Created by huangwuxian on 20-3-9.
//

#ifndef SROS_TIMER_SCHEDULE_H
#define SROS_TIMER_SCHEDULE_H

#include "abstrace_schedule.h"

namespace sros {
namespace core {

class TimerSchedule : public AbstractSchedule {
public:
    explicit TimerSchedule();
    virtual ~TimerSchedule();

    inline bool isRepeated() const { return  is_repeat_; }
    inline bool isStopByFinishTime() const { return finish_condition_ == FinishCondition::STOP_TIME; }
    inline bool isStopByLimitRepeatNumber() const { return finish_condition_ == FinishCondition::LIMIT_NUMBER; }
    inline bool isRunForever() const { return finish_condition_ == FinishCondition::FOREVER; }

    inline uint64_t getStartTime() const { return start_time_; }
    inline uint64_t getEndTime() const { return end_time_; }

    inline uint64_t getRepeatNumber() const { return repeat_number_; }
    inline uint64_t getCurrentRepeatTime() const { return cur_execute_time_; }
    inline void updateCurrentRepeateTime() { cur_execute_time_++; }

    inline uint64_t getInterval() const { return interval_; }

    bool isTimeSatisfied(uint64_t cur_time);

protected:
    virtual bool loadDetailsFromJson(const nlohmann::json &dat) override;

private:
    /**
     * 根据时间戳获取时间(不含年月日),并以ms为单位返回
     * @param timestamp
     * @return
     */
    uint32_t getDaytimeInMs(uint64_t timestamp);

private:
    uint64_t start_time_; // 开始时间,单位ms
    uint64_t interval_; // 触发的时间间隔,单位ms
    bool is_repeat_; // 是否每日执行

    FinishCondition finish_condition_; // 重复任务结束方式

    // 指定重复次数
    uint64_t repeat_number_;
    uint64_t cur_execute_time_; // 记录当前已经重复执行的次数

    // 指定结束时间的时间戳
    uint64_t end_time_;

};

typedef std::shared_ptr<TimerSchedule> TimerSchedulePtr;
}
}


#endif //SROS_TIMER_SCHEDULE_H
