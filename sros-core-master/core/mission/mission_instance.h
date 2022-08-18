#ifndef SROS_MISSION_INSTANCE_H
#define SROS_MISSION_INSTANCE_H

#include "core/state.h"

#include "mission_info.h"
#include "../map/NavigationMap.hpp"

namespace sros {
namespace core {

// 任务实例,每次启动任务时创建,包含了mission启动时的运行信息
class MissionInstance {
public:
    MissionInstance();
    ~MissionInstance();

    inline uint32_t getId() { return mission().id_; }
    inline string getName() { return mission().name_; }

    inline std::string getStartStepId () { return mission().getStartStepId(); }
    inline std::string getNextStepId (const string &cur_step_id, bool decision = true) { return mission_info_.getNextStepId(cur_step_id, decision); }

    inline std::string currentStepId() const { return cur_step_id_; }
    inline void setCurStepId(const std::string &stepId) { cur_step_id_ = stepId; }

    AbstractStepPtr getMissionStep(const std::string &step_id);
    AbstractStepPtr getCurrentMissionStep();
    AbstractStepPtr getNextMissionStep(bool decision = true);

    /**
     * 获取指定任务步骤的所经过的子任务(任务步骤)
     * 递归查询指定任务步骤的层次,例如
     * 主任务a包含子任务步骤b,子任务b包含子任务步骤c,子任务c包含步骤X
     * 则传入步骤X的id返回结果:
     * [子任务步骤b, 子任务步骤c]
     * 如果直接在主任务中查询到,返回list size = 0
     * @param step_id
     * @param child_mission_steps
     * @return
     * TODO一个子任务在主任务中出现多次时,由于任务步骤id相同,该函数未区分是哪一个子任务中的步骤,可能返回的是非期望的层次
     */
    bool traverseStep(vector<AbstractStepPtr> &child_mission_steps);

    bool isRunning() const;

    // 获取当前mission唯一id
    uint64_t getNo() const;
private:
    MissionInfo mission() { return mission_info_; }

    // 获取当前正在执行step的下一个step

public:
    uint64_t no_;  // 执行任务时给mission的编号（时间戳），同一个mission多次执行no不同，可以唯一标识运行的mission

    uint32_t seq_; // 通信seq
    uint64_t session_id_; // 会话id

    std::string user_name_; // 启动任务的用户名称

    std::string cur_step_id_; // 当前mission运行的step, 出错时，记录当前错误步骤

    ObstacleAvoidPolicy avoid_policy_; // mission避障策略

    MissionState state_; // mission运行状态

    MissionResult result_;  // mission运行结果
    uint32_t err_code_; // 出错时错误码

    uint64_t start_timestamp_; // mission启动时间戳，包括在队列中等待的时间
    uint64_t finish_timestamp_; // mission结束时间戳

    uint32_t total_cycle_time_; // 循环任务的总循环次数
    uint32_t finish_cycle_time_; // 已经完成循环的次数

    MissionInfo mission_info_; // 任务信息
    uint32_t order_weight_; // 排序权重

    bool isChildMission; // 指示该任务是否由其他mission发起

    // 缓存任务步骤，一方面解决在一个mission中多次请求同一个步骤的问题，更重要的是
    // 解决记录循环任务中当前步骤已经完成的次数问题
    std::vector<AbstractStepPtr> step_cached_;
};

typedef std::shared_ptr<MissionInstance> MissionInstancePtr;

}
}

#endif // SROS_MISSION_INSTANCE_H
