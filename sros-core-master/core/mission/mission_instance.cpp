
#include "mission_instance.h"
#include <glog/logging.h>

namespace sros {
namespace core {

MissionInstance::MissionInstance() {
    state_ = MissionState::NA;

    no_ = 0;

    isChildMission = false;

    seq_ = 0;
    session_id_ = 0;
    cur_step_id_ = "";
    avoid_policy_ = ObstacleAvoidPolicy::OBSTACLE_AVOID_WAIT;

    result_ = MissionResult::MISSION_RESULT_NA;
    err_code_ = 0;
    start_timestamp_ = 1;
    finish_timestamp_ = 1;
    total_cycle_time_ = 0;
    finish_cycle_time_ = 0;
    order_weight_ = 1024; // 默认值大，新增的mission排序时排在后面
}

MissionInstance::~MissionInstance() {
    for (uint32_t i = 0; i < step_cached_.size(); i++) {
        step_cached_[i] = nullptr;
    }
    step_cached_.clear();
}

uint64_t MissionInstance::getNo() const {
    return no_;
}

AbstractStepPtr MissionInstance::getMissionStep(const std::string &step_id) {
    for (uint32_t i = 0; i < step_cached_.size(); i++) {
        AbstractStepPtr step_ptr = step_cached_[i];
        if (!step_ptr) {
            continue;
        }
        if (step_ptr->getStepId() == step_id) {
            return step_ptr;
        }
    }

    AbstractStepPtr step = mission().getMissionStep(step_id);
    if (!step) {
        return step;
    }
    step_cached_.push_back(step);
    return step;
}

AbstractStepPtr MissionInstance::getNextMissionStep(bool decision) {
    return mission().getNextMissionStep(cur_step_id_, decision);
}

AbstractStepPtr MissionInstance::getCurrentMissionStep() {
    return getMissionStep(cur_step_id_);
}

bool MissionInstance::isRunning() const {
    return (state_ == MissionState::PAUSED || state_ == MissionState::RUNNING);
}

bool MissionInstance::traverseStep(vector<AbstractStepPtr> &child_mission_steps) {
    // 如果当前步骤未指定,直接返回
    if (cur_step_id_ == "") {
        return true;
    }
    return MissionInfo::traverseStep(mission().id_, cur_step_id_, child_mission_steps);
}

}
}
