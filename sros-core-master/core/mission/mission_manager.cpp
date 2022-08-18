//
// Created by john on 18-8-28.
//

#include "mission_manager.h"
#include <glog/logging.h>
#include "../state.h"

using namespace std;

namespace sros {
namespace core {

bool compareMission(MissionInstancePtr p1, MissionInstancePtr p2) {
    return p1->order_weight_ < p2->order_weight_;
}

MissionManager::MissionManager() : finished_missions_(FINISHED_MISSION_MAX_SIZE),
current_running_mission_(nullptr)
{
    mission_id = 0;
    for (auto instance : storage_.getFinishTasks()) {
        finished_missions_.push(instance);
    }

    for (auto instance : storage_.getUnfinishedTasks()) {
        // 未完成的任务都需要设置成pending状态,需要重新启动执行
        instance->state_ = MissionState ::PENDING;
        pending_missions_.push(instance);
    }
}

MissionManager::~MissionManager() {
    clearParentMissions();
}

MissionManager *MissionManager::getInstance() {
    static MissionManager manager;
    return &manager;
}

void MissionManager::addFinishedMission(MissionInstancePtr mission) {
    if (!mission) {
        return;
    }

    if (finished_missions_.size() >= FINISHED_MISSION_MAX_SIZE) {
        finished_missions_.pop();
    }

    finished_missions_.push(mission);
}

std::list<MissionInstancePtr> MissionManager::allMissions() {
    std::list<MissionInstancePtr> lst;
    // 注意顺序，先是已完成任务，然后是正在执行任务，最后是待执行任务
    finished_missions_.all_items(lst);
    if (current_running_mission_ && (current_running_mission_->state_ == MissionState::RUNNING ||
        current_running_mission_->state_ == MissionState::PAUSED)) {
        if (!current_running_mission_->isChildMission) {
            lst.push_back(current_running_mission_);
        } else {
            auto root_mission = getRootMission();
            if (!root_mission) {
                LOG(ERROR) << "MissionManager::allMissions(): Null root mission";
            } else {
                lst.push_back(root_mission);
            }
        }
    }

    pending_missions_.all_items(lst);
    LOG(INFO) << "MissionManager::allMissions(): " << lst.size();
    return lst;
}



MissionInstancePtr MissionManager::getMissionInstance(uint64_t mission_no) {
    std::list<MissionInstancePtr> missions;
    pending_missions_.all_items(missions);

    for (auto instance : missions) {
        if (instance->getNo() == mission_no) {
            return instance;
        }
    }
    return nullptr;
}

bool MissionManager::enqueueMission(MissionInstancePtr mission) {
    return pending_missions_.push(mission);
}

MissionInstancePtr MissionManager::dequeueMission() {
    current_running_mission_ = pending_missions_.pop();
    return current_running_mission_;
}

bool MissionManager::hasPendingMission(uint64_t mission_no) {
    return pending_missions_.has_item(mission_no);
}

bool MissionManager::reorderMissionList(std::list<uint64_t > &mission_no_lst) {

    // 设置index作为排序的权重值
    uint32_t order = 1;
    for (auto mission_no : mission_no_lst) {
        auto mission = getMissionInstance(mission_no);
        if (!mission) {
            continue;
        }
        mission->order_weight_ = order;
        order++;
    }

    pending_missions_.sort(compareMission);
    return true;
}

MissionInstancePtr MissionManager::removeMission(uint64_t mission_no) {
    return pending_missions_.remove_item(mission_no);
}

uint32_t MissionManager::getCurrentMissionId() {
    if (current_running_mission_) {
        return current_running_mission_->getId();
    }
    return mission_id;
}

uint32_t MissionManager::getCurrentRootMissionId() {
    auto root_mission = getRootMission();
    if (!root_mission) {
        return 0;
    }
    return root_mission->getId();
}

AbstractStepPtr MissionManager::getCurrentStepInfo() {
    auto current_mission = getCurrentRunningMission();
    if (!current_mission) {
        return nullptr;
    }
    return current_mission->getCurrentMissionStep();
}

AbstractStepPtr MissionManager::getMissionNextStepInfo() {
    auto current_mission = getCurrentRunningMission();
    if (current_mission == nullptr || !isMissionRunning()) {
        return nullptr;
    }

    return current_mission->getNextMissionStep(true);
}
void MissionManager::popParentMission() {
    current_running_mission_ = parent_missions_.back();
    parent_missions_.pop_back();
}

void MissionManager::pushParentMission(MissionInstancePtr mission_ptr) {
    parent_missions_.push_back(mission_ptr);
}

void MissionManager::setToRootMission() {
    // 获取最开始启动的mission
    while (hasParentMission()) {
        popParentMission();
    }
}

void MissionManager::clearParentMissions() {
    parent_missions_.clear();
}

MissionInstancePtr MissionManager::getRootMission() const {
    if (!hasParentMission()) {
        return current_running_mission_;
    }
    return parent_missions_.front();
}

bool MissionManager::isMissionRunning() {
//    if (hasPendingMissions()) {
//        return true;
//    }

    if (hasParentMission()) {
        return true;
    }

    MissionInstancePtr cur_mission = getCurrentRunningMission();
    if (!cur_mission) {
        return false;
    }
    return cur_mission->isRunning();
}

uint64_t MissionManager::getCurrentRunningMissionSessionId() {
    auto currentRunningMission = getCurrentRunningMission();
    if (!currentRunningMission) {
        return 0;
    }
    return currentRunningMission->session_id_;
}

MissionState MissionManager::getCurrentMissionState() {
    MissionState state = MissionState::NA;
    auto current_running_mission = getCurrentRunningMission();
    if (!current_running_mission) {
        if (hasPendingMissions()) {
            state = MissionState::PENDING;
        }
        return state;
    }
    return current_running_mission->state_;
}

MissionResult MissionManager::getCurrentMissionResult() {
    auto current_running_mission = getCurrentRunningMission();
    if (!current_running_mission) {
        return MissionResult::MISSION_RESULT_NA;
    }

    if (current_running_mission->state_ != MissionState::FINISHED) {
        return MissionResult::MISSION_RESULT_NA;
    }

    return current_running_mission->result_;
}

std::string MissionManager::getMissionName(int mission_id) {
    MissionInfo mission_info;
    if (!loadMissionFromStorage(mission_id, mission_info)) {
        return "";
    }

    return mission_info.name_;
}

uint64_t MissionManager::getCurrentRunningMissionUuid() {
    auto current_running_mission = getCurrentRunningMission();
    if (!current_running_mission) {
        return 0;
    }
    return current_running_mission->no_;
}

bool MissionManager::loadMissionFromStorage(uint32_t mission_id, MissionInfo &mission_info) {
    return storage_.getMission(mission_id, mission_info);
}

}
}
