//
// Created by john on 18-8-28.
//

#ifndef SROS_MISSION_MANAGER_H
#define SROS_MISSION_MANAGER_H

#include <atomic>
#include <vector>
#include <boost/thread/thread.hpp>

#include "../util/queue.h"
#include "mission_storage.h"
#include "mission_instance.h"

#define FINISHED_MISSION_MAX_SIZE 10

namespace sros {
namespace core {
typedef std::shared_ptr<boost::thread> boost_thread_ptr;

extern bool compareMission(MissionInstancePtr p1, MissionInstancePtr p2);

class MissionManager {
public:
    ~MissionManager();

    static MissionManager* getInstance();

    bool loadMissionFromStorage(uint32_t mission_id, MissionInfo &mission_info);

    // 判断任务是否正在执行，任务在队列中时也认为任务已经在执行
    bool isMissionRunning();

    bool enqueueMission(MissionInstancePtr mission);
    MissionInstancePtr dequeueMission();

    bool reorderMissionList(std::list<uint64_t > &mission_no_lst);

    bool hasPendingMission(uint64_t mission_no);
    MissionInstancePtr removeMission(uint64_t mission_no);

    void addFinishedMission(MissionInstancePtr mission);

    uint32_t pendingMissionSize() { return pending_missions_.size(); }
    inline bool hasPendingMissions() { return pendingMissionSize() > 0; }
    std::list<MissionInstancePtr> allMissions();

    // 获取当前正在执行的任务
//    uint32_t currentMissionId();

    // 提前获取下一步子任务
    AbstractStepPtr getMissionNextStepInfo ();

    std::string getMissionName (int mission_id);

    // 启动子任务时，将父任务保存到栈中
    inline bool hasParentMission() const { return !(parent_missions_.empty()); }
    void pushParentMission(MissionInstancePtr mission_ptr);
    void popParentMission();
    void setToRootMission();
    void clearParentMissions();
    MissionInstancePtr getRootMission() const;

    uint32_t getCurrentMissionId(); // 获取当前正在执行的mission(可能是子任务)
    uint32_t getCurrentRootMissionId(); // 获取当前正在执行的主任务id
    uint64_t getCurrentRunningMissionSessionId();
    uint64_t getCurrentRunningMissionUuid();  // 获取当前任务uuid(no)

    MissionState getCurrentMissionState(); // 获取当前任务状态
    MissionResult getCurrentMissionResult(); // 当前任务执行结果
    AbstractStepPtr getCurrentStepInfo();  // 获取当前正在执行的任务步骤id

    void setCurrentMission(MissionInstancePtr mission_ptr) { current_running_mission_ = mission_ptr; }
    const MissionInstancePtr getCurrentRunningMission() const { return current_running_mission_; }
    const vector<MissionInstancePtr> &getParentMissionList() const { return parent_missions_; }
private:
    MissionManager();
    MissionInstancePtr getMissionInstance(uint64_t mission_no);

private:
    uint32_t mission_id; // 启动时从数据库读取到的最近运行的mission
    MissionStorage storage_;
    MissionInstancePtr current_running_mission_; // 当前正在执行的mission

    SDQueue<MissionInstancePtr> pending_missions_;  // 队列中等待执行的mission
    SDQueue<MissionInstancePtr> finished_missions_; // 已经执行完成的mission

    // 保存所有父mission
    // 每当当前mission启动一个子mission时，将
    // 当前mission入栈；当子任务执行结束时，如果栈
    // 不为空，则从栈中弹出mission继续执行
    vector<MissionInstancePtr> parent_missions_;
};

}
}

#endif //SROS_MISSION_MANAGER_H
