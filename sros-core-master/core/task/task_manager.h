//
// Created by lhx on 17-1-12.
//

#ifndef SROS_TASK_MANAGER_H
#define SROS_TASK_MANAGER_H

#include "action_task.h"
#include "movement_task.h"

namespace sros {
namespace core {

class TaskManager {
 public:
    static TaskManager* getInstance();

    void updateMovementTask(MovementTask_ptr task);

    MovementTask_ptr getMovementTask() const;

    void updateActionTask(ActionTask_ptr task);

    ActionTask_ptr getActionTask() const;

    TaskNo_t getMovementTaskNo() const;
    TaskNo_t getActionTaskNo() const;

    bool isMovementTaskRunning() const;
    bool isActionTaskRunning() const;
    bool isActionTaskFinished() const;
    bool isTaskRunning() const;
    bool isMovementSlaveRunning() const;  // agv在移动
    bool isActionSlaveRunning() const;   // action 执行机构在运行
    bool isWaitingForCheckpoint() const; // 是否在等待关卡

    void setMovementWaitForStart(MovementTask_ptr task);
    void setActionWaitForStart(ActionTask_ptr task);

    void setMovementStart();
    void setActionStart();

    void setMovementRunning();
    void setActionRunning();

    void setMovementPause();
    void setActionPause();

    void setWaitingForCheckpoint();

    void setMovementInCancel();
    void setActionInCancel();

    void setMovementFinishFailed(uint32_t failed_code);
    void setMovementFinishSucceed();
    void setMovementFinishCanceled(uint32_t reason_code = 0);

    void updateActionState(const TaskState &state);

    void setActionFinishFailed(int failed_code);
    void setActionFinishSucceed(int result_value = 0);
    void setActionFinishCanceled(int result_value = 0);

 private:
    void setMovementFinish(MovementTask_ptr move_task, TaskResult result);
    void setActionFinish(ActionTask_ptr action_task, TaskResult result, int result_value);

    TaskManager();
    void sendMovementFinishNotifyMsg(MovementTask_ptr movement_task);
    void sendActionFinishNotifyMsg(ActionTask_ptr action_task);

    inline MovementTask_ptr loadMovementTask() const { return std::atomic_load(&move_task_); }

    inline void storeMovementTask(MovementTask_ptr ptr) { std::atomic_store(&move_task_, ptr); }

    inline ActionTask_ptr loadActionTask() const { return std::atomic_load(&action_task_); }

    inline void storeActionTask(ActionTask_ptr ptr) { std::atomic_store(&action_task_, ptr); }

    void loadMusic();
    void getDefaultActionMusic(ActionTask_ptr task);
    void getMovementMusic(MovementTask_ptr move_task);

    // NOTE：由于本类会有多线程访问，所以不允许直接操作下面两个指针，必须用上面原子操作函数操作，C++20后被废弃
    MovementTask_ptr move_task_;
    ActionTask_ptr action_task_;

    std::map<Action, int> action_music_map_; // <动作, 音乐ID>
};

}  // namespace core
}  // namespace sros

#endif  // SROS_TASK_MANAGER_H
