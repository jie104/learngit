//
// Created by lhx on 17-1-12.
//

#include "task_manager.h"
#include <glog/logging.h>
#include <time.h>
#include <random>
#include "core/db/db.h"
#include "core/logger.h"
#include "core/msg/notification_msg.hpp"
#include "core/msg_bus.h"
#include "core/music_id.h"
#include "core/settings.h"
#include "core/alarm_record.hpp"

namespace sros {
namespace core {

TaskManager::TaskManager() { loadMusic(); }

TaskManager *TaskManager::getInstance() {
    static TaskManager manager;
    return &manager;
}

void TaskManager::updateMovementTask(MovementTask_ptr task) { storeMovementTask(task); }

MovementTask_ptr TaskManager::getMovementTask() const { return loadMovementTask(); }

void TaskManager::updateActionTask(ActionTask_ptr task) { storeActionTask(task); }

ActionTask_ptr TaskManager::getActionTask() const { return loadActionTask(); }

bool TaskManager::isTaskRunning() const { return (isMovementTaskRunning() || isActionTaskRunning()); }

bool TaskManager::isWaitingForCheckpoint() const {
    auto move_task = loadMovementTask();
    return move_task->getState() == TASK_WAIT_FOR_CHECKPOINT;
}

bool TaskManager::isMovementTaskRunning() const {
    auto move_task = loadMovementTask();
    return move_task && move_task->isRunning();
}

bool TaskManager::isActionTaskRunning() const {
    auto action_task = loadActionTask();
    return action_task && action_task->isRunning();
}

bool TaskManager::isActionTaskFinished() const {
    auto action_task = loadActionTask();
    return action_task && action_task->isFinished();
}

bool TaskManager::isMovementSlaveRunning() const {
    auto move_task = loadMovementTask();
    return move_task && move_task->isSlaveRunning();
}

bool TaskManager::isActionSlaveRunning() const {
    auto action_task = loadActionTask();

    return action_task && action_task->isSlaveRunning();
}

void TaskManager::setMovementWaitForStart(MovementTask_ptr task) {
    updateMovementTask(task);
    g_state.station_camera_offset.set(DMCodeOffset());  // 启动任务时要将上一次的PGV值清空

    task->updateState(TASK_WAIT_FOR_START);
    LOGGER(INFO, MOVEMENT_TASK) << "Movement task " << task->getTaskNo() << " wait for start!";

    getMovementMusic(task);
}

void TaskManager::setActionWaitForStart(ActionTask_ptr task) {
    updateActionTask(task);

    task->updateState(TASK_WAIT_FOR_START);
    LOGGER(INFO, ACTION_TASK) << "Action task " << task->getTaskNo() << " wait for start!";

    getDefaultActionMusic(task);
}

void TaskManager::setActionFinishFailed(int failed_code) {
    auto action_task = loadActionTask();
    if (action_task) {
        AlarmRecord::getInstance().addActionTaskFailedAlarmInfo(action_task);
        setActionFinish(action_task, sros::core::TASK_RESULT_FAILED, failed_code);
        LOGGER(INFO, ACTION_TASK) << "Action task " << action_task->getTaskNo()
                                  << " slave finish failed! failed code is " << failed_code;
    }
}

void TaskManager::setActionFinishSucceed(int result_value) {
    auto action_task = loadActionTask();
    if (action_task) {
        setActionFinish(action_task, sros::core::TASK_RESULT_OK, result_value);
        LOGGER(INFO, ACTION_TASK) << "Action task " << action_task->getTaskNo()
                                  << " slave finish succeed! result value is " << result_value;
    }
}

void TaskManager::setActionFinishCanceled(int result_value) {
    auto action_task = loadActionTask();
    if (action_task) {
        setActionFinish(action_task, sros::core::TASK_RESULT_CANCELED, result_value);
        LOGGER(INFO, ACTION_TASK) << "Action task " << action_task->getTaskNo()
                                  << " slave finish canceled! result value is " << result_value;
    }
}

void TaskManager::setActionFinish(ActionTask_ptr action_task, TaskResult result, int result_value) {
    if (!action_task) {
        LOG(ERROR) << "action task is null";
        return;
    }

    action_task->finishTask(result, result_value);
    sendActionFinishNotifyMsg(action_task);
}

void TaskManager::sendActionFinishNotifyMsg(ActionTask_ptr action_task) {
    auto notify_msg = std::make_shared<sros::core::NotificationMsg>("TOPIC_NOTIFY");
    notify_msg->notify_type = sros::core::NotificationMsg::NOTIFY_ACTION_TASK_FINISHED;
    notify_msg->action_task = action_task;
    MsgBus::sendMsg(notify_msg);
}

void TaskManager::setMovementInCancel() {
    auto move_task = loadMovementTask();
    if (move_task) {
        move_task->updateState(TASK_IN_CANCEL);
        LOGGER(INFO, MOVEMENT_TASK) << "Movement task " << move_task->getTaskNo() << " in cancel!";
    }
}

void TaskManager::setActionInCancel() {
    auto action_task = loadActionTask();
    if (action_task) {
        action_task->updateState(TASK_IN_CANCEL);
        LOGGER(INFO, ACTION_TASK) << "Action task " << action_task->getTaskNo() << " in cancel!";
    }
}

void TaskManager::setMovementStart() {
    auto move_task = loadMovementTask();
    if (move_task) {
        move_task->startTask();
        LOGGER(INFO, MOVEMENT_TASK) << "Movement task " << move_task->getTaskNo() << " start!";
    }
}

void TaskManager::setActionStart() {
    auto action_task = loadActionTask();
    if (action_task) {
        action_task->startTask();
        LOGGER(INFO, ACTION_TASK) << "Action task " << action_task->getTaskNo() << " start!";
    }
}

void TaskManager::setMovementRunning() {
    auto move_task = loadMovementTask();
    if (move_task) {
        move_task->updateState(TASK_RUNNING);
        LOGGER(INFO, MOVEMENT_TASK) << "Movement task " << move_task->getTaskNo() << " in running!";
    }
}

void TaskManager::setActionRunning() {
    auto action_task = loadActionTask();
    if (action_task) {
        if (action_task->getState() != TASK_RUNNING) {
            action_task->updateState(TASK_RUNNING);
            LOGGER(INFO, ACTION_TASK) << "Action task " << action_task->getTaskNo() << " in running!";
        }
    }
}

void TaskManager::setMovementPause() {
    auto move_task = loadMovementTask();
    if (move_task) {
        move_task->updateState(TASK_PAUSED);
        LOGGER(INFO, MOVEMENT_TASK) << "Movement task " << move_task->getTaskNo() << " in paused!";
    }
}

void TaskManager::setWaitingForCheckpoint() {
    move_task_->updateState(TASK_WAIT_FOR_CHECKPOINT);
    LOGGER(INFO, MOVEMENT_TASK) << "Movement task " << move_task_->getTaskNo() << " in traffic control!";
}

void TaskManager::setActionPause() {
    auto action_task = loadActionTask();
    if (action_task) {
        action_task->updateState(TASK_PAUSED);
        LOGGER(INFO, ACTION_TASK) << "Action task " << action_task->getTaskNo() << " in paused!";
    }
}

void TaskManager::updateActionState(const TaskState &state) {
    auto action_task = loadActionTask();
    if (action_task) {
        auto old_state = action_task->getState();
        if (old_state != state) {
            action_task->updateState(state);
            LOGGER(INFO, ACTION_TASK) << "Action task " << action_task->getTaskNo() << " state changed " << old_state
                                      << " -> " << state;
        }
    }
}

void TaskManager::setMovementFinishFailed(uint32_t failed_code) {
    auto move_task = loadMovementTask();
    if (move_task) {
        move_task->setFailedCode(failed_code);
        AlarmRecord::getInstance().addMovementTaskFailedAlarmInfo(move_task);
        setMovementFinish(move_task, sros::core::TASK_RESULT_FAILED);
        LOGGER(INFO, MOVEMENT_TASK) << "Movement task " << move_task->getTaskNo()
                                    << " slave finish failed! failed code is " << failed_code;
    }
}

void TaskManager::setMovementFinishSucceed() {
    auto move_task = loadMovementTask();
    if (move_task) {
        setMovementFinish(move_task, sros::core::TASK_RESULT_OK);
        LOGGER(INFO, MOVEMENT_TASK) << "Movement task " << move_task->getTaskNo() << " slave finish succeed!";
    }
}

void TaskManager::setMovementFinishCanceled(uint32_t reason_code) {
    auto move_task = loadMovementTask();
    if (move_task) {
        move_task->setFailedCode(reason_code);
        setMovementFinish(move_task, sros::core::TASK_RESULT_CANCELED);
        LOGGER(INFO, MOVEMENT_TASK) << "Movement task " << move_task->getTaskNo() << " slave finish canceled!";
    }
}
void TaskManager::setMovementFinish(MovementTask_ptr move_task, TaskResult result) {
    if (!move_task) {
        LOG(ERROR) << "action task is null";
        return;
    }

    move_task->finishTask(result);
    sendMovementFinishNotifyMsg(move_task);
}

void TaskManager::sendMovementFinishNotifyMsg(MovementTask_ptr movement_task) {
    auto notify_msg = std::make_shared<sros::core::NotificationMsg>("TOPIC_NOTIFY");
    notify_msg->notify_type = sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED;
    notify_msg->movement_task = movement_task;
    MsgBus::sendMsg(notify_msg);
}

TaskNo_t TaskManager::getMovementTaskNo() const {
    auto move_task = loadMovementTask();
    if (move_task) {
        return move_task->getTaskNo();
    }

    return 0;
}
TaskNo_t TaskManager::getActionTaskNo() const {
    auto action_task = loadActionTask();
    if (action_task) {
        return action_task->getTaskNo();
    }

    return 0;
}

void TaskManager::loadMusic() {
    const std::string sql = "SELECT id, param0, param1, music_id FROM actions";

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        SQLite::Statement query(g_db, sql);
        while (query.executeStep()) {
            Action action;
            action.id = query.getColumn(0).getInt();
            std::string param0 = query.getColumn(1).getString();
            if (param0 == "X" || param0 == "Y") {
                action.param0 = 0;
                action.param1 = 0;
                action.static_param_num = 0;
            } else {
                action.param0 = std::stoi(param0);
                std::string param1 = query.getColumn(2).getString();
                if (param1 == "X" || param1 == "Y") {
                    action.param1 = 0;
                    action.static_param_num = 1;
                } else {
                    action.param1 = std::stoi(param1);
                    action.static_param_num = 2;
                }
            }
            int azowie_voice_file_id = query.getColumn(3).getInt();
            action_music_map_.insert(std::make_pair(action, azowie_voice_file_id));
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what() << "; sql is " << sql;
    }
}

void TaskManager::getDefaultActionMusic(ActionTask_ptr task) {
    for (auto it : action_music_map_) {
        if (it.first.id == task->getActionID()) {
            if (it.first.static_param_num == 0) {
                task->setMusicId(it.second);
            } else if (it.first.static_param_num == 1) {
                if (it.first.param0 == task->getActionParam()) {
                    task->setMusicId(it.second);
                }
            } else if (it.first.static_param_num == 2) {
                if (it.first.param1 == task->getActionParam1()) {
                    task->setMusicId(it.second);
                }
            } else {
                LOG(ERROR) << "bad static_param_num! which is " << it.first.static_param_num;
            }
            break;
        }
    }
}
void TaskManager::getMovementMusic(MovementTask_ptr move_task) {
    auto &s = sros::core::Settings::getInstance();

    auto voice_playback_mode = s.getValue<string>("hmi.movement_voice_playback_mode", "random");
    if (voice_playback_mode == "random") {
        static std::default_random_engine random(time(NULL));
        static std::uniform_int_distribution<int> dis(hmi::MUSIC_ID_MUSIC_NORMAL_RUNNING_0,
                                                      hmi::MUSIC_ID_MUSIC_NORMAL_RUNNING_9);
        move_task->setMusicId(dis(random));
    } else if (voice_playback_mode == "sequential") {
        static int last_play_music_id =
            hmi::MUSIC_ID_MUSIC_NORMAL_RUNNING_9;  // 顺序播放时，需要记录上一次播放到那一条音乐了
        if (last_play_music_id == hmi::MUSIC_ID_MUSIC_NORMAL_RUNNING_9) {
            last_play_music_id = hmi::MUSIC_ID_MUSIC_NORMAL_RUNNING_0;
        } else {
            ++last_play_music_id;
        }
        move_task->setMusicId(last_play_music_id);
    } else if (voice_playback_mode == "single_cycle") {
        auto single_cycle_voice_id = s.getValue<int>("hmi.single_cycle_voice_id", 35);
        move_task->setMusicId(single_cycle_voice_id);
    } else if (voice_playback_mode == "none") {
        move_task->setMusicId(hmi::MUSIC_ID_MUSIC_NONE);
    } else {
        LOG(ERROR) << "bad parameter hmi.movement_voice_playback_mode is " << voice_playback_mode;
    }
}

}  // namespace core
}  // namespace sros
