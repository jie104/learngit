//
// Created by lhx on 17-1-12.
//

#ifndef SROS_TASK_H
#define SROS_TASK_H

#include <memory>
#include "core/module.h"

namespace sros {
namespace core {

enum TaskState {
    TASK_NA = 0,              ///< 状态不可用, 还未启动的状态
    TASK_WAIT_FOR_START = 2,  ///< 等待开始执行，经过判断当前具备运行条件，等待执行机构开始执行
    TASK_RUNNING = 3,         ///< 任务正在执行
    TASK_PAUSED = 4,          ///< 暂停执行
    TASK_FINISHED = 5,        ///< 执行结束
    TASK_IN_CANCEL = 6,       ///< 正在取消中
    // [废弃] TASK_WAIT_FOR_ACK = 7,  // 等待ack,当收到网络发送过来的消息的时候，需要等待完ack后才算结束
    TASK_WAIT_FOR_CHECKPOINT = 8,  // 等待 关卡解锁
};

enum TaskResult {
    TASK_RESULT_NA = 0,        ///< 结果状态不可用
    TASK_RESULT_OK = 1,        ///< 任务执行完成
    TASK_RESULT_CANCELED = 2,  ///< 任务取消
    TASK_RESULT_FAILED = 3,    ///< 任务执行出错
};

typedef int TaskNo_t;

class TaskStateStruct {
 public:
    TaskStateStruct() = default;
    TaskStateStruct(TaskNo_t no, TaskState state, TaskResult result, int result_value)
        : no_(no), state_(state), result_(result), result_value_(result_value) {}

    friend std::ostream &operator<<(std::ostream &out, const TaskStateStruct &info) {
        out << "TaskStateStruct{no:" << info.no_ << ", state:" << info.state_ << ", result:" << info.result_
            << ", result_value:" << info.result_value_ << "}";
        return out;
    }

    TaskNo_t no_ = 0;
    TaskState state_ = TASK_NA;
    TaskResult result_ = TASK_RESULT_NA;
    int result_value_ = 0;  // 结果值
};

/**
 * 定义Task的基础功能
 */
class Task {
 public:
    Task(TaskNo_t no, const std::string &source_module);
    ~Task();

    TaskNo_t getTaskNo() const;

    const TaskState &getState() const;

    bool isRunning() const;  // 此处的running是表示从外界开来整个程序是否运行，包括wait_for_start、running等

    bool isSlaveRunning() const {
        return state_ == TASK_RUNNING || state_ == TASK_PAUSED || state_ == TASK_WAIT_FOR_CHECKPOINT;
    }  // 此处的running是表示从内部开来是否是正在的执行机构在运行

    bool isPaused() const { return state_ == TASK_PAUSED; }

    bool isWaitForStart() const;

    bool isFinished() const;

    bool isInCancel() const;

    TaskResult getTaskResult() const;

    uint32_t getTaskSeq() const;

    void setTaskSeq(uint32_t seq);

    uint64_t getTaskSessionId() const;

    void setTaskSessionId(uint64_t session_id);

    virtual int getProcessPercentage() const = 0;

    virtual int getRemainTime() const = 0;

    const std::string &getTaskSourceModule() const { return source_module_; }

    bool isNetworkSource() const {
        return source_module_ == "Network";
    }  // 获取是否是网络发送过来的消息，若是的话需要等待回复ack

    int getMusicId() const { return music_id_; }
    void setMusicId(int music_id) { music_id_ = music_id; }

    void updateState(const TaskState &state);
 protected:
    friend class TaskManager;
    void finishTask(TaskResult result);

    void startTask();

    TaskNo_t no_ = 0;  ///< 任务序列号

    int music_id_ = 0;

 private:
    uint32_t seq_ = 0;  // for message seq
    uint64_t session_id_ = 0;

    TaskState state_;  ///< 状态

    TaskResult result_;  ///< 结束状态

    const std::string &source_module_;  // 记录Task是由哪个模块创建的
};

typedef std::shared_ptr<Task> Task_ptr;

}  // namespace core
}  // namespace sros

#endif  // SROS_TASK_H
