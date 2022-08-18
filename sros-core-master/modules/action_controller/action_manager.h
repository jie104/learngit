//
// Created by caoyan on 1/13/21.
//

#ifndef SROS_ACTION_MANAGER_H
#define SROS_ACTION_MANAGER_H

#include "base_action.h"
#include "core/task/action_task.h"

using namespace std;
using namespace sros::core;

enum Action_TYPE {
    ACT_TYPE_NONE = 0,
    ACT_TYPE_VSC,
    ACT_TYPE_SRC,
    ACT_TYPE_SROS,
    ACT_TYPE_EAC,
};

namespace ac {

class ActionManager {
 public:
    static ActionManager* getInstance();

    void init();

    //注册动作
    void registerAction(BaseAction_ptr action_ptr);

    Action_TYPE GetActionType(int action_id);

    bool onStartSrosAction(ActionTask_ptr action_task);
    bool onStartSrcAction(ActionTask_ptr action_task);
    bool onStartEacAction(ActionTask_ptr action_task);

    void onCancelSroslAction(ActionTask_ptr action_task, int result_value = 0);
    void onCancelSrcAction(ActionTask_ptr action_task, int result_value = 0);
    void onCancelEacAction(ActionTask_ptr action_task, int result_value = 0);

    //src ac
    bool onAcFinish(ActionTask_ptr action_task);   //动作完成时,先处理，不关心动作的结果
    void onAcFinishSucceed(ActionTask_ptr action_task, int result_value);
    void onAcFinishFailed(ActionTask_ptr action_task, int result_value);
    void onAcFinishCanceled(ActionTask_ptr action_task, int result_value);

    //algo
    void onAlgoCallback(ActionTask_ptr action_task, const sros::core::base_msg_ptr &msg);

    //region obstacle
    void onRegionObstacleMsg(ActionTask_ptr action_task, const sros::core::base_msg_ptr &msg);

    void onNavCallback(ActionTask_ptr action_task, const sros::core::base_msg_ptr &msg);

    //src mc
    void onMcFinishSucceed(ActionTask_ptr action_task, int result_value);
    void onMcFinishFailed(ActionTask_ptr action_task, int result_value);

    void onTimer50ms(ActionTask_ptr action_task, uint64_t cur_time);

    bool onSubSrcActionCheck(ActionTask_ptr action_task, uint32_t& sub_src_action_no);

 private:
    ActionManager() {}


 private:
    std::map<int, BaseAction_ptr> mapActidToAction_;

};


}


#endif  // SROS_ACTION_MANAGER_H
