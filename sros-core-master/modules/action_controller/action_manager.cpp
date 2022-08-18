//
// Created by caoyan on 1/13/21.
//

#include "action_manager.h"
#include "core/state.h"
#include "core/logger.h"

#include "action_0.h"
#include "action_1.h"
#include "action_4.h"
#include "action_7.h"
#include "action_8.h"
#include "action_11.h"
#include "action_65.h"
#include "action_66.h"
#include "action_70.h"

#include "action_129.h"
#include "action_130.h"
#include "action_131.h"
#include "action_132.h"
#include "action_133.h"
#include "action_134.h"
#include "action_135.h"
#include "action_136.h"
#include "action_137.h"

#include "action_161.h"
#include "action_162.h"
#include "action_161_faster.h"
#include "action_162_faster.h"
#include "action_162_low_faster.h"
#include "action_162_redbull.h"
#include "action_163.h"
#include "action_164.h"
#include "action_165.h"
#include "action_166.h"
#include "action_171.h"
#include "action_172.h"
#include "action_172_faster.h"
#include "action_172_low_faster.h"
#include "action_172_redbull.h"
#include "action_173.h"
#include "action_180.h"
#include "action_181.h"
#include "action_188.h"
#include "action_189.h"

#include "action_206.h"
#include "action_255.h"

using namespace std;
using namespace sros::core;

namespace ac {

ActionManager * ActionManager::getInstance() {
    static ActionManager instance;
    return &instance;
}

void ActionManager::init() {
    registerAction(std::make_shared<Action0>());
    registerAction(std::make_shared<Action1>());
    registerAction(std::make_shared<Action4>());
    registerAction(std::make_shared<Action7>());
    registerAction(std::make_shared<Action8>());
    registerAction(std::make_shared<Action11>());
    registerAction(std::make_shared<Action65>());
    registerAction(std::make_shared<Action66>());
    registerAction(std::make_shared<Action70>());

    registerAction(std::make_shared<Action129>());
    registerAction(std::make_shared<Action130>());
    registerAction(std::make_shared<Action131>());
    registerAction(std::make_shared<Action132>());
    registerAction(std::make_shared<Action133>());
    registerAction(std::make_shared<Action134>());
    registerAction(std::make_shared<Action135>());
    registerAction(std::make_shared<Action136>());
    registerAction(std::make_shared<Action137>());

    auto& s = sros::core::Settings::getInstance();
    std::string mode = s.getValue<string>("forklift.load_unload_mode", "COMMON");
    if( mode == "FASTER"){
        registerAction(std::make_shared<Action161Faster>());
        registerAction(std::make_shared<Action162Faster>());
        registerAction(std::make_shared<Action172Faster>());
    }else if( mode == "LOW_FASTER"){
        registerAction(std::make_shared<Action161Faster>());
        registerAction(std::make_shared<Action162LowFaster>());
        registerAction(std::make_shared<Action172LowFaster>());
    }else if( mode == "REDBULL"){
        registerAction(std::make_shared<Action162Redbull>());
        registerAction(std::make_shared<Action172RedBull>());
    }else{
        registerAction(std::make_shared<Action161>());
        registerAction(std::make_shared<Action162>());
        registerAction(std::make_shared<Action172>());
    }

    registerAction(std::make_shared<Action163>());
    registerAction(std::make_shared<Action164>());
    registerAction(std::make_shared<Action165>());
    registerAction(std::make_shared<Action166>());
    registerAction(std::make_shared<Action171>());
    registerAction(std::make_shared<Action173>());
    registerAction(std::make_shared<Action180>());
    registerAction(std::make_shared<Action181>());
    registerAction(std::make_shared<Action188>());
    registerAction(std::make_shared<Action189>());

    registerAction(std::make_shared<Action206>());
    registerAction(std::make_shared<Action255>());
}

Action_TYPE ActionManager::GetActionType(int action_id) {
    //vsc
    if(action_id == 0x4f || action_id == 0x4e) {
        return ACT_TYPE_VSC;
    } else if((action_id > 0 && action_id < 0x7F)) {
        return ACT_TYPE_SRC;
    } else if((action_id >= 0x80 && action_id < 0xBF)) {
        return ACT_TYPE_SROS;
    } else if((action_id >= 0xC0 && action_id <= 0xFF)) {
        return ACT_TYPE_EAC;
    } else {
        return ACT_TYPE_NONE;
    }
}

void ActionManager::registerAction(BaseAction_ptr action_ptr) {
    if(action_ptr == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, registerAction failed";
        return;
    }

    int action_id = action_ptr->getRegActionId();

    if(action_id < 0) {
        LOG(ERROR) << "action_id(" << action_id << ") < 0, registerAction failed";
        return;
    }

    if(mapActidToAction_.find(action_id) != mapActidToAction_.end()) {
        LOG(ERROR) << "exist action_id(" << action_id << "), registerAction failed";
        return;
    }

    //先初始化
    if(action_ptr->initAction()) {
        mapActidToAction_[action_id] = action_ptr;
        LOG(INFO) << "register action id: " << action_id << " succeed";
    } else {
        LOG(INFO) << "register action id: " << action_id << " failed";
    }
}

bool ActionManager::onStartSrosAction(ActionTask_ptr action_task) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onStartSrosAction failed";
        return false;
    }

    bool bRet = true;
    auto action_id = action_task->getActionID();
    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->startAction(action_task);
    } else {
        bRet = false;
    }
    return bRet;
}

bool ActionManager::onStartSrcAction(ActionTask_ptr action_task) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onStartSrcAction failed";
        return false;
    }

    bool bRet = true;
    auto action_id = action_task->getActionID();
    auto act_type = ActionManager::GetActionType(action_id);
    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->startAction(action_task);
    } else {
        //如果是通用src动作指令
        if(act_type == ACT_TYPE_SRC) {
            mapActidToAction_[ACTION_ID_COMMON_SRC]->startAction(action_task);
        } else {
            bRet = false;
        }
    }
    return bRet;
}

bool ActionManager::onStartEacAction(ActionTask_ptr action_task) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onStartEacAction failed";
        return false;
    }

    bool bRet = true;
    auto action_id = action_task->getActionID();
    auto act_type = ActionManager::GetActionType(action_id);
    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->startAction(action_task);
    } else {
        //如果是通用eac动作指令
        if(act_type == ACT_TYPE_EAC) {
            mapActidToAction_[ACTION_ID_COMMON_EAC]->startAction(action_task);
        } else {
            bRet = false;
        }
    }
    return bRet;
}

void ActionManager::onCancelSroslAction(ActionTask_ptr action_task, int result_value) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onCancelSroslAction failed";
        return;
    }

    auto action_id = action_task->getActionID();

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->cancelAction(result_value);
    }
}

void ActionManager::onCancelSrcAction(ActionTask_ptr action_task, int result_value) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onCancelSrcAction failed";
        return;
    }

    auto action_id = action_task->getActionID();
    auto act_type = ActionManager::GetActionType(action_id);

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->cancelAction(result_value);
    } else {
        //如果是通用src动作指令
        if(act_type == ACT_TYPE_SRC) {
            mapActidToAction_[ACTION_ID_COMMON_SRC]->cancelAction(result_value);
        }
    }
}

void ActionManager::onCancelEacAction(ActionTask_ptr action_task, int result_value) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onCancelEacAction failed";
        return;
    }

    auto action_id = action_task->getActionID();
    auto act_type = ActionManager::GetActionType(action_id);

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->cancelAction(result_value);
    } else {
        //如果是通用eac动作指令
        if(act_type == ACT_TYPE_EAC) {
            mapActidToAction_[ACTION_ID_COMMON_EAC]->cancelAction(result_value);
        }
    }
}

bool ActionManager::onAcFinish(ActionTask_ptr action_task) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onActionFinish failed";
        return true;
    }

    auto action_id = action_task->getActionID();
    auto act_type = ActionManager::GetActionType(action_id);

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        return action_ptr->second->onSrcAcFinishFirst();
    } else {
        //如果是通用src动作指令
        if(act_type == ACT_TYPE_SRC) {
            return mapActidToAction_[ACTION_ID_COMMON_SRC]->onSrcAcFinishFirst();
        } else if(act_type == ACT_TYPE_EAC) {
            return mapActidToAction_[ACTION_ID_COMMON_EAC]->onSrcAcFinishFirst();
        }
    }

    return true;
}

void ActionManager::onAcFinishSucceed(ActionTask_ptr action_task, int result_value) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onFinishSucceed failed";
        return;
    }

    auto action_id = action_task->getActionID();
    auto act_type = ActionManager::GetActionType(action_id);

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->onSrcAcFinishSucceed(result_value);
    } else {
        //如果是通用src动作指令
        if(act_type == ACT_TYPE_SRC) {
            mapActidToAction_[ACTION_ID_COMMON_SRC]->onSrcAcFinishSucceed(result_value);
        } else if(act_type == ACT_TYPE_EAC) {
            mapActidToAction_[ACTION_ID_COMMON_EAC]->onSrcAcFinishSucceed(result_value);
        }
    }
}

void ActionManager::onAcFinishFailed(ActionTask_ptr action_task, int result_value) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onFinishFailed failed";
        return;
    }

    auto action_id = action_task->getActionID();
    auto act_type = ActionManager::GetActionType(action_id);

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->onSrcAcFinishFailed(result_value);
    } else {
        //如果是通用src动作指令
        if(act_type == ACT_TYPE_SRC) {
            mapActidToAction_[ACTION_ID_COMMON_SRC]->onSrcAcFinishFailed(result_value);
        } else if(act_type == ACT_TYPE_EAC) {
            mapActidToAction_[ACTION_ID_COMMON_EAC]->onSrcAcFinishFailed(result_value);
        }
    }
}

void ActionManager::onAcFinishCanceled(ActionTask_ptr action_task, int result_value) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onFinishCanceled failed";
        return;
    }

    auto action_id = action_task->getActionID();
    auto act_type = ActionManager::GetActionType(action_id);

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->onSrcAcFinishCanceled(result_value);
    } else {
        //如果是通用src动作指令
        if(act_type == ACT_TYPE_SRC) {
            mapActidToAction_[ACTION_ID_COMMON_SRC]->onSrcAcFinishCanceled(result_value);
        } else if(act_type == ACT_TYPE_EAC) {
            mapActidToAction_[ACTION_ID_COMMON_EAC]->onSrcAcFinishCanceled(result_value);
        }
    }
}

//algo
void ActionManager::onAlgoCallback(ActionTask_ptr action_task, const sros::core::base_msg_ptr &msg) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onAlgoCallback failed";
        return;
    }

    auto action_id = action_task->getActionID();

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->onAlgoResultCallback(msg);
    }
}

//region obstacle
void ActionManager::onRegionObstacleMsg(ActionTask_ptr action_task, const sros::core::base_msg_ptr &msg) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onRegionObstacleMsg failed";
        return;
    }

    auto action_id = action_task->getActionID();

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->onRegionObstacleMsg(msg);
    }
}

//NAV
void ActionManager::onNavCallback(ActionTask_ptr action_task, const sros::core::base_msg_ptr &msg) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onNavCallback failed";
        return;
    }

    auto action_id = action_task->getActionID();

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->onNavResultCallback(msg);
    }
}


//src mc
void ActionManager::onMcFinishSucceed(ActionTask_ptr action_task, int result_value) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onMcFinishSucceed failed";
        return;
    }

    auto action_id = action_task->getActionID();

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->onSrcMcFinishSucceed(result_value);
    }
}

void ActionManager::onMcFinishFailed(ActionTask_ptr action_task, int result_value) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_ptr == nullptr, onMcFinishFailed failed";
        return;
    }

    auto action_id = action_task->getActionID();

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->onSrcMcFinishFailed(result_value);
    }
}


void ActionManager::onTimer50ms(ActionTask_ptr action_task, uint64_t cur_time) {
    if(action_task == nullptr) {
        return;
    }

    auto action_id = action_task->getActionID();

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        action_ptr->second->onTimer50ms(cur_time);
    }
}

bool ActionManager::onSubSrcActionCheck(ActionTask_ptr action_task, uint32_t& sub_src_action_no) {
    if(action_task == nullptr) {
        return false;
    }

    auto action_id = action_task->getActionID();
    auto act_type = ActionManager::GetActionType(action_id);

    auto action_ptr = mapActidToAction_.find(action_id);
    if(action_ptr != mapActidToAction_.end()) {
        return action_ptr->second->onSubSrcActionCheck(sub_src_action_no);
    } else {
        //如果是通用src动作指令
        if(act_type == ACT_TYPE_SRC) {
            return mapActidToAction_[ACTION_ID_COMMON_SRC]->onSubSrcActionCheck(sub_src_action_no);
        } else if(act_type == ACT_TYPE_EAC) {
            return mapActidToAction_[ACTION_ID_COMMON_EAC]->onSubSrcActionCheck(sub_src_action_no);
        }
    }

    return false;
}


}
