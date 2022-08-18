//
// Created by caoyan on 1/15/21.
//

#include "action_131.h"
#include "core/logger.h"
#include "core/task/task_manager.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action131::doStart() {
    LOG(INFO) << "Action Task wait for input value";

    TaskManager::getInstance()->getActionTask()->setMusicId(action_param1_);
}

}