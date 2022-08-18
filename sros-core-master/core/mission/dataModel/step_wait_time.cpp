//
// Created by huangwuxian on 19-3-1.
//

#include "step_wait_time.h"

namespace sros {
namespace core {

StepWaitTime::StepWaitTime() : AbstractStep(MissionStepType::StepWait),
wait_time_(0), remain_time_(0){

}

StepWaitTime::~StepWaitTime() {

}

bool StepWaitTime::parseStepInfo(const nlohmann::json &step_info) {
    if (step_info.is_null() || !step_info.is_object()) {
        LOG(ERROR) << "StepWaitTime::parseStepInfo(): Invalid json dat " << step_info;
        return false;
    }
    // 转为200ms为单位的时间片
    wait_time_ = (1000 / TIMER_UNIT_200MS) * step_info.at("waitTime").get<int>();
    remain_time_ = wait_time_;
    return true;
}

}
}