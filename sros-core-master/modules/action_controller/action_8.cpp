
#include "action_8.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "core/fault_center.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action8::doStart() { 
    auto &s = sros::core::Settings::getInstance();
    int speed = s.getValue<int>("main.calibration_rotation_speed", 100);
    src_sdk->executeAction(action_no_, 8, action_param0_, speed, action_param1_);
    LOG(INFO) << "src_ac: no " << action_no_ << ", id 8, p0 " << action_param0_ << ", p1 " << speed << ", p2 " << action_param1_;;
}

void Action8::onSrcAcFinishSucceed(int result_value) {
    doActionFinishSucceed();
}

void Action8::onSrcAcFinishFailed(int result_value) {
    doActionFinishFailed(result_value);
}


}
