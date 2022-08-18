//
// Created by lbx on 2022/1/8
//

#ifndef SROS_ACTION_189_H
#define SROS_ACTION_189_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_DEBUG = 0,   // 调试动作，目前用于算法标定
class Action189 : public BaseAction {
 public:
    Action189() : BaseAction(ACTION_ID_DEBUG) {}
    virtual ~Action189() {}

    void doStart() override;
    void onAlgoResultCallback(const sros::core::base_msg_ptr& msg) override;
    void sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type,
                              const int& goal_id, 
                              Pose pose, NavigationPath_vector paths);
private:
    void detectDst3DPose();
};

}

#endif  // SROS_ACTION_189_H
