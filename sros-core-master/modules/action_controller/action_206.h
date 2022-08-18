//
// Created by caoyan on 7/20/21.
//

#ifndef SROS_ACTION_206_H
#define SROS_ACTION_206_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_BELT_CAR_BRIDGE = 206,  //皮带车搭桥
class Action206 : public BaseAction {
 public:
    Action206(): BaseAction(ACTION_ID_BELT_CAR_BRIDGE) {}
    virtual ~Action206() {}

    void doStart() override;

    void onSrcAcFinishSucceed(int result_value) override;

    void onRegionObstacleMsg(const sros::core::base_msg_ptr &msg) override;

 private:
    bool is_ignore_left_obstacle_ = false;
    bool is_ignore_right_obstacle_ = false;

};

}

#endif  // SROS_ACTION_206_H
