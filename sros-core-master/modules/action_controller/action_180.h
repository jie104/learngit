//
// Created by lbx on 10/20/21.
//

#ifndef SROS_ACTION_180_H
#define SROS_ACTION_180_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {


class Action180: public BaseAction {
 public:
    Action180(): BaseAction(ACTION_ID_HOOK_SWITCH) {}
    virtual ~Action180() {}

    void doStart() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;
    void sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type);
private:
    int result_count_ = 0;
};


}


#endif  // SROS_ACTION_180_H
