//
// Created by caoyan on 4/9/21.
//

#ifndef SROS_ACTION_163_H
#define SROS_ACTION_163_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_LOAD_PICKUP_GOODS = 163,   //载货动作
class Action163 : public BaseAction {
 public:
    Action163(): BaseAction(ACTION_ID_LOAD_PICKUP_GOODS) {}
    virtual ~Action163() {}

    void doStart() override;
    void doCancel() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;
    void onSrcAcFinishCanceled(int result_value) override;
 private:
    bool is_forklift_running_ = false;
    bool enable_action_check_pallet_signal_ = false;

};


}

#endif  // SROS_ACTION_163_H
