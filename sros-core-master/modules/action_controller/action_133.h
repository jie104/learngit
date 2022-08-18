//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_133_H
#define SROS_ACTION_133_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_GET_SVC_INFO = 133,  // 获取svc扫码信息
class Action133 : public BaseAction {
 public:
    Action133() : BaseAction(ACTION_ID_GET_SVC_INFO) {}
    virtual ~Action133() {}

    void doStart() override;

    bool doCancelToInCancel() override { return true;}

    void doTimerEvent(uint64_t cur_time) override;

 private:
    uint64_t action_133_wait_timeout_; // 动作133等待超时
};

}

#endif  // SROS_ACTION_133_H
