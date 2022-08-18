//
// Created by perry on 9/29/21.
//

#ifndef SROS_ACTION_137_H
#define SROS_ACTION_137_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_GET_SVC_INFO = 137,  // 获取svc扫码信息，单机匹配料架号(OPPO项目新增需求)
class Action137 : public BaseAction {
 public:
    Action137() : BaseAction(ACTION_ID_MATCH_SVC_INFO) {}
    virtual ~Action137() {}

    void doStart() override;

    bool doCancelToInCancel() override { return true;}

    void doTimerEvent(uint64_t cur_time) override;

    //匹配SVC扫码信息
    void doMatchSvcInfo(const std::string& _strDev);

 private:
    uint64_t action_137_wait_timeout_; // 动作137等待超时
};

}

#endif  // SROS_ACTION_136_H
