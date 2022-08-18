//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_0_H
#define SROS_ACTION_0_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_COMMON_SRC = 0,   // 通用src发送指令
class Action0 : public BaseAction {
 public:
    Action0() : BaseAction(ACTION_ID_COMMON_SRC) {}
    virtual ~Action0() {}

    void doStart() override;
};

}

#endif  // SROS_ACTION_0_H
