//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_1_H
#define SROS_ACTION_1_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_JACKING = 1,      // 顶升机构
class Action1 : public BaseAction {
 public:
    Action1(): BaseAction(ACTION_ID_JACKING) {}
    virtual ~Action1() {}

    void doStart() override;
};

}

#endif  // SROS_ACTION_1_H
