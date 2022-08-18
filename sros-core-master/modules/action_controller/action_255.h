//
// Created by caoyan on 5/28/21.
//

#ifndef SROS_ACTION_255_H
#define SROS_ACTION_255_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_COMMON_EAC = 255,   //通用eac发送指令
class Action255 : public BaseAction {
 public:
    Action255() : BaseAction(ACTION_ID_COMMON_EAC) {}
    virtual ~Action255() {}

    void doStart() override;
};

}


#endif  // SROS_ACTION_255_H
