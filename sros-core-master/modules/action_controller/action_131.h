//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_131_H
#define SROS_ACTION_131_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_WAIT_COMMAND = 131,
class Action131 : public BaseAction {
 public:
    Action131() : BaseAction(ACTION_ID_WAIT_COMMAND) {}
    virtual ~Action131() {}

    void doStart() override;
};

}

#endif  // SROS_ACTION_131_H
