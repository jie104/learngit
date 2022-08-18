//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_65_H
#define SROS_ACTION_65_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_SET_GPIO_OUTPUT = 65,
class Action65 : public BaseAction {
 public:
    Action65() : BaseAction(ACTION_ID_SET_GPIO_OUTPUT) {}
    virtual ~Action65() {}

    void doStart() override;
};

}

#endif  // SROS_ACTION_65_H
