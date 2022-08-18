//
// Created by lbx on 2022/02/24.
//

#ifndef SROS_ACTION_188_H
#define SROS_ACTION_188_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_READ_TAG = 188,  // 标签读取动作，包含rfid，pgv
class Action188 : public BaseAction {
 public:
    Action188() : BaseAction(ACTION_ID_READ_TAG) {}
    virtual ~Action188() {}

    void doStart() override;


};

}

#endif  // SROS_ACTION_188_H
