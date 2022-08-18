//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_70_H
#define SROS_ACTION_70_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_GET_SCAN_CODE = 70,  // 常州光宝在用
class Action70 : public BaseAction {
 public:
    Action70() : BaseAction(ACTION_ID_GET_SCAN_CODE) {}
    virtual ~Action70() {}

    void doStart() override;

};


}

#endif  // SROS_ACTION_70_H
