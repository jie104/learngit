//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_132_H
#define SROS_ACTION_132_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_GET_RFID_BY_INNER = 132,  // inner动作获取rfid的值
class Action132 : public BaseAction {
 public:
    Action132() : BaseAction(ACTION_ID_GET_RFID_BY_INNER) {}
    virtual ~Action132() {}

    bool doCancelToInCancel() override { return true;}

    void doInCancel() override;

    void doStart() override;

private:
    bool is_in_cancel_;

};

}

#endif  // SROS_ACTION_132_H
