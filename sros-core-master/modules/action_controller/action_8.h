//
// Created by lbx on 2022/6/8.
//

#ifndef SROS_ACTION_8_H
#define SROS_ACTION_8_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {


class Action8: public BaseAction {
 public:
    Action8(): BaseAction(ACTION_ID_CALIBRATION) {}
    virtual ~Action8() {}

    void doStart() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;

};


}


#endif  // SROS_ACTION_8_H
