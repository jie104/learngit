//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_4_H
#define SROS_ACTION_4_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_JACK_ROTATE = 4,  // 旋转顶升机构
class Action4 : public BaseAction {
 public:
    Action4(): BaseAction(ACTION_ID_JACK_ROTATE) {}
    virtual ~Action4() {}

    void doStart() override;

    bool onSrcAcFinishFirst() override;

    void onSrcAcFinishSucceed(int result_value) override;

    void onSrcAcFinishFailed(int result_value) override;

 private:
    bool enable_rotary_jack_keep_scan_;
};

}

#endif  // SROS_ACTION_4_H
