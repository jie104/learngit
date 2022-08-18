//
// Created by lbx on 2022/07/21.
//

#ifndef SROS_ACTION_181_H
#define SROS_ACTION_181_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_READ_TAG = 181,  //佛山群创2 进电梯门关闭后台雷达
class Action181 : public BaseAction {
 public:
    Action181() : BaseAction(ACTION_ID_ENABLE_BACK_OBA) {}
    virtual ~Action181() {}

    void doStart() override;
    void doCancel() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;
    void onSrcAcFinishCanceled(int result_value) override;

};

}

#endif  // SROS_ACTION_181_H
