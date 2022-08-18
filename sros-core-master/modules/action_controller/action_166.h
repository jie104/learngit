//
// Created by caoyan on 4/9/21.
//

#ifndef SROS_ACTION_166_H
#define SROS_ACTION_166_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_FORK_LIFT = 166,   //叉臂升降动作指令
//其实就是src的叉臂升降指令，包裹一层是为了失败时做急停处理
class Action166 : public BaseAction {
 public:
    Action166(): BaseAction(ACTION_ID_FORK_LIFT) {}
    virtual ~Action166() {}

    void doStart() override;
    void doCancel() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;
    void onSrcAcFinishCanceled(int result_value) override;
 private:


};


}

#endif  // SROS_ACTION_166_H
