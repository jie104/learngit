//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_7_H
#define SROS_ACTION_7_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_PGV_RECTIFY = 7,  // 下视PGV矫正
class Action7 : public BaseAction {
 public:
    Action7() : BaseAction(ACTION_ID_PGV_RECTIFY) {}
    virtual ~Action7() {}

    void doStart() override;

    bool onSrcAcFinishFirst() override;
};

}

#endif  // SROS_ACTION_7_H
