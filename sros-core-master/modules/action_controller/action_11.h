//
// Created by caoyan on 1/15/21.
//

#ifndef SROS_ACTION_11_H
#define SROS_ACTION_11_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_GET_RFID_BY_SRC = 11,    //SRC动作获取rfid的值
class Action11 : public BaseAction {
 public:
    Action11(): BaseAction(ACTION_ID_GET_RFID_BY_SRC) {}
    virtual ~Action11() {}

    void doStart() override;

    bool onSrcAcFinishFirst() override;

 private:
    bool is_get_rfid_period_ = false;  // 标记是否处于获取rfid时期，即在只有在上货的时候，另外一个线程才能去获取rfid的内容
    std::string rfid_data_;  // 记录rfid的内容,虽然两个线程读写，但是不会在同一时刻，此处就不加锁了
};


}

#endif  // SROS_ACTION_11_H
