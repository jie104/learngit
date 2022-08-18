//
// Created by lfc on 17-10-27.
//

#include "record_msg_factory.h"
#include "obstacle_record_msg.hpp"

namespace record{

BaseRecordMsg_Ptr RecordMsgFactory::getRecordMsg(RecordMsgType type) {//TODO:第五步需要在该函数创建相对应msg
    BaseRecordMsg_Ptr msg;
    switch (type) {
        case TYPE_RECORD_OBSTACLE:
            msg = std::make_shared<ObstacleRecordMsg>();
            break;
        case TYPE_RECORD_SCAN:
//            msg =
            break;
        case TYPE_RECORD_CMD:

            break;
        default:
            LOG(INFO) << "err to get the msg!" << type;
    }

    return msg;
}
}