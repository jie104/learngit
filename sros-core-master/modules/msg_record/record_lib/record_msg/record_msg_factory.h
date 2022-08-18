//
// Created by lfc on 17-10-27.
//

#ifndef SROS_RECORD_MSG_FACTORY_H
#define SROS_RECORD_MSG_FACTORY_H
#include "base_record_msg.hpp"

namespace record{
class RecordMsgFactory {
public:
    static BaseRecordMsg_Ptr getRecordMsg(RecordMsgType type);
private:


};

}


#endif //SROS_RECORD_MSG_FACTORY_H
