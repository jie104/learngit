//
// Created by john on 18-9-27.
//

#ifndef SROS_HUAWEI_COMM_DATA_MSG_HPP
#define SROS_HUAWEI_COMM_DATA_MSG_HPP


#include "base_msg.h"

namespace sros {
namespace core {

class HuaweiCommMsg : public BaseMsg{
public:
    HuaweiCommMsg() : BaseMsg("HUAWEI_TEST_LINE", TYPE_HUAWEI_COMM_DATA){ };
    virtual ~HuaweiCommMsg() {};

    virtual void getTime() {};

    huawei::SimpleSession_Ptr session_ptr; // 那个链接的指针
};

typedef std::shared_ptr<HuaweiCommMsg> HuaweiCommMsg_ptr;
}
}

#endif //SROS_HUAWEI_COMM_DATA_MSG_HPP
