//
// Created by lfc on 17-8-31.
//

#ifndef SRC_PLATFORM_CONNECT_MSG_FACTORY_HPP
#define SRC_PLATFORM_CONNECT_MSG_FACTORY_HPP
#include "base_msg.hpp"
#include "laser_scan_stamped_msg.hpp"
#include <glog/logging.h>
namespace connection{
class ConnectMsgFactory {
public:
    static BaseMsg_ptr getMsg(ConnectMsgType type) {

        BaseMsg_ptr msg;
        switch (type) {
            case SCAN_MSG_TYPE:
                msg = std::make_shared<LaserScanStampedMsg>();
                break;
            default:
                LOG(INFO) << "err to get the msg type!" << type;
                break;
        }
        return msg;
    }
};
}



#endif //SRC_PLATFORM_CONNECT_MSG_FACTORY_HPP
