//
// Created by lhx on 16-1-21.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_MSG_FACTORY_H
#define SRC_SDK_NETWORK_PROTOCOL_MSG_FACTORY_H

#include "all_msg.h"
#include <memory>

namespace record {

class MsgFactory {
public:

    static BaseMsg_ptr getMsg(MSG_TYPE type) {
        BaseMsg_ptr msg;
        switch (type) {
            case MSG_LASER_SCAN_STAMPED:
                msg = std::make_shared<LaserScanStampedMsg>();
                break;
            case MSG_POSE_STAMPED:
                msg = std::make_shared<PoseStampedMsg>();
                break;
            case PF_LASER_SCAN_STAMPED:
                msg = std::make_shared<LaserScanStampedMsg>(PF_LASER_SCAN_STAMPED);
                break;
            case UHD_LASER_SCAN_STAMPED:
                msg = std::make_shared<LaserScanStampedMsg>(UHD_LASER_SCAN_STAMPED);
                break;
            case MSG_SLAM_INFO:
                msg = std::make_shared<SlamInfoMsg>();
                break;
            default:
                break;
        }

        return msg;
    };

};

} /* namespace src */

#endif //SRC_SDK_NETWORK_PROTOCOL_MSG_FACTORY_H
