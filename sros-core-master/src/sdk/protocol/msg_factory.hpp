//
// Created by lhx on 16-1-21.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_MSG_FACTORY_H
#define SRC_SDK_NETWORK_PROTOCOL_MSG_FACTORY_H

#include "all_msg.h"
#include <memory>

namespace src {

class MsgFactory {
public:

    static BaseMsg_ptr getMsg(MSG_TYPE type) {
        BaseMsg_ptr msg;
        switch (type) {
            case MSG_PATH:
                msg = std::make_shared<PathMsg>();
                break;
            case MSG_STATE:
                msg = std::make_shared<StateMsg>();
                break;
            case MSG_POSE:
                msg = std::make_shared<PoseMsg>();
                break;
            case MSG_COMMAND:
                msg = std::make_shared<CommandMsg>();
                break;
            case MSG_VELOCITY:
                msg = std::make_shared<VelocityMsg>();
                break;
            case MSG_PARAMETER:
                msg = std::make_shared<ParameterMsg>();
                break;
            case MSG_GAZEBO_LASER_SCAN:
                msg = std::make_shared<GazeboLaserScanMsg>();
                break;
            case MSG_SIGNAL:
                msg = std::make_shared<SignalMsg>();
                break;
            case MSG_LASER_SCAN_STAMPED:
                msg = std::make_shared<LaserScanStampedMsg>();
                break;
            case MSG_POSE_STAMPED:
                msg = std::make_shared<PoseStampedMsg>();
                break;
            case MSG_USART_DATA:
                msg = std::make_shared<USARTDataMsg>();
                break;
            case PF_LASER_SCAN_STAMPED:
                msg = std::make_shared<LaserScanStampedMsg>(PF_LASER_SCAN_STAMPED);
                break;
            default:
                break;
        }

        return msg;
    };

};

} /* namespace src */

#endif //SRC_SDK_NETWORK_PROTOCOL_MSG_FACTORY_H
