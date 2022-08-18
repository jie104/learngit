//
// Created by lfc on 17-5-24.
//

#ifndef LMKSLAM_POSE_MSG_HPP
#define LMKSLAM_POSE_MSG_HPP

#include "base_msg_interface.hpp"
#include <Eigen/Dense>

namespace slam {
class PoseMsg : public BaseMsgInterface {
public:
    PoseMsg() : BaseMsgInterface(POSE_MSG) {

    }

    virtual ~PoseMsg() { }

    void normalizeYaw(float &yaw) {
        yaw = fmod(fmod(yaw, 2.0 * M_PI), 2.0 * M_PI);
        do {
            if (yaw >= M_PI) {
                yaw -= 2.0f * M_PI;
            } else if (yaw < -M_PI) {
                yaw += 2.0f * M_PI;
            }
        } while (yaw >= M_PI || yaw < -M_PI);
    }

    void setYaw(float yaw) {
        normalizeYaw(yaw);
        pose[2] = yaw;
    }

    Eigen::Vector3f pose;
    Eigen::Matrix3f cov;
    int id;
private:


};

typedef std::shared_ptr<PoseMsg> PoseMsg_Ptr;

}


#endif //LMKSLAM_POSE_MSG_HPP
