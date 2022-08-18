//
// Created by lfc on 17-10-16.
//

#ifndef SROS_ANGLE_RECOG_PROCESSOR_HPP
#define SROS_ANGLE_RECOG_PROCESSOR_HPP

#include <glog/logging.h>
#include "base_recog_processor.hpp"
#include "angle_recog/FeatureRecognition.h"
namespace recog{
class AngleRecogProcessor :public BaseRecogProcessor{
public:
    AngleRecogProcessor():BaseRecogProcessor(TYPE_RECOGPRO_ANGLE) {
        feature_recog.reset(new slam::FeatureRecognition);
    }

    virtual RecogInfos_Ptr getRecogInfo(sros::core::LaserScan_ptr scan, Eigen::Vector3f laser_pose) {
        auto pose_list = feature_recog->matchAngle(*scan, laser_pose);
        if (pose_list.size()) {
            RecogInfos_Ptr recog_infos(new RecogInfos);
            for (auto &pose:pose_list) {
                RecogInfo pose_info;
                pose_info.cov.setIdentity();
                pose_info.pose[0] = pose.x();
                pose_info.pose[1] = pose.y();
                pose_info.pose[2] = pose.yaw();
                recog_infos->recog_infos.push_back(pose_info);
            }
            return recog_infos;
        }else {
            LOG(INFO) << "err to get the recog feature! will return!";
            return 0;
        }
    }

    virtual void setProcessorPara() {
        feature_recog->setAngleFeature(recog_para->angle_feat_theta, recog_para->angle_feat_dist_1,
                                       recog_para->angle_feat_dist_2);
    }

private:

    std::shared_ptr<slam::FeatureRecognition> feature_recog;

};

}


#endif //SROS_ANGLE_RECOG_PROCESSOR_HPP
