//
// Created by lfc on 17-10-16.
//

#ifndef SROS_HACK_LMK_PROCESSOR_HPP
#define SROS_HACK_LMK_PROCESSOR_HPP

#include "base_recog_processor.hpp"
#include "feat_recog/base_recognition.hpp"
#include "feat_recog/rack_recognition.h"

namespace recog {
class HackLmkProcessor : public BaseRecogProcessor {
public:
    HackLmkProcessor() : BaseRecogProcessor(TYPE_RECOGPRO_LMKRACK) {
        feature_recog_.reset(new RackRecognition);
    }

    virtual ~HackLmkProcessor() {

    }

    virtual RecogInfos_Ptr getRecogInfo(sros::core::LaserScan_ptr scan, Eigen::Vector3f laser_pose) {

        Scan_Ptr recog_scan(new recog::ScanPoints);
        cpScan(recog_scan, scan);
        Eigen::Vector3f out_pose;
        if (feature_recog_->getPose(recog_scan, out_pose)) {
            Eigen::Affine2f laser_tf(
                    Eigen::Translation2f(laser_pose[0], laser_pose[1]) * Eigen::Rotation2Df(laser_pose[2]));
            Eigen::Vector2f world_point = laser_tf * out_pose.head<2>();
            float yaw = out_pose[2] + laser_pose[2];
            feature_recog_->normalizeAngle(yaw);
            RecogInfo recog_info;
            recog_info.pose = Eigen::Vector3f(world_point[0], world_point[1], yaw);
            recog_info.cov.setIdentity();

            RecogInfos_Ptr recog_infos(new RecogInfos);
            recog_infos->recog_infos.clear();
            recog_infos->recog_infos.push_back(recog_info);
            return recog_infos;
        }
        LOG(INFO) << "err to get the recog feature!";
        return nullptr;
    }

private:
    virtual void setProcessorPara() {
        RackRecogInfo_Ptr recog_info(new RackRecogInfo);
        std::shared_ptr<RecogRackPara> recog_rack_para = std::dynamic_pointer_cast<RecogRackPara>(recog_para);
        recog_info->intensity_thresh = recog_rack_para->intensity_thresh;
        recog_info->num_count = recog_rack_para->num_count;
        recog_info->rack_length = recog_rack_para->rack_length;
        recog_info->rack_width = recog_rack_para->rack_width;
        recog_info->split_dist = recog_rack_para->split_dist;
        feature_recog_->setInfo(recog_info);
    }

    void cpScan(Scan_Ptr recog_scan, sros::core::LaserScan_ptr scan) {
        recog_scan->angle_increment = scan->angle_increment;
        recog_scan->angle_max = scan->angle_max;
        recog_scan->angle_min = scan->angle_min;
        recog_scan->range_max = scan->range_max;
        recog_scan->range_min = scan->range_min;
        recog_scan->stamp = scan->time_;
        recog_scan->time_increment = scan->time_increment;
        recog_scan->ranges = scan->ranges;
        recog_scan->intensities = scan->intensities;
    }

private:
    BaseRecognition_Ptr hacklmk_recog;
};

}


#endif //SROS_HACK_LMK_PROCESSOR_HPP
