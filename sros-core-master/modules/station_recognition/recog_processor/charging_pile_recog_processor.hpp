//
// Created by liuyan on 18-3-9.
//

#ifndef SROS_CHARGING_PILE_RECOG_PROCESSOR_HPP
#define SROS_CHARGING_PILE_RECOG_PROCESSOR_HPP

#include "base_recog_processor.hpp"
#include "feat_recog/base_recognition.hpp"
#include "feat_recog/charging_pile_recognition.h"

namespace recog{
    class ChargingPileRecogProcessor :public BaseRecogProcessor{
    public:
        ChargingPileRecogProcessor():BaseRecogProcessor(TYPE_RECOGPRO_CHARGINGPILE){
            feature_recog_.reset(new ChargingPileRecognition);
        }
        virtual ~ChargingPileRecogProcessor(){}

        virtual RecogInfos_Ptr getRecogInfo(sros::core::LaserScan_ptr scan, Eigen::Vector3f laser_pose){
            Scan_Ptr recog_scan(new recog::ScanPoints);
            changeLaserScanToScan(recog_scan,scan);
            Eigen::Vector3f out_pose;
            if(feature_recog_->getPose(recog_scan,out_pose)){
                RecogInfo recog_info;
                LOG(INFO) << "!!!!!!!!!!!!!!!!!!!!!!!";
                if((laser_pose[0] == 0)&&(laser_pose[1]==0)&&(laser_pose[2]==0)){
                    recog_info.pose = out_pose;//返回在ｓｃａｎ坐标系下的位置
                }
                else {
                    Eigen::Affine2f laser_tf(
                            Eigen::Translation2f(laser_pose[0], laser_pose[1]) * Eigen::Rotation2Df(laser_pose[2]));
                    Eigen::Vector2f world_point = laser_tf * out_pose.head<2>();
                    float yaw = out_pose[2] + laser_pose[2] ;
                    feature_recog_->normalizeAngle(yaw);
                    recog_info.pose = Eigen::Vector3f(world_point[0], world_point[1], yaw);
                    recog_info.cov.setIdentity();
                }
                RecogInfos_Ptr recog_infos(new RecogInfos);
                recog_infos->recog_infos.clear();
                recog_infos->recog_infos.push_back(recog_info);
                return recog_infos;
            }
            return nullptr;
        }

    private:
        virtual void setProcessorPara() {
            ChargingRecogInfo_Ptr recog_info(new ChargingRecogInfo);
            std::shared_ptr<RecogChargingPilePara> recog_charging_pile_para = std::dynamic_pointer_cast<RecogChargingPilePara>(recog_para);
            recog_info->intensity_thresh = recog_charging_pile_para->intensity_thresh;
            recog_info->num_count = recog_charging_pile_para->num_count;
            recog_info->charging_pile_width = recog_charging_pile_para->charging_pile_width;
            recog_info->split_dist = recog_charging_pile_para->split_dist;
            recog_info->reflective_stickers_length  = recog_charging_pile_para->reflective_stickers_length;

            feature_recog_->setInfo(recog_info);
        }
        void changeLaserScanToScan(Scan_Ptr recog_scan, sros::core::LaserScan_ptr scan){
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
        BaseRecognition_Ptr charging_pile_recog_;
    };
}


#endif //SROS_CHARGING_PILE_RECOG_PROCESSOR_HPP
