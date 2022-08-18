/*
 * MyThread.h
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#ifndef SROS_STATION_RECOGNITION_MODULE_H_
#define SROS_STATION_RECOGNITION_MODULE_H_

#include "core/core.h"

#include "core/pose.h"

#include "core/msg/str_msg.hpp"
#include "core/msg/laser_scan_msg.hpp"
#include "recog_processor/base_recog_processor.hpp"

namespace recog {
class BaseRecogProcessor;
}

namespace sros {

enum FindFeatureType{
    FIND_ANGLE_CMD = 1,
    FIND_LMKRACK_CMD = 2,
    FIND_CHARGINGPILE_CMD=3,
};

class StationRecognitionModule: public core::Module {
public:
    StationRecognitionModule();
    virtual ~StationRecognitionModule();

    virtual void run();

private:
    bool enable_station_recognation_;

    void onNavigationCommandMsg(sros::core::base_msg_ptr m);

    void onLaserScanMsg(sros::core::base_msg_ptr m);

    void onParameterMsg(sros::core::base_msg_ptr m);

    bool findStationPose(sros::core::LaserScan_ptr scan_ptr,
                         sros::core::Pose& result_pose);

    bool getFindAverageResult(sros::core::Pose& out_pose);

    double getDistance(core::Pose &p1, core::Pose &p2) const;

    enum FindState {
        FIND_NONE,
        FIND_WAITING_SCAN, // 等待激光雷达数据
    };

    FindState find_state_;

    sros::core::Pose cur_finding_pose_; // 当前正在检测的站点的地图位置

    std::shared_ptr<recog::BaseRecogProcessor> feature_recog_;

    int find_cnt_; // 检测的次数

    sros::core::Pose_Vector station_pose_list_; // 已检测到的站点Pose

    double param_feature_theta_;
    double param_feature_d1_;
    double param_feature_d2_;

    double param_station_distance_threshold_; // 检测到的站点位置与地图位置偏差阈值
    int param_average_cnt_; // 求平均值需要的检测次数

    double param_laser_coordx;
    double param_laser_coordy;
    double param_laser_coordyaw;

    recog::RecogPara_Ptr recog_para;
    recog::RecogProType recog_type;
};

} /* namespace sros */

#endif /* SROS_STATION_RECOGNITION_MODULE_H_ */
