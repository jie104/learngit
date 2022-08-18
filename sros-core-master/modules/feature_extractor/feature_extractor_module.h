//
// Created by lfc on 2020/12/23.
//

#ifndef SROS_FEATURE_EXTRACTOR_MODULE_H
#define SROS_FEATURE_EXTRACTOR_MODULE_H
#include <core/msg/common_state_msg.hpp>
#include <core/msg/data_matrix_code_msg.hpp>
#include <core/msg/feature_info_msg.hpp>
#include <deque>
#include "core/core.h"
#include "core/msg/str_msg.hpp"
#include "core/pose.h"
#include "scan_feature/scan_feature_extractor.hpp"

namespace sros{

class FeatureExtractorModule: public sros::core::Module{
 public:

    enum AlignmentStatus{
        ERROR_DEVIATION_ISLARGE = 1,
        ERROR_SPECIFIED_FEATURE_NOT_RECOGNIZED = 2,
        ERROR_RECOGNIZED_FEATURE = 3,
        SUCCESSFUL = 4,
    };

    enum ExtractorState{
        STATE_IDLE = 0,//默认
        STATE_SEND_SCAN_TO_INTERACT = 1,//交互 传给上位机
        STATE_SEND_SCAN_TO_LOCATION = 2,//定位
        STATE_CHECK_SCAN_RESO = 3,//对接状态 确认
        STATE_SEND_VISION_TO_INTERACT = 4,
        STATE_SEND_VISION_TO_LOCATION = 5,
        STATE_CHECK_VISION_RESO = 6,
        STATE_LOCATION_VISION_FUSION = 7,
        STATE_RECORD_SCAN_FEATURE = 8,
    };

    struct MatchedFeatureInfo{
        slam::tf::TransForm pose;
        std::string feature_name;
        std::string sensor_name;
    };


    FeatureExtractorModule();

    virtual ~FeatureExtractorModule();

    virtual void run();

 private:

    void extractCommandCallback(sros::core::base_msg_ptr cmd);

    void scanCallback(sros::core::base_msg_ptr scan);

    void visionCallback(sros::core::base_msg_ptr msg);

    void computeAligenSensorTF();

    bool getSpecifyFeature(const MatchedFeatureInfo& matched_info,std::vector<extractor::FeatureInfo_ptr>& infos,extractor::FeatureInfo_ptr& info);

    bool convertToDataMsg(const extractor::FeatureInfo_ptr& info,sros::core::DataMatrixCodeMsg_ptr& msg_info);

    void convertDataMsgToFeatureInfo(const sros::core::DataMatrixCodeMsg_ptr& msg_info,extractor::FeatureInfo_ptr& feature_info);

    bool convertToFeatureMsg(const extractor::FeatureInfo_ptr& info,const slam::tf::TransForm& mean_tf,sros::core::FeatureInfoMsg_ptr& msg_info);

    bool getWorldTF(const extractor::FeatureInfo_ptr& info,const slam::tf::TransForm& mean_tf,slam::tf::TransForm& world_tf);

    void computeMeanPose(const extractor::FeatureInfo_ptr& info,
                         const std::deque<std::vector<extractor::FeatureInfo_ptr>>& info_queues,
                         slam::tf::TransForm& pose);

    bool isMeasureEnough(const extractor::FeatureInfo_ptr& info,
                         const std::deque<std::vector<extractor::FeatureInfo_ptr>>& info_queues,
                         slam::tf::TransForm& pose);

    bool isSameFeature(const extractor::FeatureInfo_ptr& base_info,const extractor::FeatureInfo_ptr& test_info,const float &dist_thresh);

    bool checkAlignmentResult(const extractor::FeatureInfo_ptr& base_info,const slam::tf::TransForm& feature,const MatchedFeatureInfo& match_info,std::shared_ptr<sros::core::CommonStateMsg<bool>>& state);

    void sendUnWatchedState(const MatchedFeatureInfo& match_info);
    ExtractorState scan_extractor_state_;
    ExtractorState camera_extractor_state_;
    bool in_scan_debug_state_ = false;
    bool in_camera_debug_state_ = false;
    std::map<std::string,slam::tf::TransForm> sensor_transforms_;

    extractor::ScanFeatureExtractor scan_extractor_;

    MatchedFeatureInfo scan_matched_info_;
    MatchedFeatureInfo camera_matched_info_;
    std::deque<std::vector<extractor::FeatureInfo_ptr>> info_queues_;
    std::deque<std::vector<extractor::FeatureInfo_ptr>> debug_info_queues_;
    double check_feature_dist_para = 0.02;
    int check_feature_not_watched_count_ = 0;
    bool need_close_camera_ = false;
    const int mean_queue_size_ = 10;
    const int check_feature_not_watched_thresh_ = 10;
};


}

#endif  // SROS_FEATURE_EXTRACTOR_MODULE_H
