//
// Created by lfc on 17-9-11.
//

#ifndef SROS_SLAM_LOC_PROCESSOR_H
#define SROS_SLAM_LOC_PROCESSOR_H

#include <modules/slam/include/msg/pose_slammsg.hpp>
#include <core/msg/laser_scan_msg.hpp>
#include "../location_processor.h"
#include "modules/slam/include/normalslam/normal_location_singleton.h"
#include "core/tf/TFOperator.h"
#include <map>

namespace sros{
namespace core{
    class PoseStampedMsg;
}
}

namespace location{
class SlamLocProcessor: public LocationProcessor{
public:
    enum FusionCodeState{
        STATE_FUSION_IDLE = 0,
        STATE_FUSION_RECORD_CODE = 1,
        STATE_FUSION_CODE = 2,
        STATE_ALIGNMENT = 3,
    };

    SlamLocProcessor();

    virtual bool handleStartLocation(sros::core::SlamCommandMsg &syscommand);

    virtual bool handleStopLocation(sros::core::SlamCommandMsg &syscommand);

    virtual bool handleStopLocalLocation(sros::core::SlamCommandMsg &syscommand);

    virtual bool handleStartLocalLocation(sros::core::SlamCommandMsg &syscommand);

    virtual bool handleStartLocalRealTimeSlamLoc(sros::core::SlamCommandMsg &syscommand);

    virtual bool handleStopLocalRealTimeSlamLoc(sros::core::SlamCommandMsg &syscommand);

    virtual bool handleRelocation(sros::core::SlamCommandMsg &syscommand);

    virtual bool processPara(sros::core::base_msg_ptr base_ptr);

    void updatePara(std::shared_ptr<slam::LocationInfoSlamMsg>& loc_info);

    void computeAligenSensorTF(std::shared_ptr<slam::LocationInfoSlamMsg>& loc_info);

    virtual bool processScan(sros::core::base_msg_ptr base_ptr);

    virtual void processPgvPose(sros::core::base_msg_ptr base_ptr);

    virtual void processDmCode(sros::core::base_msg_ptr base_ptr);
    virtual void computeLocationError(sros::core::base_msg_ptr base_ptr, slam::LocationCodeSlammsg_Ptr code_msg);

    virtual void processAlignment(sros::core::base_msg_ptr base_ptr);

    virtual bool loadMap(std::string map_name);

    virtual void reloadupdatedMap(const std::string map_path,const std::string map_name);

    void locOutputCallback(slam::LocationOutput_Ptr loc_output);


    virtual ~SlamLocProcessor() {

    }

    virtual void onTime50msLoop();
private:

    Eigen::Vector3d computeDeltaOdoPose(const int64_t &old_time,const int64_t &new_time,int range_index);

    std::vector<std::string> splitStr(const std::string &str, const char delim);

    void reset();

    void setLocationState() {
        is_location_state = true;
    }

    void processStopLocation();

    void resetLocationState() {
        is_location_state = false;
    }

    bool getLocationState() {
        return is_location_state;
    }

    void filtScans(slam::ScanSlam_Ptr scan);

    slam::ScanSlam_Ptr getSlamScan(sros::core::LaserScan_ptr scan_msg);

    void sendMatchMsg(slam::LocationOutput_Ptr loc_output);

    void sendObstacle(slam::ScanSlam_Ptr scan);

    void initialLocThread(sros::core::SlamCommandMsg &syscommand);

    void sendPose(slam::PoseSlam_Ptr pose_msg,float score = 0.0f);

    void setLastPose(const slam::PoseSlam_Ptr& pose_msg);

    slam::PoseSlam_Ptr getLastPose();
    std::string map_name_;

    std::string map_path_;

    std::shared_ptr<slam::LocationInfoSlamMsg> loc_info;

    std::shared_ptr<slam::NormalLocationSingleton> loc_processor;

    sros::core::LaserScan_ptr scan_backup;

    bool is_location_state;
    std::shared_ptr<slam::tf::TFOperator> tf_base_to_odo;

    std::shared_ptr<slam::tf::TFOperator> tf_base_to_world;

    slam::PoseSlam_Ptr last_odo_;

    std::shared_ptr<sros::core::PoseStampedMsg> curr_pgv_pose_;
    slam::PoseSlam_Ptr last_match_pose_;
    boost::mutex last_match_mutex;
    bool need_filter_noise_points = false;
    std::map<std::string,slam::tf::TransForm> sensor_transforms_;
    FusionCodeState fusion_code_state_;
    std::set<std::string> last_fusion_devices_;
    std::string last_record_device_ = "False";
};

}



#endif //SROS_SLAM_LOC_PROCESSOR_H
