//
// Created by lfc on 17-7-10.
//

#ifndef SROS_LMK_PROCESSOR_H
#define SROS_LMK_PROCESSOR_H

#include <boost/thread/pthread/mutex.hpp>
#include <modules/location/record_data/pose_record.h>
#include <boost/thread/pthread/condition_variable.hpp>
#include <modules/slam/include/lmkslam/msg/lmk_para_msg.hpp>
#include "../location_processor.h"
#include "core/tf/TFOperator.h"

namespace slam{
class LmkModuleSingleton;

class SensorInputMsg;

class PoseMsg;

class MapOutputMsg;
}
namespace location{
class LmkLocProcessor : public LocationProcessor {
public:
    LmkLocProcessor();

    virtual bool handleStartLocation(sros::core::SlamCommandMsg &syscommand);

    virtual bool handleStopLocation(sros::core::SlamCommandMsg &syscommand);

    virtual bool processPara(sros::core::base_msg_ptr base_ptr);

    virtual bool processScan(sros::core::base_msg_ptr base_ptr);

    virtual bool processScan(sros::core::base_msg_ptr base_ptr, sros::core::Pose base_pose);

    virtual bool loadMap(std::string map_name);

    virtual ~LmkLocProcessor() {

    }


private:
    bool checkScanState(sros::core::base_msg_ptr base_ptr);

    void processInitialPoseThread(sros::core::SlamCommandMsg &syscommand);

    bool processInitialPose(std::shared_ptr<slam::PoseMsg> pose_msg);

    bool processStopLoc();

    void convertToScan(sros::core::base_msg_ptr base_ptr);

    void sendMatchPose(std::shared_ptr<slam::PoseMsg> pose_msg);

    void getScanToOdoTF();

    void monitorLoop();

    void wakeupInfo();

    static void systemExitTrace(int signum);

    void mapOutputCallback(std::shared_ptr<slam::MapOutputMsg> match_output);

    Eigen::Vector3f getBaseToScanPose();

    void sendObstacleMsg(sros::core::base_msg_ptr base_ptr,
                         std::shared_ptr<slam::PoseMsg> pose_msg);

    boost::mutex wakeup_mutex;
    boost::condition_variable_any condition;

    boost::mutex thread_mutex;

    std::shared_ptr<slam::LmkModuleSingleton> lmk_module_ptr;

    int64_t last_scan_time_stamp;

    int err_loc_count;
    const int err_loc_count_thresh = 5;

    std::shared_ptr<slam::SensorInputMsg> sensor_msg;
    std::shared_ptr<slam::PoseMsg> last_pose_msg;

    std::string map_name;
    std::string map_path;

    bool is_location_state;

    Eigen::Vector3f odo2scan_pose;
    Eigen::Vector3f scan2odo_pose;

    record::PoseRecord_ptr pose_record;


    sros::core::base_msg_ptr last_scan; //用于发布scan数据

    slam::LmkParaMsg_Ptr lmkslam_para;

    std::shared_ptr<slam::tf::TFOperator> tf_base_to_odo;

    std::shared_ptr<slam::tf::TFOperator> tf_base_to_world;

};

}


#endif //SROS_LMK_PROCESSOR_H
