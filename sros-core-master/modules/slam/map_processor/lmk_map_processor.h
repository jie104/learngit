//
// Created by lfc on 17-7-12.
//

#ifndef SROS_LMK_MAP_PROCESSOR_H
#define SROS_LMK_MAP_PROCESSOR_H

#include "../mapping_processor.h"
#include "../include/map_manager/grid_map_officer.h"
#include <boost/thread.hpp>
#include <modules/slam/include/lmkslam/msg/sensor_input_msg.hpp>
#include <modules/slam/include/lmkslam/msg/map_output_msg.hpp>
#include <modules/slam/include/lmkslam/msg/lmk_para_msg.hpp>

#ifndef LOG_WARNING
#define LOG_WARNING LOG(WARNING)
#endif

namespace slam{
    class LmkModuleSingleton;
}
namespace mapping{
class LmkMapProcessor :public MappingProcessor{
public:

    LmkMapProcessor();

    virtual ~LmkMapProcessor(){

    }


    virtual bool handleStartMapping(sros::core::SlamCommandMsg &syscommand) ;

    virtual bool handleStopMapping(sros::core::SlamCommandMsg &syscommand) ;

    virtual bool handleCancelMapping(sros::core::SlamCommandMsg &syscommand);

    virtual bool processScan(sros::core::base_msg_ptr base_ptr);

    virtual bool processPara(sros::core::base_msg_ptr base_ptr);

    virtual void saveMap(std::string map_name);


protected:
    virtual bool checkScanState(sros::core::base_msg_ptr base_ptr);
private:

    void saveOccMap(std::string map_name);


    bool startMapThread(std::string map_name_with_path);

    bool stopMapThread(std::string map_name_with_path);

    void convertToScan(sros::core::base_msg_ptr base_ptr);

    void getScanToOdoTF();

    void sendMatchPose(std::shared_ptr<slam::PoseMsg> pose_msg);

    void updateOccMap(std::shared_ptr<slam::PoseMsg> pose_msg,
                      sros::core::base_msg_ptr base_ptr);

    void convertScantoPoints(slam::ScanMsg_Ptr scan_ptr,
                             mapping::PointContainer_Ptr points);

    void mapOutputCallback(slam::MapOutputMsg_Ptr map_output);

    GridMapOfficer_Ptr map_officer;

    boost::mutex thread_mutex;
    boost::mutex stop_mapping_mutex;

    std::shared_ptr<slam::LmkModuleSingleton> lmk_module_ptr;

    std::shared_ptr<slam::SensorInputMsg> sensor_msg;
    std::shared_ptr<slam::PoseMsg> last_pose_msg;
    std::shared_ptr<slam::PoseMsg> last_updatemap_pose;

    float updatemap_dist_thresh;
    float updatemap_yaw_thresh;

    float map_resolution;
    int map_level;

    float laser_min_dist;
    float laser_max_dist;

    const int update_free_step = 20;

    std::string map_name;
    std::string map_path;

    bool is_mapping_state;

    Eigen::Vector3f scan2odo_pose;

    slam::LmkParaMsg_Ptr lmkslam_para;



};

}


#endif //SROS_LMK_MAP_PROCESSOR_H
