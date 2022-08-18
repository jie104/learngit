//
// Created by lfc on 17-9-5.
//

#ifndef SROS_SLAM_MAP_PROCESSOR_H
#define SROS_SLAM_MAP_PROCESSOR_H

#include "../mapping_processor.h"
#include "../include/normalslam/normal_mapping_singleton.h"
namespace mapping{
class SlamMapProcessor: public MappingProcessor{
public:
    SlamMapProcessor();

    virtual ~SlamMapProcessor();


    virtual bool handleStartMapping(sros::core::SlamCommandMsg &syscommand) ;

    virtual bool handleStopMapping(sros::core::SlamCommandMsg &syscommand) ;

    virtual bool handleCancelMapping(sros::core::SlamCommandMsg &syscommand);

    virtual bool processScan(sros::core::base_msg_ptr base_ptr);

    virtual bool processPara(sros::core::base_msg_ptr base_ptr);

    void updatePara(std::shared_ptr<slam::MappingInfoSlamMsg> mapping_info);

    virtual bool processPose(sros::core::base_msg_ptr base_ptr);

    virtual void saveMap(std::string map_name);

    void mapOutputCallback(slam::MapOutputSlam_Ptr map_output);

private:
    void setMappingState() {
        is_mapping_state = true;
    }

    void resetMappingState() {
        is_mapping_state = false;
    }

    bool getMappingState() {
        return is_mapping_state;
    }

    void processStopMapping();

    void sendPose(slam::PoseSlam_Ptr pose_msg);
    std::string map_name;

    std::string map_path;

    std::shared_ptr<slam::MappingInfoSlamMsg> mapping_info;
    std::shared_ptr<slam::NormalMappingSingleton> mapping_processor;
    bool is_mapping_state = false;
};
}



#endif //SROS_SLAM_MAP_PROCESSOR_H
