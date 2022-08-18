//
// Created by lfc on 17-5-11.
//

#ifndef SROS_SLAM_MODULE_H
#define SROS_SLAM_MODULE_H

#include "core/core.h"

#include "core/pose.h"

#include "core/msg/str_msg.hpp"

#include "core/msg/SlamCommandMsg.h"
#include "mapping_processor.h"


namespace mapping {
class SlamModule : public sros::core::Module{
public:
    SlamModule();

    virtual ~SlamModule();

    virtual void run();

    void syscommandCallback(sros::core::base_msg_ptr base_ptr);

    void sendStateMsg(sros::core::SLAM_STATE_CODE state_msg);

    void scanCallback(sros::core::base_msg_ptr base_ptr);

    void paraCallback(sros::core::base_msg_ptr base_ptr);

    void poseCallback(sros::core::base_msg_ptr base_ptr);

    int getVersion();

    std::string getVersionStr();
private:

    MAP_PROCESSOR_TYPE  map_processor_type;
    MappingProcessor_Ptr map_processor;


};
}


#endif //SROS_SLAM_MODULE_H
