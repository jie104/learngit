/**
 * @file location_module.cpp
 *
 * @author lhx
 * @date 2016/12/01
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef MODULES_LOCATION_LOCATION_MODULE_H_
#define MODULES_LOCATION_LOCATION_MODULE_H_

#include <Eigen/Dense>
#include "core/core.h"
#include "core/msg/SlamCommandMsg.h"
#include "core/msg/str_msg.hpp"
#include "core/pose.h"
#include "core/tf/TFOperator.h"

#include "location_processor.h"
#include "core/msg/data_matrix_code_msg.hpp"
namespace location {

class LocationModule : public sros::core::Module {
 public:
    LocationModule();

    virtual ~LocationModule();

    virtual void run();

    void syscommandCallback(sros::core::base_msg_ptr base_ptr);

    void scanCallback(sros::core::base_msg_ptr base_ptr);

    void pgvPoseCallback(sros::core::base_msg_ptr base_ptr);

    void dmCodeCallback(sros::core::base_msg_ptr base_ptr);

    void alignmentCallback(sros::core::base_msg_ptr base_ptr);

    void paraCallback(sros::core::base_msg_ptr base_ptr);

    void sendStateMsg(sros::core::SLAM_STATE_CODE state_msg);

    static void systemExitTrace(int signum);

    void onTime50msLoop(sros::core::base_msg_ptr base_ptr);

    void onUpdateMap(sros::core::base_msg_ptr base_ptr);

 private:
    LOC_PROCESSOR_TYPE loc_processor_type;
    LocationProcessor_Ptr loc_processor;
};

}  // namespace location

#endif  // MODULES_LOCATION_LOCATION_MODULE_H_
