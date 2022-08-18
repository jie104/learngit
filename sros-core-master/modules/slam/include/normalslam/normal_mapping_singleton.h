//
// Created by lfc on 17-9-1.
//

#ifndef SROS_NORMAL_SLAM_SINGLETON_H
#define SROS_NORMAL_SLAM_SINGLETON_H
#include <memory>
#include <boost/thread.hpp>
#include "../msg/base_slammsg_interface.hpp"
#include "../msg/mapping_cmd_slammsg.hpp"
#include "../msg/mapping_info_slammsg.hpp"
#include "../msg/mapping_output_slammsg.hpp"
#include "../msg/scan_slammsg.hpp"
#include <Eigen/Dense>
namespace slam{
typedef boost::function<void(slam::MapOutputSlam_Ptr)> MapOutputCallbackFunc;
class NormalMappingSingleton {
public:
    static std::shared_ptr<NormalMappingSingleton> getInstance(MappingInfo_Ptr info);

    bool handleMapCmd(slam::MappingCmd_Ptr cmd);

    virtual ~NormalMappingSingleton();

    void processMsg(slam::BaseSlamMsgInterface_Ptr msg);

    void setMappingInfo(MappingInfo_Ptr info);

    void setMapOutputCallback(MapOutputCallbackFunc func);
};


}


#endif //SROS_NORMAL_SLAM_SINGLETON_H
