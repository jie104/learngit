//
// Created by lfc on 17-9-11.
//

#ifndef SROS_NORMAL_LOCATION_SINGLETON_H
#define SROS_NORMAL_LOCATION_SINGLETON_H

#include <memory>
#include <boost/thread.hpp>
#include "../msg/scan_slammsg.hpp"
#include "../msg/location_info_slammsg.hpp"
#include "../msg/location_cmd_slammsg.hpp"
#include "../msg/location_output_slammsg.hpp"
#include "../msg/location_code_slammsg.hpp"
#include <core/tf/TransForm.h>
namespace slam{

typedef boost::function<void(LocationOutput_Ptr)> LocOutputCallbackFunc;
class NormalLocationSingleton {
public:

    static std::shared_ptr<NormalLocationSingleton> getInstance(LocationInfo_Ptr info);

    bool handleLocCmd(slam::LocationCmd_Ptr cmd);

    virtual ~NormalLocationSingleton();

    void setLocationInfo(LocationInfo_Ptr info);

    void processMsg(slam::BaseSlamMsgInterface_Ptr msg);

    void setMapOutputCallback(LocOutputCallbackFunc func);

    bool getLastMatchedPoseStamp(int64_t &scan_stamp,Eigen::Vector3f& scan_matched_pose);

    bool getLastOdoPoseStamp(int64_t &odo_stamp,Eigen::Vector3f& odo_pose);

    bool recordCodeMsg(slam::BaseSlamMsgInterface_Ptr msg);

    bool fusionWithCodeMsg(slam::BaseSlamMsgInterface_Ptr msg);

    bool recordCodeMsg(const slam::tf::TransForm &pose,std::string code_id);

    bool removeCodeMsg(const slam::tf::TransForm &pose,std::string code_id);

    bool saveCodeMsg();

    bool nearFusionCode(const Eigen::Vector3f& curr_pose,const float& check_dist_thresh,std::string& device_name) const;

    void resetFusionState() const;

    bool convertDMCodeToWorld(const std::string& map_path, const std::string& map_name,
                              const Eigen::Vector3f& code_pose, const std::string code_info,
                              Eigen::Vector3f& world_pose);
    void reloadMap(const std::string map_path,const std::string map_name,const std::string layer_suffix);
};

}


#endif //SROS_NORMAL_LOCATION_SINGLETON_H
