//
// Created by lfc on 18-12-13.
//

#ifndef SROS_OBSTACLE_MODULE_H
#define SROS_OBSTACLE_MODULE_H
#include "core/core.h"
#include "core/pose.h"
#include "core/msg/gulf_avdoba_msg.hpp"
#include "obstacle_module_para.hpp"
#include "sensor_manager/base_sensor_manager.hpp"
#include "core/msg/common_command_msg.hpp"


namespace oba{
using AvdObaCommandMsgPtr = sros::core::AvdObaCommandMsgPtr;      

class ObstacleModule :public sros::core::Module{
public:
    ObstacleModule();

    virtual ~ObstacleModule();

    virtual void run();

private:
    void updatePara();

    void onLaserScanMsg(sros::core::base_msg_ptr m);

    void onR2100SensorMsg(sros::core::base_msg_ptr m);

    void onStereoCameraMsg(sros::core::base_msg_ptr m);

    void onIFM3DCameraMsg(sros::core::base_msg_ptr m);

    void onRegionControlMsg(sros::core::base_msg_ptr m);

    void switchAvoidObaState(sros::core::base_msg_ptr m);

    void enablePublishBackLaserObstacle(const sros::core::base_msg_ptr &m);

    void processPSObstacles(const std::string cmd,
                            const std::shared_ptr<sros::core::CommonCommandMsg<std::string>> msg);

    void obaMsgCallback(sros::core::base_msg_ptr m);
    
    void gulfAvdobaCmdMsg(sros::core::base_msg_ptr m);

    void updateParaLoop(sros::core::base_msg_ptr m);

    void changeParticularTofState(const std::string& disable_particular_tofs,bool clear_old_state = true);

    void changeSoftTouchTofState(const std::string& disable_particular_tofs);

    void creatUstDevice(sensor::BaseSensorManager_Ptr &sensor_manager, const std::string &device_name,
                        const sros::device::DeviceID &device_id,const std::string &prefix, const bool &enable_state);

    std::vector<int> splitStrsToInt(const std::string &s, const char seperator) {
        std::vector<int> result;
        std::string::size_type i = 0;
        std::string::size_type j = 0;
        if (s.empty()) {
            return result;
        }
        char c = s[0];
        if (c == '"')           //chip 设置的话一般带“”
            j = 1;
        //LOG(INFO) << "the s is:" << s;
        while (i < s.size()) {
            if (s[i] == seperator || i == s.size() - 1) {
                if (j != i || i == s.size() - 1) {
                    auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                    std::string item_s = s.substr(j, len);
                    if (item_s == "\"")
                        break;
                    try {
                        if (item_s.size()) {
                            auto item = stoi(item_s);
                            result.push_back(item);
                        }
                    }catch (std::exception& e){
                        LOG(INFO) << "catch exception:" << e.what() << "," << s << "," << item_s << ","
                                  << item_s.size();
                    }
                }
                j = i + 1;
            }
            i++;
        }
        return result;
    }

    ObstacleModulePara_Ptr oba_para;
    sensor::BaseSensorManager_Ptr laser_sensor_manager;
    sensor::BaseSensorManager_Ptr r2100_sensor_manager;
    sensor::BaseSensorManager_Ptr stereo_camera_manager;
    sensor::BaseSensorManager_Ptr sh100_sensor_manager;
    sensor::BaseSensorManager_Ptr eu100_tim320_sensor_manager;
    sensor::BaseSensorManager_Ptr ust_back_sensor_manager;
    sensor::BaseSensorManager_Ptr ust_left_sensor_manager;
    sensor::BaseSensorManager_Ptr ust_right_sensor_manager;
    sensor::BaseSensorManager_Ptr ust_forward_sensor_manager;
    sensor::BaseSensorManager_Ptr ifm_camera_manager;
    const double MM_TO_M = 0.001;
    const double DEG_TO_RAD = M_PI/180.0;
    std::string last_cmd_;
    sros::core::Pose cmd_position;
    bool is_enable_avd = false;
    bool enable_record_ifm_oba = false;

};

}


#endif //SROS_OBSTACLE_MODULE_H
