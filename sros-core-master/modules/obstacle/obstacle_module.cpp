//
// Created by lfc on 18-12-13.
//

#include "obstacle_module.h"
#include <core/src.h>
#include <core/rack/rack_operator_instance.hpp>
#include "core/settings.h"
#include "core/msg/common_msg.hpp"
#include "rack_oba_filter_singleton.h"
#include "sensor_manager/sensor_process_alg.h"
#include "sensor_manager/sh100_sensor_manager.hpp"
#include "sensor_manager/ust05la_sensor_manager.hpp"
#include "rack_oba_filter_singleton.h"
#include "sensor_manager/ust05la_sensor_manager.hpp"
#include "sensor_manager/keli_sensor_manager.hpp"
#include "core/msg/common_command_msg.hpp"
#include "sensor_manager/ifm_sensor_manager.hpp"

namespace oba {

ObstacleModule::ObstacleModule() : Module("Obstacle") {}

ObstacleModule::~ObstacleModule() {}

void ObstacleModule::run() {
    subscribeTopic("TOPIC_LASER", CALLBACK(&ObstacleModule::onLaserScanMsg));
    subscribeTopic("R2100_DATA", CALLBACK(&ObstacleModule::onR2100SensorMsg));
    subscribeTopic("IFM_ORIGIN_POINTS", CALLBACK(&ObstacleModule::onStereoCameraMsg));
    subscribeTopic("OBSTACLE_REGION_SWITCH", CALLBACK(&ObstacleModule::onRegionControlMsg));
    subscribeTopic("TIMER_50MS", CALLBACK(&ObstacleModule::updateParaLoop));
    subscribeTopic("TOPIC_BACK_LASER_ENABLE_PUBLISH", CALLBACK(&ObstacleModule::enablePublishBackLaserObstacle));
    subscribeTopic("IFM3D_IMG", CALLBACK(&ObstacleModule::onIFM3DCameraMsg));
    subscribeTopic("GULF_GOODS_AVDOBA_CMD", CALLBACK(&ObstacleModule::gulfAvdobaCmdMsg));

    updatePara();

    dispatch();
}

void ObstacleModule::updatePara() {
    using namespace sros::core;
    oba_para.reset(new ObstacleModulePara);
    auto &s = sros::core::Settings::getInstance();
    oba_para->laser_coord_x = s.getValue("posefilter.laser_coordx", 0.29);
    oba_para->laser_coord_y = s.getValue("posefilter.laser_coordy", 0.0);
    oba_para->laser_coord_yaw = s.getValue("posefilter.laser_coordyaw", 0.0);
    oba_para->laser_angle_min = s.getValue("slam.laser_angle_min", -2.1);
    oba_para->laser_angle_max = s.getValue("slam.laser_angle_max", 2.10);

    oba_para->enable_r2100_points_when_load_full =
        s.getValue<std::string>("obstacle.enable_r2100_points_when_load_full", "True") == "True";

    oba_para->r2100_coord_x = (double)(s.getValue<int>("device.r2100_install_x_offset", 0)) / 1000.0;
    oba_para->r2100_coord_y = (double)(s.getValue<int>("device.r2100_install_y_offset", 0)) / 1000.0;
    oba_para->r2100_coord_yaw = s.getValue<int>("device.r2100_install_yaw", 0.0) * sros::core::DEG_TO_RAD;
    // 过滤掉超过雷达避障范围的点，不让其加入运算，加快运算速度
    oba_para->oba_laser_range_min = s.getValue("obstacle.oba_laser_range_min", 0.1);//近距离是需要过滤的
//    oba_para->oba_laser_range_max = s.getValue("obstacle.oba_laser_range_max", 5.0);
    oba_para->stereo_camera_coord_x = s.getValue("camera.d435_install_x_offset", 390.0) * MM_TO_M;
    oba_para->stereo_camera_coord_y = s.getValue("camera.d435_install_y_offset", 0.0) * MM_TO_M;
    oba_para->stereo_camera_coord_yaw = s.getValue("camera.d435_install_yaw", 0.0) * DEG_TO_RAD;
    oba_para->car_heght = s.getValue<float>("nav.vehicle_heght", 10.0);

    oba_para->stereo_camera_2_coord_x = s.getValue("camera.d435_2_install_x_offset", 390.0) * MM_TO_M;
    oba_para->stereo_camera_2_coord_y = s.getValue("camera.d435_2_install_y_offset", 0.0) * MM_TO_M;
    oba_para->stereo_camera_2_coord_yaw = s.getValue("camera.d435_2_install_yaw", 0.0) * DEG_TO_RAD;
    // IFM camera install offset
    oba_para->ifm_camera_coord_x = s.getValue("camera.o3d303_install_x_offset", 0) * MM_TO_M;
    oba_para->ifm_camera_coord_y = s.getValue("camera.o3d303_install_y_offset", 0.0) * MM_TO_M;
    oba_para->ifm_camera_coord_yaw = s.getValue("camera.o3d303_install_yaw", 0.0) * DEG_TO_RAD ;
    

    // IFM obstacle detection range 
    std::vector<float> range3d_str_vct;
    const int decode_param_num = common_func::decodeStringParam("obstacle.obstacle_range3d", range3d_str_vct);
    if(decode_param_num == 6){
        oba_para->detect_range.min_x = range3d_str_vct[0];
        oba_para->detect_range.max_x = range3d_str_vct[1];
        oba_para->detect_range.min_y = range3d_str_vct[2];
        oba_para->detect_range.max_y = range3d_str_vct[3];
        oba_para->detect_range.min_z = range3d_str_vct[4];
        oba_para->detect_range.max_z = range3d_str_vct[5];
    }else {
        LOG(ERROR) << "obstacle_range3d param size error!";
    }

    oba_para->rack_leg_center_length = s.getValue<double>("rack.rack_leg_center_length", 1060.0) * MM_TO_M;
    oba_para->rack_leg_center_width = s.getValue<double>("rack.rack_leg_center_width", 600.0) * MM_TO_M;
    oba_para->rack_leg_diameter = s.getValue<double>("rack.rack_leg_diameter", 100.0) * MM_TO_M;
    oba_para->rack_backlash_rotate_angle = s.getValue<double>("rack.rack_leg_backlash_rotate_angle", 2.0) * DEG_TO_RAD;
    oba_para->rack_radius_offset = s.getValue<double>("obstacle.rack_region_radius_offset", 50.0) * MM_TO_M;
    oba_para->enable_remove_rack_leg = s.getValue<std::string>("obstacle.enable_remove_rack_leg", "False") == "True";
    int action_mode = g_state.getActionControllerType();
    if (action_mode != ACTION_CONTROLLER_TYPE_SRC_JACKING_ROTATE && action_mode != ACTION_CONTROLLER_TYPE_SRC_PUTTER_JACKING) {
        // 如果动作机构不是顶升货架类机构，则不启用货架腿滤除功能
        bool enable_load_rack = s.getValue<std::string>("rack.enable_load_rack", "False") == "True";
        LOG(INFO) << "src.continuous_mode isn't 4 or 6, disable rack leg filter function";
        if (enable_load_rack) {
            LOG(INFO) << "enable load rack!";
        }else{
            oba_para->enable_remove_rack_leg = false;
        }
    }
    int lidar_type = s.getValue<int>("slam.lidar_type", 3);

    if (lidar_type == 0x03 || lidar_type == 0x07||lidar_type == 0x14) {
        oba_para->enable_filter_low_intensity_points = true;
        if(lidar_type == 0x14){
            LOG(INFO) << "will larger the intensity distance! the lidar is dual lidar!";
            oba_para->min_filter_length = 1.8;
        }else{
            auto laser_oba_narrow_offset = s.getValue("obstacle.oba_laser_angle_narrow_offset", 0) * sros::core::DEG_TO_RAD;
            LOG(INFO) << "laser_oba_narrow_offset:" << laser_oba_narrow_offset;
            if (fabs(laser_oba_narrow_offset) > 40 * sros::core::DEG_TO_RAD) {
                LOG(ERROR) << "the laser oba narrow offset sets wrong! will set to zero!";
                laser_oba_narrow_offset = 0.0;
            }
            oba_para->laser_angle_min += fabs(laser_oba_narrow_offset);
            oba_para->laser_angle_max -= fabs(laser_oba_narrow_offset);
        }
    }else{
        LOG(INFO) << "will disable filter low intensity points!";
        oba_para->enable_filter_low_intensity_points = false;
    }

    oba_para->enable_filter_only_load_full =
        s.getValue<std::string>("obstacle.rack_enable_filter_only_load_full", "True") == "True";
    RackObaFilterSingleton::getInstance()->updateRackPara(oba_para);

    laser_sensor_manager = sensor::getSensorManager(oba_para, boost::bind(&ObstacleModule::obaMsgCallback, this, _1),
                                                    sensor::TYPE_LASER_SENSOR);
    r2100_sensor_manager = sensor::getSensorManager(oba_para, boost::bind(&ObstacleModule::obaMsgCallback, this, _1),
                                                    sensor::TYPE_R2100_SENSOR);
    stereo_camera_manager = sensor::getSensorManager(oba_para, boost::bind(&ObstacleModule::obaMsgCallback, this, _1),
                                                     sensor::TYPE_STEREO_CAMERA);
    ifm_camera_manager = sensor::getSensorManager(oba_para, boost::bind(&ObstacleModule::obaMsgCallback, this, _1),
                                                         sensor::TYPE_IFM_SENSOR);

    const std::string sh100_enable = "sh100.enable_sh100_";
    const std::string sh100_can = "sh100.sh100_";
    const std::string can_id = "_can_id";
    try {
        auto tof_num = s.getValue<std::string>("sh100.each_sh100_tof_num", "4tofs_each_sh100");
        if (tof_num == "4tofs_each_sh100") {
            oba_para->each_sh100_tof_num = 4;
        } else if (tof_num == "6tofs_each_sh100") {
            oba_para->each_sh100_tof_num = 6;
        }
        LOG(INFO) << "each sh100 tof num:" << oba_para->each_sh100_tof_num << "," << tof_num;
        for (int j = 0; j < oba_para->sh100_nums; ++j) {
            int sh100_id = j + 1;
            std::string enable_para_name = sh100_enable + std::to_string(sh100_id);
            oba_para->sh100_infos[j].sh100_enable_state =
                (s.getValue<std::string>(enable_para_name, "False") == "True");
            std::string sh100_can_id = sh100_can + std::to_string(sh100_id) + can_id;
            oba_para->sh100_infos[j].sh100_id = std::stoi(s.getValue<std::string>(sh100_can_id, "0x310"), nullptr, 16);
        }
    } catch (...) {
        LOG(ERROR) << "sh100 address sets wrong!";
    }
    
    LOG(INFO) << "will enable sh100 sensor manager!";
    const std::string sh100 = "sh100.";
    const std::string tof = "tof";
    const std::string coord = "_coord_";
    const std::string x = "x";
    const std::string y = "y";
    const std::string yaw = "yaw";
    const std::string enable = "enable_";
    for (int i = 0; i < oba_para->tof_nums; ++i) {
        int tof_id = i + 1;
        std::string coord_x_name = sh100 + tof + std::to_string(tof_id) + coord + x;
        std::string coord_y_name = sh100 + tof + std::to_string(tof_id) + coord + y;
        std::string coord_yaw_name = sh100 + tof + std::to_string(tof_id) + coord + yaw;
        std::string enable_tof = sh100 + enable + tof + std::to_string(tof_id);
        oba_para->tof_coords[i].x() = s.getValue(coord_x_name, 0.0) * MM_TO_M;
        oba_para->tof_coords[i].y() = s.getValue(coord_y_name, 0.0) * MM_TO_M;
        oba_para->tof_coords[i].z() = s.getValue(coord_yaw_name, 0.0) * DEG_TO_RAD;
        oba_para->sh100_tof_enable_states[i] = (s.getValue<std::string>(enable_tof, "False") == "True");
    }
    oba_para->enable_sh100_debug_output =
        (s.getValue<std::string>("debug.enable_sh100_debug_output", "False") == "True");

    //增加tof设备SH200协议兼容
    auto tof_device_model = s.getValue<std::string>("device.shxxx_tof_device_model", "SH100");
    if (tof_device_model == "SH100")
    {
        sh100_sensor_manager = sensor::getSensorManager(
            oba_para, boost::bind(&ObstacleModule::obaMsgCallback, this, _1), sensor::TYPE_SH100_SENSOR);
    }
    else if (tof_device_model == "SH200")
    {
        sh100_sensor_manager = sensor::getSensorManager(
            oba_para, boost::bind(&ObstacleModule::obaMsgCallback, this, _1), sensor::TYPE_SH200_SENSOR);
    }

    auto particular_tofs = (s.getValue<std::string>("sh100.disable_particular_tofs", "0;"));
    changeParticularTofState(particular_tofs);
    auto soft_touch_tofs = (s.getValue<std::string>("sh100.soft_touch_tofs", "0;"));
    changeSoftTouchTofState(soft_touch_tofs);

    oba_para->eu100_tim320_enable = (s.getValue<std::string>("device.enable_eu100_tim320", "False") == "True");
    if (oba_para->eu100_tim320_enable) {
        LOG(INFO) << "will enable eu100 sensor manager!";
        oba_para->eu100_tim320_stop_state = s.getValue<int>("obstacle.eu100_tim320_stop_state", 0);
        if (oba_para->eu100_tim320_stop_state < 0 || oba_para->eu100_tim320_stop_state > 4) {
            LOG(INFO) << "oba para is wrong!" << oba_para->eu100_tim320_stop_state;
            oba_para->eu100_tim320_stop_state = 0;
        }
        oba_para->enable_left_tim320 = (s.getValue<std::string>("device.enable_left_eu100_tim320", "True") == "True");
        oba_para->enable_right_tim320 = (s.getValue<std::string>("device.enable_right_eu100_tim320", "True") == "True");
        oba_para->enable_back_tim320 = (s.getValue<std::string>("device.enable_back_eu100_tim320", "False") == "True");

        oba_para->left_tim320_id =
            std::stoi(s.getValue<std::string>("device.left_eu100_tim320_id", "0x331"), nullptr, 16);
        oba_para->right_tim320_id =
            std::stoi(s.getValue<std::string>("device.right_eu100_tim320_id", "0x332"), nullptr, 16);
        oba_para->back_tim320_id =
            std::stoi(s.getValue<std::string>("device.back_eu100_tim320_id", "0x333"), nullptr, 16);
        oba_para->enable_switch_region_avoid_obstacle =
            (s.getValue<std::string>("obstacle.enable_switch_region_avoid_obstacle", "False") == "True");
        eu100_tim320_sensor_manager = sensor::getSensorManager(
            oba_para, boost::bind(&ObstacleModule::obaMsgCallback, this, _1), sensor::TYPE_EU100_TIM320_SENSOR);
    }

    oba_para->ust_detect_min_height = s.getValue("obstacle.ust_detect_min_height", 30) * MM_TO_M;
    oba_para->ust_detect_max_height = s.getValue("obstacle.ust_detect_max_height", 1000) * MM_TO_M;
    oba_para->enable_ust_back_device = (s.getValue<std::string>("device.enable_ust_back_device", "False") == "True");
    if(oba_para->enable_ust_back_device){
        creatUstDevice(ust_back_sensor_manager, sros::device::DEVICE_UST_LIDAR_BACK, sros::device::DEVICE_ID_UST05_1,
                       "ust_back", oba_para->enable_ust_back_online);
    }
    oba_para->enable_ust_left_device = (s.getValue<std::string>("device.enable_ust_left_device", "False") == "True");
    if(oba_para->enable_ust_left_device){
        creatUstDevice(ust_left_sensor_manager, sros::device::DEVICE_UST_LIDAR_LEFT, sros::device::DEVICE_ID_UST05_2,
                       "ust_left", oba_para->enable_ust_left_online);
    }
    oba_para->enable_ust_right_device = (s.getValue<std::string>("device.enable_ust_right_device", "False") == "True");
    if(oba_para->enable_ust_right_device){
        creatUstDevice(ust_right_sensor_manager, sros::device::DEVICE_UST_LIDAR_RIGHT, sros::device::DEVICE_ID_UST05_3,
                       "ust_right", oba_para->enable_ust_right_online);
    }
    oba_para->enable_ust_forward_device = (s.getValue<std::string>("device.enable_ust_forward_device", "False") == "True");
    if (oba_para->enable_ust_forward_device) {
        creatUstDevice(ust_forward_sensor_manager, sros::device::DEVICE_UST_LIDAR_FORWARD, sros::device::DEVICE_ID_UST05_3,
                       "ust_forward", oba_para->enable_ust_forward_online);
    }
}

void ObstacleModule::onLaserScanMsg(sros::core::base_msg_ptr m) {
    if (laser_sensor_manager) {
        laser_sensor_manager->processMsg(m);
    }
}

void ObstacleModule::onR2100SensorMsg(sros::core::base_msg_ptr m) {
    if (r2100_sensor_manager) {
        r2100_sensor_manager->processMsg(m);
    }
}

void ObstacleModule::obaMsgCallback(sros::core::base_msg_ptr m) { sendMsg(m); }

void ObstacleModule::onStereoCameraMsg(sros::core::base_msg_ptr m) { stereo_camera_manager->processMsg(m); }

void ObstacleModule::onIFM3DCameraMsg(sros::core::base_msg_ptr m) { 
    if (ifm_camera_manager && is_enable_avd){
        ifm_camera_manager->processMsg(m); 
    }
}
void ObstacleModule::enablePublishBackLaserObstacle(const sros::core::base_msg_ptr &m) {
    auto& s = sros::core::Settings::getInstance();
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);

    auto check_and_enable_laser_obstacle = [&](const std::string& param, const std::string& device_name,
                                               const sensor::BaseSensorManager_Ptr& sensor_manager) {
        auto function_enable = (s.getValue<std::string>(param, "False") == "True");
        if (function_enable) {
            // 根据m->flag设置是否对外发布后向激光的障碍物
            if (msg->flag) {
                LOG(INFO) << device_name << " obstacle function => enable";
                sensor_manager->enable();
            } else {
                LOG(INFO) << device_name << " obstacle function => disable";
                if(sensor_manager!= nullptr){
                    sensor_manager->disable();
                }
            }
        } else {
            LOG(WARNING) << "fail: " << param << "=false";
        }
    };

    if (sros::device::DEVICE_UST_LIDAR_BACK == msg->str_0_) {
        auto enable_ust_back_device = (s.getValue<std::string>("device.enable_ust_back_device", "False") == "True");
        if (enable_ust_back_device) {
            check_and_enable_laser_obstacle("obstacle.enable_ust_back_online", sros::device::DEVICE_UST_LIDAR_BACK,
                                            ust_back_sensor_manager);
        } else {
            LOG(INFO) << "enable_ust_back_device=false " <<  sros::device::DEVICE_UST_LIDAR_BACK << " obstacle function => disable";
            if(ust_back_sensor_manager!= nullptr){
                ust_back_sensor_manager->disable();
            }
        }
    } else if (sros::device::DEVICE_LIDAR == msg->str_0_) {
        check_and_enable_laser_obstacle("obstacle.enable_r2000_obs_online", sros::device::DEVICE_LIDAR,
                                        laser_sensor_manager);
    } else {
        LOG(WARNING) << "Unknown message received str_0_=" << msg->str_0_;
        return;
    }
}

void ObstacleModule::updateParaLoop(sros::core::base_msg_ptr m) {
    auto changeDevicePublishState = [&](sensor::BaseSensorManager_Ptr& sensor_manager,bool enable_online_state){
      if(sensor_manager){
          if (enable_online_state != sensor_manager->enableState()) {
              if(enable_online_state){
                  sensor_manager->enableState();
              }else{
                  sensor_manager->disable();
              }
          }
      }
    };

    if (oba_para) {
        auto &s = sros::core::Settings::getInstance();
        oba_para->use_stereo_points = (s.getValue<std::string>("obstacle.use_stereo_points", "True") == "True");
        oba_para->enable_sh100_debug_output = (s.getValue<std::string>("debug.enable_sh100_debug_output", "False") == "True");
        if (oba_para->enable_sh100_debug_output) {
            /*const std::string sh100 = "sh100.";
            const std::string tof = "tof";
            const std::string coord = "_coord_";
            const std::string x = "x";
            const std::string y = "y";
            const std::string yaw = "yaw";
            const std::string enable = "enable_";
            for (int i = 0; i < oba_para->tof_nums; ++i) {
                int tof_id = i + 1;
                std::string coord_x_name = sh100 + tof + std::to_string(tof_id) + coord + x;
                std::string coord_y_name = sh100 + tof + std::to_string(tof_id) + coord + y;
                std::string coord_yaw_name = sh100 + tof + std::to_string(tof_id) + coord + yaw;
                std::string enable_tof = sh100 + enable + tof + std::to_string(i + 1);
                oba_para->tof_coords[i].x() = s.getValue(coord_x_name, 0.0) * MM_TO_M;
                oba_para->tof_coords[i].y() = s.getValue(coord_y_name, 0.0) * MM_TO_M;
                oba_para->tof_coords[i].z() = s.getValue(coord_yaw_name, 0.0) * DEG_TO_RAD;
                oba_para->sh100_tof_enable_states[i] = (s.getValue<std::string>(enable_tof, "False") == "True");
            }*/
        }
        oba_para->oba_laser_range_min = s.getValue("obstacle.oba_laser_range_min", 0.1);//近距离是需要过滤的
        oba_para->enable_ust_left_online = (s.getValue<std::string>("obstacle.enable_ust_left_online", "True") == "True");
        changeDevicePublishState(ust_left_sensor_manager, oba_para->enable_ust_left_online);
        oba_para->enable_ust_back_online = (s.getValue<std::string>("obstacle.enable_ust_back_online", "True") == "True");
        changeDevicePublishState(ust_back_sensor_manager, oba_para->enable_ust_back_online);
        oba_para->enable_ust_right_online = (s.getValue<std::string>("obstacle.enable_ust_right_online", "True") == "True");
        changeDevicePublishState(ust_right_sensor_manager, oba_para->enable_ust_right_online);
        oba_para->enable_r2000_obs_online = (s.getValue<std::string>("obstacle.enable_r2000_obs_online", "Ture") == "True");
        changeDevicePublishState(laser_sensor_manager, oba_para->enable_r2000_obs_online);
        auto particular_tofs = (s.getValue<std::string>("sh100.disable_particular_tofs", "0;"));
        changeParticularTofState(particular_tofs);

        auto soft_touch_tofs = (s.getValue<std::string>("sh100.soft_touch_tofs", "0;"));
        changeSoftTouchTofState(soft_touch_tofs);
        // once recieved command, change target position
        
        ifm_camera_manager->set_target_position(cmd_position);

//        oba_para->enable_r2100_points_when_load_full =
//            s.getValue<std::string>("obstacle.enable_r2100_points_when_load_full", "True") == "True";
        oba_para->near_tof_dist_with_high_strength =
            s.getValue("sh100.near_tof_dist_with_high_strength", 500.0) * MM_TO_M;
        oba_para->near_tof_dist_strength = s.getValue("sh100.near_tof_dist_strength", 500.0);

        if (oba_para->enable_remove_rack_leg &&
            (!oba_para->enable_filter_only_load_full || g_state.load_state == sros::core::LOAD_FULL)) {
            oba_para->enable_r2100_points_when_load_full =
                s.getValue<std::string>("obstacle.enable_r2100_points_when_load_full", "True") == "True";
                auto particular_tofs = (s.getValue<std::string>("sh100.disable_load_full_particular_tofs", "0;"));
                changeParticularTofState(particular_tofs, false);//若空闲状态为disable,那么顶升也必然是disable
                oba_para->is_load_full_state = true;
                double rotate_value = 0.0;  //货架相对于小车的旋转角度,逆时针为正
                rotate_value = (double)g_state.rotate_value / 1000.0;
                auto rack_filter_singleton = RackObaFilterSingleton::getInstance();
                rack_filter_singleton->computeRackDirection(rotate_value);

                double rack_length = oba_para->rack_leg_center_length;
                double rack_width = oba_para->rack_leg_center_width;
                double rack_diameter = oba_para->rack_leg_diameter;
                auto rack_op = rack::RackOperatorInstance::getInstance();
                if (rack_op) {
                    auto rack_info = rack_op->getRackInfo();
                    if (rack_info) {
                        rack_length = rack_info->rack_para.rack_length;
                        rack_width = rack_info->rack_para.rack_width;
                        rack_diameter = rack_info->rack_para.rack_leg_diameter;
                    }
                }

                if (rack_length != oba_para->rack_leg_center_length || rack_width != oba_para->rack_leg_center_width ||
                    rack_diameter != oba_para->rack_leg_diameter) {
                    LOG(INFO) << "will recompute rack size!";
                    oba_para->rack_leg_center_length = rack_length;
                    oba_para->rack_leg_center_width = rack_width;
                    oba_para->rack_leg_diameter = rack_diameter;
                    rack_filter_singleton->updateRackPara(oba_para);
                }

        } else {
            oba_para->is_load_full_state = false;
        }

        bool enable_record_ust05_left = (s.getValue<std::string>("debug.enable_record_ust05_left_data", "False") == "True");
        if (enable_record_ust05_left) {
            s.setValue("debug.enable_record_ust05_left_data", "False");
            if (ust_left_sensor_manager) {
                ust_left_sensor_manager->startRecord();
            }
        }
        bool enable_record_ust05_right = (s.getValue<std::string>("debug.enable_record_ust05_right_data", "False") == "True");
        if (enable_record_ust05_right) {
            s.setValue("debug.enable_record_ust05_right_data", "False");
            if (ust_right_sensor_manager) {
                ust_right_sensor_manager->startRecord();
            }
        }
        bool enable_record_ust05_back = (s.getValue<std::string>("debug.enable_record_ust05_back_data", "False") == "True");
        if (enable_record_ust05_back) {
            s.setValue("debug.enable_record_ust05_back_data", "False");
            if (ust_back_sensor_manager) {
                ust_back_sensor_manager->startRecord();
            }
        }
    }
}
void ObstacleModule::onRegionControlMsg(sros::core::base_msg_ptr m) {
    if (eu100_tim320_sensor_manager) {
        if (oba_para->enable_switch_region_avoid_obstacle) {
            eu100_tim320_sensor_manager->processMsg(m);
        }else{

        }
    }else{
        switchAvoidObaState(m);
    }
}

void ObstacleModule::changeParticularTofState(const std::string& disable_particular_tofs,bool clear_old_state) {
    auto size = oba_para->online_sh100_tof_enable_states.size();
    if (clear_old_state) {
        for (int i = 0; i < size; ++i) {
            oba_para->online_sh100_tof_enable_states[i] = true;//先清空旧的状态,否则,一旦该值设置为false一次,将无法自动恢复成true
        }
    }
    auto particular_tofs = splitStrsToInt(disable_particular_tofs,';');

    for (auto& tof : particular_tofs) {
        if (tof > 0 && tof <= size) {
            oba_para->online_sh100_tof_enable_states[tof - 1] = false;
        }
    }
}

bool convertStringToTF(const std::string &string_data, slam::tf::TransForm &tf) {
    const double MM_TO_M = 0.001;
    const double DEG_TO_RAD = M_PI/180.0;
    std::stringstream string_data_stream(string_data);
    std::string item;
    std::vector<double> data_array;
    try {
        while (std::getline(string_data_stream, item, ';')) {
            data_array.push_back(std::stod(item));
        }
    } catch (...) {
        LOG(INFO) << "cannot get quat! will return normal quat!";
    }
    if (data_array.size() == 6) {
        LOG(INFO) << "get right size array!";
        tf.position.x() = data_array[0] * MM_TO_M;
        tf.position.y() = data_array[1] * MM_TO_M;
        tf.position.z() = data_array[2] * MM_TO_M;
        tf.rotation.roll() = data_array[3] * DEG_TO_RAD;
        tf.rotation.pitch() = data_array[4] * DEG_TO_RAD;
        tf.rotation.yaw() = data_array[5] * DEG_TO_RAD;
        return true;
    } else {
        LOG(INFO) << "cannot get right size! will set normal quat!" << data_array.size();
        return false;
    }
}

bool convertStringToPoint(const std::string &string_data, std::vector<Eigen::Vector2f>& data_array) {
    const double MM_TO_M = 0.001;
    const double DEG_TO_RAD = M_PI/180.0;
    std::stringstream string_data_stream(string_data);
    std::string item;
    try {
        while (std::getline(string_data_stream, item, ';')) {
            std::stringstream point_stream(item);
            std::vector<double> point_array;
            std::string value;
            while (std::getline(point_stream, value, ',')){
                point_array.push_back(std::stod(value)*MM_TO_M);
            }
            if (point_array.size() == 2) {
                data_array.push_back(Eigen::Vector2f(point_array[0], point_array[1]));
            }else{
                LOG(INFO) << "wrong point array size! please check!" << point_array.size();
                for (auto& point : point_array) {
                    LOG(INFO) << "value:" << point;
                }
            }
        }
    } catch (...) {
        LOG(INFO) << "cannot get quat! will return normal quat!";
    }
    if (data_array.size() >= 3) {
        LOG(INFO) << "get right size array!" << data_array.size();
        for (auto& point : data_array) {
            LOG(INFO) << "point:" << point[0] << "," << point[1];
        }
        return true;
    } else {
        LOG(INFO) << "cannot get right size! will set normal quat!" << data_array.size();
        data_array.clear();
        return false;
    }
}

void ObstacleModule::creatUstDevice(sensor::BaseSensorManager_Ptr& sensor_manager,const std::string &device_name, const sros::device::DeviceID& device_id,const std::string& ust_prefix,const bool& enable_state) {
    auto&s = sros::core::Settings::getInstance();
    std::string obstacle_prefix = "obstacle.";
    std::string device_prefix = "device.";
    std::string coord_x = "coord_x";
    std::string coord_y = "coord_y";
    std::string coord_yaw = "coord_yaw";
    std::string angle_max = "angle_max";
    std::string angle_min = "angle_min";
    std::string host_name = "ip_address";
    std::string install_direction = "install_direction";
    std::string port = "port";
    std::string coord_info = "coord_info";
    std::string laser_region_info = "laser_region_info";
    auto ip_address = s.getValue<std::string>(device_prefix + ust_prefix + "_" + host_name, "192.168.23.110");
    int port_number = s.getValue<int>(device_prefix + ust_prefix + "_" + port, 2112);
    auto ust_coord_x = s.getValue<float>(obstacle_prefix + ust_prefix + "_" + coord_x, 0) * MM_TO_M;
    auto ust_coord_y = s.getValue<float>(obstacle_prefix + ust_prefix + "_" + coord_y, 0) * MM_TO_M;
    auto ust_coord_yaw = s.getValue<float>(obstacle_prefix + ust_prefix + "_" + coord_yaw, 0) * DEG_TO_RAD;
    auto ust_laser_angle_max = s.getValue<float>(obstacle_prefix + ust_prefix + "_" + angle_max, 0) * DEG_TO_RAD;
    auto ust_laser_angle_min = s.getValue<float>(obstacle_prefix + ust_prefix + "_" + angle_min, 0) * DEG_TO_RAD;
    slam::tf::TransForm scan_coord;
    std::string tf_data,region_data;
    tf_data = s.getValue<std::string>(obstacle_prefix + ust_prefix + "_" + coord_info, "0;");
    if (!convertStringToTF(tf_data, scan_coord)) {
        LOG(INFO) << "will use 2D coord tf!";
        scan_coord.position.x() = ust_coord_x;
        scan_coord.position.y() = ust_coord_y;
        scan_coord.position.z() = 0.0;
        scan_coord.rotation.yaw() = ust_coord_yaw;
        scan_coord.rotation.roll() = 0.0;
        scan_coord.rotation.pitch() = 0.0;
    }
    region_data = s.getValue<std::string>(obstacle_prefix + ust_prefix + "_" + laser_region_info, "0;");

    bool is_right_install_direction =
        (s.getValue<std::string>(obstacle_prefix + ust_prefix + "_" + install_direction, "True") == "True");

    std::string radar_type =s.getValue<std::string>("device.obstacle_radar_type", "UST");
    if(radar_type == "KELI"){
        std::shared_ptr<sensor::KeliSensorManager> keli_manager;
        keli_manager.reset(new sensor::KeliSensorManager(
            device_name, device_id, ip_address, port_number, scan_coord, ust_laser_angle_max, ust_laser_angle_min,
            is_right_install_direction, enable_state, oba_para, boost::bind(&ObstacleModule::obaMsgCallback, this, _1)));
        
        std::vector<Eigen::Vector2f> region_points;
        if (convertStringToPoint(region_data, region_points)) {
            keli_manager->updateSimplyRegion(region_points);
        }
        sensor_manager = keli_manager;
        LOG(INFO) << "USE KELI LASER!";
    }else{
        std::shared_ptr<sensor::Ust05laSensorManager> ust_manager;
        ust_manager.reset(new sensor::Ust05laSensorManager(
            device_name, device_id, ip_address, port_number, scan_coord, ust_laser_angle_max, ust_laser_angle_min,
            is_right_install_direction, enable_state, oba_para, boost::bind(&ObstacleModule::obaMsgCallback, this, _1)));

        std::vector<Eigen::Vector2f> region_points;
        if (convertStringToPoint(region_data, region_points)) {
            ust_manager->updateSimplyRegion(region_points);
        }
        sensor_manager = ust_manager;
        LOG(INFO) << "USE UST LASER!";
    }
}

void ObstacleModule::switchAvoidObaState(sros::core::base_msg_ptr m) {
    std::shared_ptr<sros::core::CommonCommandMsg<std::string>> region_cmd_msg =
        std::dynamic_pointer_cast<sros::core::CommonCommandMsg<std::string>>(m);
    const auto& msg_cmd = region_cmd_msg->command;
    std::string cmd;
    if (msg_cmd == "LINE FORWARD") {
        if (region_cmd_msg->param0 == 1) {
            cmd = "GO STRAIGHT FULL";
        } else {
            cmd = "GO STRAIGHT EMPTY";
        }
    } else if (msg_cmd == "LINE BACKWARD") {
        if (region_cmd_msg->param0 == 1) {
            cmd = "GO BACK FULL";
        } else {
            cmd = "GO BACK EMPTY";
        }
    } else if (msg_cmd == "TURN RIGHT FORWARD") {
        cmd = msg_cmd;
    } else if (msg_cmd == "TURN LEFT FORWARD") {
        cmd = msg_cmd;
    } else if (msg_cmd == "TURN RIGHT BACKWARD") {
        cmd = msg_cmd;
    } else if (msg_cmd == "TURN LEFT BACKWARD") {
        cmd = msg_cmd;
    } else if (msg_cmd == "ROTATE RIGHT") {
        cmd = "ROTATE RIGHT";
    } else if (msg_cmd == "ROTATE LEFT") {
        cmd = "ROTATE LEFT";
    } else if (msg_cmd == "DEFAULT") {
        cmd = "DEFAULT";
    }
    if (region_cmd_msg->param1 == 1) {
        //LOG(INFO) << "进叉";
        cmd = "ENTER FORK";
    } else if (region_cmd_msg->param1 == 2) {
        //LOG(INFO) << "退叉";
        cmd = "BACK FORK";
    }
    if (oba_para->enable_photoelectric_switch_avoid_obstacle) {
        processPSObstacles(cmd, region_cmd_msg);
    }
}

//光电
void ObstacleModule::processPSObstacles(const std::string cmd,
                                        const std::shared_ptr<sros::core::CommonCommandMsg<std::string>> msg) {
    sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("OBSTACLES"));
    oba_msg->oba_name = "PS_OBA";
    oba_msg->time_ = sros::core::util::get_time_in_us();
    oba_msg->is_region_oba = true;
    oba_msg->oba_state = sros::core::ObstacleMsg::STATE_OBA_FREE;
    if ((cmd == "ENTER FORK") || (cmd == "TURN RIGHT BACKWARD") || (cmd == "TURN LEFT BACKWARD") ||
        (cmd == "GO BACK FULL") || (cmd == "GO BACK EMPTY") || (cmd == "BACK FORK")) {
        auto enable_ps_switch = (sros::core::Settings::getInstance().getValue<std::string>(
                                     "obstacle.enable_photoelectric_switch_obstacle", "False") == "True");
        if (enable_ps_switch || g_state.photoelectric_switch_on_action) {
            bool left_on = false;
            bool right_on = false;
            if (msg->str0 == "PS LEFT ON") {
                left_on = true;
            }
            if (msg->str1 == "PS RIGHT ON") {
                right_on = true;
            }
            if (left_on | right_on) {
                oba_msg->oba_state = sros::core::ObstacleMsg::ObaState(oba_para->eu100_tim320_stop_state);
            }
        }
        // LOG(INFO) << "oba_msg->oba_state ="<<  oba_msg->oba_state;
    }
    sendMsg(oba_msg);
}

void ObstacleModule::changeSoftTouchTofState(const std::string& disable_particular_tofs) {
    auto soft_states = splitStrsToInt(disable_particular_tofs, ';');
    auto size = oba_para->soft_touch_tof_states.size();
    for (auto& soft_touch : soft_states) {
        if (soft_touch <= size && soft_touch > 0) {
            oba_para->soft_touch_tof_states[soft_touch - 1] = true;
        }
    }
}

void ObstacleModule::gulfAvdobaCmdMsg(sros::core::base_msg_ptr m){

    AvdObaCommandMsgPtr cmd = std::dynamic_pointer_cast<sros::core::AvdObaCommandMsg>(m);
    
    cmd_position = cmd->command.goal_position;

    is_enable_avd = cmd->command.is_enable;

}
}
