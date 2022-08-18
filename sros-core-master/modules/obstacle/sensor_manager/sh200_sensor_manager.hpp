//
// Created by lfc on 19-5-24.
//

#ifndef SROS_SH200_SENSOR_MANAGER_HPP
#define SROS_SH200_SENSOR_MANAGER_HPP

#include <Eigen/Dense>
#include "../rack_oba_filter_singleton.h"
#include "base_sensor_manager.hpp"
#include "core/device/can_interface.h"
#include "core/hardware/SH200.h"
#include "functional"

namespace sensor {
class Sh200SensorManager : public BaseSensorManager {
 public:
    Sh200SensorManager(oba::ObstacleModulePara_Ptr para, ObaMsgCallback msg)
        : BaseSensorManager(para, msg, TYPE_SH200_SENSOR) {
        oba_name_ = "SH200_OBA";

        sh200_tfs_.resize(para->tof_nums);//需在设备初始化前完成赋值，否则容易死机。
        tof_devices_.resize(para->tof_nums);
        for (uint32_t i = 0; i < para->tof_nums; ++i) {
            sh200_tfs_[i] = Eigen::Affine2d(Eigen::Translation2d(para->tof_coords[i][0],para->tof_coords[i][1]) *
                                            Eigen::Rotation2Dd(para->tof_coords[i][2]));
            LOG(INFO) << "tof" << i + 1 << "coord info:" << para->tof_coords[i][0] << "," << para->tof_coords[i][1]
                      << "," << para->tof_coords[i][2];
        }
        LOG(INFO) << "successfully to initialize sh200!";

        sh200_ptr_agv_.resize(para->sh100_nums);
        int sh200_device_id = 631;
        const int each_sh200_tof_num = para->each_sh100_tof_num;
        for (uint32_t j = 0; j < para->sh100_nums; ++j) {
            if (para->sh100_infos[j].sh100_enable_state) {
                int sh200_id = j + 1;
                std::string device_name = "SH200_TOF_" + std::to_string(sh200_id);
                sros::device::DeviceID device_id = (sros::device::DeviceID)(sh200_device_id + j);
                sh200_ptr_agv_[j] = sros::device::createDevice<sros::device::SH200>(
                    device_name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_CAN_1,
                    std::make_shared<sros::device::CanInterface>(para->sh100_infos[j].sh100_id));                    
                auto func = std::bind(&Sh200SensorManager::onDataRecieve, this, sh200_id, std::placeholders::_1,
                                      std::placeholders::_2, std::placeholders::_3);
                sh200_ptr_agv_[j]->setTofDataCallback(func);

                for (int i = 0; i < each_sh200_tof_num; ++i) {
                    auto tof_id = j * each_sh200_tof_num + i;
                    if (tof_id < para->tof_nums) {
                      if(para->sh100_tof_enable_states[tof_id]){
                        std::string tof_name = sros::device::DEVICE_TOF;
                        tof_name += std::to_string(tof_id + 1);//从1开始计数
                        tof_devices_[tof_id].reset(new sros::device::Device(
                            tof_name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_CAN_1,
                            sros::device::DEVICE_MOUNT_SROS));
                        tof_devices_[tof_id]->setStateOK();
                        sros::device::DeviceManager::getInstance()->addDevice(tof_devices_[tof_id]);
                      }
                    }else{
                        LOG(INFO) << "cannot current tof device:" << tof_id << ",total num:" << para->tof_nums;
                    }
                }

                //初始化完所有TOF后再初始化协议（激活设备），否则会导致数据回调使用初始化设备指针为空，从而程序崩溃
                sh200_ptr_agv_[j]->initProtocol();

                // get and set device info
                sh200_ptr_agv_[j]->setModelNo( sh200_ptr_agv_[j]->getDeviceModelStr() );
                sh200_ptr_agv_[j]->setVersionNo( sh200_ptr_agv_[j]->getDeviceVersionStr() );
                sh200_ptr_agv_[j]->setSerialNo("-");
            }
        }

    }

 private:
    // (id, distance, strength)
    void onDataRecieve(int device_id, uint8_t id, uint16_t distance, uint16_t strength) {
        if (id > para->each_sh100_tof_num) {
            LOG_EVERY_N(INFO,20) << "curr id is larger than enable_max_id!" << id << "," << para->each_sh100_tof_num;
            return;
        }
        uint32_t tof_index = (uint32_t)id + para->each_sh100_tof_num * (device_id - 1);
        if (tof_index <= 0) {
            LOG(INFO) << "err tof id:" << tof_index;
        }
        uint32_t tof_array_index = tof_index - 1;
        double tof_dist = 0.0;
        const double cm_to_m = 0.01;
        Eigen::Vector2d tof_base_coord;

        if (tof_array_index < para->tof_nums) {

            tof_dist = (double)distance * cm_to_m;
            tof_base_coord = sh200_tfs_[tof_array_index] * Eigen::Vector2d(tof_dist, 0.0);
            if (para->enable_sh100_debug_output) {
                /*LOG(INFO) << "device id:" << device_id << ","
                          << "tof id:" << (uint32_t)id << "all id:" << tof_index
                          << ",tf info:" << para->tof_coords[tof_array_index].x() << ","
                          << para->tof_coords[tof_array_index].y() << "," << para->tof_coords[tof_array_index].z()
                          << ",distance:" << tof_dist << ",strength:" << strength;*/
            }
        } else {
            LOG(INFO) << "tof id is wrong!" << tof_index << "," << (uint32_t)id;
            return;
        }
        if(para->sh100_tof_enable_states[tof_array_index]){
            tof_devices_[tof_array_index]->keepAlive();//喂狗
        }else{
            return;
        }

        sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("OBSTACLES"));
        oba_msg->oba_name = oba_name_ + std::to_string(tof_index);
        int64_t curr_time = sros::core::util::get_time_in_us();
        oba_msg->time_ = curr_time;
        if (tof_dist < para->near_tof_dist_with_high_strength && strength < para->near_tof_dist_strength) {
            if(para->enable_sh100_debug_output){
                /*LOG(INFO) << "thresh out! the id:" << tof_index << ",tf info:" << para->tof_coords[tof_array_index][0] << ","
                          << para->tof_coords[tof_array_index][1] << "," << para->tof_coords[tof_array_index][2] << ",distance:" << tof_dist
                          << "," << strength;*/
            }
        } else if (strength > 200 && strength < 65534 && tof_dist > 0.02 && tof_dist < 5.0) {
            bool tf_state =
                para->sh100_tof_enable_states[tof_array_index] & para->online_sh100_tof_enable_states[tof_array_index];
            if (tf_state) {
                if (para->soft_touch_tof_states[tof_array_index]) {
                    oba_msg->is_region_oba = true;
                    oba_msg->oba_state = sros::core::ObstacleMsg::STATE_OBA_FREE;
                    if (fabs(para->tof_coords[tof_array_index][0]) > tof_dist) {
                        oba_msg->oba_state = (sros::core::ObstacleMsg::ObaState)para->eu100_tim320_stop_state;
                    }
                    if (para->enable_sh100_debug_output) {
                        /*LOG(INFO) << "soft touch state!" << oba_msg->oba_state << "," << tof_array_index << ","
                                  << tof_dist;*/
                    }
                } else {
                    Eigen::Vector2d tof_world_coord;
                    if (!convertToWorldPoint(curr_time, tof_base_coord, tof_world_coord)) {
                        return;
                    }
                    if (para->is_load_full_state) {
                        if (oba::RackObaFilterSingleton::getInstance()->isRackPoint(tof_base_coord)) {
                            return;
                        }
                    }
                    oba_msg->point_cloud.emplace_back();
                    oba_msg->point_cloud.back() = sros::core::Location(tof_world_coord[0], tof_world_coord[1]);
                }
            }
        }
        if (sendObaMsg) {
            if (para->enable_sh100_debug_output) {
                /*LOG(INFO) << "tof name:" << oba_msg->oba_name << "," << oba_msg->point_cloud.size() << ","
                          << tof_base_coord[0] << "," << tof_base_coord[1];*/
            }
            sendObaMsg(oba_msg);
        }
        //        LOG(INFO) << "id:" << (int)id << ",distance:" << distance << ",strength:" << strength;
    }

    bool convertToWorldPoint(int64_t curr_time, const Eigen::Vector2d &base_coord, Eigen::Vector2d &tof_world_coord) {
        slam::tf::TransForm curr_pose;
        if (!base_to_world_tf->lookUpTransForm(curr_time, curr_pose, para->delta_time_thresh)) {
            //LOG(INFO) << "oba err to get the realtime msg!";
            return false;
        }
        if (std::isnan(curr_pose.rotation.yaw() || !finite(curr_pose.rotation.yaw()))) {
            LOG(WARNING) << "yaw is nan value!" << curr_pose.rotation.yaw();
            return false;
        }
        if (std::isnan(curr_pose.position.x() || !finite(curr_pose.position.x()))) {
            LOG(WARNING) << "x is nan value!" << curr_pose.position.x();
            return false;
        }
        if (std::isnan(curr_pose.position.y() || !finite(curr_pose.position.y()))) {
            LOG(WARNING) << "y is nan value!" << curr_pose.position.y();
            return false;
        }

        Eigen::Affine2d world_tf = Eigen::Translation2d(curr_pose.position.x(), curr_pose.position.y()) *
                                   Eigen::Rotation2Dd(curr_pose.rotation.yaw());
        tof_world_coord = world_tf * base_coord;
        return true;
    }

    std::vector<Eigen::Affine2d> sh200_tfs_;
    std::vector<sros::device::Device_ptr> tof_devices_;
    std::string oba_name_;
    std::vector<sros::device::SH200_ptr> sh200_ptr_agv_;
};

}  // namespace sensor

#endif  // SROS_SH100_SENSOR_MANAGER_HPP
