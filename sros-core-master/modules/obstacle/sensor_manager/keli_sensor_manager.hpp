//
// Created by lfc on 2020/9/2.
//

#ifndef SROS_KELI_SENSOR_MANAGER_HPP
#define SROS_KELI_SENSOR_MANAGER_HPP
#include <memory>
#include <string>
#include <sys/types.h>  /* basic system data types */
#include <sys/socket.h> /* basic socket definitions */
#include <netinet/in.h>
#include <arpa/inet.h>
#include "base_sensor_manager.hpp"
#include "ring_buffer.hpp"
#include "simply_connect_region.hpp"
#include "core/tf/transform_3d.hpp"
#include "modules/laser/standard_lidar_protocol/udp_scan_data_receiver.hpp"
#include "modules/laser/standard_lidar_protocol/keli_obstacle_laser_protocol.hpp"

namespace sensor {
class KeliSensorManager : public BaseSensorManager {
 public:
    KeliSensorManager(const std::string& device_name, const sros::device::DeviceID& device_id,
                         const std::string& host_name, const int& port, const slam::tf::TransForm& scan_pose,
                         float angle_max, float angle_min, bool install_direction,const bool& enable_publish_obstacle,
                         const oba::ObstacleModulePara_Ptr para, ObaMsgCallback msg)
        : host_name_(host_name), port_(port), laser_angle_max(angle_max), laser_angle_min(angle_min),
          is_right_install_direction(install_direction), BaseSensorManager(para, msg, TYPE_KELI_SENSOR) {

        enable_publish_obstacle_ = enable_publish_obstacle;
        oba_name_ = device_name;
        scan_tf_ = slam::tf::Transform3D(scan_pose);
        LOG(INFO) << "!!!!!keli sensor info:" << scan_pose.position.x() << "," << scan_pose.position.y() << ","
                  << scan_pose.position.z() << ",roll," << scan_pose.rotation.roll() << ",pitch,"
                  << scan_pose.rotation.pitch() << ",yaw," << scan_pose.rotation.yaw() << "," << laser_angle_max << ","
                  << laser_angle_min << "," << is_right_install_direction;
        keli_device_.reset(new sros::device::Device(device_name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_ETH_1, sros::device::DEVICE_MOUNT_SROS));
        sros::device::DeviceManager::getInstance()->addDevice(keli_device_);
        ok_ = true;
        boost::thread(boost::bind(&KeliSensorManager::grabScanLoop, this));
    }

    void updateSimplyRegion(std::vector<Eigen::Vector2f>& points){
        if (points.size() >= 3) {
            simply_region_.reset(new SimplyConnectRegion(points));
        }
    }

 private:
    bool openDevice(std::string host_name,int port){
        int try_count = 0;
        int try_count_thresh = 10;
        while (try_count++ < try_count_thresh) {
            try {
                KeliProtocol_.reset(new sros::KeliObstacleLaserProtocol);

                bool is_connect = false;
                while (!is_connect) {
                    data_receiver_.reset(new sros::UdpScanDataReceiver(KeliProtocol_, host_name, port));
                    is_connect = data_receiver_->isConnected();
                }
                keli_device_->setStateOK();
                keli_device_->setTimeoutTime(1000);
                LOG(INFO) << "!!!!!keli - succesfully to open it!" << host_name << "," << port;
                return true;
            }catch (std::exception& e){
                LOG(ERROR) << "err to open the lidar:" << host_name << "," << port << "," << e.what();
                LOG(INFO) << "will try more time to open it!";
                KeliProtocol_.reset();
                sleep(1);
            }
        }
        return false;
    }

    void grabScanLoop(){
        int ungrabed_count = 0;
        while (ok_) {
            sros::core::LaserScan_ptr scan(new sros::core::LaserScanMsg);
            if (data_receiver_ && data_receiver_->getScan(scan)) {
                scan->time_ = sros::core::util::get_time_in_us();
                ungrabed_count = 0;
               if (!is_right_install_direction) {
                    inverseLidarPoint(scan);
                }
                keli_device_->keepAlive();
                if (isRecording()) {
                    serialization::ScanRecord record;
                    record.range_min = scan->range_min;
                    record.range_max = scan->range_max;
                    record.angle_min = scan->angle_min;
                    record.angle_max = scan->range_max;
                    record.angle_increment = scan->angle_increment;
                    record.ranges = scan->ranges;
                    record.intensities = scan->intensities;
                    recordData(record);
                }
                sensor::filterScanTrailPoint<sros::core::LaserScan_ptr>(scan, para->min_filter_length * 2.0);
                filterScanIsolatedPoint(scan);
                processMsg(scan);
                if(sendObaMsg){
                    scan->topic_ = "KELI_SCAN";
                    scan->sensor_name = oba_name_;
                    sendObaMsg(scan);
                }
                if (simply_region_) {
                    sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("LASER_REGION_STATE"));
                    oba_msg->oba_name = oba_name_;
                    oba_msg->time_ = scan->time_;
                    convertToRegionState(scan, oba_msg);
                    if (sendObaMsg) {
                        sendObaMsg(oba_msg);
                    }
                }
            } else {
                sleep(1);
                ungrabed_count++;
                if (!KeliProtocol_ || ungrabed_count > 10) {
                    keli_device_->setStateOpenFailed();
                    LOG(INFO) << "!!!!!keli - device state failed!";
                    if (openDevice(host_name_, port_)) {
                        ungrabed_count = 0;
                        keli_device_->setStateOK();
                        LOG(INFO) << "!!!!!keli - succesfully to open Device";
                    } else {
                        LOG(INFO) << "cannot open it will retry!";
                    }
                }
            }
        }
    }

    virtual void processMsg(sros::core::LaserScan_ptr& scan_msg){
        sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("OBSTACLES"));
        oba_msg->oba_name = oba_name_;
        oba_msg->time_ = scan_msg->time_;

        if(enable_publish_obstacle_){
            slam::tf::TransForm curr_pose;
            if(!base_to_world_tf->lookUpTransForm(scan_msg->time_, curr_pose,para->delta_time_thresh)){
                //LOG(INFO)<<"!!!!!keli -  err to get the realtime msg!";
                return;
            }

            if(std::isnan(curr_pose.rotation.yaw()||!finite(curr_pose.rotation.yaw()))) {
                LOG(WARNING) << "yaw is nan value!" << curr_pose.rotation.yaw();
                return;
            }
            if(std::isnan(curr_pose.position.x()||!finite(curr_pose.position.x()))) {
                LOG(WARNING) << "x is nan value!" << curr_pose.position.x();
                return;
            }
            if(std::isnan(curr_pose.position.y()||!finite(curr_pose.position.y()))) {
                LOG(WARNING) << "y is nan value!" << curr_pose.position.y();
                return;
            }
            slam::tf::Transform3D curr_tf(curr_pose);
            slam::tf::Transform3D scan_to_world_tf;
            curr_tf.transformTF(scan_tf_, scan_to_world_tf);
            convertToObas(scan_to_world_tf, scan_msg, oba_msg);
        }

        if (sendObaMsg) {
            sendObaMsg(oba_msg);
        }else {
            LOG(INFO) << "have not set the sendMsgCallback! cannot transform the obas!";
        }
    }

    void convertToObas(slam::tf::Transform3D& world_tf,const sros::core::LaserScan_ptr &scan_msg,sros::core::ObstacleMsg_ptr& oba_msg){

        double start_angle = scan_msg->angle_min;
        double angle_incre = scan_msg->angle_increment;
        oba_msg->point_cloud.reserve(scan_msg->ranges.size());
        double max_range_min =
            scan_msg->range_min > para->oba_laser_range_min ? scan_msg->range_min : para->oba_laser_range_min;
        double min_range_max =
            scan_msg->range_max < para->oba_laser_range_max ? scan_msg->range_max : para->oba_laser_range_max;

        for (auto &range:scan_msg->ranges) {
            bool is_valid = true;
            if (start_angle > laser_angle_min && start_angle < laser_angle_max && range < min_range_max &&
                range > max_range_min) {
                Eigen::Vector3d curr_point(range * cos(start_angle), range * sin(start_angle),0.0);
                Eigen::Vector3d world_point;
                world_tf.transformPoint(curr_point, world_point);
                if (scan_tf_.point()[2] >= para->ust_detect_min_height) {
                    if (world_point[2] < para->ust_detect_min_height) {
                        is_valid = false;
                    }
                }
                if (is_valid && world_point[2] < para->ust_detect_max_height) {//最高障碍
                    sros::core::Location loc(world_point[0], world_point[1], world_point[2]);
                    oba_msg->point_cloud.push_back(loc);
                }
            }
            start_angle += angle_incre;
        }
    }

    void convertToRegionState(const sros::core::LaserScan_ptr &scan_msg,sros::core::ObstacleMsg_ptr& oba_msg){
        oba_msg->oba_state = sros::core::ObstacleMsg::STATE_OBA_FREE;
        if (!simply_region_) {
            return;
        }
        double start_angle = scan_msg->angle_min;
        double angle_incre = scan_msg->angle_increment;
        if (!is_right_install_direction) {//激光即便是倒着装，也应该按照正向进行判断
            start_angle = scan_msg->angle_max;
            angle_incre = -scan_msg->angle_increment;
        }
        oba_msg->point_cloud.reserve(scan_msg->ranges.size());
        double max_range_min =
            scan_msg->range_min > para->oba_laser_range_min ? scan_msg->range_min : para->oba_laser_range_min;
        double min_range_max =
            scan_msg->range_max < para->oba_laser_range_max ? scan_msg->range_max : para->oba_laser_range_max;

        for (auto &range:scan_msg->ranges) {
            if (start_angle > laser_angle_min && start_angle < laser_angle_max && range < min_range_max &&
                range > max_range_min) {
                Eigen::Vector2d curr_point(range * cos(start_angle), range * sin(start_angle));
                if (simply_region_->inRegion(curr_point)) {
                    oba_msg->oba_state = sros::core::ObstacleMsg::STATE_OBA_STOP_0;
                    return;
                }
            }
            start_angle += angle_incre;
        }
    }

    void inverseLidarPoint(const sros::core::LaserScan_ptr &scan) {
        auto point_size = scan->ranges.size();
        if (point_size != 0) {
            std::vector<float> ranges(point_size);
            std::vector<float> intens(point_size);
            if (scan->intensities.size() != point_size) {
                scan->intensities.resize(point_size, 100);
            }
            for (int i = 0; i < point_size; ++i) {
                ranges[i] = scan->ranges[point_size - 1 - i];
                intens[i] = scan->intensities[point_size - 1 - i];
            }
            scan->ranges.swap(ranges);
            scan->intensities.swap(intens);
        }
    }

    std::string host_name_;
    std::string oba_name_;
    const int port_;
    std::shared_ptr<sros::KeliObstacleLaserProtocol> KeliProtocol_;
    std::shared_ptr<sros::UdpScanDataReceiver> data_receiver_;
    std::shared_ptr<SimplyConnectRegion> simply_region_;
    sros::device::Device_ptr keli_device_;
    slam::tf::Transform3D scan_tf_;

    bool is_right_install_direction;
    bool ok_ = true;
    bool using_intensity = false;
    bool using_echo = false;
    double min_angle = -M_PI * 3 / 4;
    double max_angle = M_PI * 3 / 4;
    double laser_angle_max = 0;
    double laser_angle_min = 0;
    const bool intensity = true;
    const int cluster = 1;
    const int skip = 1;
};

}  // namespace sensor

#endif  // SROS_KELI_SENSOR_MANAGER_HPP
