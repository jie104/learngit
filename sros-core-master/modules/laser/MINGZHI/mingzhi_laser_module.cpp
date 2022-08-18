//
// Created by jin on 2020/4/16.
//
#include "mingzhi_laser_module.h"
#include "glog/logging.h"
namespace laser {
    MingzhiLaserModule::MingzhiLaserModule() : scan_seq_(0), connect_status_(false), get_config_(true),
                                               scanParamInitialized_(false) {
        angle_min_ = -2.356;
        angle_max_ = 2.356;
        scan_range_min_ = 0.0;
        scan_range_max_ = 100.0;
        frame_id_ = "laser";
        host_ip_ = "192.168.1.100";
        port_ = 2112;
    }

    MingzhiLaserModule::~MingzhiLaserModule() {

    }

    bool MingzhiLaserModule::doOpen() {
//    ROS_INFO_STREAM("Connecting to laser at " << host_ip_);
        LOG(INFO) << "Connecting to laser at " << host_ip_;
        laser.connect(host_ip_, port_);
        if (!laser.isConnected()) {
//        ROS_WARN("Unable to connect, retrying.");
            LOG(INFO) << "Unable to connect, retrying.";
//            ros::Duration(1).sleep();
            sleep(1);
            return false;
        }
        if(!heart_beat_flag_){
            heart_beat_flag_ = true;
            heart_beat_thread_ = std::thread(std::bind(&MingzhiLaserModule::sendHB2Lidar, this));//beat线程
        }
//        heart_beat_flag_ = true;
//        if(heart_beat_thread_){
//            heart_beat_thread_ = std::thread(std::bind(&MingzhiLaserModule::sendHB2Lidar, this));//beat线程
//        }
        connect_status_ = true;
//    ROS_INFO("Connected to laser.");
        LOG(INFO) << "Connected to laser.";
        return true;
    }

    void MingzhiLaserModule::doStart() {
        LOG(INFO) << "Do open!";//关于start的工作实际上在第一次getScan时进行
    }

    void MingzhiLaserModule::doStop() {
        laser.stopMeas();
        LOG(INFO) << "Module stop";
    }

    void MingzhiLaserModule::doClose() {
        heart_beat_flag_ = false;
        if(heart_beat_thread_.joinable()){
            heart_beat_thread_.join();
        }
        laser.disconnect();
        LOG(INFO) << "Module close";
    }

// Get Laser config.
    bool MingzhiLaserModule::getConfig() {
        return laser.getConfig();
    }

    void MingzhiLaserModule::initialize_scan_param() {
//        scan_msg_.header.frame_id = frame_id_;
        scan_msg_.range_min = 0;
        scan_msg_.range_max = uld_config_.nMR / 100.;
        scan_msg_.angle_min = (uld_config_.nSA[0] - 90) / (180.) * M_PI;
        scan_msg_.angle_max = (uld_config_.nSA[1] - 90) / (180.) * M_PI;

        LOG(INFO) << "scan_range_min: " << scan_msg_.range_min;
        LOG(INFO) << "scan_range_max: " << scan_msg_.range_max;
        LOG(INFO) << "scan_angle_min: " << scan_msg_.angle_min;
        LOG(INFO) << "scan_angle_max: " << scan_msg_.angle_max;
        LOG(INFO) << "lase RPM: " << uld_config_.nSAV;
        LOG(INFO) << "lase nSAP: " << uld_config_.nSAP;
        LOG(INFO) << "lase nPF: " << uld_config_.nPF;

        scanParamInitialized_ = true;
    }

// Start continous laser scan msgs
    void MingzhiLaserModule::startMesure() {
        laser.startMeas();
    }

// Send Heart Beat to laser.
    void MingzhiLaserModule::sendHB2Lidar() {
        while(heart_beat_flag_){
            laser.sendHB();
            LOG(INFO) << "Beat!";
            sleep(3);
        }
    }

    bool MingzhiLaserModule::getScanData(sros::core::LaserScan_ptr& scan) {
        LOG(INFO) << "getScanData, connect status: " << connect_status_;
        if (!connect_status_) {
            LOG(INFO) << "connect----------";
            doOpen();
        } else {
            // Send laser configuration request.
            if (get_config_) {
                getConfig();//首次需要
                LOG(INFO) << "SEND: Request laser config ...";
                get_config_ = false;
            }

            // if receive packet from tcp
//            auto before_get_time = sros::core::util::get_time_in_us();
            if (laser.packetDecodeExt(&uld_config_)) {//100ms的时间内从laser读取数据到buffer,否则返回false
                // decode laser packet.
                scan->angle_min = scan_msg_.angle_min;
                scan->angle_max = scan_msg_.angle_max;
                scan->angle_increment = scan_msg_.angle_increment;
                scan->range_min = scan_msg_.range_min;
                scan->range_max = scan_msg_.range_max;
                scan->time_increment = 1.0/(25.0*720.0);
//                auto scan_time = sros::core::util::get_time_in_us();
//                scan->time_ = before_get_time;
                if (laser.GetALim(&uld_config_, scan.get())) {
                    uint64_t sync_time = sros::core::util::get_time_in_us();//获取数据后，sros给出的时间
                    uint64_t temp1 = sync_time;//
                    LOG(INFO) << "-------sros time: " << sync_time;
                    uint64_t lidar_scan_time = scan->time_;//雷达给出的，scan结束的时间,us
                    LOG(INFO) << "-------lidar time: " << scan->time_;
                    if(lidar_scan_time < last_lidar_stamp){//getScan函数被调用的前两次，没有对scan操作，但是时间保留了接近sros的时间，因此这里需要清空避免delta=0的情况
                        LOG(ERROR) << "Lidar timestamp is wrong! Last lidar stamp: " << last_lidar_stamp << ", this lidar stamp: " << lidar_scan_time;
                        delta_timestamp_.resize(100);
                    }
                    last_lidar_stamp = lidar_scan_time;
                    if(delta_timestamp_.empty()){
                        delta_timestamp_.resize(100);
                    }
                    uint64_t delta_time = sync_time - lidar_scan_time;//us
                    LOG(INFO) << "-------delta_time: " << delta_time;
                    delta_timestamp_.push_back(delta_time);
                    uint64_t  min_delta_time = delta_timestamp_.getMinValue();
                    LOG(INFO) << "-------min_delta_time: " << min_delta_time;
                    if(min_delta_time != 0){
//                        if(std::abs(sync_time - lidar_scan_time - min_delta_time) > 1e5){
//                            LOG(INFO) << "Calibration is too large!" << std::abs(sync_time - lidar_scan_time - min_delta_time)*1e-6;
//                        }
                        sync_time = lidar_scan_time + min_delta_time;
                    }
                    scan->time_ = sync_time - scan->time_increment * scan->ranges.size() * 0.5 * 1e6;//time_increment单位是s
                    LOG(INFO) << "-------final_scan_time_ " << scan->time_;
                    uint64_t temp2 = scan->time_;
                    LOG(INFO) << "Change: " << (temp1 - temp2)*1e-3;//ms

                    if (laser.initializedLaserConfig()) {

                        // if not intialized laser config.
                        if (!scanParamInitialized_)//首次需要
                        {
                            initialize_scan_param();
                            startMesure();
                            HB_time_ = sros::core::util::get_time_in_us();
                        } else {
//                            if (scan_time - HB_time_ > 5.0) {
//                                // Send Heart beat to laser every 5 sec.
////                                sendHB2Lidar();
//                                HB_time_ = scan_time;
//                            }
                        }
                        // publish sensor_msgs/LaserScan

                        if (scan->ranges.size() == 601) {
//                            auto curr_time = sros::core::util::get_time_in_us();
//                            static int64_t last_curr_time = 0;
//                            LOG(INFO) << "delta time:" << curr_time - before_get_time << "," << curr_time - last_curr_time
//                                      << "," << scan_time - before_get_time << "dist:" << scan->range_max;
//                            last_curr_time = curr_time;
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
}