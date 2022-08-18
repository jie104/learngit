//
// Created by lfc on 16-11-2.
//

#include <glog/logging.h>
#include "uam05lp_module.h"
#include "core/msg/ObstacleMsg.hpp"
#include "core/msg/sonar_data_msg.hpp"
#include "core/msg_bus.h"

laser::Uam05lpModule::Uam05lpModule(std::string ip_address,int ip_port):ip_address_(ip_address),ip_port_(ip_port) {
    // ip_address_ = "192.168.23.100";
    // ip_port_ = 10940;
//    publish_intensity = true;
    publish_multiecho = false;
    LOG(INFO) << "Uam05lpModule start";
}

laser::Uam05lpModule::~Uam05lpModule() {
    doStop();
    doClose();
}

bool laser::Uam05lpModule::serialOpen(){
    try {
        bool publish_intensity = config.intensity;
        serial_laser_.reset(new urg_node::URGCWrapper(9600, config.port, publish_intensity, publish_multiecho));

        LOG(INFO) << "Device opened successfully. serial";
        return true;

    } catch (hokuyo::Exception &e) {
        LOG(ERROR) << "can't open laser: serial" << e.what();
        sleep(1); // wait for 1 seconds
        // FIXME 消除递归调用
        //return doOpen();
        return false;
    }
}

bool laser::Uam05lpModule::doOpen() {
    bool is_open = false;
    int count = 0;
    while (!is_open) {
        count++;
        if (count > 3) {
            LOG(INFO) << "error to open the uam05lp!";
            return false;
        }
        try {
            bool publish_intensity = config.intensity;
            laser_.reset(new urg_node::URGCWrapper(ip_address_, ip_port_, publish_intensity, publish_multiecho));
            LOG(INFO) << "Open laser successfully: " << laser_->getDeviceID();
            LOG(INFO) << "getSerialBaud: " << laser_->getSerialBaud();
            LOG(INFO) << "getVendorName: " << laser_->getVendorName();
            LOG(INFO) << "getProductName: " << laser_->getProductName();
            LOG(INFO) << "getFirmwareVersion: " << laser_->getFirmwareVersion();
            LOG(INFO) << "getProtocolVersion: " << laser_->getProtocolVersion();
            LOG(INFO) << "getUserTimeOffset: " << laser_->getUserTimeOffset();
            LOG(INFO) << "getSensorStatus: " << laser_->getSensorStatus();
            LOG(INFO) << "getSensorState: " << laser_->getSensorState();
            
            std::stringstream ss;
            ss << "Connected to";
            if (publish_multiecho) {
                ss << " multiecho";
            }
            if (ip_address_ != "") {
                ss << " network";
            } else {
                ss << " serial";
            }
            ss << " device with";
            if (publish_intensity) {
                ss << " intensity and";
            }
            ss << " ID: " << laser_->getDeviceID();
            LOG(INFO) << ss.str();
            if (config.calibrate_time) {
                LOG(INFO) << "Starting calibration.";
                uint64_t latency = laser_->computeLatency(10);
                LOG(INFO) << "Claibration finished. Latency is " << latency;
            } else {
                laser_->clearLatency();
            }

            //serialOpen();

        } catch (std::runtime_error &e) {
            LOG(ERROR) << "Laser return abnormal status";
            LOG(INFO) << "Error connecting to Hokuyo" << e.what();
            usleep(1e6);
            continue;
        } catch (...) {
            LOG(ERROR) << "Laser return abnormal status";
            LOG(INFO) << "Unknown error connecting to Hokuyo";
            usleep(1e6);
            continue;
        }

        LOG(INFO) << "Device opened successfully.";
        is_open = true;
        return true;
    }
    return false;
}

void laser::Uam05lpModule::doStart() {
    bool is_start = false;
    double min_angle=config.min_ang;
    double max_angle=config.max_ang;
    laser_->setAngleLimitsAndCluster(min_angle,max_angle,config.cluster);
    laser_->setSkip(config.skip);
    while (!is_start) {
        try {
            laser_->start();
            if (!laser_->isStarted()) {
                LOG(INFO) << "Failed to request scans from device.  Status: " << laser_->isStarted();
                break;
            }
            LOG(INFO) << "Streaming data.";
            is_start = true;
        } catch (std::runtime_error &e) {
            LOG(ERROR) << "Error starting Hokuyo: %s" << e.what();
            laser_->stop();
            doOpen();
            usleep(1e6);
            continue; // Return to top of main loop
        } catch (...) {
            LOG(ERROR) << "Unknown error starting Hokuyo";
            laser_->stop();
            usleep(1e6);
            doOpen();
            continue; // Return to top of main loop
        }
    }

}

void laser::Uam05lpModule::doStop() {

}

void laser::Uam05lpModule::doClose() {
    try {
        laser_->stop();
    } catch (...) {
        return;
    }

}

static int64_t convertNTPtoUs(const std::uint64_t &ntp_time){
    std::uint64_t hsw = (ntp_time >> 32) * 1e6;
    std::uint64_t lsw = ((ntp_time << 32) >> 32) * 1.0e6 / 4294967296;
    return (int64_t)(double)(std::uint64_t)(hsw + lsw);
}

bool laser::Uam05lpModule::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
    try{
        if (laser_->grabScan(scan_ptr)) {
            return true;
        }else {
            doClose();
            if(!doOpen()) {
                throw std::runtime_error("error to open uam05lp lidar!");
            }
            doStart();
            return false;
        }
    }catch (...) {
        LOG(ERROR) << "Unknown error grabbing Hokuyo scan.";
        doClose();
        if(!doOpen()) {
            throw std::runtime_error("error to open uam05lp lidar!");
        }
        doStart();
        return false;
    }
}
