//
// Created by lfc on 16-11-3.
//

#include "ust10lx_module.h"

laser::Ust10lxModule::Ust10lxModule() {
    ip_address = "192.168.23.100";
    ip_port = 10940;
//    publish_intensity = true;
    publish_multiecho = false;
//    calibrate_time = true;
}

laser::Ust10lxModule::~Ust10lxModule() {
    doStop();
    doClose();
}

bool laser::Ust10lxModule::doOpen() {
    bool is_open = false;
//    ip_address = "192.168.1.210";
//    ip_port = 10940;
    int count = 0;
    while (!is_open) {
        count++;
        if (count > 3) {
            LOG(INFO) << "error to open the UST!";
            return false;
        }
        try {
            bool publish_intensity = config.intensity;
            laser_.reset(new urg_node::URGCWrapper(ip_address, ip_port, publish_intensity, publish_multiecho));
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
            if (ip_address != "") {
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

void laser::Ust10lxModule::doStart() {
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

void laser::Ust10lxModule::doStop() {

}

void laser::Ust10lxModule::doClose() {
    try {
        laser_->stop();
    } catch (...) {
        return;
    }

}

bool laser::Ust10lxModule::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
    try{
        if (laser_->grabScan(scan_ptr)) {
            return true;
        }else {
            doClose();
            if(!doOpen()) {
                throw std::runtime_error("error to open ust lidar!");
            }
            doStart();
            return false;
        }
    }catch (...) {
        LOG(ERROR) << "Unknown error grabbing Hokuyo scan.";
        doClose();
        if(!doOpen()) {
            throw std::runtime_error("error to open ust lidar!");
        }
        doStart();
        return false;
    }
}
