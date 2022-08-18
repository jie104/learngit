//
// Created by lfc on 16-11-2.
//

#include <glog/logging.h>
#include "utm30lx_module.h"
#include "hokuyo.h"
laser::Utm30lxModule::Utm30lxModule() {


}

laser::Utm30lxModule::~Utm30lxModule() {
    doStop();
    doClose();
}

bool laser::Utm30lxModule::doOpen() {
    try {
        laser_.open(config_.port.c_str());

        device_id_ = laser_.getID();
        device_status_ = laser_.getStatus();
        if (device_status_ != std::string("Sensor works well.")) {
            doClose();
            LOG(ERROR) << "Laser return abnormal status: " << device_status_;
            return false;
        }

        LOG(INFO) << "Open laser successfully: " << device_id_;

        for (int retries = 10;;retries--) {
            try {
                laser_.laserOn();
                break;
            } catch (hokuyo::Exception &e) {
                if (!retries) throw e; // has retry 10 times
                LOG(WARNING) << "Could not turn on laser, will retry for 10 seconds";
                sleep(1); // wait for 1 seconds
            }
        }

        if (config_.calibrate_time) {
            LOG(INFO) << "Staring calibration.";
            int time_out = 5000;
            double latency = laser_.calcLatency(config_.intensity, config_.min_ang, config_.max_ang, config_.cluster, config_.skip,0,time_out) * 1e-9;
            LOG(INFO) << "Calibration finished. Latency is " << latency;
        } else {
            laser_.clearLatency();
        }

        LOG(INFO) << "Device opened successfully.";
        return true;

    } catch (hokuyo::Exception &e) {
        laser_.close();
        LOG(ERROR) << "can't open laser: " << e.what();
        sleep(1); // wait for 1 seconds
        // FIXME 消除递归调用
        return doOpen();
    }
}

void laser::Utm30lxModule::doStart() {
    try {
        laser_.laserOn();

        int status = laser_.requestScans(config_.intensity, config_.min_ang, config_.max_ang, config_.cluster,
                                         config_.skip);

        if (status != 0) {
            LOG(INFO) << "Failed to request scans from device.  Status: " << status;
            return;
        }
    }catch (hokuyo::Exception& e){
            doClose();
            LOG(INFO) << "Exception thrown while starting Hokuyo. " << e.what();
            return;
        }
        LOG(INFO) << "Waiting for first scan.";
}

void laser::Utm30lxModule::doStop() {

}

void laser::Utm30lxModule::doClose() {
    try {
        laser_.close();
    } catch (hokuyo::Exception &e) {
        return;
    }
}

bool laser::Utm30lxModule::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
    try {
        int status = laser_.serviceScan(scan_);
        if (status != 0) {
            LOG(ERROR) << "Error getting scan: " << status;
            doClose();
            if(!doOpen()) {
                throw std::runtime_error("error to open utm lidar!");
            }
            doStart();
            return false;
        }else {
            scan_ptr->time_ = scan_.system_time_stamp / 1000;
            scan_ptr->angle_min = scan_.config.min_angle;
            scan_ptr->angle_max = scan_.config.max_angle;
            scan_ptr->angle_increment = scan_.config.ang_increment;
            scan_ptr->time_increment = scan_.config.time_increment;
            scan_ptr->scan_time = scan_.config.scan_time;
            scan_ptr->range_min = scan_.config.min_range;
            scan_ptr->range_max = scan_.config.max_range;
            scan_ptr->ranges = scan_.ranges;
            scan_ptr->intensities = scan_.intensities;
            return true;
        }

    } catch (hokuyo::CorruptedDataException &e) {
        LOG(INFO) << "Skipping corrupted data";
        return false;
    } catch (hokuyo::Exception &e) {
        LOG(ERROR) << "Exception thrown while trying to get scan: " << e.what();
        doClose();
        if(!doOpen()) {
            throw std::runtime_error("error to open utm lidar!");
        }
        doStart();
        return false;
    }
}
