//
// Created by lhx on 15-12-23.
//

#include "laser_module.h"
#include "core/msg/laser_scan_msg.hpp"
#include "core/settings.h"

laser::LaserModule::LaserModule()
        : Module("Laser"),
          device_state_(CLOSED),
          publish_freq_(40),
          scan_no_(0) {
    publish_freq_ = sros::core::Settings::getInstance().laser_scan_freq;
}

laser::LaserModule::~LaserModule() {
    doStop();
    doClose();
}

void laser::LaserModule::run() {
    LOG(INFO) << "laser start running";
    doOpen();
    doStart();
}

void laser::LaserModule::doOpen() {
    try {
        laser_.open(config_.port.c_str());

        device_id_ = laser_.getID();
        device_status_ = laser_.getStatus();
        if (device_status_ != std::string("Sensor works well.")) {
            doClose();
            LOG(ERROR) << "Laser return abnormal status: " << device_status_;
            return;
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
            double latency = laser_.calcLatency(config_.intensity, config_.min_ang, config_.max_ang, config_.cluster, config_.skip) * 1e-9;
            LOG(INFO) << "Claibration finished. Latency is " << latency;
        } else {
            laser_.clearLatency();
        }

        LOG(INFO) << "Device opened successfully.";
        device_state_ = OPENED;

    } catch (hokuyo::Exception &e) {
        laser_.close();
        LOG(ERROR) << "can't open laser: " << e.what();
    }
}

void laser::LaserModule::doStart() {
    try {
        laser_.laserOn();

        int status = laser_.requestScans(config_.intensity, config_.min_ang, config_.max_ang, config_.cluster, config_.skip);

        if (status != 0) {
            LOG(INFO) << "Failed to request scans from device.  Status: " << status;
            return;
        }

        LOG(INFO) << "Waiting for first scan.";
        device_state_ = RUNNING;

        while (device_state_ == RUNNING) {
            try {
                int status = laser_.serviceScan(scan_);
                if (status != 0) {
                    LOG(ERROR) << "Error getting scan: " << status;
                    break;
                }
            } catch (hokuyo::CorruptedDataException &e) {
                LOG(INFO) << "Skipping corrupted data";
                continue;
            } catch (hokuyo::Exception &e) {
                LOG(ERROR) << "Exception thrown while trying to get scan: " << e.what();
                doClose();
                break;
            }
            publicScanData(scan_);
        }

    } catch (hokuyo::Exception& e) {
        doClose();
        LOG(INFO) << "Exception thrown while starting Hokuyo. " << e.what();
        return;
    }
}

void laser::LaserModule::doStop() {
    device_state_ = OPENED;
}

void laser::LaserModule::doClose() {
    try {
        laser_.close();
        device_state_ = CLOSED;
    } catch (hokuyo::Exception &e) {
        return;
    }
}

void laser::LaserModule::publicScanData(hokuyo::LaserScan &scan) {
//    LOG(INFO) << "publicScanData: " << scan.self_time_stamp << ", " << scan.ranges.size();

    auto laser_scan = std::make_shared<sros::core::LaserScanMsg>();
//    laser_scan->time_=sros::core::util::get_time_in_us();
    laser_scan->time_ = scan.system_time_stamp/1000;
    laser_scan->angle_min = scan.config.min_angle;
    laser_scan->angle_max = scan.config.max_angle;
    laser_scan->angle_increment = scan.config.ang_increment;
    laser_scan->time_increment = scan.config.time_increment;
    laser_scan->scan_time = scan.config.scan_time;
//    laser_scan->range_min = scan.config.min_range;
    laser_scan->range_min = 0.1;
    laser_scan->range_max = scan.config.max_range;
    laser_scan->ranges = scan.ranges;
    laser_scan->intensities = scan.intensities;

//    scan_no_++;

    // 调节激光雷达数据发布速率
    // 此处假定雷达原始数据发布频率为40Hz
//    if (scan_no_ % (40 / publish_freq_) == 0) {
        sendMsg(laser_scan);
//    }

//    for (int i = 0; i < scan.ranges.size(); i++) {
//        printf("%0.4f ", scan.ranges[i]);
//    }
//    printf("\n");
}
