//
// Created by lfc on 16-11-2.
//

#include <glog/logging.h>
#include <math.h>
#include "lms1xx_module.h"
#define DEG2RAD M_PI/180.0
laser::Lms1xxModule::Lms1xxModule():host("192.168.1.14"),laser(nullptr) {

}

laser::Lms1xxModule::~Lms1xxModule() {
    if (laser) {
        delete laser;
        laser = nullptr;
    }

}

bool laser::Lms1xxModule::doOpen() {
    if (laser) {
        delete laser;
        laser = nullptr;
    }
    laser = new LMS1xx;
    int count = 0;
    do {
        laser->connect(host);

        if (laser->isConnected()) {
            laser->login();
            cfg = laser->getScanCfg();
            printf("the frequency is:%d\n", cfg.scaningFrequency);
            if (cfg.scaningFrequency == 2500) {
                printf("the frequency is 25HZ! will not reset the lidar!\n");
            } else {
                cfg.angleResolution = 2500;
                cfg.scaningFrequency = 2500;

                laser->setScanCfg(cfg);
                cfg = laser->getScanCfg();

                LOG(INFO) <<
                "Laser configuration: scaningFrequency" << cfg.scaningFrequency << "angleResolution" <<
                cfg.angleResolution << "startAngle" << cfg.startAngle << "stopAngle" << cfg.stopAngle;
            }
            outputRange = laser->getScanOutputRange();
            outputRange.angleResolution = 2500;
        }else {
            count++;
            usleep(1000*100);
            if (count > 200) {
                LOG(INFO) << "error to open the LMS5XX!";
                count = 0;
//                return false;
            }
        }


        //check if laser is fully initialized, else reconnect
        //assuming fully initialized => scaningFrequency=5000
        if (cfg.scaningFrequency != 2500) {
            laser->disconnect();
            LOG(INFO) << "Waiting for laser to initialize...";
        }

    } while (!laser->isConnected() || cfg.scaningFrequency != 2500);

    return true;


}

void laser::Lms1xxModule::doStart() {
    if (!laser) {
        doOpen();
    }
    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;
    dataCfg.timestamp = true;
    double start_angle = outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
    double end_angle = outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;
    double incerement = outputRange.angleResolution / 10000.0 * DEG2RAD;
    laser->setScanNumberCheck((int) ((end_angle - start_angle) / incerement + 1));
    laser->setScanDataCfg(dataCfg);
    laser->startMeas();
    status_t stat = laser->queryStatus();
    while (stat != ready_for_measurement) // wait for ready status
    {
        stat = laser->queryStatus();
        LOG(INFO) << "will sleep for the lidar start!";
        LOG(INFO) << "the status is:"<<stat;
        sleep(1);
    }
    laser->startDevice(); // Log out to properly re-enable system after config
    laser->scanContinous(1);
}

void laser::Lms1xxModule::doStop() {
    if (laser) {
        laser->stopMeas();
    }


}

void laser::Lms1xxModule::doClose() {
    if (laser) {
        laser->disconnect();
    }
}

bool laser::Lms1xxModule::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
//TODO:时间戳需要重新做
    scanData data;
    if (laser->getScanData(&data)) {
        scan_ptr->time_ = sros::core::util::get_time_in_us();
        scan_ptr->range_min = 0.10;
        scan_ptr->range_max = 50.0;
        scan_ptr->scan_time = 100.0 / cfg.scaningFrequency;
        scan_ptr->angle_increment = (double) outputRange.angleResolution / 10000.0 * DEG2RAD;
        scan_ptr->angle_min = (double) outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
        scan_ptr->angle_max = (double) outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;
        scan_ptr->angle_max = scan_ptr->angle_max - scan_ptr->angle_increment;
        int num_values =(int)round((scan_ptr->angle_max - scan_ptr->angle_min) / scan_ptr->angle_increment + 1);
        scan_ptr->ranges.resize(num_values);
        scan_ptr->intensities.resize(num_values);
        for (int i = 0; i < data.dist_len1; i++) {
            scan_ptr->ranges[i] = data.dist1[i] * 0.001;
            if (scan_ptr->ranges[i] < scan_ptr->range_min) {
                scan_ptr->ranges[i] = scan_ptr->range_max + 0.1;
            }
        }
        for (int i = 0; i < data.rssi_len1; i++) {
            scan_ptr->intensities[i] = data.rssi1[i];
        }
        return true;
    }else {
        LOG(INFO) << "error to get the scan!";
        doClose();
        if(!doOpen()) {
            throw std::runtime_error("error to open lms1xx lidar!");
        }
        doStart();
        return false;
    }
}
