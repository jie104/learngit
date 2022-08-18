//
// Created by lfc on 17-2-11.
//

#include <glog/logging.h>
#include "LMS5xx.h"
#include "lms5xx_module.h"
#include <math.h>
#include <signal.h>
#include <execinfo.h>

#define DEG2RAD M_PI/180.0
namespace laser {


Lms5xxModule::Lms5xxModule():host("192.168.1.115") {
    lms511_cfg.resolution = 1667;
    lms511_cfg.frequency = 2500;

//    signal(SIGSEGV, Lms5xxModule::systemExitTrace); //Invaild memory address
//    signal(SIGABRT, Lms5xxModule::systemExitTrace); // Abort signal
//    signal(SIGQUIT, Lms5xxModule::systemExitTrace); // Abort signal
//    signal(SIGINT, Lms5xxModule::systemExitTrace); // Abort signal
//    signal(SIGKILL, Lms5xxModule::systemExitTrace); // Abort signal
//    signal(SIGSTOP, Lms5xxModule::systemExitTrace); // Abort signal
}

Lms5xxModule::~Lms5xxModule() {
    lms_511.reset();
}

bool Lms5xxModule::doOpen() {
    if (lms_511) {
        lms_511.reset();
    }
    lms_511.reset(new lms5xx::LMS5xx);
    int count = 0;
    do {
        lms_511->connect(host);

        if (lms_511->isConnected()) {
            lms_511->login();
            cfg = lms_511->getScanCfg();
            printf("the frequency is:%d\n", cfg.scaningFrequency);
            if (cfg.scaningFrequency == lms511_cfg.frequency&&cfg.angleResolution == lms511_cfg.resolution) {
                LOG(INFO) << ("the frequency is 25HZ! will not reset the lidar!\n");
            } else {
                cfg.angleResolution = lms511_cfg.resolution;
                cfg.scaningFrequency = lms511_cfg.frequency;

                lms_511->setScanCfg(cfg);
                cfg = lms_511->getScanCfg();

                LOG(INFO) <<
                "lms_511 configuration: scaningFrequency" << cfg.scaningFrequency << "angleResolution" <<
                cfg.angleResolution << "startAngle" << cfg.startAngle << "stopAngle" << cfg.stopAngle;
            }
            outputRange = lms_511->getScanOutputRange();
            outputRange.angleResolution = lms511_cfg.resolution;
            LOG(INFO) <<
            "lms_511 configuration: angleResolution," <<
                    outputRange.angleResolution << ",startAngle," << outputRange.startAngle << ",stopAngle," << cfg.stopAngle;
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
        if (cfg.scaningFrequency != lms511_cfg.frequency) {
            lms_511->disconnect();
            LOG(INFO) << "Waiting for lms_511 to initialize...";
        }

    } while (!lms_511->isConnected() || cfg.scaningFrequency != lms511_cfg.frequency);

    return true;
}

void Lms5xxModule::doStart() {
    if (!lms_511) {
        doOpen();
    }
    dataCfg.outputChannel = 0;
    dataCfg.remission = true;
    dataCfg.resolution = 0;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;
    dataCfg.timestamp = false;
    double start_angle = outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
    double end_angle = outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;
    double incerement = outputRange.angleResolution / 10000.0 * DEG2RAD;


    lms_511->setScanNumberCheck((roundf(((end_angle - start_angle) / incerement))+1));
    lms_511->setScanDataCfg(dataCfg);

    echo_cfg.echo_cfg = 1;
    lms_511->setEchoCfg(echo_cfg);

    lms_511->startDevice(); // Log out to properly re-enable system after config
    lms_511->startMeas();


    lms5xx::status_t stat = lms_511->queryStatus();
    while (stat != lms5xx::ready_for_measurement) // wait for ready status
    {
        stat = lms_511->queryStatus();
        LOG(INFO) << "will sleep for the lidar start!";
        LOG(INFO) << "the status is:"<<stat;
        sleep(1);
    }
    lms_511->scanContinous(1);
//    lms_511->startDevice(); // Log out to properly re-enable system after config

}

void Lms5xxModule::doStop() {
    lms_511.reset();
}

void Lms5xxModule::doClose() {
    lms_511.reset();
}

bool Lms5xxModule::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
    lms5xx::scanData data;
    if (lms_511->getScanData(&data)) {

        scan_ptr->time_ = sros::core::util::get_time_in_us();
        scan_ptr->range_min = 0.10;
        scan_ptr->range_max = 80.0;
        scan_ptr->scan_time = 100.0 / cfg.scaningFrequency;
        scan_ptr->angle_increment = (double) outputRange.angleResolution / 10000.0 * DEG2RAD;
        scan_ptr->angle_min = (double) outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
        scan_ptr->angle_max = (double) outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;
        scan_ptr->angle_max = scan_ptr->angle_max;
        int num_values =(int)roundf((scan_ptr->angle_max - scan_ptr->angle_min) / scan_ptr->angle_increment+1);
        scan_ptr->ranges.resize(num_values);
        scan_ptr->intensities.resize(num_values);
        for (int i = 0; i < data.dist_len1; i++) {
            scan_ptr->ranges[i] = data.dist1[num_values-i-1] * 0.001;
            if (scan_ptr->ranges[i] < scan_ptr->range_min) {
                scan_ptr->ranges[i] = scan_ptr->range_max + 0.1;
            }
        }
        for (int i = 0; i < data.rssi_len1; i++) {
            scan_ptr->intensities[i] = data.rssi1[num_values-i-1];
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

void Lms5xxModule::systemExitTrace(int signum) {
    const int len = 1024;
    void *func[len];
    size_t size;
    int i;
    char **funs;
    signal(signum, SIG_DFL);
    size = backtrace(func, len);
    funs = (char **) backtrace_symbols(func, size);
    LOG(INFO) << "get the backtrace!";
    LOG(INFO) << "the no is:" << signum;
    LOG(INFO) << "System error, Stack trace:";
    for (i = 0; i < size; ++i){
        LOG(INFO) << "the id is:" << i << ",the fun is:" << funs[i];
    }
    free(funs);
    exit(1);
}
}
