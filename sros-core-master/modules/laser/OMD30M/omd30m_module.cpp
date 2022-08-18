//
//  Created by lfc on 16-11-10.
//

#include <glog/logging.h>
#include "omd30m_module.h"
#include "core/settings.h"


laser::Omd30mModule::Omd30mModule(std::string ip) : scanner_ip_(ip){
    if (scanner_ip_.empty()) {
        scanner_ip_ = "192.168.1.100";
    }
    LOG(INFO) << "scan ip:" << scanner_ip_;

    scan_frequency_ = 20;
    samples_per_scan_ = 3600;
    scan_frequency_ = sros::core::Settings::getInstance().getValue<int>(//getValue = mainSetting.ini中参数
            "laser.scan_frequency", 20);
    samples_per_scan_ = sros::core::Settings::getInstance().getValue<int>(//getValue = mainSetting.ini中参数
            "laser.samples_per_scan", 3600);
}

laser::Omd30mModule::Omd30mModule(LidarType type,std::string ip):scanner_ip_(ip),driver_(nullptr) {
    if (type == OMD30M) {
        scanner_ip_ = "192.168.1.100";
        scan_frequency_ = 20;
        samples_per_scan_ = 3600;
    }else if (type == OMDUHD) {
        scanner_ip_ = "192.168.1.100";
        scan_frequency_ = 20;
        samples_per_scan_ = 12600;
    }

    scan_frequency_ = sros::core::Settings::getInstance().getValue<int>(//getValue = mainSetting.ini中参数
            "laser.scan_frequency",20);
    samples_per_scan_ = sros::core::Settings::getInstance().getValue<int>(//getValue = mainSetting.ini中参数
            "laser.samples_per_scan",12600);

}

laser::Omd30mModule::~Omd30mModule() {

}

bool laser::Omd30mModule::doOpen() {
    LOG(INFO) << "!!!the frequency is:" << scan_frequency_;
    LOG(INFO) << "!!!the samples is:" << samples_per_scan_;
    // Connecting to laser range finder
    //-------------------------------------------------------------------------

    driver_.reset(new pepperl_fuchs::R2000Driver());
    LOG(INFO) << "Connecting to scanner at " << scanner_ip_ << " ... ";

    std::string display_input;
    display_input=sros::core::Settings::getInstance().getValue<std::string>(//getValue = mainSetting.ini中参数
            "laser.display_str", "");
    LOG(INFO) << "the display_input is:" << display_input;

//    scanner_ip_=sros::core::Settings::getInstance().getValue<std::string>(//getValue = mainSetting.ini中参数
//        "laser.first_lidar_ip_address", "192.168.23.100");
    LOG(INFO) << "Connecting to scanner at " << scanner_ip_ << " ... ";
    std::stringstream display_stream;
    std::string first_str;
    std::string second_str;
    display_stream << display_input;
    display_stream >> first_str;
    display_stream >> second_str;
    bool is_connect=false;
    while (!is_connect) {

        if( driver_->connect(scanner_ip_,80) ){
            LOG(INFO) << "OK" << std::endl;
            is_connect = true;
        }else{
            LOG(INFO)<<"cannot connect the lidar! will sleep 1s";
            sleep(1);
        }

    }
//
//    else
//    {
//        LOG(INFO) << "FAILED!" << std::endl;
//        LOG(INFO) << "Connection to scanner at " << scanner_ip_ << " failed!" << std::endl;
//        return false;
//    }

    // Setting, reading and displaying parameters
    //-------------------------------------------------------------------------
    driver_->setScanFrequency(scan_frequency_);
    driver_->setSamplesPerScan(samples_per_scan_);

    auto params = driver_->getParameters();
    LOG(INFO) << "Current scanner settings:" << std::endl;
    LOG(INFO) << "============================================================" << std::endl;
//    for( const auto& p : params )
//        LOG(INFO) << p.first << " : " << p.second << std::endl;
    LOG(INFO) << "Serial NO: " << params["serial"];
    LOG(INFO) << "Product: " << params["product"];
    LOG(INFO) << "============================================================" << std::endl;

    serial_no_ = params["serial"];
    model_no_ = params["product"];
    version_no_ = params["revision_fw"] + "/" + params["revision_hw"];

    // Start capturing scanner data
    //-------------------------------------------------------------------------
    LOG(INFO) << "Starting capturing: ";
    int count_sleep = 10;
    int count = 0;
    while( !driver_->startCapturingUDP() ) {
        LOG(INFO) << "FAILED!" << "count" << count;
        if (count > count_sleep) {
            LOG(INFO) << "try too many times! will return false!";
            return false;
        }
        count++;
        sleep(1);
    }
    LOG(INFO) << "OK" << std::endl;

    std::string command_name="hmi_display_mode";
    std::string command_value="application_text";
    driver_->setParameter(command_name, command_value);
    command_name = "hmi_application_text_1";
    command_value = first_str;
    driver_->setParameter(command_name, command_value);
    command_name = "hmi_application_text_2";
    command_value = second_str;
    driver_->setParameter(command_name, command_value);
    return true;
}

void laser::Omd30mModule::doStart() {
    LOG(INFO) << "Starting measuring: ";
    LOG(INFO) << "!!!the frequency is:" << scan_frequency_;
    LOG(INFO) << "!!!the samples is:" << samples_per_scan_;
}

void laser::Omd30mModule::doStop() {
    driver_->stopCapturing();
}

void laser::Omd30mModule::doClose() {
    if (driver_) {
        driver_->disconnect();
    }
}

int64_t convertNTPtoUs(const std::uint64_t &ntp_time){
    std::uint64_t hsw = (ntp_time >> 32) * 1e6;
    std::uint64_t lsw = ((ntp_time << 32) >> 32) * 1.0e6 / 4294967296;
    return (int64_t)(double)(std::uint64_t)(hsw + lsw);
}

bool laser::Omd30mModule::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
    if( !driver_->isCapturing() )
    {
        LOG(WARNING) << "ERROR: Laser range finder disconnected. Trying to reconnect..." << std::endl;
        while( !doOpen() )
        {
            LOG(INFO) << "ERROR: Reconnect failed. Trying again in 2 seconds..." << std::endl;
            usleep((2*1000000));
        }
    }
    auto scandata = driver_->getFullScan();
    if( scandata.amplitude_data.empty() || scandata.distance_data.empty() )
        return false;
    unsigned int point_num = 0;
    if ((int)(scandata.headers.size())) {
        point_num = scandata.headers[0].num_points_scan;
        if (point_num != scandata.distance_data.size()) {
            LOG(INFO) << "cannot get enough points! will return false!" << scandata.distance_data.size();
            return false;
        }
        if (point_num != scandata.amplitude_data.size()) {
            return false;
        }
    }else {
        LOG(INFO) << "err to get the header!";
        return false;
    }
    if (point_num == 0 || point_num > 100000) {
        LOG(INFO) << "the lidar is wrong! will return!";
        return false;
    }

    scan_ptr->angle_min = -M_PI;
    scan_ptr->angle_increment = 2.0*M_PI/double(point_num);
    scan_ptr->angle_max = +M_PI - scan_ptr->angle_increment;
   

    scan_ptr->scan_time = 1 / std::atof(driver_->getParametersCached().at("scan_frequency").c_str());
    //防止计数不一样
    // scan_ptr->time_increment = 1. / (double)scan_frequency_ / double(point_num);
    scan_ptr->time_increment = scan_ptr->scan_time / double(point_num);

    int64_t sync_time = scandata.time_stamp;
    sync_time -= scan_ptr->time_increment * scandata.headers[0].num_points_packet * 1e6;
    if (!scandata.headers.empty()) {
        int64_t lidar_timestamp = convertNTPtoUs(scandata.headers[0].timestamp_raw);
        if (last_lidar_stamp > lidar_timestamp) {
            LOG(WARNING) << "lidar timestamp is wrong! will clear laser stamp array!" << "," << lidar_timestamp << ","
                         << last_lidar_stamp << "," << (last_lidar_stamp - lidar_timestamp) / 1.0e6;
        }
        last_lidar_stamp = lidar_timestamp;
        int64_t delta_sync_time = sync_time - lidar_timestamp;
        if(delta_timestamp_.empty()){
            delta_timestamp_.resize(max_stamp_cache_size_);
        }
        delta_timestamp_.push_back(delta_sync_time);
        int64_t min_delta_stamp = delta_timestamp_.getMinValue();
        if (min_delta_stamp != 0) {
            sync_time = min_delta_stamp + lidar_timestamp;
        }
    }
    sync_time += scan_ptr->time_increment * point_num * 0.5 * 1e6;
    scan_ptr->time_ =  sync_time;
    int64_t curr_time = sros::core::util::get_time_in_us();
    if((curr_time-sync_time)>1e5){
        LOG(INFO) << "scan delta sync time is large!" << (curr_time - sync_time) / 1.e6;
        delta_timestamp_.resize(max_stamp_cache_size_);
    }

    //    scan_ptr->range_min = std::atof(driver_->getParametersCached().at("radial_range_min").c_str());
    scan_ptr->range_min = 0.1;
    scan_ptr->range_max = std::atof(driver_->getParametersCached().at("radial_range_max").c_str());
    double range_min = 0.1;
    double range_max = scan_ptr->range_max;

    scan_ptr->ranges.resize(point_num);
    scan_ptr->intensities.resize(point_num);

    for (unsigned int i = 0; i < point_num; i++) {
        scan_ptr->ranges[i] = float(scandata.distance_data[i]) / 1000.0f;
        auto& range = scan_ptr->ranges[i];
        if (isnanf(range)) {
            LOG(INFO) << "find one nan number!";
            range = 0.0f;
        }
        if (range > range_min && range < range_max) {

        }else {
            range = 0.0f;
        }
        scan_ptr->intensities[i] = (float)(scandata.amplitude_data[i]);
        auto &intensity = scan_ptr->intensities[i];
        if(isnanf(intensity)) {
            LOG(INFO) << "find one nan number!";
            scan_ptr->intensities[i] = 0.0f;
        }
        if (intensity > 0.0f && intensity < 10000.0f) {

        } else {
            intensity = 0.0f;
        }
    }
    return true;
}

std::string laser::Omd30mModule::getSerialNO() const {
    return serial_no_;
}

std::string laser::Omd30mModule::getModelNO() const {
    return model_no_;
}

std::string laser::Omd30mModule::getVersionNO() const {
    return version_no_;
}
