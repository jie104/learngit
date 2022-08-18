//
//  Created by lfc on 16-11-10.
//

#include "micro_scan3_module.h"
#include <glog/logging.h>
#include "core/settings.h"
#include "sick_safetyscanners/SickSafetyscanners.h"

namespace laser {
MicroScan3Module::MicroScan3Module(std::string ip_address,int ip_port) : scanner_ip_(ip_address),ip_port_(ip_port) {
    //    scanner_ip_ = "192.168.1.30";
    LOG(INFO) << "scanner ip:" << scanner_ip_ << "," << ip_port;
    scan_frequency_ = 20;
    samples_per_scan_ = 3600;
    //    scan_frequency_ = sros::core::Settings::getInstance().getValue<int>(  // getValue = mainSetting.ini中参数
    //        "laser.scan_frequency", 20);
    //    samples_per_scan_ = sros::core::Settings::getInstance().getValue<int>(  // getValue = mainSetting.ini中参数
    //        "laser.samples_per_scan", 3600);
    communication_settings_.setSensorIp(scanner_ip_);
    std::string host_ip_adress = "192.168.1.112";
    communication_settings_.setHostIp(host_ip_adress);

    communication_settings_.setHostUdpPort(ip_port);
    int channel = 0;
    communication_settings_.setChannel(channel);
    bool enabled = false;
    communication_settings_.setEnabled(enabled);
    int skip = 0;
    communication_settings_.setPublishingFrequency(skip + 1);
    float angle_start = -135;
    float angle_end = 135;
    communication_settings_.setStartAngle(radToDeg(angle_start) - angle_offset_);
    communication_settings_.setEndAngle(radToDeg(angle_end) - angle_offset_);

    bool general_system_state = true;
    bool derived_settings = true;
    bool measurement_data = true;
    bool intrusion_data = true;
    bool application_io_data = true;
    communication_settings_.setFeatures(general_system_state, derived_settings, measurement_data, intrusion_data,
                                        application_io_data);
}

MicroScan3Module::~MicroScan3Module() {}

bool MicroScan3Module::doOpen() {
    try {
        LOG(ERROR) << "will open!";
        communication_settings_.setSensorTcpPort(2122);
        device_ = std::make_shared<sick::SickSafetyscanners>(
            boost::bind(&MicroScan3Module::receivedUDPPacket, this, _1), &communication_settings_);
        device_->run();
        sick::datastructure::TypeCode type_code;
        device_->requestTypeCode(communication_settings_, type_code);
        LOG(INFO) << "type:" << type_code.getInterfaceType() << "," << type_code.getMaxRange();
        communication_settings_.setEInterfaceType(type_code.getInterfaceType());
        range_min_ = 0.1;
        range_max_ = type_code.getMaxRange();
        sick::datastructure::ConfigData config_data;
        //  LOG(INFO)<<"config data:"<<config_data.getStartAngle()<<","<<config_data.getEndAngle();
        device_->requestPersistentConfig(communication_settings_, config_data);
        LOG(INFO) << "config data:" << config_data.getStartAngle() << "," << config_data.getEndAngle();

        communication_settings_.setStartAngle(config_data.getStartAngle());
        communication_settings_.setEndAngle(config_data.getEndAngle());
        device_->changeSensorSettings(communication_settings_);

        LOG(INFO) << "Connecting to scanner at " << scanner_ip_ << " ... ";
    } catch (std::exception& e) {
        LOG(INFO) << "exception:" << e.what();
        return false;
    }

    //    bool is_connect = false;
    return true;
}

void MicroScan3Module::doStart() {
    LOG(INFO) << "Starting measuring: ";
    LOG(INFO) << "!!!the frequency is:" << scan_frequency_;
    LOG(INFO) << "!!!the samples is:" << samples_per_scan_;
}

void MicroScan3Module::doStop() {}

void MicroScan3Module::doClose() { device_.reset(); }

int64_t convertNTPtoUs(const std::uint64_t& ntp_time) {
    std::uint64_t hsw = (ntp_time >> 32) * 1e6;
    std::uint64_t lsw = ((ntp_time << 32) >> 32) * 1.0e6 / 4294967296;
    return (int64_t)(double)(std::uint64_t)(hsw + lsw);
}

bool MicroScan3Module::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
    if (scan_queue_.empty()) {
        std::unique_lock<std::mutex> lock(condition_mutex_);
        auto state = condition_.wait_for(lock, std::chrono::seconds(1));
        if (state == std::cv_status::timeout) {
            LOG(INFO) << "wait for lidar time out!";
        }
    }
    if (scan_queue_.size()) {
        scan_ptr = popScan();
    } else {
        LOG(INFO) << "cannot get scan data!";
        return false;
    }

    return true;
}

std::string MicroScan3Module::getSerialNO() const { return serial_no_; }

std::string MicroScan3Module::getModelNO() const { return model_no_; }

std::string MicroScan3Module::getVersionNO() const { return version_no_; }

void MicroScan3Module::receivedUDPPacket(const sick::datastructure::Data& data) {
    // TODO:添加时间戳同步程序
    auto scan_stamp = sros::core::util::get_time_in_us();
    sros::core::LaserScan_ptr scan_ptr(new sros::core::LaserScanMsg);
    if (buildScan(scan_stamp, data, scan_ptr)) {
        pushScan(scan_ptr);
        condition_.notify_one();
    }
}

bool MicroScan3Module::buildScan(const int64_t stamp, const sick::datastructure::Data& data,
                                 sros::core::LaserScan_ptr& scan_ptr) {
    auto& scan = *scan_ptr;
    const uint64_t from_ms_to_us = 1000;
    uint16_t num_scan_points = data.getDerivedValuesPtr()->getNumberOfBeams();
    if (num_scan_points == 0) {
        return false;
    }
    scan.angle_min = degToRad(data.getDerivedValuesPtr()->getStartAngle() + angle_offset_);
    if (data.getMeasurementDataPtr()->getScanPointsVector().size() != num_scan_points) {
        LOG(ERROR) << "scan size not equal!" <<","<<data.getMeasurementDataPtr()->getScanPointsVector().size()<<","<< num_scan_points;
        return false;
    }
    if (!have_get_version_) {
        serial_no_ = std::to_string(data.getDataHeaderPtr()->getSerialNumberOfDevice());
        model_no_ = std::to_string(data.getDataHeaderPtr()->getSerialNumberOfSystemPlug());
        version_no_ = std::to_string(data.getDataHeaderPtr()->getVersionMajorVersion()) + "." +
                      std::to_string(data.getDataHeaderPtr()->getVersionMinorVersion()) + ".0";
    }

    uint64_t lidar_time = (uint64_t)data.getDataHeaderPtr()->getTimestampTime() * from_ms_to_us;
    scan.time_ = getSyncStamp(lidar_time, stamp);
    boost::posix_time::microseconds time_increment =
        boost::posix_time::microseconds(data.getDerivedValuesPtr()->getInterbeamPeriod());
    scan.time_increment = time_increment.total_microseconds() * 1e-6;
    boost::posix_time::milliseconds scan_time =
        boost::posix_time::milliseconds(data.getDerivedValuesPtr()->getScanTime());
    scan.scan_time = scan_time.total_microseconds() * 1e-6;
    scan.time_ -= scan.scan_time / 2.0 * 1e6;

    double angle_max = degToRad(data.getMeasurementDataPtr()
                                    ->getScanPointsVector()
                                    .at(data.getMeasurementDataPtr()->getScanPointsVector().size() - 1)
                                    .getAngle() +
                                angle_offset_);
    scan.angle_max = angle_max;
    scan.angle_increment = degToRad(data.getDerivedValuesPtr()->getAngularBeamResolution());
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.ranges.resize(num_scan_points);
    scan.intensities.resize(num_scan_points);

    std::vector<sick::datastructure::ScanPoint> scan_points = data.getMeasurementDataPtr()->getScanPointsVector();
    for (uint16_t i = 0; i < num_scan_points; ++i) {
        const sick::datastructure::ScanPoint& scan_point = scan_points.at(i);
        scan.ranges[i] = static_cast<float>(scan_point.getDistance()) *
                         data.getDerivedValuesPtr()->getMultiplicationFactor() * 1.0e-3;  // mm -> m
        scan.intensities[i] = static_cast<float>(scan_point.getReflectivity());
        if (scan_point.getReflectorBit()) {
            if (scan.ranges[i] < scan.range_max - 0.1f) {
                scan.intensities[i] = 2000;
            }
        }
    }
    return true;
}

sros::core::LaserScan_ptr MicroScan3Module::popScan() {
    sros::core::LaserScan_ptr scan;
    read_write_mutex_.lock();
    if (scan_queue_.size()) {
        scan = scan_queue_.front();
        scan_queue_.pop();
    }
    read_write_mutex_.unlock();
    return scan;
}

void MicroScan3Module::pushScan(const sros::core::LaserScan_ptr& scan) {
    if (scan_queue_.size() >= 2) {
        LOG(INFO) << "too many scan!" << scan_queue_.size();
    }
    read_write_mutex_.lock();
    scan_queue_.push(scan);
    if (scan_queue_.size() >= 10) {
        while (scan_queue_.size() >= 2) {
            scan_queue_.pop();
        }
    }
    read_write_mutex_.unlock();
}

int64_t MicroScan3Module::getSyncStamp(int64_t lidar_time, int64_t system_time) {
    int64_t delta_sync_time = system_time - lidar_time;
    if (delta_timestamp_.empty()) {
        delta_timestamp_.resize(100);
    }
    delta_timestamp_.push_back(delta_sync_time);
    int64_t min_delta_stamp = delta_timestamp_.getMinValue();
    if (min_delta_stamp != 0) {
        auto sync_time = min_delta_stamp + lidar_time;
        if (abs(system_time - sync_time) > 1e5) {
            LOG(INFO) << "sync is wrong! will use input time!";
            delta_timestamp_.resize(100);
        } else {
            system_time = sync_time;
        }
    }
    return system_time;
}

void MicroScan3Module::setHostIpAddress(std::string host_ip) { communication_settings_.setHostIp(host_ip); }

}