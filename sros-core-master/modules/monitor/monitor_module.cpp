/**
 * @file monitor_module.cpp
 *
 * @author lhx
 * @date 2017/12/07
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "modules/monitor/monitor_module.h"

#include <boost/filesystem.hpp>

#include <core/monitor/monitor.h>
#include <core/settings.h>
#include <core/msg/common_msg.hpp>

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "core/fault_center.h"
#include "core/util/record_file_manager.hpp"
#include "core/msg/str_msg.hpp"
#include "core/logger.h"
#include "file_autoclean.h"
#include "core/alarm_record.hpp"

using namespace std;

namespace monitor {

const char MONITOR_FILE_SAVE_PATH[] = "/sros/monitor/";
const string FILE_TYPE_MF = "mf";
const string FILE_TYPE_JSON = "json";

void write_len_to_file(shared_ptr<fstream> output, size_t len) {
    assert(len <= 0xffff);

    char low_byte = static_cast<char>(len);
    char high_byte = static_cast<char>(len >> 8);

    *output << high_byte << low_byte;
}

size_t get_len_from_file(ifstream &input) {
    char low_byte = 0, high_byte = 0;

    input >> high_byte >> low_byte;

    return ((size_t)high_byte << 8) + low_byte;
}

MonitorModule::MonitorModule()
    : Module("MonitorModule"), proto_header_(), proto_record_(), pre_packets_out_(0), pre_bytes_out_(0) {
    cpu_monitor_ = make_shared<CPUMonitor>();
    cpu_monitor_->snapshot();

    network_monitor_ = make_shared<NetworkMonitor>();
    network_monitor_->snapshot();
    code_buffer_.clear();
}

MonitorModule::~MonitorModule() {}

void MonitorModule::run() {
    //    waitForStartCommand();

    LOG(INFO) << "MonitorModule module start running";

    subscribeTopic("TIMER_1S", CALLBACK(&MonitorModule::onTimer_1s));
    subscribeTopic("TIMER_20S", CALLBACK(&MonitorModule::onTimer_20s));
    subscribeTopic("MONITOR_PONG", CALLBACK(&MonitorModule::onMonitorPongMsg));
    // subscribeTopic("NAV_COMMAND", CALLBACK(&MonitorModule::onRecordPoseError));
    // subscribeTopic("DM_CODE_INFO", CALLBACK(&MonitorModule::onDMCodeInfoMsg));
    // subscribeTopic("LOCATION_CODE_ERROR", CALLBACK(&MonitorModule::onDMCodeScanPoseError));
    subscribeTopic("MONITOR_PRINT_LOG", CALLBACK(&MonitorModule::onPrintLogMonitor));
    dispatch();
}

void MonitorModule::onPrintLogMonitor(sros::core::base_msg_ptr m) {
    auto mm = dynamic_pointer_cast<sros::core::StrMsg>(m);
    if (mm == nullptr) return;

    LOGGER(INFO, SROS) << mm->data;
}

void MonitorModule::writeHeaderToFile(shared_ptr<fstream> output) {
    auto nick_name = sros::core::Settings::getInstance().getValue<string>("main.nickname", "NA");
    auto serial_number = sros::core::Settings::getInstance().getValue<string>("main.serial_no", "NA");

    proto_header_.set_nick_name(nick_name);
    proto_header_.set_serial_number(serial_number);
    proto_header_.set_start_timestamp(sros::core::util::get_timestamp_in_ms());

    string buf;

    proto_header_.SerializeToString(&buf);

    write_len_to_file(output, buf.size());
    *output << buf;
}

void MonitorModule::onTimer_1s(sros::core::base_msg_ptr m) {
    // LOG(INFO) << "+1s";

    if (!cur_file_) {
        updateCurMonitorFile();
    }

    // 重用proto_record_对象，节约对象构建的资源消耗

    record_SystemHardware(&proto_record_);

    record_NetworkData(&proto_record_);

    record_ModuleState(&proto_record_);

    record_HardwareState(&proto_record_);

    record_MovementData(&proto_record_);

    proto_record_.set_timestamp(sros::core::util::get_timestamp_in_ms());

    string buf;
    proto_record_.SerializeToString(&buf);

    auto msg = std::make_shared<sros::core::CommonMsg>("MONITOR_DATA");
    msg->str_0_ = buf;
    sendMsg(msg);

    write_len_to_file(cur_file_, buf.size());
    *cur_file_ << buf;

    sendMonitorPingMsg();
}

void MonitorModule::onTimer_20s(sros::core::base_msg_ptr m) {
    // 每 20s 检查更新一次当前文件
    updateCurMonitorFile();

    // 每 24h 删除sros/log及其子文件夹下十天前的日志文件
    onTimer_days();
}

void MonitorModule::sendMonitorPingMsg() {
    pings_.clear();  // 清空上次记录的数据

    auto ping_msg = std::make_shared<sros::core::CommonMsg>("MONITOR_PING");
    ping_msg->time_ = sros::core::util::get_time_in_us();
    ping_msg->is_real_time_ = true;
    sendMsg(ping_msg);
}

void MonitorModule::record_SystemHardware(Record *record) {
    size_t total_ram_size, avail_ram_size;
    total_memory_usage(total_ram_size, avail_ram_size);
    auto memory_usage = 1000 - avail_ram_size * 1000.0 / total_ram_size;

    auto self_memory_size = self_memory_usage_rss();
    auto self_memory_usage = self_memory_size * 1000.0 / total_ram_size;

    size_t total_disk_size, available_disk_size;
    auto get_disk_usage = [&]() -> double {
        disk_info(total_disk_size, available_disk_size);
        auto iRet = 1000 - available_disk_size * 1000.0 / total_disk_size;
        return iRet;
    };
    auto disk_usage = get_disk_usage();

    static uint64_t count_s = 0;
    // 新增磁盘自动清理机制：70%-80%-90%
    // 新增自动清理周期：运行中10分钟检测一次
    if (((count_s++ % 600) == 1) && CFileAutoClean::doClean(disk_usage/10))
    {
        auto disk_check = [&] (const int& chk_disk_usage) -> bool {
            auto compare_func = [=](const int& limit) -> bool {return disk_usage >= limit;};
            if (compare_func(chk_disk_usage)) {
                disk_usage = get_disk_usage();
                if (compare_func(chk_disk_usage)) {
                    LOG(INFO) << "disk usage is too big!";
                }
                return true;
            }
            return false;
        };

        if (!disk_check(900)) {
            if (!disk_check(800)) {
                disk_check(700);
            }
        }
    }

    cpu_monitor_->snapshot();

    g_state.cpu_usage = static_cast<int>(cpu_monitor_->getTotalUsage() * 1000);
    g_state.memory_usage = memory_usage;
    g_state.cpu_temperature = get_cpu_temperature();
    g_state.board_temperature = get_board_temperature();

    auto fault_center = sros::core::FaultCenter::getInstance();
    fault_center->track(sros::core::FAULT_CODE_CPU_LOAD_IS_TOO_HIGH, [&]() {
        static std::list<uint32_t> cpu_usage_list;
        cpu_usage_list.push_back(g_state.cpu_usage);
        if (cpu_usage_list.size() > 3 * 60) {
            cpu_usage_list.pop_front();

            auto sum = 0.0;
            for (auto cpu_usage : cpu_usage_list) {
                sum += cpu_usage;
            }
            auto average = sum / cpu_usage_list.size();
            if (average > 900) {
                LOG(WARNING) << "CPU load is too high!!! current 3 minute average cpu load is " << average / 10 << "%";
                return true;
            }
        }

        return false;
    });

    fault_center->track(sros::core::FAULT_CODE_CPU_PEAK_LOAD_IS_TOO_HIGH, [&]() {
        if (g_state.cpu_usage > 900) {
            LOG(WARNING) << "CPU peak load is too high!!! current cpu load is " << g_state.cpu_usage / 10 << "%";
            return true;
        }
        return false;
    });

    fault_center->track(sros::core::FAULT_CODE_FREE_MEMORY_IS_VERY_LOW, [&]() {
        if (g_state.memory_usage > 900) {
            LOG(WARNING) << "Free memory is too low!!! current memory usage is " << g_state.memory_usage / 10 << "%";
            return true;
        }
        return false;
    });

    fault_center->track(sros::core::FAULT_CODE_FREE_DISK_IS_VERY_LOW, [&]() {
        if (disk_usage > 900) {
            LOG(WARNING) << "Free disk is too low!!! current disk usage is " << disk_usage / 10 << "%";
            return true;
        }
        return false;
    });

    fault_center->track(sros::core::FAULT_CODE_CPU_TEMPERATURE_IS_TOO_HIGH, [&]() {
        if (g_state.cpu_temperature > 90000) {
            LOG(WARNING) << "CPU temperature is too high!!! current temperature is " << g_state.cpu_temperature / 1000
                         << "℃";
            return true;
        }
        return false;
    });

    fault_center->track(sros::core::FAULT_CODE_BOARD_TEMPERATURE_IS_TOO_HIGH, [&]() {
        if (g_state.board_temperature > 90000) {
            LOG(WARNING) << "Board temperature is too high!!! current temperature is "
                         << g_state.board_temperature / 1000 << "℃";
            return true;
        }
        return false;
    });

    auto system_hardware = record->mutable_system_hardware();
    system_hardware->set_cpu_usage(g_state.cpu_usage);
    system_hardware->set_memory_usage(g_state.memory_usage);
    system_hardware->set_self_memory_used_space(static_cast<int>(self_memory_size));
    system_hardware->set_self_memory_usage(static_cast<int>(self_memory_usage));

    system_hardware->set_cpu_temperature(g_state.cpu_temperature);
    system_hardware->set_board_temperature(g_state.board_temperature);

    system_hardware->set_disk_remain_space(static_cast<int>(available_disk_size));
    system_hardware->set_disk_usage(static_cast<int>(disk_usage));

    // 每600次（大约10分钟）输出一次
    LOG_EVERY_N(INFO, 60 * 10) << "MONITOR: cpu_usage = " << g_state.cpu_usage / 10 << "%";
    LOG_EVERY_N(INFO, 60 * 10) << "MONITOR: memory_usage = " << memory_usage / 10 << "%";
    LOG_EVERY_N(INFO, 60 * 10) << "MONITOR: self_memory_usage = " << self_memory_usage / 10 << "%";
    LOG_EVERY_N(INFO, 60 * 10) << "MONITOR: disk_usage = " << disk_usage / 10 << "%";
    LOG_EVERY_N(INFO, 60 * 10) << "MONITOR: available_disk_size = " << available_disk_size / 1024 << "MB";
}

void MonitorModule::updateCurMonitorFile() {
    auto file_name = getMonitorFileName();
    if (file_name.empty()) {
        return;
    }

    if (cur_file_name_ != file_name) {
        // 需要创建新文件
        // 首先关闭当前正在写入的文件
        if (cur_file_) {
            cur_file_->close();
        }

        // 创建新文件
        cur_file_.reset(new fstream(file_name, ios_base::out | ios_base::app | ios_base::binary));

        // 如果文件为空，那么写入文件头
        if (cur_file_->tellg() == 0) {
            writeHeaderToFile(cur_file_);
        }
    }

    cur_file_name_ = file_name;
}

std::string MonitorModule::getMonitorFileName() const {
    using boost::filesystem::directory_iterator;
    using boost::filesystem::path;

    const int NEW_FILE_TIME_INTERVAL = 60 * 60;

    // 单位 h
    auto monitor_file_save_time = sros::core::Settings::getInstance().getValue<int>("main.monitor_file_save_time", 240);

    int old_file_time_interval = monitor_file_save_time * 60 * 60;  // 单位 s

    vector<int> ts;

    // 遍历文件列表，得到每个文件对应的时间戳
    path mf_extension(".mf");

    if (!exists(path(MONITOR_FILE_SAVE_PATH))) {
        create_directory(path(MONITOR_FILE_SAVE_PATH));
    }

    try {
        for (directory_iterator pos(MONITOR_FILE_SAVE_PATH), end; pos != end; ++pos) {
            path p = pos->path();

            // NOTE:
            // 由于此处只判断了.mf后缀，以前有一个bug，没有删除掉.json后缀的文件，导致可能一些车的.json文件没有被删除掉，
            // 由于占用的不多，所以此处没有用代码处理掉这个问题，请悉知。后期若看到/sros/monitor文件夹中有多余的老.json文件，可以手动删除掉
            if (p.has_filename() && p.extension() == mf_extension) {
                string name = p.stem().string();
                string time_str = name.substr(2);  // 文件名格式为"m_*.mf"
                int timestamp = std::stoi(time_str);

                ts.push_back(timestamp);
            }
        }
    } catch (const std::exception &e) {
        LOG(ERROR) << e.what();
        return std::string();
    }

    auto cur_ts = sros::core::util::get_timestamp_in_s();

    // 清理过时文件
    for (auto t : ts) {
        if (cur_ts - t > old_file_time_interval) {
            // 后台code.py根据*.mf文件转为*.json文件，删除时要一起删除
            string mf_file = generateFileNameString(t, FILE_TYPE_MF);
            string json_file = generateFileNameString(t, FILE_TYPE_JSON);
            if (exists(path(mf_file))) {
                LOG(INFO) << "removing old monitor mf file : " << mf_file;

                boost::system::error_code ec;
                if (!boost::filesystem::remove(path(mf_file), ec)) {
                    LOG(WARNING) << "remove old monitor mf file failed : " + ec.message();
                }
            }

            if (exists(path(json_file))) {
                LOG(INFO) << "removing old monitor json file : " << json_file;

                boost::system::error_code ec;
                if (!boost::filesystem::remove(path(json_file), ec)) {
                    LOG(WARNING) << "remove old monitor json file failed : " + ec.message();
                }
            }
        }
    }

    int max_ts = 0;
    if (!ts.empty()) {
        // 找到时间戳最大的
        max_ts = *max_element(ts.begin(), ts.end());
    }

    // 根据时间差值决定是否新建文件
    auto new_ts = (cur_ts - max_ts >= NEW_FILE_TIME_INTERVAL) ? cur_ts : max_ts;

    return generateFileNameString(new_ts, FILE_TYPE_MF);
}

string MonitorModule::generateFileNameString(int64_t new_ts, const std::string file_type) const {
    string file_path = string(MONITOR_FILE_SAVE_PATH) + "m_" + to_string(new_ts);
    if (file_type == "json") {
        file_path += ".json";
    } else {
        file_path += ".mf";
    }
    return file_path;
}

int getMapValue(const std::map<std::string, int> &m, const std::string &key) {
    auto iter = m.find(key);
    return (iter == m.end()) ? 0 : iter->second;
}

void MonitorModule::record_NetworkData(monitor::Record *record) {
    auto system_hardware = record->mutable_system_hardware();

    network_monitor_->snapshot();

    auto eth0_stat = network_monitor_->getAdapterStat("eth0");

    system_hardware->set_network_eth0_rx_bps(eth0_stat.rx_bps);
    system_hardware->set_network_eth0_rx_pps(eth0_stat.rx_pps);
    system_hardware->set_network_eth0_tx_bps(eth0_stat.tx_bps);
    system_hardware->set_network_eth0_tx_pps(eth0_stat.tx_pps);

    auto enp3s0_stat = network_monitor_->getAdapterStat("enp3s0");
    if (enp3s0_stat.name.empty()) {
        // 如果enp3s0网卡不存在，那么尝试另外一个别名
        enp3s0_stat = network_monitor_->getAdapterStat("enp1s0");
    }

    system_hardware->set_network_enp3s0_rx_bps(enp3s0_stat.rx_bps);
    system_hardware->set_network_enp3s0_rx_pps(enp3s0_stat.rx_pps);
    system_hardware->set_network_enp3s0_tx_bps(enp3s0_stat.tx_bps);
    system_hardware->set_network_enp3s0_tx_pps(enp3s0_stat.tx_pps);

    auto can0_stat = network_monitor_->getAdapterStat("can0");

    system_hardware->set_network_can0_rx_bps(can0_stat.rx_bps);
    system_hardware->set_network_can0_rx_pps(can0_stat.rx_pps);
    system_hardware->set_network_can0_tx_bps(can0_stat.tx_bps);
    system_hardware->set_network_can0_tx_pps(can0_stat.tx_pps);
}

void MonitorModule::record_ModuleState(monitor::Record *record) {
    auto module_state = record->mutable_module_state();

    // 如果pings_里没有对应的module_name，那么pings_返回0
    module_state->set_module_ping_main(getMapValue(pings_, "Main"));
    module_state->set_module_ping_network(getMapValue(pings_, "Network"));
    module_state->set_module_ping_navigaton(getMapValue(pings_, "Navigation"));
    module_state->set_module_ping_location(getMapValue(pings_, "LocationModule"));
    module_state->set_module_ping_slam(getMapValue(pings_, "SlamModule"));
    module_state->set_module_ping_monitor(getMapValue(pings_, "MonitorModule"));
}

void MonitorModule::record_HardwareState(monitor::Record *record) {
    auto hardware_state = record->mutable_hardware_state();

    hardware_state->set_battery_remain_percentage(g_state.battery_percentage * 10);
    hardware_state->set_battery_temperature(g_state.battery_temperature * 10);
    hardware_state->set_battery_velotage(g_state.battery_voltage);
    hardware_state->set_battery_current(g_state.battery_current);
    hardware_state->set_battery_remain_time(g_state.battery_remain_time);

    if (g_state.battery_temperature > 50) {
        LOG(WARNING) << "Battery temperature is too high!!! current temperature is " << g_state.battery_temperature
                     << "℃";
    }

    auto power = g_state.battery_current * g_state.battery_voltage / 1000;  // 单位mW
    hardware_state->set_battery_power(power);

    // TODO(lhx) 待完善

    // 每600次（大约10分钟）输出一次
    LOG_EVERY_N(INFO, 60 * 10) << "MONITOR: battery_percentage = " << static_cast<int>(g_state.battery_percentage)
                               << "%";
    LOG_EVERY_N(INFO, 60 * 10) << "MONITOR: battery_voltage = " << g_state.battery_voltage << "mV";
    LOG_EVERY_N(INFO, 60 * 10) << "MONITOR: battery_current = " << g_state.battery_current << "mA";
    LOG_EVERY_N(INFO, 60 * 10) << "MONITOR: battery_remain_time = " << g_state.battery_remain_time << "min";
}

void MonitorModule::record_MovementData(monitor::Record *record) {
    auto movement_data = record->mutable_movement_data();

    movement_data->set_actual_linear_velocity(g_src_state.cur_v);
    movement_data->set_actual_angular_velocity(g_src_state.cur_w);

    movement_data->set_dst_linear_velocity(g_src_state.cur_v);
    movement_data->set_dst_angular_velocity(g_src_state.cur_w);

    // TODO(lhx) 待完善
}

void MonitorModule::onMonitorPongMsg(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<sros::core::CommonMsg>(m);

    auto module_name = msg->str_0_;

    auto cur_ts = sros::core::util::get_time_in_us();
    auto send_ts = msg->time_;
    auto peer_recv_ts = msg->int64_;

    auto ping_value = peer_recv_ts - send_ts;
    auto pong_value = cur_ts - peer_recv_ts;

    //    LOG(INFO) << "PING-PONG: " << module_name << " -> "
    //              << ping_value << " + " << pong_value
    //              << " = " << ping_value + pong_value << "us";

    pings_[module_name] = static_cast<int>(ping_value);
}

void MonitorModule::timedClearSrosLogBefore10()
{
    //从开机开始，每24h删除sros/log文件夹下以sros开头10天前的文件，并递归删除其子文件下10天前的文件
    std::string cmd_str;
    cmd_str = 
        "find /sros/log -name 'sros*' -mtime +10 -exec rm -rf {} \\; \
        && \
        for dir in $(find /sros/log/* -type d); do find $dir -type f -mtime +10 -exec rm -rf {} \\;; done";
    
    
    if (!systemWrapper(cmd_str)) {
        LOG(INFO) << "Failed to clear SROS Log";
    }
    else{
        LOG(INFO) << "SROS Log cleared successfully";
    }
}
void MonitorModule::onRecordPoseError(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<sros::core::CommonCommandMsg<sros::core::NavigationCommand>>(m);
    if (msg->command == sros::core::COMMAND_NAV_FINISH) {
        static std::fstream record_loc_stream;
        if (!record_loc_stream.is_open()) {
            auto file_name = record::RecordFileManager::creatImgName(record::RecordFileManager::getCurrSaveTime());
            std::string map_path = "/sros/record/";
            std::string suffix = ".txt";
            record::RecordFileManager::manageRecordFile(map_path, suffix, 5);
            LOG(INFO) << "will reopen record file:" << map_path << "," << file_name;
            record_loc_stream.open(map_path + file_name + "_pose_errors" + suffix, std::ios_base::out);
            // record_loc_stream << "time, code_id, motion_error_x, motion_error_y, motion_error_yaw, location_error_x, location_error_y, location_error_yaw" << std::endl;
            record_loc_stream << "time, code_id, motion_error_x, motion_error_y, motion_error_yaw, time_stamp" << std::endl;
        } else {
            // auto curr_time = record::RecordFileManager::getCurrSaveTime();
            // record_loc_stream << curr_time->tm_mday << "-" << curr_time->tm_hour << ":" << curr_time->tm_min << ":" << curr_time->tm_sec << "." << record::RecordFileManager::timeinus() << ",";
            // record_loc_stream << code_id_ << "," << code_x_ << "," << code_y_ << "," << code_yaw_*180/3.14 << "," <<
            //                   loc_error_x_ << "," << loc_error_y_ << "," << loc_error_yaw_*180/3.14 << std::endl;
            for (auto it : code_buffer_) {
                auto curr_time = record::RecordFileManager::getCurrSaveTime();
                record_loc_stream << curr_time->tm_mday << "-" << curr_time->tm_hour << ":" << curr_time->tm_min << ":" << curr_time->tm_sec << "." << record::RecordFileManager::timeinus() << ",";
                record_loc_stream << it.code_id << "," << it.code_x << "," << it.code_y << "," << it.code_yaw*180/3.14 << "," << it.time_stamp << std::endl;
            }
            code_buffer_.clear();
        }
    }
}

void MonitorModule::onDMCodeInfoMsg(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(m);
    monitor::CodeRcd code_temp;
    code_temp.code_x = msg->x_;
    code_temp.code_y = msg->y_;
    code_temp.code_yaw = msg->angle_;
    code_temp.code_id = msg->code_str_;
    code_temp.time_stamp = msg->getTimestamp();
    // code_x_ = msg->x_;
    // code_y_ = msg->y_;
    // code_yaw_ = msg->angle_;
    // code_id_ = msg->code_str_;
    if (code_buffer_.size() == 0) {
        code_buffer_.push_back(code_temp);      
    } else if(code_buffer_.size() < 4 && 
              msg->getTimestamp() - code_buffer_.back().time_stamp > 5e3) { 
            code_buffer_.push_back(code_temp);
    }
}

void MonitorModule::onDMCodeScanPoseError(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(m);
    loc_error_x_ = msg->loc_x_err_;
    loc_error_y_ = msg->loc_y_err_;
    loc_error_yaw_ = msg->loc_yaw_err_;
    // LOG(INFO) << "监控中： x: " << msg->loc_x_err_ << " y: " << msg->loc_y_err_;
}

void MonitorModule::onTimer_days() {
    static uint64_t times = 0;
    if(times % 4320 == 0) {
        times = 0;
        timedClearSrosLogBefore10();
        checkDataBaseSize();
    }
    ++times;
}

void MonitorModule::checkDataBaseSize() {
    sros::core::AlarmRecord::getInstance().checkAlarmTableSize();
}

} /* namespace monitor */
