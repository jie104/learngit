/**
 * @file monitor_module.h
 *
 * @author lhx
 * @date 2017/12/07
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_MODULES_MONITOR_MONITOR_MODULE_H_
#define SROS_MODULES_MONITOR_MONITOR_MODULE_H_

#include <fstream>
#include <memory>
#include <string>
#include <map>

#include "core/core.h"

#include "core/pose.h"

#include "core/msg/str_msg.hpp"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/data_matrix_code_msg.hpp"

#include "core/monitor/cpu_monitor.h"
#include "core/monitor/network_monitor.h"

#include "core/proto/monitor.pb.h"


namespace monitor {

struct CodeRcd
{
   double code_x;
   double code_y;
   double code_yaw;
   std::string code_id;
   std::uint64_t time_stamp;
};

class MonitorModule: public sros::core::Module {
 public:
    MonitorModule();

    virtual ~MonitorModule();

    virtual void run();

 private:
    void onTimer_1s(sros::core::base_msg_ptr m);
    void onTimer_20s(sros::core::base_msg_ptr m);
    void onTimer_days();

    std::string getMonitorFileName() const;

    std::string generateFileNameString(int64_t new_ts, const std::string file_type) const;

    void writeHeaderToFile(std::shared_ptr<std::fstream> output);

    void record_SystemHardware(monitor::Record *record);
    void record_ModuleState(monitor::Record *record);
    void record_HardwareState(monitor::Record *record);
    void record_MovementData(monitor::Record *record);
    void record_NetworkData(monitor::Record *record);

    void sendMonitorPingMsg();
    void onMonitorPongMsg(sros::core::base_msg_ptr m);
    void onRecordPoseError(sros::core::base_msg_ptr m);
    void onDMCodeInfoMsg(sros::core::base_msg_ptr m); 
    void onDMCodeScanPoseError(sros::core::base_msg_ptr m);

    void onPrintLogMonitor(sros::core::base_msg_ptr m);

    monitor::Header proto_header_;
    monitor::Record proto_record_;
    CPUMonitor_ptr cpu_monitor_;
    NetworkMonitor_ptr network_monitor_;

    double code_x_;
    double code_y_;
    double code_yaw_;
    double loc_error_x_;
    double loc_error_y_;
    double loc_error_yaw_;
    std::string code_id_;

    std::vector<monitor::CodeRcd> code_buffer_; 

    std::map<std::string, int> pings_;

    uint64_t pre_packets_out_;
    uint64_t pre_bytes_out_;

    std::string cur_file_name_;

    std::shared_ptr<std::fstream> cur_file_;

    void updateCurMonitorFile();

    void timedClearSrosLogBefore10();

    void checkDataBaseSize();
};
} /* namespace monitor */

#endif /* SROS_MODULES_MONITOR_MONITOR_MODULE_H_ */
