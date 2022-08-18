//
// Created by caoyan on 6/15/21.
//

#include "dump.h"
#include "core/settings.h"
#include "glog/logging.h"
#include "core/device/device.h"
#include "core/util/utils.h"
#include "boost/filesystem.hpp"
#include <fstream>

namespace sros {
namespace core {

const std::string SROS_DEBUG_DATA_PATH = "/sros/debug_data/";
const std::string START_DUMP_EXEC_FILE = "/sros/bin/start_dump.sh";
const std::string STOP_DUMP_EXEC_FILE = "/sros/bin/stop_dump.sh";

Dump* Dump::getInstance() {

    static Dump instance;
    return &instance;
}

void Dump::init() {
    if (!boost::filesystem::exists(boost::filesystem::path(SROS_DEBUG_DATA_PATH))) {
        std::string cmd_str = "mkdir " + SROS_DEBUG_DATA_PATH;
        LOG(INFO) << cmd_str;
        systemWrapper(cmd_str);
    }

    st_double_buffer_src_.szBuffer_[0].id_ = 0;
    st_double_buffer_src_.szBuffer_[0].full_ = false;
    st_double_buffer_src_.szBuffer_[0].buffer_ = "";

    st_double_buffer_src_.szBuffer_[1].id_ = 1;
    st_double_buffer_src_.szBuffer_[1].full_ = false;
    st_double_buffer_src_.szBuffer_[1].buffer_ = "";

    cur_buffer_src_ = &st_double_buffer_src_.szBuffer_[0];


    st_double_buffer_vsc_.szBuffer_[0].id_ = 0;
    st_double_buffer_vsc_.szBuffer_[0].full_ = false;
    st_double_buffer_vsc_.szBuffer_[0].buffer_ = "";

    st_double_buffer_vsc_.szBuffer_[1].id_ = 1;
    st_double_buffer_vsc_.szBuffer_[1].full_ = false;
    st_double_buffer_vsc_.szBuffer_[1].buffer_ = "";

    cur_buffer_vsc_ = &st_double_buffer_vsc_.szBuffer_[0];

    auto &s = sros::core::Settings::getInstance();
    debug_timeout = s.getValue<unsigned int>("debug.data_vision_debug_timeout", 600);
}

void Dump::judgeTimeout() {
    auto cur_time = sros::core::util::get_time_in_s();
    if(running_dump_tcp_eth0_ && cur_time - running_dump_tcp_eth0_time_ > debug_timeout) {
        stopDumpTcpEth0();
    }

    if(running_dump_tcp_enp3s0_ && cur_time - running_dump_tcp_enp3s0_time_ > debug_timeout) {
        stopDumpTcpEnp3s0();
    }

    if(running_dump_can_ && cur_time - running_dump_can_time_ > debug_timeout) {
        stopDumpCan();
    }

    if(running_dump_src_ && cur_time - running_dump_src_time_ > debug_timeout) {
        stopDumpSrc();
    }

    if(running_dump_vsc_ && cur_time - running_dump_vsc_time_ > debug_timeout) {
        stopDumpVsc();
    }
}

bool Dump::startDump(const std::string& strFunc) {

    auto &s = sros::core::Settings::getInstance();
    auto exec_func = [=](const std::string& _strCmd) -> bool {
        if (systemWrapper(_strCmd)) {
            return true;
        }
        return false;
    };

    std::string strCmd = "";
    char cmdShell[100] = {0};
    if (strFunc == sros::device::DEVICE_ETH0 && !running_dump_tcp_eth0_) 
    {
        running_dump_tcp_eth0_time_ = sros::core::util::get_time_in_s();
        auto eth0_ip = s.getValue<std::string>("network.eth0_ip", "192.168.23.112");
        sprintf(cmdShell, "%s %s %s %s &",START_DUMP_EXEC_FILE.c_str(),strFunc.c_str(),SROS_DEBUG_DATA_PATH.c_str(),eth0_ip.c_str());
    } 
    else if (strFunc == sros::device::DEVICE_ENP3S0 && !running_dump_tcp_enp3s0_) 
    {
        running_dump_tcp_enp3s0_time_ = sros::core::util::get_time_in_s();
        auto enp3s0_ip = s.getValue<std::string>("network.enp3s0_ip", "192.168.71.50");
        sprintf(cmdShell, "%s %s %s %s &",START_DUMP_EXEC_FILE.c_str(),strFunc.c_str(),SROS_DEBUG_DATA_PATH.c_str(),enp3s0_ip.c_str());
    } 
    else if (strFunc == sros::device::DEVICE_CAN && !running_dump_can_) 
    {
        running_dump_can_time_ = sros::core::util::get_time_in_s();
        sprintf(cmdShell, "%s %s %s &",START_DUMP_EXEC_FILE.c_str(),strFunc.c_str(),SROS_DEBUG_DATA_PATH.c_str());
    }
    else 
    {
        return false;
    }

    strCmd = cmdShell;
    if (exec_func(strCmd)) {
        LOG(INFO) << "start dump " << strFunc << " success!";
        return true;
    } else {
        LOG(INFO) << "start dump " << strFunc << " failed!";
        return false;
    }
}

bool Dump::stopDump(const std::string& strFunc) {

    auto exec_func = [=](const std::string& _strCmd) -> bool {
        if (systemWrapper(_strCmd)) {
            return true;
        }
        return false;
    };

    std::string strCmd = "";
    char cmdShell[60] = {0};
    sprintf(cmdShell, "%s %s%s.data",STOP_DUMP_EXEC_FILE.c_str(),SROS_DEBUG_DATA_PATH.c_str(),strFunc.c_str());
    strCmd = cmdShell;
    if (exec_func(strCmd)) {
        LOG(INFO) << "stop dump " << strFunc << " success!";
        return true;
    } else {
        LOG(INFO) << "stop dump " << strFunc << " failed!";
        return false;
    }
}

void Dump::dumpTcpEth0() {

    if (startDump(sros::device::DEVICE_ETH0)) {
        running_dump_tcp_eth0_ = true;
    } else {
        running_dump_tcp_eth0_ = false;
    }
}

void Dump::stopDumpTcpEth0() {
    
    if (stopDump(sros::device::DEVICE_ETH0)) {
        running_dump_tcp_eth0_ = false;
        running_dump_tcp_eth0_time_ = 0;
    }
}

void Dump::dumpTcpEnp3s0() {

    if (startDump(sros::device::DEVICE_ENP3S0)) {
        running_dump_tcp_enp3s0_ = true;
    } else {
        running_dump_tcp_enp3s0_ = false;
    }
}

void Dump::stopDumpTcpEnp3s0() {
    
    if (stopDump(sros::device::DEVICE_ENP3S0)) {
        running_dump_tcp_enp3s0_ = false;
        running_dump_tcp_enp3s0_time_ = 0;
    }
}

void Dump::dumpCan() {

    if (startDump(sros::device::DEVICE_CAN)) {
        running_dump_can_ = true;
    } else {
        running_dump_can_ = false;
    }
}

void Dump::stopDumpCan() {

    if (stopDump(sros::device::DEVICE_CAN)) {
        running_dump_can_ = false;
        running_dump_can_time_ = 0;
    }
}

void Dump::dumpSrc() {

    if(running_dump_src_) {
        return;
    }

    auto doDumpSrc = [&]() {
        std::string file = SROS_DEBUG_DATA_PATH + sros::device::DEVICE_SRC + ".data";
        LOG(INFO) << file;

        std::ofstream outFile;
        outFile.open(file.c_str());
        if(!outFile.is_open()) {
            LOG(ERROR) << "open fail: " << file;
            return;
        }

        running_dump_src_ = true;
        running_dump_src_time_ = sros::core::util::get_time_in_s();

        while(running_dump_src_) {
            for (auto &it : st_double_buffer_src_.szBuffer_) {
                if(it.full_) {
                    LOG(INFO) << "write src buffer id: " << it.id_ << ", size: " << it.buffer_.size();
                    outFile << it.buffer_;
                    it.buffer_.clear();
                    it.full_ = false;
                }
            }
            sleep(0.5);
        }

        LOG(INFO) << "stop dump src";

        sleep(0.5);
        for (auto &it : st_double_buffer_src_.szBuffer_) {
            LOG(INFO) << "write src buffer id: " << it.id_ << ", size: " << it.buffer_.size();
            outFile << it.buffer_;
            it.buffer_.clear();
            it.full_ = false;
        }

        outFile.flush();
        outFile.close();

        //debug
        for (auto &it : st_double_buffer_src_.szBuffer_) {
            LOG(INFO) << "fin src buffer id: " << it.id_ << ", size: " << it.buffer_.size();
        }
    };

    std::thread td(doDumpSrc);
    td.detach();

}

void Dump::stopDumpSrc() {
    running_dump_src_ = false;
    running_dump_src_time_ = 0;
}

void Dump::dumpVsc() {
    if(running_dump_vsc_) {
        return;
    }

    auto doDumpSrc = [&]() {
      std::string file = SROS_DEBUG_DATA_PATH + sros::device::DEVICE_VSC + ".data";
      LOG(INFO) << file;

      std::ofstream outFile;
      outFile.open(file.c_str());
      if(!outFile.is_open()) {
          LOG(ERROR) << "open fail: " << file;
          return;
      }

      running_dump_vsc_ = true;
      running_dump_vsc_time_ = sros::core::util::get_time_in_s();

      while(running_dump_vsc_) {
          for (auto &it : st_double_buffer_vsc_.szBuffer_) {
              if(it.full_) {
                  LOG(INFO) << "write vsc buffer id: " << it.id_ << ", size: " << it.buffer_.size();
                  outFile << it.buffer_;
                  it.buffer_.clear();
                  it.full_ = false;
              }
          }
          sleep(0.5);
      }

      LOG(INFO) << "stop dump vsc";

      sleep(0.5);
      for (auto &it : st_double_buffer_vsc_.szBuffer_) {
          LOG(INFO) << "write vsc buffer id: " << it.id_ << ", size: " << it.buffer_.size();
          outFile << it.buffer_;
          it.buffer_.clear();
          it.full_ = false;
      }

      outFile.flush();
      outFile.close();

      //debug
      for (auto &it : st_double_buffer_vsc_.szBuffer_) {
          LOG(INFO) << "fin vsc buffer id: " << it.id_ << ", size: " << it.buffer_.size();
      }

    };

    std::thread td(doDumpSrc);
    td.detach();

}

void Dump::stopDumpVsc() {
    running_dump_vsc_ = false;
    running_dump_vsc_time_ = 0;
}

void Dump::appendSrcData(const std::string& data) {
    if(!running_dump_src_) {
        return;
    }

    cur_buffer_src_->buffer_.append(data);

    //LOG(INFO) << "cur src buffer id: " << cur_buffer_src_->id_ << ", size: " << cur_buffer_src_->buffer_.size();

    if(cur_buffer_src_->isFull()) {
        LOG(INFO) << "src buffer full id: " << cur_buffer_src_->id_ << ", size: " << cur_buffer_src_->buffer_.size();
        int id = (cur_buffer_src_->id_ + 1) % 2;
        StBuffer* temp = &st_double_buffer_src_.szBuffer_[id];
        if(!temp->isFull()) {
            //通知去写磁盘
            cur_buffer_src_->full_ = true;
            cur_buffer_src_ = temp;
        }
    }
}

void Dump::appendVscData(const std::string& data) {
    if(!running_dump_vsc_) {
        return;
    }

    cur_buffer_vsc_->buffer_.append(data);

    //LOG(INFO) << "cur vsc buffer id: " << cur_buffer_vsc_->id_ << ", size: " << cur_buffer_vsc_->buffer_.size();

    if(cur_buffer_vsc_->isFull()) {
        LOG(INFO) << "vsc buffer full id: " << cur_buffer_vsc_->id_ << ", size: " << cur_buffer_vsc_->buffer_.size();
        int id = (cur_buffer_vsc_->id_ + 1) % 2;
        StBuffer* temp = &st_double_buffer_vsc_.szBuffer_[id];
        if(!temp->isFull()) {
            //通知去写磁盘
            cur_buffer_vsc_->full_ = true;
            cur_buffer_vsc_ = temp;
        }
    }
}

}
}