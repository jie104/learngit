/**
 * @file pipe_module.cpp
 *
 * @author pengjiali
 * @date 18-12-13.
 *
 * @describe1、C++中管道不能用ifstream操作，ifstream能打开管道，但是Python那边打开管道就会自动崩溃，所以只能用C那一套读写文件
 * 2、C++open管道默认打开模式是阻塞方式，也就是到等待对方打开管道的另一端，open才会返回。
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "pipe_module.h"
#include <fcntl.h>
#include <glog/logging.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <boost/filesystem.hpp>
#include <thread>
#include "core/map_manager.h"
#include "core/msg/common_msg.hpp"
#include "core/msg/command_msg.hpp"

namespace sros {

const char *SROS_PIPE_READ = "/tmp/sros_pipe_read_fifo";    // sros读文件的管道
const char *SROS_PIPE_WRITE = "/tmp/sros_pipe_write_fifo";  // sros写文件的管道

PipeModule::PipeModule() : Module("PipeModule") {}

void PipeModule::run() {
    LOG(INFO) << "PipeModule module start running";

    subscribeTopic("TOPIC_UPDATA_SROS_REPLY", CALLBACK(&PipeModule::onUpgradeResult));
    subscribeTopic("TOPIC_UPDATA_SRC_REPLY", CALLBACK(&PipeModule::onUpgradeResult));

    if (!init()) {
        stop();
        return;
    }

    boost::thread(boost::bind(&PipeModule::doRead, this));

    LOG(INFO) << "PipeModule module dispatch!";

    dispatch();
}

bool PipeModule::init() {
    // 若管道不存在就创建
    auto mk_fifo_fun = [&](const char *file_path) {
        if (access(file_path, F_OK) == -1) {
            if (mkfifo(file_path, 0777) != 0) {
                LOG(ERROR) << "PipeModule: Could not create fifo, file_path: " << file_path;
                return false;
            }
        }

        return true;
    };

    if (!mk_fifo_fun(SROS_PIPE_READ)) {
        return false;
    }

    if (!mk_fifo_fun(SROS_PIPE_WRITE)) {
        return false;
    }

    //    write_fd_ = open(SROS_PIPE_WRITE, O_WRONLY);
    //    if (read_fd_ == -1) {
    //        LOG(ERROR) << "PipeModule: Could not open fifo, file_path: " << SROS_PIPE_READ;
    //        return false;
    //    }

    return true;
}

void PipeModule::doRead() {
    read_fd_ = open(SROS_PIPE_READ, O_RDONLY);
    if (read_fd_ == -1) {
        LOG(ERROR) << "PipeModule: Could not open fifo, file_path: " << SROS_PIPE_READ;
        return;
    }
    LOG(INFO) << "PipeModule: read pipe opened!";

    while (true) {
        std::string cmd, file_path;
        bool get_cmd = true;  // 当前的字符是用来组成cmd的
        while (true) {
            char ch;

            auto ret = read(read_fd_, &ch, 1);
            if (ret == -1 || ret == 0) {
                LOG(ERROR) << "PipeModule: read error! ret: " << ret;
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));  // 防止出现错误时1，导致此处循环过快
                continue;
            }

            if (ch == '\n') {
                break;
            }

            //            std::cout << ch << std::endl;
            if (ch == '\x20') {  //  若是空格，空格后面的为文件路径
                get_cmd = false;
                continue;
            }

            if (get_cmd) {
                cmd.push_back(ch);
            } else {
                file_path.push_back(ch);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 防止出现错误时1，导致此处循环过快
        LOG(INFO) << "PipeModule: cmd: " << cmd << ", file_path: " << file_path;

        // 解析收到的消息
        if (cmd == "update_sros") {
            auto upgrade_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_SROS");
            upgrade_msg->str_0_ = file_path;
            sendMsg(upgrade_msg);
        } else if (cmd == "update_src") {
            // 通知UpgradeModule对SRC进行升级
            auto upgrade_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_SRC");
            upgrade_msg->str_0_ = file_path;
            sendMsg(upgrade_msg);
        } else if (cmd == "update_map") {  // 地图更新
            // NOTE: file_path实质传进来的就是个地图名
            auto upgrade_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATE_MAP");
            upgrade_msg->str_0_ = file_path;
            sendMsg(upgrade_msg);
        } else if (cmd == "update_schedule") {
            auto upgrade_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATE_SCHEDULE");
            upgrade_msg->str_0_ = file_path;
            sendMsg(upgrade_msg);
        } else if(cmd == "update_db") {
            auto reset_msg = std::make_shared<sros::core::CommandMsg>(getName());
            reset_msg->command = sros::core::CMD_RESET_SROS;
            sendMsg(reset_msg);
        } else if(cmd == "update_db_config") {
            auto upgrade_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATE_SRC_PARA");
            sendMsg(upgrade_msg);
        }
    }
}

void PipeModule::onUpgradeResult(sros::core::base_msg_ptr m) {
    //    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);
    //    bool succeed = msg->int_0_ != -1;
    //
    //    const char * const UPDATE_FAILED = "update_failed\n";
    //    if (write(write_fd_, UPDATE_FAILED, strlen(UPDATE_FAILED)) == -1) {
    //        LOG(ERROR) << "PipeModule: write error";
    //    }
}

}  // namespace sros
