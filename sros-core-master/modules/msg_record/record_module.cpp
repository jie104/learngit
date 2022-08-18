//
// Created by lfc on 17-10-27.
//

#include <glog/logging.h>
#include <dirent.h>
#include <sys/stat.h>
#include "record_module.h"
#include "core/msg/ObstacleMsg.hpp"
#include "record_lib/record_operator.h"
#include "record_lib/record_msg/obstacle_record_msg.hpp"
#include "core/settings.h"
//#include <bits/signum.h>
#include <signal.h>
#include <execinfo.h>
#include "debug_lib/base_record_function.h"
#include "debug_lib/record_function_operator.hpp"
#include <modules/slam/include/lmkslam/debug_tool/RecordFunc.hpp>
using namespace std;
namespace record{


RecordModule::RecordModule():Module("RecordModule") {
//    signal(SIGSEGV, RecordModule::systemExitTrace); //Invaild memory address
//    signal(SIGABRT, RecordModule::systemExitTrace); // Abort signal
//    signal(SIGQUIT, RecordModule::systemExitTrace); // Abort signal
//    signal(SIGINT, RecordModule::systemExitTrace); // Abort signal
//    signal(SIGKILL, RecordModule::systemExitTrace); // Abort signal
//    signal(SIGSTOP, RecordModule::systemExitTrace); // Abort signal

}

RecordModule::~RecordModule() {

}

void RecordModule::run() {
    LOG(INFO) << "RecordModule start running";
    auto &s = sros::core::Settings::getInstance();

    enable_record_ = (s.getValue<string>("main.enable_record", "False") == "True");

    if (!enable_record_) {
        LOG(WARNING) << "RecordModule module stop running(disable)";
        stop();
        return;
    }
    subscribeTopic("RECORD_TOPIC_OBSTACLE", CALLBACK(&RecordModule::obstacleCallback));

    dispatch();
}

void RecordModule::obstacleCallback(sros::core::base_msg_ptr base_ptr) {
    sros::core::ObstacleMsg_ptr oba_msg = std::dynamic_pointer_cast<sros::core::ObstacleMsg>(base_ptr);
    if (oba_msg) {
        if (creatRecordFile()) {
            record::ObstacleRecordMsg_Ptr record_msg(new ObstacleRecordMsg);
            record_msg->stamp = oba_msg->time_;
            for (auto &oba:oba_msg->point_cloud) {
                record_msg->points.emplace_back();
                record_msg->points.back().x = oba.x();
                record_msg->points.back().y = oba.y();
            }
            record_op->recordMsg(record_msg);
        }
    }else {
        LOG(INFO) << "err to get the oba msg!";
    }
}

bool RecordModule::creatRecordFile() {
    if (record_op) {
        return true;
    }else{
        if (opendir(file_path.c_str()) == NULL) {
            LOG(INFO) << "no dir! will create!" << file_path;
            if (mkdir(file_path.c_str(), 0777) < 0) {
                LOG(INFO) << "error to create!";
                return false;
            }
        }

        timespec time;
        clock_gettime(CLOCK_REALTIME, &time);  //获取相对于1970到现在的秒数
        tm nowtime;
        localtime_r(&time.tv_sec, &nowtime);
        std::stringstream time_str;
        time_str<<nowtime.tm_year<<nowtime.tm_mon+1<<nowtime.tm_mday<<nowtime.tm_hour<<nowtime.tm_min;
        std::string file_name = file_path + "/record" + time_str.str()+".record";
        LOG(INFO) << "will creat the log file name:" << file_name;

        record_op.reset(new RecordOperator);

        record_op->open(file_name.c_str());
        return true;
    }
}

void RecordModule::systemExitTrace(int signum) {
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
    auto func_recorder = debug::BaseRecordFunction::getInstance();
    debug::RecordFunctionOperator func_operator("stack trace");
    for (i = 0; i < size; ++i){
        LOG(INFO) << "the id is:" << i << ",the fun is:" << funs[i];
        func_operator.pushBackFunc(funs[size-1-i]);
    }
    debug::RecordFunctionOperator slam_func_operator("slammodule");//将slam信息输出出来.
    slam_func_operator.pushBackFunc(debug::RecordFunc::getStr());
    func_recorder->outputAllFuncs();
    free(funs);
    exit(-1);
}
}
