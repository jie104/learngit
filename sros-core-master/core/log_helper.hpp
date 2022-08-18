//
// Created by lhx on 15-12-16.
//

#ifndef SROS_LOG_HELPER_H
#define SROS_LOG_HELPER_H

#include <glog/logging.h>

namespace sros {
namespace core {

class LogHelper {
public:
    /// 初始化设置Glog，可在此指定log文件存储位置
    LogHelper(char* argv) {
        google::InitGoogleLogging(argv);
        FLAGS_colorlogtostderr = true;
        FLAGS_minloglevel = 0;
        FLAGS_logbufsecs = 0;
        FLAGS_max_log_size = 100;//日志大小100M

        // 警告日志和错误日志现在都没人看，都是INFO的日志，有时候还有人误吧警告日志当成INFO日志，所有先去除。
        google::SetLogDestination(google::WARNING, "");
        google::SetLogDestination(google::ERROR, "");

        google::SetStderrLogging(google::INFO);
        google::InstallFailureSignalHandler();
    };

    ~LogHelper() {
        google::ShutdownGoogleLogging();
    };
};

} // namespace core
} // namespace sros

#define VDEBUG VLOG(-3)
#define VINFO VLOG(-4)
#define VWARNING VLOG(-5)
#define VERROR VLOG(-6)
#define VFATAL VLOG(-7)

#define VINFO_EVERY_N(num) VLOG_EVERY_N(-4, num) // 将num个打印消息折叠起来打印一次

#endif //SROS_LOG_HELPER_H

