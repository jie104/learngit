//
// Created by caoyan on 6/15/21.
//

#ifndef SROS_DUMP_H
#define SROS_DUMP_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include <thread>

namespace sros {
namespace core {
class Dump {
 public:
    static Dump* getInstance();
    ~Dump() {}

    void init();
    void judgeTimeout();
    bool runningDumpSrc() { return running_dump_src_; }
    bool runningDumpVsc() { return running_dump_vsc_; }

    void dumpTcpEth0();
    void stopDumpTcpEth0();

    void dumpTcpEnp3s0();
    void stopDumpTcpEnp3s0();

    void dumpCan();
    void stopDumpCan();

    void dumpSrc();
    void stopDumpSrc();
    void appendSrcData(const std::string& data);

    void dumpVsc();
    void stopDumpVsc();
    void appendVscData(const std::string& data);

 private:
    Dump() {}

    // 改用脚本启动功能，防止关闭进程导致SROS崩溃
    bool startDump(const std::string& strFunc);
    bool stopDump(const std::string& strFunc);

 private:

    struct StBuffer {
        int id_;
        bool full_;
        std::string buffer_;

        bool isFull() {
            //1M = 1024 * 1024
            return (buffer_.size() > 1048576);
        }
    };

    struct StDoubleBuffer {
        StBuffer szBuffer_[2];
    };

    StDoubleBuffer st_double_buffer_src_;
    StDoubleBuffer st_double_buffer_vsc_;

    StBuffer *cur_buffer_src_;
    StBuffer *cur_buffer_vsc_;

    bool running_dump_src_ = false;
    uint64_t running_dump_src_time_;

    bool running_dump_vsc_ = false;
    uint64_t running_dump_vsc_time_;


    //tcp
    bool running_dump_tcp_eth0_ = false;
    uint64_t running_dump_tcp_eth0_time_;
    std::string dump_tcp_eth0_str_;

    bool running_dump_tcp_enp3s0_ = false;
    uint64_t running_dump_tcp_enp3s0_time_;
    std::string dump_tcp_enp3s0_str_;

    //can
    bool running_dump_can_ = false;
    uint64_t running_dump_can_time_;
    std::string dump_can_str_;

    // 调试自动结束时间
    uint64_t debug_timeout = 0;

};
}
}

#endif  // SROS_DUMP_H
