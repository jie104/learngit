//
// Created by lhx on 17-12-18.
//

#ifndef SROS_CPU_MONITOR_H
#define SROS_CPU_MONITOR_H

#include <string>
#include <vector>

#include <memory>

namespace monitor {

const int NUM_CPU_STATES = 10;

enum CPUStates {
    S_USER = 0,
    S_NICE,
    S_SYSTEM,
    S_IDLE,
    S_IOWAIT,
    S_IRQ,
    S_SOFTIRQ,
    S_STEAL,
    S_GUEST,
    S_GUEST_NICE
};

class CPUData {
public:
    std::string cpu;
    size_t times[NUM_CPU_STATES];
};

typedef std::vector<CPUData> CPUData_list;

class CPUMonitor {
public:
    CPUMonitor();
    ~CPUMonitor();

    void snapshot();

    double getTotalUsage(); // 返回两次snapshot之间CPU的使用情况

    int getCPUCounts();

private:

    void readStatsCPU(CPUData_list &entries);
    double getTotalCPUUsage(const CPUData_list &entries1, const CPUData_list &entries2);
    size_t getIdleTime(const CPUData &e);
    size_t getActiveTime(const CPUData &e);

    int snap_cnt_;

    CPUData_list pre_cpu_data_;
    CPUData_list cur_cpu_data_;

};

typedef std::shared_ptr<CPUMonitor> CPUMonitor_ptr;

}

#endif //SROS_CPU_MONITOR_H
