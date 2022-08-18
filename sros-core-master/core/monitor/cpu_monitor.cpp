//
// Created by lhx on 17-12-18.
//

#include "cpu_monitor.h"

#include <fstream>
#include <sstream>

namespace monitor {

CPUMonitor::CPUMonitor()
        : snap_cnt_(0) {

}

CPUMonitor::~CPUMonitor() {

}

void CPUMonitor::snapshot() {
    pre_cpu_data_ = cur_cpu_data_;

    readStatsCPU(cur_cpu_data_);

    snap_cnt_ += 1;
}

double CPUMonitor::getTotalUsage() {
    if (snap_cnt_ < 2) {
        return -1;
    } else {
        return getTotalCPUUsage(pre_cpu_data_, cur_cpu_data_);
    }
}

int CPUMonitor::getCPUCounts() {
    if (snap_cnt_ < 2) {
        return -1;
    } else {
        return (int) (pre_cpu_data_.size() - 1);
    }
}

void CPUMonitor::readStatsCPU(CPUData_list &entries) {
    std::ifstream fileStat("/proc/stat");

    std::string line;

    const std::string STR_CPU("cpu");
    const std::size_t LEN_STR_CPU = STR_CPU.size();
    const std::string STR_TOT("tot");

    entries.clear();

    while (std::getline(fileStat, line)) {
        // cpu stats line found
        if (!line.compare(0, LEN_STR_CPU, STR_CPU)) {
            std::istringstream ss(line);

            // store entry
            entries.emplace_back(CPUData());
            CPUData &entry = entries.back();

            // read cpu label
            ss >> entry.cpu;

            // remove "cpu" from the label when it's a processor number
            if (entry.cpu.size() > LEN_STR_CPU)
                entry.cpu.erase(0, LEN_STR_CPU);
                // replace "cpu" with "tot" when it's total values
            else
                entry.cpu = STR_TOT;

            // read times
            for (int i = 0; i < NUM_CPU_STATES; ++i)
                ss >> entry.times[i];
        }
    }
}

size_t CPUMonitor::getIdleTime(const CPUData &e) {
    return e.times[S_IDLE] +
           e.times[S_IOWAIT];
}

size_t CPUMonitor::getActiveTime(const CPUData &e) {
    return e.times[S_USER] +
           e.times[S_NICE] +
           e.times[S_SYSTEM] +
           e.times[S_IRQ] +
           e.times[S_SOFTIRQ] +
           e.times[S_STEAL] +
           e.times[S_GUEST] +
           e.times[S_GUEST_NICE];
}

double CPUMonitor::getTotalCPUUsage(const CPUData_list &entries1, const CPUData_list &entries2) {
    const size_t NUM_ENTRIES = entries1.size();

    const int TOTAL_CPU_INDEX = 0;

    const CPUData &e1 = entries1[TOTAL_CPU_INDEX];
    const CPUData &e2 = entries2[TOTAL_CPU_INDEX];

    const size_t active_time = getActiveTime(e2) - getActiveTime(e1);
    const size_t idle_time = getIdleTime(e2) - getIdleTime(e1);
    const size_t total_time = active_time + idle_time;

    if (total_time > 0) {
        return (1.f * active_time / total_time);
    } else {
        return 0;
    }
}

}

