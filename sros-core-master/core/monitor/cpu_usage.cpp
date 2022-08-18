//
// Created by lhx on 17-12-7.
//

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <cassert>

#include "cpu_usage.h"

size_t getIdleTime(const CPUData &e);

size_t getActiveTime(const CPUData &e);

void printStats(const std::vector<CPUData> &entries1, const std::vector<CPUData> &entries2);

double getTotalCPUUsage(const std::vector<CPUData> &entries1, const std::vector<CPUData> &entries2) {
    const size_t NUM_ENTRIES = entries1.size();

    assert(NUM_ENTRIES > 0);

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

int main2(int argc, char *argv[]) {
    std::vector<CPUData> entries1;
    std::vector<CPUData> entries2;

    // snapshot 1
    readStatsCPU(entries1);

    // 100ms pause
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // snapshot 2
    readStatsCPU(entries2);

    // print output
    std::cout << "total: " << getTotalCPUUsage(entries1, entries2) << std::endl;

    printStats(entries1, entries2);

    return 0;
}

void readStatsCPU(std::vector<CPUData> &entries) {
    std::ifstream fileStat("/proc/stat");

    std::string line;

    const std::string STR_CPU("cpu");
    const std::size_t LEN_STR_CPU = STR_CPU.size();
    const std::string STR_TOT("tot");

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

size_t getIdleTime(const CPUData &e) {
    return e.times[S_IDLE] +
           e.times[S_IOWAIT];
}

size_t getActiveTime(const CPUData &e) {
    return e.times[S_USER] +
           e.times[S_NICE] +
           e.times[S_SYSTEM] +
           e.times[S_IRQ] +
           e.times[S_SOFTIRQ] +
           e.times[S_STEAL] +
           e.times[S_GUEST] +
           e.times[S_GUEST_NICE];
}

void printStats(const std::vector<CPUData> &entries1, const std::vector<CPUData> &entries2) {
    const size_t NUM_ENTRIES = entries1.size();

    for (size_t i = 0; i < NUM_ENTRIES; ++i) {
        const CPUData &e1 = entries1[i];
        const CPUData &e2 = entries2[i];

        std::cout.width(3);
        std::cout << e1.cpu << "] ";

        const float ACTIVE_TIME = static_cast<float>(getActiveTime(e2) - getActiveTime(e1));
        const float IDLE_TIME = static_cast<float>(getIdleTime(e2) - getIdleTime(e1));
        const float TOTAL_TIME = ACTIVE_TIME + IDLE_TIME;

        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout.width(6);
        std::cout.precision(2);
        std::cout << (100.f * ACTIVE_TIME / TOTAL_TIME) << "%";

        std::cout <<  std::endl;
    }
}