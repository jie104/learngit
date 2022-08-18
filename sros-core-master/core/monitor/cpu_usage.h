//
// Created by lhx on 17-12-9.
//

#ifndef SYSTEM_UTILS_CPU_USAGE_H
#define SYSTEM_UTILS_CPU_USAGE_H

#include <string>

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

typedef struct CPUData {
    std::string cpu;
    size_t times[NUM_CPU_STATES];
} CPUData;


void readStatsCPU(std::vector<CPUData> &entries);

double getTotalCPUUsage(const std::vector<CPUData> &entries1, const std::vector<CPUData> &entries2);

#endif //SYSTEM_UTILS_CPU_USAGE_H
