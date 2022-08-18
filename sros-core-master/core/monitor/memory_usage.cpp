//
// Created by lhx on 17-12-13.
//

#include "memory_usage.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <chrono>


// 返回的单位为KB
size_t get_meminfo_value(const std::string& label) {
//    MemTotal:        1972464 kB
//    MemFree:         1028752 kB
//    Buffers:           16060 kB
//    Cached:           668592 kB
//    SwapCached:            0 kB

    std::ifstream f("/proc/meminfo");

    std::string line;
    std::string x;

    size_t value = 0;

    while (std::getline(f, line)) {
        // label line found
        if (!line.compare(0, label.size(), label)) {
            std::istringstream ss(line);

            ss >> x;
            ss >> value;
        }

    }

    return value;
}

int total_memory_usage(size_t& total_ram_size, size_t & avail_ram_size) {
    // 通过sysinfo()获取的内存数据没有Cached的大小

    auto mem_total = get_meminfo_value("MemTotal");
    auto mem_free = get_meminfo_value("MemFree");
    auto mem_buffers = get_meminfo_value("Buffers");
    auto mem_cached = get_meminfo_value("Cached");

    total_ram_size = mem_total; // 单位KB
    avail_ram_size = mem_free + mem_buffers + mem_cached;

    return 0;
}

size_t self_memory_usage(const std::string &label) {
    std::ifstream fileStat("/proc/self/status");

    std::string line;

    std::string x;
    size_t vm_size = 0;

    while (std::getline(fileStat, line)) {
        // VmSize line found
        if (!line.compare(0, label.size(), label)) {
            std::istringstream ss(line);

            ss >> x;
            ss >> vm_size;
        }
    }
    return vm_size; // 单位KB
}

size_t self_memory_usage() {
    return self_memory_usage("VmSize");
}

size_t self_memory_usage_rss() {
    return self_memory_usage("VmRSS");
}
