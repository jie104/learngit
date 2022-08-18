//
// Created by lhx on 17-12-13.
//

#include "temperature.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <chrono>

#include <linux/kernel.h>
#include <sys/sysinfo.h>
#include <sys/statvfs.h>
#include <thread>
#include <sys/time.h>
#include "core/state.h"

/*
 * Toradex Apalis TK1核心板温度传感器分布：
 *
 * 1). 首先apalis TK1包含Tegra K1芯片内部的sensor和外部sensor两部分。
 *
 * 2). 下面四个读出的是K1芯片内部的temperature sensor的数据，其中sensor的位置分别是靠近CPU区域，GPU区域，Memory区域以及PLL区域
 * # cat /sys/class/thermal/thermal_zone[0-3]/type
 * CPU-therm: Tsensor inside TK1, close to CPU part.
 * GPU-therm: Tsensor inside TK1, close to GPU part.
 * MEM-therm: Tsensor inside TK1, close to MEM part.
 * PLL-therm: Tsensor inside TK1, close to PLL part.
 *
 * 3). 另外在Apalis TK1模块上，芯片外部，还通过I2C总线连接了一个TPM451 temperature控制器，这个控制器有两个作用，一个是读取K1芯片
 * 内部的一个thermal diode部件的温度，用于配合PMIC对K1进行一些温度保护；另外其自身内部也有一个temperature sensor，可以测量PCB的
 * 温度，这两个温度读取值分别是Tdiode_tegra(thermal_zone5)为K1内部thermal diode温度和Tboard_tegra(thermal_zone4)为TPM451内
 * 部温度也就是K1外部PCB温度。
 */

int get_sys_temperature_value(const char * path) {

    std::ifstream fileStat(path);

    std::string line;

    int temp = 0;

    std::getline(fileStat, line);
    std::istringstream ss(line);

    ss >> temp;

    return temp; // 单位 1/100 ℃
}

int get_cpu_temperature() {
    std::string TEMP_PATH;
    if (g_state.is_kernel_release_4_14()) {
        TEMP_PATH = "/sys/class/hwmon/hwmon0/temp2_input";
    } else {
        TEMP_PATH = "/sys/devices/virtual/thermal/thermal_zone5/temp";
    }

    return get_sys_temperature_value(TEMP_PATH.c_str());
}

int get_board_temperature() {
    std::string TEMP_PATH;
    if (g_state.is_kernel_release_4_14()) {
        TEMP_PATH = "/sys/class/hwmon/hwmon0/temp1_input";
    } else {
        TEMP_PATH = "/sys/devices/virtual/thermal/thermal_zone4/temp";
    }
    return get_sys_temperature_value(TEMP_PATH.c_str());
}
