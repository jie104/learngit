//
// Created by lhx on 17-12-13.
//

#include "disk_usage.h"

#include <boost/filesystem.hpp>


int disk_info(size_t& total_disk_size, size_t& avail_disk_size) {
    // 认为根目录文件系统代表了整个磁盘
    const char * ROOT_PATH = "/";

    auto s = boost::filesystem::space(ROOT_PATH);

    total_disk_size = s.capacity / 1024; // 转换为单位KB
    avail_disk_size = s.available / 1024; // 转换为单位KB

    return 0;
}
