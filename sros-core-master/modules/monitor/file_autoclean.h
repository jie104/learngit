
#ifndef FILE_AUTOCLEAN_H
#define FILE_AUTOCLEAN_H
#include <cstdio>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <map>
#include <dirent.h>
#include <time.h>
#include <glog/logging.h>
using namespace std;

#define SROS_ROOT_DIR "/sros"
#define SROS_MAP_DIR "/sros/map"
#define SROS_BIN_DIR "/sros/bin"
#define SROS_LOG_DIR "/sros/log"
#define SROS_STM32_DIR "/sros/stm32"
#define SROS_BACKUP_DIR "/sros/backup"
#define SROS_RECORD_DIR "/sros/record"
#define SROS_MONITOR_DIR "/sros/monitor"
#define SROS_DEBUG_DIR "/sros/debug_data"
#define SROS_SCAN_BACKUP_DIR "/sros/scan_back_up"

namespace monitor
{
    class CFileAutoClean
    {
    public:
        CFileAutoClean(/* args */);
        ~CFileAutoClean();

        // 根据磁盘占用清理目录
        static bool doClean(const int& _disk_usage);

    protected:
        // 清除指定天数外的目录文件
        static bool doClean(const std::string& _strDIR,const int& _days);

        // 清除/sros/bin目录文件
        static bool doCleanBin(const int& _reserveNum);

    private:
        // 获取文件修改时间
        static uint64_t get_file_modify_time(std::string filepath);

        // 指定期限内时间
        static bool date_from_now(long now, long modify, int limit);
    };
    
} // namespace monitor


#endif