
#include "file_autoclean.h"

namespace monitor
{
    CFileAutoClean::CFileAutoClean(/* args */)
    {
    }
    
    CFileAutoClean::~CFileAutoClean()
    {
    }

    bool CFileAutoClean::doClean(const int& _disk_usage)
    {
        int iLimitDays = 0;     //清除指定天数外的文件
        int iReserveNum = 0;    //保留指定文件数
        if (_disk_usage >= 90) {
            iLimitDays = 1;
            iReserveNum = 3;
        } else if (_disk_usage >= 80) {
            iLimitDays = 3;
            iReserveNum = 5;
        } else if (_disk_usage >= 70) {
            iLimitDays = 10;
            iReserveNum = 10;
        } else {
            return false;
        }

        bool bRet = true;
        bRet &= doCleanBin(iReserveNum);
        bRet &= doClean(SROS_LOG_DIR,iLimitDays);
        bRet &= doClean(SROS_STM32_DIR,iLimitDays);
        bRet &= doClean(SROS_BACKUP_DIR,iLimitDays);
        bRet &= doClean(SROS_RECORD_DIR,iLimitDays);
        bRet &= doClean(SROS_MONITOR_DIR,iLimitDays);
        bRet &= doClean(SROS_DEBUG_DIR,iLimitDays);
        bRet &= doClean(SROS_SCAN_BACKUP_DIR,iLimitDays);
        return bRet;
    }

    bool CFileAutoClean::doClean(const std::string& _strDIR,const int& _days)
    {
        struct stat s;
        stat(_strDIR.c_str(), &s);
        if(!S_ISDIR(s.st_mode)) {
            return false;
        }

        LOG(INFO) << "auto clean dir : " << _strDIR;

        time_t now;
        time(&now);
        DIR *dirhand = opendir(_strDIR.c_str());
        if(nullptr == dirhand){
            return false;
        }

        dirent *fp = nullptr;
        while((fp = readdir(dirhand)) != nullptr)
        {
            if(fp->d_name[0] == '.') {
                continue;
            }

            std::string filename = _strDIR + "/" + std::string(fp->d_name);
            struct stat filemod;
            stat(filename.c_str(), &filemod);
            if(S_ISDIR(filemod.st_mode)) {
                doClean(filename, _days);
            }
            else if (S_ISREG(filemod.st_mode)){
                if (date_from_now(now,get_file_modify_time(filename),_days)) {
                    LOG(INFO) << "auto remove file : " << filename;
                    remove(filename.c_str());
                }
            }
        }
        closedir(dirhand);
        return true;
    }

    bool CFileAutoClean::doCleanBin(const int& _reserveNum)
    {
        std::string strBinDir = SROS_BIN_DIR;
        struct stat s;
        stat(strBinDir.c_str(), &s);
        if(!S_ISDIR(s.st_mode)) {
            return false;
        }

        LOG(INFO) << "auto clean dir : " << strBinDir;

        time_t now;
        time(&now);
        DIR *dirhand = opendir(strBinDir.c_str());
        if(nullptr == dirhand){
            return false;
        }

        dirent *fp = nullptr;
        std::map<uint64_t,std::string> fileMap;
        while((fp = readdir(dirhand)) != nullptr)
        {
            if(fp->d_name[0] == '.') {
                continue;
            }

            std::string fileName = std::string(fp->d_name);
            std::size_t iFind = fileName.find("sros_");
            if (iFind != std::string::npos && (0 == iFind)) {
                std::string fileFullPath = strBinDir + "/" + fileName;
                uint64_t offSet = now - get_file_modify_time(fileFullPath);
                fileMap[offSet] = fileFullPath;
            }
        }

        if (fileMap.size() <= _reserveNum) {
            return true;
        }

        // 保留最近的文件数，清理多余的可执行文件
        int iCount = 0;
        for (auto it = fileMap.begin();it != fileMap.end();++it) {
            if (++iCount > _reserveNum) {
                LOG(INFO) << "auto remove sros : " << it->second;
                remove(it->second.c_str());
            }
        }
        closedir(dirhand);
        return true;
    }

    uint64_t CFileAutoClean::get_file_modify_time(string filepath)
    {
        struct stat filehand;
        FILE *fp;
        fp = fopen(filepath.c_str(), "r");
        if (nullptr == fp) {
            return 0;
        }

        int fileid = fileno(fp);
        fstat(fileid, &filehand);
        fclose(fp);
        return filehand.st_mtime;
    }

    bool CFileAutoClean::date_from_now(long now, long modify, int limit)
    {
        int dis = int((1.0 * (now - modify) / 86400 + 0.5));
        return dis >= limit;
    }
    
} // namespace monitor