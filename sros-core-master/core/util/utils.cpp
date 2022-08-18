//
// Created by lhx on 18-3-14.
//

#include "utils.h"

#include <glog/logging.h>
#include <cmath>
#include <iostream>

using namespace std;

// FIXME: 字符串最后一个字符必须是seperator，否则会少分割一个
vector<int> spilt_strs_to_ints(const string &s, const char seperator) {
    vector<int> result;
    typedef string::size_type string_size;

    string_size i = 0;
    string_size j = 0;
    char c = s[0];
    if (c == '"')  // chip 设置的话一般带“”
        j = 1;
    // LOG(INFO) << "the s is:" << s;
    while (i < s.size()) {
        if (s[i] == seperator || i == s.size() - 1) {
            if (j != i || i == s.size() - 1) {
                auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                string item = s.substr(j, len);
                if (item == "\"") break;
                int item_int = atoi(item.c_str());
                result.push_back(item_int);

                // LOG(INFO) << "spilt got item: " << item << ", " << item_int;
            }
            j = i + 1;
        }
        i++;
    }

    return result;
}

std::ostream &operator<<(std::ostream &out, std::vector<uint8_t> data) {
    for (auto d : data) {
        out << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << (int)d << " ";
    }
    return out;
}

// 数据同步写入磁盘, 防止意外断电数据丢失
void syncDisk() {
    LOG(INFO) << "==> sync";

    string cmd_str = "sync";
    systemWrapper(cmd_str);
}

bool systemWrapper(const std::string &cmd) {
    pid_t status = system(cmd.c_str());
    if (status == -1) {
        LOG(ERROR) << "system() error while run command " << cmd << " " << strerror(errno);
        return false;
    } else {
        if (WIFEXITED(status)) {
            if (0 == WEXITSTATUS(status)) {
                LOG(INFO) << "system(\"" << cmd << "\") exit successfully";
                return true;
            } else {
                LOG(INFO) << "system(\"" << cmd << "\") exit with code =  " << WEXITSTATUS(status)
                          << " (failed to run)!";
                return false;
            }
        } else {
            LOG(INFO) << "system(\"" << cmd << "\") exit with code =  " << WEXITSTATUS(status) << " (success to run)!";
            return true;
        }
    }
}

double get2PointDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

double normalizeYaw(double yaw) {
    yaw = fmod(fmod(yaw, 2.0 * M_PI), 2.0 * M_PI);
    do {
        if (yaw >= M_PI) {
            yaw -= 2.0f * M_PI;
        } else if (yaw < -M_PI) {
            yaw += 2.0f * M_PI;
        }
    } while (yaw >= M_PI || yaw < -M_PI);

    return yaw;
}

double normalizeYaw0To2Pi(double yaw) {
    yaw = std::abs(fmod(fmod(yaw, 2.0 * M_PI), 2.0 * M_PI));

    return yaw;
}

double minRotate(double yaw1, double yaw2) { return std::abs(normalizeYaw(yaw1 - yaw2)); }

std::string getCurrentRFC3339Time() {
    time_t now = time(NULL);
    struct tm *tm;
    int off_sign;
    int off;

    if ((tm = localtime(&now)) == NULL) {
        return "";
    }
    off_sign = '+';
    off = (int)tm->tm_gmtoff;
    if (tm->tm_gmtoff < 0) {
        off_sign = '-';
        off = -off;
    }
    //    printf("%04d-%02d-%02dT%02d:%02d:%02d%c%02d:%02d", tm->tm_year + 1900,
    //           tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec,
    //           off_sign, off / 3600, off % 3600);

    std::string buf(25 + 1, 0);  // note +1 for null terminator
    std::snprintf(&buf[0], buf.size(), "%04d-%02d-%02dT%02d:%02d:%02d%c%02d:%02d", tm->tm_year + 1900, tm->tm_mon + 1,
                  tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, off_sign, off / 3600, off % 3600);
    buf.pop_back();  // remove null for terminator
    return buf;
}

std::string execShell(const std::string cmd) {
    if (cmd.empty()) {
        LOG(ERROR) << "cmd is empty!!!";
        return std::string();
    }

    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }

    if (!result.empty()) {
        result.pop_back();  // 删除末尾的/n 字符
    }
    return result;
}

std::string versionUint2Str(uint32_t version_num) {
    std::string str = "v";
    str += std::to_string(version_num / 1000000);
    str += ".";
    str += std::to_string(version_num % 1000000 / 1000);
    str += ".";
    str += std::to_string(version_num % 1000);
    return str;
}

std::string versionUint16ToStr(uint16_t version_num) {
    std::string str = "v";
    str += std::to_string(version_num / 1000);
    str += ".";
    str += std::to_string(version_num % 1000 / 10);
    str += ".";
    str += std::to_string(version_num % 10);
    return str;
}