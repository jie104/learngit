//
// Created by lhx on 18-3-14.
//

#ifndef SROS_UTILS_H
#define SROS_UTILS_H

#include <math.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

std::vector<int> spilt_strs_to_ints(const std::string &s, const char seperator);

void syncDisk();

bool systemWrapper(const std::string &cmd);

template <typename T>
std::string int_to_hex(T i) {
    // Ensure this function is called with a template parameter that makes sense. Note: static_assert is only available
    // in C++11 and higher.
    static_assert(std::is_integral<T>::value,
                  "Template argument 'T' must be a fundamental integer type (e.g. int, short, etc..).");

    std::stringstream stream;
    stream << std::setfill('0') << std::setw(sizeof(T) * 2) << std::hex;

    // If T is an 8-bit integer type (e.g. uint8_t or int8_t) it will be
    // treated as an ASCII code, giving the wrong result. So we use C++17's
    // "if constexpr" to have the compiler decides at compile-time if it's
    // converting an 8-bit int or not.
    if (std::is_same<std::uint8_t, T>::value) {
        // Unsigned 8-bit unsigned int type. Cast to int (thanks Lincoln) to
        // avoid ASCII code interpretation of the int. The number of hex digits
        // in the  returned string will still be two, which is correct for 8 bits,
        // because of the 'sizeof(T)' above.
        stream << static_cast<int>(i);
    } else if (std::is_same<std::int8_t, T>::value) {
        // For 8-bit signed int, same as above, except we must first cast to unsigned
        // int, because values above 127d (0x7f) in the int will cause further issues.
        // if we cast directly to int.
        stream << static_cast<int>(static_cast<uint8_t>(i));
    } else {
        // No cast needed for ints wider than 8 bits.
        stream << i;
    }

    return stream.str();
}

// 将number的容器中的数据转化成string，主要方便打印容器中的值
template <class InputIt>
std::string numberListToStr(InputIt first, InputIt last) {
    std::stringstream out;
    for (; first != last; ++first) {
        out << std::setw(2) << std::uppercase << std::setfill('0') << std::hex << (int)(*first) << " ";
    }
    return out.str();
}

std::ostream &operator<<(std::ostream &out, std::vector<uint8_t> data);

/**
 * @brief 获取两个点之间的距离
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @note 参数不能用int，原因是100m用mm表示就是100000，平方就超出了32位了
 * @return 两个点之间的距离值
 */
double get2PointDistance(double x1, double y1, double x2, double y2);

const double DEGREE_TO_RAD = M_PI / 180.0;  // 角度转弧度
const double RAD_TO_DEGREE = 180.0 / M_PI;  // 弧度转角度
const double METER_TO_MM = 1000.0; // 米到毫米换算，1m = 1000.0mm
const double METER_TO_CM = 100.0; //米到厘米间换算,1m = 100.0cm
const double CM_TO_METER = 1 / 100.0; //厘米到米间换算,1m = 100.0cm

/**
 * @brief 将角度限制在[-PI, PI]中
 * @param yaw
 * @return yaw
 */
double normalizeYaw(double yaw);

/**
 * @brief 将角度限制在[0, 2PI)中
 * @param yaw
 * @return yaw
 */
double normalizeYaw0To2Pi(double yaw);

/**
 * @brief 从yaw1到yaw2最小需要旋转多少，可以顺时针转，也可以逆时针转
 * @param yaw1
 * @param yaw2
 * @return
 */
double minRotate(double yaw1, double yaw2);

template <typename T>
void getArgsStrSlave(std::stringstream &stream, T value) {
    stream << value;
}

template <typename T, typename... Targs>
void getArgsStrSlave(std::stringstream &stream, T value, Targs... args) {
    stream << value << " ";
    getArgsStrSlave(stream, args...);
}

template <typename... Targs>
std::string getArgStr(Targs... args) {
    std::stringstream stream;
    getArgsStrSlave(stream, args...);
    return stream.str();
}

/**
 * @brief 获取当前RFC339格式的字符串
 * @return 当前RFC339格式的字符串
 */
std::string getCurrentRFC3339Time();

/**
 * @brief 执行shell脚本，并返回其结果
 * @param cmd shell 脚本
 * @return 脚本结果
 */
std::string execShell(const std::string cmd);

/**
 * 将数字版本号转换成字符串版本号， 数字版本号 = 主版本号 × 1000 × 1000 + 次版本号 × 1000 + 修订版本号
 * @param version_num 数字版本号
 * @return 字符串版本号
 */
std::string versionUint2Str(uint32_t version_num);

/**
 * 将数字版本号转换成字符串版本号， 数字版本号 = 主版本号 × 1000 + 次版本号 × 10 + 修订版本号
 * @param version_num 数字版本号
 * @return 字符串版本号
 */
std::string versionUint16ToStr(uint16_t version_num);

template <typename T>
std::string convertInt32ListToStr(T cbegin, T cend) {
    const int MAX_LEN = 32;
    char buf[MAX_LEN + 1] = {'\0'};

    int i = 0;
    for (auto it = cbegin; it != cend; ++it) {
        char ch = '\0';
        for (int k = 0; k <= 3; k++) {
            ch = (char)(((uint32_t)(*it) >> (8 * k)) & 0xFF);
            if (ch == '\0') {
                break;
            }
            if (!isprint(ch)) {
                // 如果发现有乱码字符，则直接返回NA
                return std::string("NA");
            }
            if (i < MAX_LEN) {
                buf[i] = ch;
                i += 1;
            } else {
                // 停止遍历
                ch = '\0';
                break;
            }
        }
        if (ch == '\0') {
            break;
        }
    }

    return std::string(buf);
}

#endif  // SROS_UTILS_H
