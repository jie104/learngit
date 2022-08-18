//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_UART_INFO_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_UART_INFO_MSG_H

#include "base_msg.hpp"

namespace src {

/// @brief SRC反馈的硬件状态信息
///
/// 运动里程
/// 电机驱动
/// IMU
/// 手动控制器
///
class InfoMsg : public BaseMsg {
public:
    InfoMsg() : BaseMsg(MSG_INFO) { };

    virtual ~InfoMsg() { };

    virtual bool encodeBody() override {
        char buf[SRC_GIT_VERSION_STR_SIZE + 1] = {0};

        encode_field(version_no);

        strncpy(buf, git_version_str.c_str(), SRC_GIT_VERSION_STR_SIZE);
        encode_field(buf, SRC_GIT_VERSION_STR_SIZE);

        encode_field(total_power_cycle);
        encode_field(total_poweron_time);
        encode_field(total_mileage);

        encode_field(m1_status);
        encode_field(m1_status_code);
        encode_field(m1_mileage);

        encode_field(m2_status);
        encode_field(m2_status_code);
        encode_field(m2_mileage);

        encode_field(m3_status);
        encode_field(m3_status_code);
        encode_field(m3_mileage);

        encode_field(m4_status);
        encode_field(m4_status_code);
        encode_field(m4_mileage);

        encode_field(device0_status);
        encode_field(device1_status);
        encode_field(device2_status);
        encode_field(device3_status);
        encode_field(device4_status);
        encode_field(device5_status);
        encode_field(device6_status);
        encode_field(device7_status);

        encode_field(reserved_field);
        return true;
    }

    virtual bool decodeBody() override {
        char buf[SRC_GIT_VERSION_STR_SIZE + 1] = {0};

        decode_field(version_no);

        decode_field(buf, SRC_GIT_VERSION_STR_SIZE);
        git_version_str = string(buf);

        decode_field(total_power_cycle);
        decode_field(total_poweron_time);
        decode_field(total_mileage);

        decode_field(m1_status);
        decode_field(m1_status_code);
        decode_field(m1_mileage);

        decode_field(m2_status);
        decode_field(m2_status_code);
        decode_field(m2_mileage);

        decode_field(m3_status);
        decode_field(m3_status_code);
        decode_field(m3_mileage);

        decode_field(m4_status);
        decode_field(m4_status_code);
        decode_field(m4_mileage);

        decode_field(device0_status);
        decode_field(device1_status);
        decode_field(device2_status);
        decode_field(device3_status);
        decode_field(device4_status);
        decode_field(device5_status);
        decode_field(device6_status);
        decode_field(device7_status);

        decode_field(reserved_field);
        return true;
    }

    string versionStr() {
        int major = version_no / (1000 * 1000);
        int minor = (version_no / 1000) % 1000;
        int patch = version_no % 1000;
        return to_string(major) + "." + to_string(minor) + "." + to_string(patch) + "(" + git_version_str + ")";
    }

public:
    uint32_t version_no;
    string git_version_str;

    uint32_t total_power_cycle; // 上电次数
    uint32_t total_poweron_time; // 总开机时间，单位s
    uint32_t total_mileage; // 总运动里程，单位m

    // 电机1
    uint8_t m1_status; // 状态
    uint32_t m1_status_code; // 状态码
    uint32_t m1_mileage; // 运动里程

    // 电机2
    uint8_t m2_status; // 状态
    uint32_t m2_status_code; // 状态码
    uint32_t m2_mileage; // 运动里程

    // 电机3
    uint8_t m3_status; // 状态
    uint32_t m3_status_code; // 状态码
    uint32_t m3_mileage; // 运动里程

    // 电机4
    uint8_t m4_status; // 状态
    uint32_t m4_status_code; // 状态码
    uint32_t m4_mileage; // 运动里程


    uint8_t device0_status; // IMU
    uint8_t device1_status; // PGV1(下视)
    uint8_t device2_status; // PGV2(上视）
    uint8_t device3_status; // 手动控制器
    uint8_t device4_status;
    uint8_t device5_status;
    uint8_t device6_status;
    uint8_t device7_status;

    uint64_t reserved_field;

};

typedef std::shared_ptr<InfoMsg> InfoMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_UART_INFO_MSG_H
