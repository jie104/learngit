/**
 * describe: 
 * Created by caoyan on 2021-3-4.
**/

#ifndef SROS_RFID_PROTOCOL_PEPPERL_FUCHS_H
#define SROS_RFID_PROTOCOL_PEPPERL_FUCHS_H

#include "rfid_protocol_interface.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <set>

#define SS_MODE "SS_MODE"
#define ES_MODE "ES_MODE"

#define SINGLE_LABEL "SINGLE_LABEL"
#define MULTI_LABEL "MULTI_LABEL"

namespace ac {

class RfidProtPepperlFuchs : public RfidProtInterface {
public:
    RfidProtPepperlFuchs();
    virtual ~RfidProtPepperlFuchs();

    bool init() override;

    void parseMsg(std::vector<uint8_t>& data) override;

    std::string getRfid() override;

    void asyncGetRfidCmd() override;

    std::string asyncGetRfidData() override;

private:
    //设置单标签模式
    bool setSLabelMode();

    //设置功率
    bool setPowerValue(uint16_t value);

    //设置退出模式(进入EF/ES高级读模式后，收到应答后需退出该模式)
    bool setQuitMOde();

    //发送数据
    bool setDataAndRecv(std::vector<uint8_t>& send_data, std::vector<uint8_t>& recv);

    //只发送数据
    bool setData(std::vector<uint8_t>& send_data);

    //解析rifd
    bool parseRfid(const std::vector<uint8_t>& recv_data, std::string& rfid_string);

    //数据是否为ascci码
    bool isAscii(std::vector<uint8_t>& send_data);

private:
    //设置单标签模式指令
    std::vector<uint8_t> set_slabel_data_ = {0x57, 0x50, 0x55, 0x51, 0x56, 0x00, 0x01, 0x53, 0x23, 0x0D};

    //设置多标签模式指令
    std::vector<uint8_t> set_multi_slabel_data_ = {0x57, 0x50, 0x55, 0x51, 0x56, 0x00, 0x01, 0x4D, 0x23, 0x0D};

    //设置功率指令
    //0x00, 0x00 为设置的功率值
    std::vector<uint8_t> set_power_data_ = {0x57, 0x50, 0x55, 0x50, 0x54, 0x00, 0x02, 0x00, 0x00, 0x23, 0x0D};

    //单标签模式下读取epc指令
    std::vector<uint8_t> get_epc_data_ = {0x53, 0x53, 0x30, 0x23, 0x0D};

    //单标签模式下es模式读取epc指令
    std::vector<uint8_t> get_epc_data_es_ = {0x45, 0x53, 0x30, 0x23, 0x0D};

    //退出高级读模式命令（QU）
    std::vector<uint8_t> quit_enhanced_data_ = {0x51, 0x55, 0x23, 0x0D};

private:
    std::mutex mutex_;

    std::string aysnc_rfid_string_;

    bool is_async_get_rfid_ = false;

    std::set<std::string> set_async_rfid_string_;

};


}

#endif //SROS_RFID_PROTOCOL_PEPPERL_FUCHS_H


