//
// Created by lhx on 16-12-01.
//

#ifndef SROS_USART_DATA_MSG_H
#define SROS_USART_DATA_MSG_H

#include "base_msg.h"
#include <memory>

namespace sros {
namespace core {

class UsartDataMsg : public BaseMsg{
public:
    UsartDataMsg(topic_t topic) : BaseMsg(topic, TYPE_USART_DATA){ };
    virtual ~UsartDataMsg() {};

    virtual void getTime() {};

    void generateRawData() {
        raw_data.push_back(0xAA);
        raw_data.push_back(0x00); // 长度需要最后回填
        raw_data.push_back(type);
        raw_data.push_back(command);

        for (auto it : data) {
            raw_data.push_back(it);
        }

        // 回填长度字段
        raw_data[1] = (uint8_t) (raw_data.size() - 2);

        // 计算异或校验值
        uint8_t check_v = 0;
        for (uint8_t i = 0; i < raw_data[1]; i++) {
            check_v ^= raw_data[2 + i];
        }

        raw_data.push_back(check_v);
        raw_data.push_back(0x55);
    }

    bool decodeRawData() {
        // TODO 校验接收到的数据
        if (raw_data.size() < 6) {
            return false;
        }

        if (raw_data[0] != 0xAA) {
            return false;
        }

        int data_len = raw_data[1];

        // 校验长度字段正确性
        if (data_len != raw_data.size() - 4) {
            return false;
        }

        type = raw_data[2];
        command = raw_data[3];

        data.clear();
        for (int i = 4; i < data_len + 2; i++) {
            data.push_back(raw_data[i]);
        }

        return true;
    }

    std::vector<uint8_t> raw_data; // 原始完整数据帧

    uint8_t type; // 类型
    uint8_t command; // 指令

    std::vector<uint8_t> data; // 数据段

};

typedef std::shared_ptr<UsartDataMsg> UsartDataMsg_ptr;

}
}


#endif //SROS_USART_DATA_MSG_H
