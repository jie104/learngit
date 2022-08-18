//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_PARAMETER_MSG
#define SRC_SDK_NETWORK_PROTOCOL_PARAMETER_MSG

#include "base_msg.hpp"

namespace src {

// 指令
class ParameterMsg : public BaseMsg {
 public:
    ParameterMsg() : BaseMsg(MSG_PARAMETER) {}

    virtual ~ParameterMsg() {}

    bool isError() { return (op_ == 0x91 || op_ == 0x90); }

    virtual bool encodeBody() {
        encode_field(op_);
        encode_field(start_addr_);
        encode_field(count_);

        for (auto value : values_) {
            encode_field(value);
        }

        return true;
    }

    virtual bool decodeBody() {
        int32_t value = 0;

        decode_field(op_);
        decode_field(start_addr_);
        decode_field(count_);

        values_.clear();
        for (int i = 0; i < count_; i++) {
            decode_field(value);
            values_.push_back(value);
        }

        return true;
    }

    void setAddr(uint16_t start_addr) {
        start_addr_ = start_addr;
        count_ = 1;

        values_.clear();

        op_ = 0x10;
    }

    void setAddr(uint16_t start_addr, uint16_t count) {
        start_addr_ = start_addr;
        count_ = count;

        values_.clear();

        op_ = 0x10;
    }

    void setValue(uint16_t start_addr, const int &value) {
        start_addr_ = start_addr;

        values_.clear();
        values_.push_back(value);

        op_ = 0x11;
    }

    void setValues(uint16_t start_addr, const std::vector<int> &values) {
        start_addr_ = start_addr;
        values_ = values;

        count_ = static_cast<uint16_t>(values_.size());

        op_ = 0x11;
    }

    // private:
    uint8_t op_;  // 0x10: 读取; 0x11: 写入; 0x91: 出错（srtos添加）

    uint16_t start_addr_;
    uint16_t count_;

    std::vector<int> values_;
};

typedef std::shared_ptr<ParameterMsg> ParameterMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_PARAMETER_MSG
