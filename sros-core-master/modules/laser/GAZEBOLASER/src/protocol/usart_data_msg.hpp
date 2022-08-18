//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_USART_DATA_MSG
#define SRC_SDK_NETWORK_PROTOCOL_USART_DATA_MSG

#include "base_msg.hpp"

namespace network {

class USARTDataMsg : public BaseMsg {
public:
    USARTDataMsg() : BaseMsg(MSG_USART_DATA) {

    }

    virtual ~USARTDataMsg() {

    }

    virtual bool encodeBody() {
        char raw_data[SRC_USART_DATA_SIZE];

        for (int i = 0; i < value_.size(); i++) {
            raw_data[i] = value_[i];
        }

        encode_field(raw_data, value_.size());
        return true;
    }

    virtual bool decodeBody() {
        uint8_t raw_data[SRC_USART_DATA_SIZE];
        decode_field(raw_data, getBodyLength());

        value_.clear();

        for (int i = 0; i < getBodyLength(); i++) {
            value_.push_back((uint8_t) raw_data[i]);
        }

        return true;
    }

    bool setUSARTData(std::vector<uint8_t> data) {
        value_ = data;
        return true;
    }

    std::vector<uint8_t> getUSARTData() {
        return value_;
    }

private:
    std::vector<uint8_t> value_;
    size_t size_;
};

typedef std::shared_ptr<USARTDataMsg> USARTDataMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_USART_DATA_MSG
