//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_PARAMETER_MSG
#define SRC_SDK_NETWORK_PROTOCOL_PARAMETER_MSG

#include "base_msg.hpp"

namespace network {

// 指令
class ParameterMsg : public BaseMsg {
public:
    ParameterMsg() : BaseMsg(MSG_PARAMETER) {

    }

    virtual ~ParameterMsg() {

    }

    virtual bool encodeBody() {
        encode_field(name_, SRC_PARAMETER_FIELD_SIZE);
        encode_field(value_, SRC_PARAMETER_FIELD_SIZE);
        return true;
    }

    virtual bool decodeBody() {
        decode_field(name_, SRC_PARAMETER_FIELD_SIZE);
        decode_field(value_, SRC_PARAMETER_FIELD_SIZE);
        return true;
    }

    bool setParameter(const char *name, const char *value) {
        strncpy(name_, name, SRC_PARAMETER_FIELD_SIZE);
        strncpy(value_, value, SRC_PARAMETER_FIELD_SIZE);
        return true;
    }

    void getParameter(char *name, char *value) {
        strncpy(name, name_, SRC_PARAMETER_FIELD_SIZE);
        strncpy(value, value_, SRC_PARAMETER_FIELD_SIZE);
    }

private:
    char name_[SRC_PARAMETER_FIELD_SIZE];
    char value_[SRC_PARAMETER_FIELD_SIZE];
};

typedef std::shared_ptr<ParameterMsg> ParameterMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_PARAMETER_MSG
