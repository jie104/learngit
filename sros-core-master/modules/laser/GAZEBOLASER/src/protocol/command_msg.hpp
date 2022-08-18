//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_COMMAND_MSG
#define SRC_SDK_NETWORK_PROTOCOL_COMMAND_MSG

#include "base_msg.hpp"

namespace network {

// 指令
class CommandMsg : public BaseMsg {
public:
    CommandMsg() : BaseMsg(MSG_COMMAND),
                   command_(COMMAND_STOP),
                   param_0_(0),
                   param_1_(0) {

    }

    virtual ~CommandMsg() {

    }

    virtual bool encodeBody() {
        encode_field(&command_, COMMAND_FIELD_SIZE);
        encode_field(param_0_);
        encode_field(param_1_);
        return true;
    }

    virtual bool decodeBody() {
        decode_field(&command_, COMMAND_FIELD_SIZE);
        decode_field(param_0_);
        decode_field(param_1_);
        return true;
    }

    const COMMAND_t &getCommand() const {
        return command_;
    }

    void setCommand(const COMMAND_t &command) {
        command_ = command;
    }

    int32_t getParam0() const {
        return param_0_;
    }

    void setParam0(int32_t param_0) {
        param_0_ = param_0;
    }

    int32_t getParam1() const {
        return param_1_;
    }

    void setParam1(int32_t param_1) {
        param_1_ = param_1;
    }

private:
    COMMAND_t command_;
    int32_t param_0_; // 自定义参数0
    int32_t param_1_;  // 自定义参数1
};

typedef std::shared_ptr<CommandMsg> CommandMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_COMMAND_MSG
