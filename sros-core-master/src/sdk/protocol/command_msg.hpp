//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_COMMAND_MSG
#define SRC_SDK_NETWORK_PROTOCOL_COMMAND_MSG

#include "base_msg.hpp"

namespace src {

// 指令
class CommandMsg : public BaseMsg {
public:
    CommandMsg() : BaseMsg(MSG_COMMAND),
                   command_(COMMAND_STOP),
                   param_0_(0),
                   param_1_(0),
                   param_2_(0),
                   param_3_(0) {

    }

    virtual ~CommandMsg() {

    }

    virtual bool encodeBody() {
        encode_field(&command_, COMMAND_FIELD_SIZE);
        encode_field(param_0_);
        encode_field(param_1_);
        encode_field(param_2_);
        encode_field(param_3_);
        encode_field(seq_no_);
        return true;
    }

    virtual bool decodeBody() {
        decode_field(&command_, COMMAND_FIELD_SIZE);
        decode_field(param_0_);
        decode_field(param_1_);
        decode_field(param_2_);
        decode_field(param_3_);
        decode_field(seq_no_);
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

    void setParam0(int32_t param) {
        param_0_ = param;
    }

    int32_t getParam1() const {
        return param_1_;
    }

    void setParam1(int32_t param) {
        param_1_ = param;
    }

    int32_t getParam2() const {
        return param_2_;
    }

    void setParam2(int32_t param) {
        param_2_ = param;
    }

    int32_t getParam3() const {
        return param_3_;
    }

    void setParam3(int32_t param) {
        param_3_ = param;
    }

    uint32_t getSeqNO() const {
        return seq_no_;
    }

    void setSeqNO(uint32_t seq_no) {
        seq_no_ = seq_no;
    }

private:
    COMMAND_t command_;
    int32_t param_0_;  // 自定义参数0
    int32_t param_1_;  // 自定义参数1
    int32_t param_2_;  // 自定义参数2
    int32_t param_3_;  // 自定义参数3

    uint32_t seq_no_; // 自增序列号
};

typedef std::shared_ptr<CommandMsg> CommandMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_COMMAND_MSG
