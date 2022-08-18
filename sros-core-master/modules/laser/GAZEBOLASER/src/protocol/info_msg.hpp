//
// Created by lhx on 16-5-26.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_INFO_MSG
#define SRC_SDK_NETWORK_PROTOCOL_INFO_MSG

#include "base_msg.hpp"

namespace network {

// 指令
class InfoMsg : public BaseMsg {
public:
    InfoMsg() : BaseMsg(MSG_INFO),
                src_version_(0),
                odo_version_(0) {

    }

    virtual ~InfoMsg() {

    }

    virtual bool encodeBody() {
        encode_field(src_version_);
        encode_field(src_version_);
        return true;
    }

    virtual bool decodeBody() {
        decode_field(src_version_);
        decode_field(src_version_);
        return true;
    }

    uint32_t getSRCVersion() const {
        return src_version_;
    }

    void setSRCVersion(uint32_t v) {
        src_version_ = v;
    }

    uint32_t getODOVersion() const {
        return odo_version_;
    }

    void setODOVersion(uint32_t v) {
        odo_version_ = v;
    }

private:
    uint32_t src_version_;
    uint32_t odo_version_;
};

typedef std::shared_ptr<InfoMsg> InfoMsg_ptr;

}

#endif //SRC_SDK_NETWORK_PROTOCOL_INFO_MSG
