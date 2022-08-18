/**
 * @file sros_state_msg
 *
 * @author pengjiali
 * @date 2020/6/22.
 *
 * @describe
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_SROS_STATE_MSG_HPP
#define SROS_SROS_STATE_MSG_HPP


#include "base_msg.hpp"

namespace src {

/**
 * 向src发送一些sros相关的信息
 */
class SrosStateMsg : public BaseMsg {
 public:
    SrosStateMsg() : BaseMsg(MSG_SROS_STATE){}

    virtual ~SrosStateMsg(){}

    virtual bool encodeBody() override {
        encode_field(heart_beat);

        return true;
    }

    virtual bool decodeBody() override {
        decode_field(heart_beat);

        return true;
    }

 public:
    uint32_t heart_beat = 0; // 向src发送TK1启动后的时间（ms）
};

typedef std::shared_ptr<SrosStateMsg> SrosStateMsg_Ptr;

}  // namespace src

#endif  // SROS_SROS_STATE_MSG_HPP
