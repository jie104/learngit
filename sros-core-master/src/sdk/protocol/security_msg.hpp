/**
 * @file security_msg
 *
 * @author pengjiali
 * @date 2020/6/19.
 *
 * @describe
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_SECURITY_MSG_H
#define SROS_SECURITY_MSG_H

#include "base_msg.hpp"

namespace src {

/**
 * 安全状态反馈
 */
class SecurityMsg : public BaseMsg {
 public:
    SecurityMsg() : BaseMsg(MSG_SECURITY){}

    virtual ~SecurityMsg(){}

    virtual bool encodeBody() override {
        encode_field(safety_state);

        if (src_length == 2) {
            encode_field((uint16_t)emergency_src);
        }
        else if (src_length == 4){
            encode_field(emergency_src);
        }

        return true;
    }

    virtual bool decodeBody() override {
        decode_field(safety_state);

        if (src_length == 2) {
            uint16_t src_int16 = 0;
            decode_field(src_int16);
            emergency_src = src_int16;
        }
        else if (src_length == 4){
            decode_field(emergency_src);
        }
        decode_field(error_code);

        return true;
    }

 public:
    /**
     * 取值范围：EmergencyState
     * enum EmergencyState {
     *      STATE_EMERGENCY_NA = 0x00,  // 紧急状态不可用
     *      STATE_EMERGENCY_NONE = 0x01,         // 不处于急停状态
     *      STATE_EMERGENCY_TRIGER = 0x02,       // 急停状态触发
     *      STATE_EMERGENCY_RECOVERABLE = 0x03,  // 可恢复急停状态
     *  };
     */
    uint8_t safety_state = 0;

    /** 急停触发源（VSC/CE/SRC）
     * 
     *  修改记录：新增SRC安全单元触发源，且扩充到4个字节（2021/10/18）
     * 
     *  SRC安全单元（独有）
     *  ESI4    ESI3    ESI2    ESI1    触发源
     *  0       1       1       1       外部自定义(0x0800)
     *  0       0       1       1       急停按钮(0x0001)
     *  0       0       0       1       触边(0x0010)
     *  0       0       0       0       内部自定义(0x0400)
     * 
     *  安全单元类型        EmSource        触发源
     *  VSC/CE/SRC        0x0001          急停按钮1
     *  VSC/CE            0x0002          急停按钮2
     *  VSC/CE            0x0004          急停按钮3
     *  VSC/CE            0x0008          急停按钮4
     *  VSC/CE/SRC        0x0010          触边1
     *  VSC/CE            0x0020          触边2
     *  VSC/CE            0x0040          触边3
     *  VSC/CE            0x0080          触边4
     *  VSC/CE/SRC        0x1000          sros软件触发1
     *  VSC/CE            0x2000          sros软件触发2
     *  VSC/CE            0x4000          sros软件触发3
     *  VSC/CE            0x8000          sros软件心跳超时
     *  VSC/CE            0x0100          电池舱门被打开
     *  SRC               0x0400          src内部自定义
     *  SRC               0x0800          src外部自定义
     *  SRC               0x10000         硬件NXP触发(NXP_ESTOP)
     *  SRC               0x20000         硬件STM32触发(ST_ESTOP)
     *  SRC               0x40000         硬件TK1触发(TK1_ESTOP)
     *  SRC               0x80000         硬件急停锁存电路触发(ESTOP_OUT)
     */
    uint32_t emergency_src = 0;

    /**
     * 增加此参数用于拓展触发源解析长度
     */
    uint16_t src_length = 2;

    // 增加触发源错误码解析
    uint16_t error_code = 0;
};

typedef std::shared_ptr<SecurityMsg> SecurityMsg_ptr;

}  // namespace src

#endif  // SROS_SECURITY_MSG_H
