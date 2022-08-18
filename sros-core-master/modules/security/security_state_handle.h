/**
 * @file security_state_handle
 *
 * @author pengjiali
 * @date 20-1-21.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_SECURITY_STATE_HANDLE_H
#define SROS_SECURITY_STATE_HANDLE_H

#include "core/state.h"

namespace security {

/** 急停触发源（VSC/CE/SRC）
 * 
 *  修改记录：新增SRC安全单元触发源，且扩充到4个字节（2021/10/18）
 * 
 *  SRC安全单元（独有）
 *  ESI4    ESI3    ESI2    ESI1    触发源
 *  0       1       1       1       外部自定义(0x0400)
 *  0       0       1       1       急停按钮(0x0001)
 *  0       0       0       1       触边(0x0010)
 *  0       0       0       0       内部自定义(0x0200)
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

class SecurityStateHandle {
 public:

    // SRC新增触急停发源，字节扩充到4字节
    void setNewState(sros::core::EmergencyState emergency_state, uint32_t emergency_src);

 private:
    void updateNavigationState(sros::core::NavigationState state);

    sros::core::EmergencySource extractEmergencySource(uint32_t emergency_src);

    // 解析触发源并输出详细信息供分析使用
    std::string extractEmergencySourceString(uint32_t emergency_src);

    // 根据急停触发源进行故障码报错
    void handleEmergencyFaultCode(sros::core::EmergencyState emergency_state, uint32_t emergency_src);

    bool is_waiting_emergency_recover_ = false;  // 是否正在等待安全单元恢复正常
};

}  // namespace security

#endif  // SROS_SECURITY_STATE_HANDLE_H
