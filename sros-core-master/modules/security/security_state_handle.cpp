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

#include "security_state_handle.h"
#include "core/fault_center.h"
#include "core/msg/command_msg.hpp"
#include "core/msg/common_state_msg.hpp"
#include "core/msg/emergency_msg.h"
#include "core/msg_bus.h"
#include "core/src.h"
#include "core/settings.h"
#include "core/alarm_record.hpp"

namespace security {

void SecurityStateHandle::setNewState(sros::core::EmergencyState emergency_state, uint32_t emergency_src) {
    // NOTE：由于vsc触发异常时，第一时间捕获不到急停触发源，所以不能是触发的那一瞬间就打印，正真的触发源，可能来自触发急停的下一帧。
    static uint32_t old_emergency_src = 0;
    if (old_emergency_src != emergency_src) {
        if (emergency_src == 0) {
            LOGGER(INFO, SROS) << "emergency source cleaned: 0x" << std::hex << old_emergency_src << " -> 0x00";
        } else {
            //封装接口用于区分VC300(vsc/ce)、VC400(src)触发源解析
            std::string source_str = extractEmergencySourceString(emergency_src);
            LOGGER(ERROR, SROS) << "emergency source changed: 0x" << std::hex << old_emergency_src << " -> 0x"
                                << std::hex << emergency_src << "(" << source_str
                                << ")";
            sros::core::AlarmRecord::getInstance().addEmergencyAlarmInfo(emergency_src, source_str);
        }
        old_emergency_src = emergency_src;
    }

    g_state.emergency_source = extractEmergencySource(emergency_src);
    static auto old_mergency_state = g_state.emergency_state;

    if (emergency_state == sros::core::STATE_EMERGENCY_NONE) {  // 正常状态（急停未触发）
        // 恢复系统急停状态
        g_state.emergency_state = sros::core::STATE_EMERGENCY_NONE;

        // 如果需要，向相关模块发送急停取消命令
        if (is_waiting_emergency_recover_) {
            // 向SRC发送急停继续
            src_sdk->emergencyContinue(EMERGENCY_PAUSE_EMERGENCY_STOP);
            LOG(INFO) << "Set src emergency continue, last emergency pause source is emergency stop!";

            is_waiting_emergency_recover_ = false;  // 清除标志
        }
    } else if (emergency_state == sros::core::STATE_EMERGENCY_TRIGER) {  //急停触发
        // 更新系统急停状态
        g_state.emergency_state = sros::core::STATE_EMERGENCY_TRIGER;

        if (!is_waiting_emergency_recover_) {  // 如果已处于急停状态，则不向相关模块在发送急停指令
            // 向SRC发送急停暂停
            LOG(INFO) << "Set src emergency pause for emergency stop!";
            src_sdk->emergencyPause(EMERGENCY_PAUSE_EMERGENCY_STOP, g_state.emergency_source);
        }

        // 设置标志，等待状态恢复正常后恢复其他模块状态
        is_waiting_emergency_recover_ = true;
    } else if (emergency_state == sros::core::STATE_EMERGENCY_RECOVERABLE) {  // 急停可恢复
        if (g_state.emergency_state == sros::core::STATE_EMERGENCY_NONE ||
            g_state.emergency_state == sros::core::STATE_EMERGENCY_NA) {
            // 从NONE状态直接转为“可恢复”说明触发了急停

            if (!is_waiting_emergency_recover_) {  // 如果已处于急停状态，则不向相关模块在发送急停指令
                // 向SRC发送急停暂停
                LOG(INFO) << "Set src emergency pause for emergency stop!";
                src_sdk->emergencyPause(EMERGENCY_PAUSE_EMERGENCY_STOP, g_state.emergency_source);
            }

            // 设置标志，等待状态恢复正常后恢复其他模块状态
            is_waiting_emergency_recover_ = true;
        }

        g_state.emergency_state = sros::core::STATE_EMERGENCY_RECOVERABLE;
    } else {
        g_state.emergency_state = sros::core::STATE_EMERGENCY_NONE;
    }

    if (old_mergency_state != g_state.emergency_state) {
        auto msg = std::make_shared<sros::core::EmergencyMsg>();
        msg->emergency_source = g_state.emergency_source;
        msg->emergency_state = g_state.emergency_state;
        sros::core::MsgBus::sendMsg(msg);

        old_mergency_state = g_state.emergency_state;
    }

    // 处理急停故障代码
    handleEmergencyFaultCode(emergency_state,emergency_src);
}

void SecurityStateHandle::updateNavigationState(sros::core::NavigationState state) {
    auto mm = std::make_shared<sros::core::CommonStateMsg<sros::core::NavigationState>>("NAV_STATE");
    mm->state = state;
    sros::core::MsgBus::getInstance()->sendMsg(mm);
}

sros::core::EmergencySource SecurityStateHandle::extractEmergencySource(uint32_t emergency_src) {
    auto out_emergency_src = sros::core::EMERGENCY_SRC_NONE;

    uint8_t d;
    d = (uint8_t)((emergency_src & 0x00F0) >> 4);
    if (d != 0) {
        if ((d & 0b0001) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_EDGE_1;
        } else if ((d & 0b0010) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_EDGE_2;
        } else if ((d & 0b0100) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_EDGE_3;
        } else if ((d & 0b1000) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_EDGE_4;
        }
    }

    d = (uint8_t)(emergency_src & 0x000F);
    if (d != 0) {
        if ((d & 0b0001) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_BUTTON_1;
        } else if ((d & 0b0010) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_BUTTON_2;
        } else if ((d & 0b0100) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_BUTTON_3;
        } else if ((d & 0b1000) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_BUTTON_4;
        }
    }

    d = (uint8_t)((emergency_src & 0xF000) >> 12);
    if (d != 0) {
        if ((d & 0b0001) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_SOFTWARE_1;
        } else if ((d & 0b0010) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_SOFTWARE_2;
        } else if ((d & 0b0100) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_SOFTWARE_3;
        } else if ((d & 0b1000) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_SOFTWARE_4;
        }
    }

    d = (uint8_t)((emergency_src & 0x0F00) >> 8);
    if (d != 0) {
        if ((d & 0b0001) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_BATTERY_DOOR;
        } else if ((d & 0b0010) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_INTERNAL_FAULT;
        } else if ((d & 0b0100) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_EXTERNAL_FAULT;
        }
    }

    d = (uint8_t)((emergency_src & 0x0F0000) >> 0x10);
    if (d != 0) {
        if ((d & 0b0001) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_NXP_ESTOP;
        } else if ((d & 0b0010) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_ST_ESTOP;
        } else if ((d & 0b0100) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_TK1_ESTOP;
        } else if ((d & 0b1000) != 0) {
            out_emergency_src = sros::core::EMERGENCY_SRC_ESTOP_OUT;
        }
    }
    return out_emergency_src;
}

std::string SecurityStateHandle::extractEmergencySourceString(uint32_t emergency_src) {
    std::string strRet = "";
    if ((emergency_src & 0x000F) != 0) {
        strRet += "emergency button";
    }
    
    if ((emergency_src & 0x00F0) != 0) {
        strRet += strRet.empty() ? "" : " && ";
        strRet += "edge trigger";
    }
    
    if ((emergency_src & 0x0F00) != 0) {
        strRet += strRet.empty() ? "" : " && ";
        if ((emergency_src & 0x0100) != 0) {
            strRet += "battery door opened";
        } else if ((emergency_src & 0x0400) != 0) {
            strRet += "internal defined";
        } else if ((emergency_src & 0x0800) != 0) {
            strRet += "external defined";
        } else {
            strRet += "safety lidar";
        }
    }
    
    if ((emergency_src & 0xF000) != 0) {
        strRet += strRet.empty() ? "" : " && ";
        if ((emergency_src & 0x8000) != 0) {
            strRet += "sros heart beat timeout";
        } else {
            strRet += "software trigger";
        }
    }
    
    if ((emergency_src & 0x10000) != 0) {
        strRet += strRet.empty() ? "" : " && ";
        strRet += "nxp estop";
    }
    
    if ((emergency_src & 0x20000) != 0) {
        strRet += strRet.empty() ? "" : " && ";
        strRet += "st estop";
    }
    
    if ((emergency_src & 0x40000) != 0) {
        strRet += strRet.empty() ? "" : " && ";
        strRet += "tk1 estop";
    }
    
    if ((emergency_src & 0x80000) != 0) {
        strRet += strRet.empty() ? "" : " && ";
        strRet += "estop out";
    }
    return strRet;
}

void SecurityStateHandle::handleEmergencyFaultCode(sros::core::EmergencyState emergency_state, uint32_t emergency_src)
{
    auto fault_center = sros::core::FaultCenter::getInstance();
    auto can_automatically_recover_func = [&]() -> bool { return g_state.isEmergencyRecoverable(); };
    auto try_to_recover_func = [&]() {
      auto d_msg = std::make_shared<sros::core::CommandMsg>("security");
      d_msg->command = sros::core::CMD_CANCEL_EMERGENCY;
      sros::core::MsgBus::sendMsg(d_msg);
    };
    auto is_emergency_func = [&]() -> bool{
      return emergency_state == sros::core::STATE_EMERGENCY_TRIGER || emergency_state == sros::core::STATE_EMERGENCY_RECOVERABLE;
    };

    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_BUTTON, [&]() { return is_emergency_func() && ((emergency_src & 0x000F) != 0); },
        can_automatically_recover_func, try_to_recover_func);
    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_EDGE, [&]() { return is_emergency_func() && ((emergency_src & 0x00F0) != 0); },
        can_automatically_recover_func, try_to_recover_func);
    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_SOFTWARE, [&]() { return is_emergency_func() && ((emergency_src & 0xF000) != 0); },
        can_automatically_recover_func, try_to_recover_func);
    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_BATTERY_DOOR_OPEN, [&]() { return is_emergency_func() && ((emergency_src & 0x0100) != 0); },
        can_automatically_recover_func, try_to_recover_func);
    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_UNKNOWN, [&]() { return is_emergency_func() && (emergency_src == 0); },
        can_automatically_recover_func, try_to_recover_func);

    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_NXP_ESTOP, [&]() { return is_emergency_func() && ((emergency_src & 0x10000) != 0); },
        can_automatically_recover_func, try_to_recover_func);
    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_ST_ESTOP, [&]() { return is_emergency_func() && ((emergency_src & 0x20000) != 0); },
        can_automatically_recover_func, try_to_recover_func);
    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_TK1_ESTOP, [&]() { return is_emergency_func() && ((emergency_src & 0x40000) != 0); },
        can_automatically_recover_func, try_to_recover_func);
    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_ESTOP_OUT, [&]() { return is_emergency_func() && ((emergency_src & 0x80000) != 0); },
        can_automatically_recover_func, try_to_recover_func);
    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_SRC_INTERNAL_FAULT, [&]() { return is_emergency_func() && ((emergency_src & 0x0200) != 0); },
        can_automatically_recover_func, try_to_recover_func);
    fault_center->track(
        sros::core::FAULT_CODE_EMERGENCY_TRIGGER_SRC_EXTERNAL_FAULT, [&]() { return is_emergency_func() && ((emergency_src & 0x0400) != 0); },
        can_automatically_recover_func, try_to_recover_func);
}

}  // namespace security
