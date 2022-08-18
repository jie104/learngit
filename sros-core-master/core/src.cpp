//
// Created by lhx on 16-1-29.
//

#include "src.h"
#include "src/sdk/src_sdk.h"
#include "src/sdk/src_sdk_v1.h"
#include "src/sdk/src_sdk_v2.h"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/common_state_msg.hpp"
#include "core/msg_bus.h"
#include "core/state.h"
#include "core/task/task_manager.h"
#include "settings.h"

// 全局变量
sdk::SrcSdk *src_sdk = new sdk::SrcSdk();
Src src_car(*src_sdk);

Src::Src(sdk::SrcSdk &sdk) : sdk_(sdk) {
    //    auto &s = sros::core::Settings::getInstance();
    //    user_set_speed_level_ = s.getValue<int>("main.user_speed_level", 100);
}
void Src::updateSRCSpeedLevel() {
    // src暂停后，会丢失speed level信息，需要在continue后立即重新设置speed level
    // 选择最小的速度作为当前速度level
    g_state.speed_level = std::min(std::min(nav_set_speed_level_, user_set_speed_level_), area_set_speed_level_);

    LOG(INFO) << "update speed level to " << static_cast<int>(g_state.speed_level)
              << "; nav_speed_=" << static_cast<int>(nav_set_speed_level_)
              << ", user_speed_=" << static_cast<int>(user_set_speed_level_)
              << ", area_speed_=" << static_cast<int>(area_set_speed_level_);

    src_sdk->setSpeedLevel(g_state.speed_level);
}

void Src::setSpeedLevel(SpeedLimitSource source, uint8_t speed_level) {
    if (speed_level < 0) {
        speed_level = 0;
    } else if (speed_level > 100) {
        speed_level = 100;
    }

    switch (source) {
        case SPEED_LIMIT_SOURCE_AREA: {
            if (speed_level != area_set_speed_level_) {
                LOG(INFO) << "area set speed level change: " << static_cast<int>(area_set_speed_level_) << " -> "
                          << static_cast<int>(speed_level);
                area_set_speed_level_ = speed_level;
            } else {
                return;
            }
            break;
        }
        case SPEED_LIMIT_SOURCE_NAV: {
            if (speed_level != nav_set_speed_level_) {
                LOG(INFO) << "navigation set speed level change: " << static_cast<int>(nav_set_speed_level_) << " -> "
                          << static_cast<int>(speed_level);
                nav_set_speed_level_ = speed_level;
            } else {
                return;
            }
            break;
        }
        case SPEED_LIMIT_SOURCE_USER: {
            if (speed_level != user_set_speed_level_) {
                LOG(INFO) << "user set speed level change: " << static_cast<int>(user_set_speed_level_) << " -> "
                          << static_cast<int>(speed_level);
                user_set_speed_level_ = speed_level;
            } else {
                return;
            }
            break;
        }
        default: {
            LOG(ERROR) << "unreachable!";
            break;
        }
    }

    updateSRCSpeedLevel();
}

void Src::setPauseState(PauseSource source, bool paused, int pause_level, bool is_latch) {
    if (pause_level < SRC_MIN_PAUSE_LEVEL || pause_level > SRC_MAX_PAUSE_LEVEL) {
        LOG(WARNING) << "updateSRCPauseState : illegal pause level value: " << pause_level;
        pause_level = SRC_MIN_PAUSE_LEVEL;
    }

    switch (source) {
        case PAUSE_SOURCE_USER: {
            user_set_paused_ = paused;
            break;
        }
        case PAUSE_SOURCE_NAV: {
            nav_set_paused_ = paused;
            break;
        }
        case PAUSE_SOURCE_IO: {
            io_set_paused_ = paused;
            break;
        }
        case PAUSE_SOURCE_FAULT: {
            fault_set_paused_ = paused;
            if (paused && is_latch) {
                user_set_paused_ = true;
                // FIXME(pengjiali) 重构状态机时，此处写法不优雅
                if (sros::core::TaskManager::getInstance()->isMovementTaskRunning()) {
                    auto m = std::make_shared<sros::core::CommonStateMsg<sros::core::NavigationState>>("NAV_STATE");
                    m->state = sros::core::STATE_NAV_PAUSED;
                    m->param_int = 0;
                    sros::core::MsgBus::sendMsg(m);

                    auto mm =
                        std::make_shared<sros::core::CommonCommandMsg<sros::core::NavigationCommand>>("NAV_COMMAND");
                    mm->command = sros::core::COMMAND_NAV_PAUSE;
                    sros::core::MsgBus::sendMsg(mm);
                }
            }
            break;
        }
        default: {
            LOG(ERROR) << "unreachable!!!";
            break;
        }
    }

    // 如果新设置的暂停减速级别值比当前级别值还要低，那么更新减速级别
    pause_level_ = (pause_level < pause_level_) ? pause_level : pause_level_;

    updatePauseState();
}

void Src::updatePauseState() {
    if (user_set_paused_ || nav_set_paused_ || fault_set_paused_ || io_set_paused_) {
        LOG(INFO) << "update pause state: user_set_paused_:" << user_set_paused_
                  << ", nav_set_paused_:" << nav_set_paused_ << ", fault_set_paused_:" << fault_set_paused_
                  << ", io_set_paused_:" << io_set_paused_;

        //if (sros::core::TaskManager::getInstance()->isMovementSlaveRunning()) {
            LOG(INFO) << "updateSRCPauseState true! level = " << pause_level_;
            src_sdk->pauseMovement(pause_level_);
        //}
    } else {
        LOG(INFO) << "updateSRCPauseState false!";
        src_sdk->continueMovement();

        pause_level_ = SRC_MAX_PAUSE_LEVEL;

        // src暂停后，会丢失speed level信息，需要在continue后立即重新设置speed level
        src_car.updateSRCSpeedLevel();
    }
}

bool Src::getPauseState(PauseSource source) const {
    auto paused = false;
    switch (source) {
        case PAUSE_SOURCE_USER: {
            paused = user_set_paused_;
            break;
        }
        case PAUSE_SOURCE_NAV: {
            paused = nav_set_paused_;
            break;
        }
        case PAUSE_SOURCE_IO: {
            paused = io_set_paused_;
            break;
        }
        case PAUSE_SOURCE_FAULT: {
            paused = fault_set_paused_;
            break;
        }
        default: {
            LOG(ERROR) << "unreachable!!!";
            break;
        }
    }

    return paused;
}
void Src::reset() {
    nav_set_speed_level_ = 100;

    nav_set_paused_ = false;
    setPauseState(PAUSE_SOURCE_USER, false);
    updateSRCSpeedLevel();
}
