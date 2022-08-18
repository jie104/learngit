//
// Created by lhx on 16-1-29.
//

#ifndef SROS_SRC_H
#define SROS_SRC_H

#include "src/sdk/src_sdk.h"

const char STM32_UPDATE_LOG[] = "/sros/stm32_update.log";
const char STM32_BACKUP_PATH[] = "/sros/stm32";
const char STM32_UPGRADE_DEFAULT_NAME[] = "stm32_upgrade.bin";

const auto default_upgrade_file_symlink =
    boost::filesystem::path(STM32_BACKUP_PATH) / boost::filesystem::path(STM32_UPGRADE_DEFAULT_NAME);

class Src;

// 全局变量
extern sdk::SrcSdk *src_sdk;
extern Src src_car;

enum SpeedLimitSource {
    SPEED_LIMIT_SOURCE_AREA = 1,
    SPEED_LIMIT_SOURCE_NAV,
    SPEED_LIMIT_SOURCE_USER,
};

enum PauseSource {
    PAUSE_SOURCE_USER = 1,
    PAUSE_SOURCE_NAV = 2,
    PAUSE_SOURCE_IO,
    PAUSE_SOURCE_FAULT,
};

// 暂停时的减速级别：0 - 5 ,值越小减速越快，越快停下来
const int SRC_MAX_PAUSE_LEVEL = 5;
const int SRC_MIN_PAUSE_LEVEL = 0;

/**
 * 我们给src加入一些sros层的逻辑
 */
class Src {
 public:
    Src(sdk::SrcSdk& sdk);
    void setSpeedLevel(SpeedLimitSource source, uint8_t speed_level);

    void updateSRCSpeedLevel();

    void setPauseState(PauseSource source, bool paused, int pause_level = SRC_MIN_PAUSE_LEVEL, bool is_latch = false);

    /**
     * 当src出现和sros不同的暂停状态时，调用此函数就会将sros的暂停状态更新到src
     */
    void updatePauseState();

    bool getPauseState(PauseSource source) const;

    /**
     * 每次重新开始任务时重置速度，重置暂停状态
     */
    void reset();

 private:
    sdk::SrcSdk& sdk_;

    // NOTE:
    // 由于我们的策略是速度没有变化就不设置速度，src刚开始速度是0，我们必须制造一次变化，让第一次跑路径的时候设置速度，不然src会以龟速运行，而不报错
    uint8_t nav_set_speed_level_ = 101;   // 导航控制的速度级别
    uint8_t user_set_speed_level_ = 100;  // 用户控制的速度级别
    uint8_t area_set_speed_level_ = 100;  // 根据所在区域设置的速度级别

    bool nav_set_paused_ = false;            // 导航控制的暂停
    bool user_set_paused_ = false;           // 用户控制的暂停
    bool fault_set_paused_ = false;          // 故障设置的暂停
    bool io_set_paused_ = false;             // IO设置的暂停
    int pause_level_ = SRC_MAX_PAUSE_LEVEL;  // 暂停时的减速级别：0 - 5 ,值越小减速越快，越快停下来
};

#endif  // SROS_SRC_H
