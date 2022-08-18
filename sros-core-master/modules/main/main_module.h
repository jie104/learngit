/**
 * @file main_module.h
 *
 * @author lhx
 * @date 2015年12月2日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef MODULES_MAIN_MAIN_MODULE_H_
#define MODULES_MAIN_MAIN_MODULE_H_

#include <future>
#include <string>
#include <vector>
#include <list>

#include "core/core.h"
#include "core/navigation_path.h"
#include "core/pose.h"

#include "core/msg/command_msg.hpp"
#include "core/msg/laser_scan_msg.hpp"
#include "core/msg/notification_msg.hpp"

#include "core/src.h"

#include "core/map/mark/AreaMark.hpp"
#include "core/task/action_task.h"
#include "core/task/movement_task.h"

namespace sros {

class MainModule : public core::Module {
 public:
    MainModule();

    virtual ~MainModule() = default;

    void run() final;

 private:
    bool is_src_connected_;  // SRC是否连接成功
    bool is_src_resetting_;  // 是否正在等待SRC重启完成
    bool is_navigation_okay_;
    bool is_slam_okay_;

    bool is_wait_for_draw_map_;  // 是否正在等待定位结束然后开始绘制地图

    bool need_convert_map_;  // 是否需要转换地图

    std::string cur_drawing_map_name_;  // 当前正在绘制地图名

    core::NavigationPathi_vector path_list_to_run_;

    core::Pose initial_pose_;

    int pre_io_pause_input_value_ = 0;  // 记录上次IO信号的值

    bool manual_enable_laser_oba_;  // 手动控制是否启用激光雷达避障

    sros::core::LoadState set_load_state_;

    sros::core::BatteryState pre_battery_state_;

    void stopMusic();

    //站点特征识别错误码
    enum FRFailedCode {
        FR_ERR_DEPARTURE = 1,       // 特征偏差过大
        FR_ERR_NOT_DISCERN = 2,     // 未识别到指定特征
        FR_ERR_NOT_MATCH = 3,       // 识别到特征但不匹配
    };

    void onFeatureStateMsg(const core::base_msg_ptr& m);

    void onSlamStateMsg(const core::base_msg_ptr& m);

    void onNavigationStateMsg(const core::base_msg_ptr& m);

    void handleSetCurrentMap(const core::CommandMsg_ptr& msg);

    sros::core::ResultState handleStartLocation(const core::CommandMsg_ptr& msg);

    void handleNewMovementTask(const core::base_msg_ptr& m);

    void handlePathReplace(const core::base_msg_ptr& m);

    sros::core::ResultState handleMapSwitching(core::CommandMsg_ptr m);

    void onPathMsg(const core::base_msg_ptr& m);

    void onDebugCmdMsg(const core::base_msg_ptr& m);

    void onMapUpdated(
        const core::base_msg_ptr& m);  // 响应地图被更新的函数，如地图被http更新了，若是当前地图，需要重新加载当前地图

    void startDoRunPath(const core::MovementTask_ptr& task, int32_t checkpoint);

    void startMoveTo(const core::MovementTask_ptr& task);

    void sendNavCommand(sros::core::NavigationCommand command);

    void sendSlamCommand(core::SLAM_COMMAND_TYPE command);

    void startSlamDrawMap(const std::string& map_name);

    void startSlamLocationManual(const std::string& map_name, bool absolute_location = false);
    // 重定位
    void startSlamRelocationManual(const std::string& map_name, bool absolute_location = false);

    void stopSlamLocation();

    void setPGVInfoToTask(const core::MovementTask_ptr& task, core::StationNo_t dst_station_no);

    void initSRCCar();

    void onSRCState(const SRCState &state);

    void onSRCPathFinish();

    void onSRCPathAborted();

    void onSRCPathPaused();

    void checkModuleInitState();

    /**
     * @brief
     * @param paths
     * @param auto_run
     * @param checkpoint 交通管制点，默认为0,0就不给src设置, -1为停在路径的最前面
     */
    void doRunPath(const core::NavigationPathi_vector& paths, bool auto_run = true, int32_t checkpoint = 0);

    void onTimer_20s(const core::base_msg_ptr& m);

    void onTimer_5s(const core::base_msg_ptr& m);

    void onTimer_1s(const core::base_msg_ptr& m);

    void onTimer_100ms(const core::base_msg_ptr& m);

    void onTimer_50ms(const core::base_msg_ptr& m);

    // 更新g_state.gpio的值
    void updateGPIOState();

    void performAreaOperation();

    void updateParameterByLoadState();

    void updateMovementTaskProcess();

    // 向各模块发送参数配置msg
    void sendModuleParameters();

    // 向src发送叉车参数
    void sendSrcForkParams();

    void stopAllModules();

    void onDMCodeInfoMsg(const sros::core::base_msg_ptr& m);

    void resetSRCCar();

    void checkChargeStation();

    void updateIOPauseState();

    void updateIOCancelEmergencyState();

    void checkUpSvc100ScanState();

    /**
     * 更新触摸屏翻转的状态，300c有触摸屏翻转的功能
     */
    void updateIOScreenTurnedUpState();

    void updateTrafficControlState() const;

    void updateNavState(sros::core::NavigationState state, int param_int = 0);

    void updateDownCameraRealtimeInfo();  // 更新pgv实时信息

    void loadingCheckPalletSignal();

    // 更新任务状态（暂停或继续）
    void updateActionState(const core::base_msg_ptr &msg,bool _bPauseOrConinue);

    bool obstacle_avoid_enable_;
    bool pre_area_disable_oba_;
    bool pre_area_no_enter_ = false;

    sros::core::SystemState sys_state_before_pause_;
    sros::core::NavigationState nav_state_before_pause_;

    sros::core::SystemState sys_state_before_stop_location_ = sros::core::SYS_STATE_ZERO;  // 停止暂停前的sys_state

    void responseCommand(const sros::core::CommandMsg_ptr &msg);

    void togglePowerMode();

    std::vector<sros::map::AreaMark> area_list_;  // 当前所在区域列表
    bool resetting_area_info_ = false;            // 标记是否需要重新检测区域信息。（如：
                                        // 地图的区域信息更新了，但是位置没有变化，此时需要重新设置区域参数）

    std::string target_switching_map_;                               // 需要切换的目标地图
    std::shared_ptr<std::promise<void>> promise_slam_state_change_;  // slam状态变化的promise

    std::list<int> lst_sliding_window_calc_;      //滑动窗口计算
    bool is_last_sliding_window_calc_ = false;            //上次是否使用滑动窗口计算
    int manual_control_time_record = 0; //手动控制状态下3min没有收到速度指令即退出

    bool is_src_check_parameters_;    //src参数校验标志。
};

}  // namespace sros

#endif  // MODULES_MAIN_MAIN_MODULE_H_
