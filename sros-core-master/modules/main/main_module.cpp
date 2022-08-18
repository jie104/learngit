/**
 * @file main_module.cpp
 *
 * @author lhx
 * @date 2015年12月2日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include <unistd.h>
#include <sys/reboot.h>
#include "modules/main/main_module.h"
#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <utility>

#include "core/device/device_manager.h"
#include "core/settings.h"

#include "core/msg_bus.h"

#include "core/map_manager.h"
#include "core/mission/mission_manager.h"
#include "core/task/task_manager.h"

#include "core/msg/SlamCommandMsg.h"
#include "core/msg/SlamStateMsg.h"
#include "core/msg/command_msg.hpp"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/common_msg.hpp"
#include "core/msg/common_state_msg.hpp"
#include "core/msg/data_matrix_code_msg.hpp"
#include "core/msg/hmi_msg.hpp"
#include "core/msg/parameter_msg.hpp"
#include "core/msg/path_msg.hpp"
#include "core/msg/str_msg.hpp"
#include "core/msg/usart_data_msg.hpp"

#include "core/util/md5.h"
#include "core/util/pose_saver.h"
#include "core/util/utils.h"

#include "core/version.h"

#include "core/log/run_logger.h"

#include "core/device/device.h"
#include "core/exec_error.hpp"
#include "core/fault_center.h"
#include "core/map/mark/AreaMark.hpp"
#include "core/meta_setting.h"
#include "core/src.h"
#include "core/user_manager.h"
#include "core/util/time.h"
#include "path_checker.h"
#include "pose_checker.h"
#include "modules/action_controller/action_manager.h"

using namespace std;
using namespace sros::core;
using namespace sros::map;

namespace sros {

MainModule::MainModule()
    : Module("Main"),
      is_src_connected_(false),
      is_src_check_parameters_(false),
      is_src_resetting_(false),
      is_navigation_okay_(false),
      is_slam_okay_(false),
      is_wait_for_draw_map_(false),
      need_convert_map_(true),
      manual_enable_laser_oba_(true),
      set_load_state_(LOAD_NONE),
      obstacle_avoid_enable_(false),
      pre_area_disable_oba_(false) {
    g_state.kernel_release = execShell("uname -r");
    auto &s = sros::core::Settings::getInstance();
    auto fleet_mode = s.getValue<std::string>("main.fleet_mode", "FLEET_MODE_OFFLINE");
    g_state.fleet_mode = (fleet_mode == "FLEET_MODE_ONLINE") ? FLEET_MODE_ONLINE : FLEET_MODE_OFFLINE;

    if (s.getValue<std::string>("device.enable_eac", "False") == "True" &&
        s.getValue<std::string>("main.action_unit", "") == "eac") {
        g_state.action_unit = ACTION_UNIT_EAC;
    }

    FaultCenter::getInstance();
}

void MainModule::checkModuleInitState() {
    auto &s = sros::core::Settings::getInstance();
    bool enable_vsc_module = (s.getValue<string>("main.enable_vsc", "False") == "True");

    // 等待电源初始化成功后才允许去初始化其他硬件设备
    if (enable_vsc_module) {
        for (auto i = 0; i < 10 * 20; ++i) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
            if (g_state.power_state == POWER_NORMAL) {
                break;
            }
            LOGGER(INFO, SROS) << "Waiting battery initialization, current power state is " << g_state.power_state;
        }
    }

    initSRCCar();

    for (auto i = 0; i < 10 * 40; ++i) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

        if (!is_navigation_okay_) {
            LOGGER(INFO, SROS) << "Waiting navigation initialization, current navigation state is "
                               << g_state.nav_state;
            continue;
        }
        if (!is_slam_okay_) {
            LOGGER(INFO, SROS) << "Waiting slam initialization, current slam state is " << g_state.slam_state;
            continue;
        }
        if (!is_src_connected_) {
            LOGGER(INFO, SROS) << "Waiting src initialization";
            // src重启要17秒，这里设置成20s src还未初始化成功
            if (i == 200) {
                src_sdk->setConnSrcFailFlag(true);
            }
            continue;
        }

        if (g_state.isLaserError()) {
            LOGGER(INFO, SROS)
                << "Waiting laser initialization, current laser state is "
                << device::DeviceManager::getInstance()->getDeviceByName(device::DEVICE_LIDAR)->getState();
            continue;
        }
        g_state.main_state = STATE_IDLE;
        g_state.sys_state = SYS_STATE_IDLE;
        g_state.ready_for_new_movement_task = false;
        LOG(INFO) << "初始化完成!";

        auto &s = sros::core::Settings::getInstance();
        bool auto_start_location = s.getValue<string>("main.auto_start_location", "True") == "True";
        auto auto_start_location_delay = s.getValue<int>("main.auto_start_location_delay", 0);
        auto auto_start_location_by_station_no = s.getValue<uint16_t>("main.auto_start_location_by_station_no", 0);
        if (auto_start_location) {
            LOG(INFO) << "自动启动定位";
            std::string map_name;
            if (PoseSaver::load(initial_pose_, map_name, g_state.station_no)) {
                if (!MapManager::getInstance()->setCurrentMap(map_name)) {
                    LOG(INFO) << "load map failed!, cannot auto start location!";
                } else {
                    g_state.setCurMapName(map_name);
                    LOGGER(INFO, SROS) << "Last map is " << map_name << ", last station is " << g_state.station_no
                                       << ", last pose is " << initial_pose_;

                    if (auto_start_location_by_station_no == 0) {  // 默认不按照站点定位
                        // 如果此处设置的站点不正确，当开启station_auto_detect功能时，会自动清除或设置为正确的站点
                        LOG(INFO) << "Auto start location after " << auto_start_location_delay << " seconds!";

                        std::thread t([this, auto_start_location_delay, map_name] {
                            std::this_thread::sleep_for(std::chrono::seconds(auto_start_location_delay));
                            startSlamLocationManual(map_name);
                        });
                        t.detach();

                    } else {
                        // 开机的时候强制按照站点定位，主要针对那些只有一个地方能定位成功
                        // 当定位失败时，将agv推到对应的站点，断电重启即可
                        LOGGER(INFO, SROS) << "Auto start location by station " << g_state.station_no << " after "
                                           << auto_start_location_delay << " seconds!";

                        // 带有站点信息,说明是使用站点位姿作为初始位姿
                        auto station = MapManager::getInstance()->getStation(auto_start_location_by_station_no);
                        if (station.id == auto_start_location_by_station_no) {
                            initial_pose_.x() = station.pos.x / 100;
                            initial_pose_.y() = station.pos.y / 100;
                            initial_pose_.yaw() = station.pos.yaw;
                            std::thread t([this, auto_start_location_delay, map_name] {
                                std::this_thread::sleep_for(std::chrono::seconds(auto_start_location_delay));
                                startSlamLocationManual(map_name);
                            });
                            t.detach();
                        } else {  // 地图中不存在这个站点
                            LOGGER(ERROR, SROS) << "Location by station " << auto_start_location_by_station_no
                                                << ", but station not exist in map " << map_name;
                        }
                    }
                }
            } else {
                LOGGER(ERROR, SROS) << "Auto start location failed, can not get the last info.";
            }
        }
        break;
    }
    sendModuleParameters();

    std::string vehicleType = s.getValue<std::string>("main.vehicle_type","");
    std::string type = vehicleType.substr(0,4);
    if(type == "gulf" || type == "Gulf"){   //叉车系列
        sendSrcForkParams();
    }
   

    if (g_state.main_state == STATE_IDLE) {
        // 发送start Msg来启动Timer等module
        auto msg = make_shared<core::StrMsg>("COMMAND");
        msg->data = "start";
        sendMsg(msg);

        // 记录本次开机信息
        auto &run_logger = sros::core::RunLogger::getInstance();
        run_logger.newRun(SROS_VERSION_STR, src_sdk->getSRCVersionStr());
        // 将当前里程值作为零点偏移量, 现在src和sros一同重启，所以默认值为0，防止默认值为未初始化的情况出现
        // run_logger.setMileageOffset(g_src_state.total_mileage);
    } else {
        LOG(ERROR) << "SROS初始化失败!";
        g_state.main_state = STATE_IDLE;
        g_state.sys_state = SYS_STATE_IDLE;

        if (!is_src_connected_) {
            LOG(ERROR) << "-> SRC连接失败";
        }

        if (!is_navigation_okay_) {
            LOG(ERROR) << "-> Navigation初始化失败";
        }

        if (!is_slam_okay_) {
            LOG(ERROR) << "-> SLAM初始化失败";
        }
    }
}

void MainModule::run() {
    LOG(INFO) << "MainModule 正在初始化...";

    subscribeTopic("TOPIC_SLAMSTATE", CALLBACK(&MainModule::onSlamStateMsg));
    subscribeTopic("TOPIC_ALIGNMENT_STATE", CALLBACK(&MainModule::onFeatureStateMsg));
    subscribeTopic("NAV_STATE", CALLBACK(&MainModule::onNavigationStateMsg));
    subscribeTopic("NAV_PATH", CALLBACK(&MainModule::onPathMsg));
    subscribeTopic("DM_CODE_INFO", CALLBACK(&MainModule::onDMCodeInfoMsg));

    subscribeTopic("DEBUG_CMD", CALLBACK(&MainModule::onDebugCmdMsg));
    subscribeTopic("TOPIC_UPDATE_MAP", CALLBACK(&MainModule::onMapUpdated));

    subscribeTopic("TIMER_20S", CALLBACK(&MainModule::onTimer_20s));
    subscribeTopic("TIMER_5S", CALLBACK(&MainModule::onTimer_5s));
    subscribeTopic("TIMER_1S", CALLBACK(&MainModule::onTimer_1s));
    subscribeTopic("TIMER_100MS", CALLBACK(&MainModule::onTimer_100ms));
    subscribeTopic("TIMER_50MS", CALLBACK(&MainModule::onTimer_50ms));

    boost::thread t(boost::bind(&MainModule::checkModuleInitState, this));

    dispatch();

    LOG(INFO) << "MainModule停止执行.";

    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    sros::core::MsgBus::getInstance()->stop();

    stopAllModules();
}

void MainModule::onPathMsg(const core::base_msg_ptr &m) {
    auto msg = dynamic_pointer_cast<PathMsg>(m);
    LOG(INFO) << "收到navigation生成的路径 :" << msg->paths.size();
    for (auto p : msg->paths) {
        LOG(INFO) << "-> 路径"
                  << ": 类型" << p.type_ << "(sx,sy)= (" << p.sx_ << "," << p.sy_ << "); (ex,ey)= (" << p.ex_ << ","
                  << p.ey_ << ") rotate_angle = " << p.rotate_angle_;
        LOG(INFO) << "-> 方向"
                  << ": " << p.direction_;
    }

    try{
        if (msg->paths.size() >= 128){
            throw EXEC_ERROR(ERROR_CODE_MOVEMENT_PATHS_IS_TOO_LONG,"path is more than 128");
        }
    } catch (const ExecError & e) {
        SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_MOVEMENT_PATHS_IS_TOO_LONG,"path is more than 128");
        sendNavCommand(COMMAND_NAV_CANCEL);
        return;
    }

    try {
        PathChecker::checkDisableRotateArea(msg->paths);
    } catch (const ExecError &error) {
        SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_MOVEMENT_ROTATE_IN_DISABLE_ROTATE_AREA, "");
        sendNavCommand(COMMAND_NAV_CANCEL);
        return;
    }
    try {
        PathChecker::checkChassisType(msg->paths);
    } catch (const ExecError &error) {
        SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_MOVEMENT_PATHS_CHASSIS_NOT_SUPPORT, "");
        sendNavCommand(COMMAND_NAV_CANCEL);
        return;
    }

    NavigationPathi_vector paths_in_mm;

    // from m to mm
    for (const auto &p : msg->paths) {
        sros::core::NavigationPathi pp;

        pp.type_ = p.type_;

        pp.sx_ = static_cast<int>(p.sx_ * 1000);
        pp.sy_ = static_cast<int>(p.sy_ * 1000);
        pp.ex_ = static_cast<int>(p.ex_ * 1000);
        pp.ey_ = static_cast<int>(p.ey_ * 1000);
        pp.cx_ = static_cast<int>(p.cx_ * 1000);
        pp.cy_ = static_cast<int>(p.cy_ * 1000);
        pp.dx_ = static_cast<int>(p.dx_ * 1000);
        pp.dy_ = static_cast<int>(p.dy_ * 1000);

        pp.radius_ = (p.radius_ * 1000);
        pp.rotate_angle_ = (p.rotate_angle_ * 1000);
        pp.limit_v_ = static_cast<int>(p.limit_v_ * 1000);
        pp.limit_w_ = static_cast<int>(p.limit_w_ * 1000);

        pp.direction_ = p.direction_;

        paths_in_mm.push_back(pp);
    }

    path_list_to_run_ = paths_in_mm;

    auto movement_task = TaskManager::getInstance()->getMovementTask();
    if (movement_task->isSlaveRunning()) {
        // 运行过程中最重新规划路径
        movement_task->pathsReplace(paths_in_mm);
    } else {
        //    auto task = TaskManager::getInstance()->getMovementTask();
        //    if (g_state.load_state == sros::core::LOAD_FULL &&
        //                task->getDstStationType() == sros::core::DS_NO_ROTATE &&
        //                !path_list_to_run_.empty() && path_list_to_run_.back().type_ == 4) {
        //         LOG(INFO)<<"Remove last rotate path!";
        //         path_list_to_run_.pop_back();
        //     }
        // 任务开始规划的路径
        movement_task->setPaths(paths_in_mm);

        LOG(INFO) << "发送给SRC的路径 :" << path_list_to_run_.size();
        for (auto p : path_list_to_run_) {
            LOG(INFO) << "-> 路径"
                      << ": 类型" << p.type_ << "(sx,sy)= (" << p.sx_ << "," << p.sy_ << "); (ex,ey)= (" << p.ex_ << ","
                      << p.ey_ << " ) rotate_angle = " << p.rotate_angle_;
            LOG(INFO) << "-> 方向"
                      << ": " << p.direction_;

            LOG(INFO) << "-> s_facing_= " << p.s_facing_ << " e_facing_ " << p.e_facing_ << " control point ( " << p.cx_
                      << "," << p.cy_ << " ) control point 2" << p.dx_ << "," << p.dy_ << " limit_w_ " << p.limit_w_
                      << " limit_v " << p.limit_v_;
        }

        // 自由导航遇到障碍物减速，暂停后，重新规划路线后造成速度丢失
        // 因此重新规划路径或者导航结束，都应该重置速度
        src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);

        // FIXME(pengjiali):重构状态机
        // 应该和执行路径一样，由避障判断没有障碍的情况下才启动，而不是启动后，然后去检测障碍，这样会导致前面有障碍也会走一点点
        doRunPath(path_list_to_run_, true);
    }
}

/**
 * @brief 雷达状态切换
 * @param m 雷达状态消息
 */
void MainModule::onSlamStateMsg(const core::base_msg_ptr &m) {
    auto msg = std::dynamic_pointer_cast<core::SlamStateMsg>(m);

    auto pre_slam_state = g_state.slam_state;
    g_state.slam_state = msg->slam_state;  // 更新slam状态
    LOG(INFO) << "slam state update: " << pre_slam_state << " -> " << g_state.slam_state
              << " main_state: " << g_state.main_state << " location_state: " << g_state.location_state;

    if (promise_slam_state_change_) {
        promise_slam_state_change_->set_value();
    }

    if (g_state.main_state == STATE_INITIALING) {
        if (g_state.slam_state == STATE_SLAM_IDLE) {
            LOG(INFO) << "SLAM 初始化完成";
            is_slam_okay_ = true;
        }
    } else if (g_state.main_state == STATE_TASK_NEW_MAP) {
        if (g_state.slam_state == STATE_SLAM_DRAWING) {
            if (pre_slam_state == STATE_SLAM_LOCATING_AMCL) {
                // 从重定位状态中恢复
                LOG(INFO) << "绘图:从重定位状态中恢复";
            } else {
                // 开始绘图
                src_sdk->stop();
                src_sdk->enterVelocityMode();
                src_sdk->run();
                LOG(INFO) << "开始绘图";
                g_state.sys_state = SYS_STATE_TASK_NEWMAP_DRAWING;
            }
            g_state.location_state = LOCATION_STATE_RUNNING;
            src_sdk->setPoseAvailable(true);
        } else if (g_state.slam_state == STATE_SLAM_LOCATING_AMCL) {
            LOG(WARNING) << "绘图:STATE_SLAM_LOCATING_AMCL";
            g_state.location_state = LOCATION_STATE_RELOCATING;
        } else if (g_state.slam_state == STATE_SLAM_LOCATING) {
        } else if (g_state.slam_state == STATE_SLAM_ERROR) {
            src_sdk->stop();
            LOG(ERROR) << "绘图:STATE_SLAM_ERROR";
            g_state.location_state = LOCATION_STATE_ERROR;
            src_sdk->setPoseAvailable(false);
        } else if (g_state.slam_state == STATE_SLAM_SAVING_MAP) {
            if (pre_slam_state != STATE_SLAM_SAVING_MAP) {
                // 第一次收到SAVING_MAP才进行stop操作
                src_sdk->stop();
                LOG(INFO) << "绘图:STATE_SLAM_SAVING_MAP";
                g_state.sys_state = SYS_STATE_TASK_NEWMAP_SAVING;
                src_sdk->setPoseAvailable(false);
            }
            // 每次收到SAVING_MAP都更新progress
            g_state.progress = msg->progress;  // 更新保存进度
            //            DLOG(INFO) << "g_state.progress: " << static_cast<int> g_state.progress;
        } else if (g_state.slam_state == STATE_SLAM_IDLE) {
            if (is_wait_for_draw_map_) {
                is_wait_for_draw_map_ = false;
                LOG(INFO) << "定位已停止,正在启动SLAM绘图";
                // 启动SLAM绘图
                startSlamDrawMap(cur_drawing_map_name_);
            } else {
                // 绘图任务SLAM模块彻底结束
                src_sdk->stop();
                src_sdk->enterPathMode();
                LOG(INFO) << "绘图任务SLAM模块彻底结束";

                // 开始地图转换任务
                if (need_convert_map_ && pre_slam_state != STATE_SLAM_ERROR) {
                    LOG(INFO) << "开始转换任务:STATE_SLAM_IDLE";
                    auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("NAV_COMMAND");
                    mm->command = COMMAND_NAV_CONVERT_MAP;
                    mm->str0 = getGrayMapPath(cur_drawing_map_name_);
                    mm->str1 = getNavigationMapPath(cur_drawing_map_name_);
                    sendMsg(mm);
                } else {
                    LOG(INFO) << "绘图结束，不需要转换地图，恢复系统状态";
                    g_state.main_state = STATE_IDLE;  // 恢复系统状态
                    g_state.sys_state = SYS_STATE_IDLE;
                    g_state.ready_for_new_movement_task = false;
                    g_state.location_state = LOCATION_STATE_NONE;  // 绘图完成后，即使没有保存地图，定位状态也是未启动
                }
                src_sdk->setPoseAvailable(false);
            }
        }
    } else if (g_state.main_state == STATE_TASK_NAVIGATING) {
        if (g_state.slam_state == STATE_SLAM_LOCATING_AMCL) {
            LOG(WARNING) << "定位:STATE_SLAM_LOCATING_AMCL";
            g_state.location_state = LOCATION_STATE_RELOCATING;
        } else if (g_state.slam_state == STATE_SLAM_ERROR) {
            g_state.location_state = LOCATION_STATE_ERROR;
            LOG(ERROR) << "定位:STATE_SLAM_ERROR";
        } else if (g_state.slam_state == STATE_SLAM_IDLE) {
            LOG(ERROR) << "定位:STATE_SLAM_IDLE";
        } else if (g_state.slam_state == STATE_SLAM_LOCATING) {
            if (pre_slam_state == STATE_SLAM_LOCATING_AMCL) {
                // 从重定位状态中恢复
                LOG(INFO) << "user set paused: " << src_car.getPauseState(PAUSE_SOURCE_USER);
                if (src_car.getPauseState(PAUSE_SOURCE_USER)) {
                    g_state.sys_state = SYS_STATE_TASK_MANUAL_PAUSED;
                } else {
                    if (sys_state_before_stop_location_ == SYS_STATE_ZERO) {
                        g_state.sys_state = SYS_STATE_TASK_PATH_WAITING_FINISH;
                    } else {
                        g_state.sys_state = sys_state_before_stop_location_;
                    }
                }
                LOG(INFO) << "定位:从重定位状态中恢复";
                g_state.location_state = LOCATION_STATE_RUNNING;
            } else {
                // 定位初始化完成,启动Navigation
            }
        }
    } else if (g_state.main_state == STATE_TASK_RUN_PATH) {
        if (g_state.slam_state == STATE_SLAM_LOCATING_AMCL) {
            LOG(WARNING) << "定位:STATE_SLAM_LOCATING_AMCL";
            g_state.location_state = LOCATION_STATE_RELOCATING;
        } else if (g_state.slam_state == STATE_SLAM_ERROR) {
            LOG(ERROR) << "定位:STATE_SLAM_ERROR";
            g_state.location_state = LOCATION_STATE_ERROR;
        } else if (g_state.slam_state == STATE_SLAM_WAITING_INITIAL_POSE) {
        } else if (g_state.slam_state == STATE_SLAM_LOCATING) {
            // 定位完成,可以执行路径
            LOG(INFO) << "定位完成,正在执行路径...";
            if (src_car.getPauseState(PAUSE_SOURCE_USER)) {
                g_state.sys_state = SYS_STATE_TASK_MANUAL_PAUSED;
            } else {
                if (sys_state_before_stop_location_ == SYS_STATE_ZERO) {
                    g_state.sys_state = SYS_STATE_TASK_PATH_WAITING_FINISH;
                } else {
                    g_state.sys_state = sys_state_before_stop_location_;
                }
            }
            g_state.location_state = LOCATION_STATE_RUNNING;
        } else if (g_state.slam_state == STATE_SLAM_IDLE) {
            LOG(ERROR) << "定位:STATE_SLAM_IDLE";
        }

    } else if (g_state.main_state == STATE_IDLE) {
        if (g_state.slam_state == STATE_SLAM_LOCATING) {
            LOGGER(INFO, SROS) << "Location succeed!";
            g_state.sys_state = SYS_STATE_IDLE;
            g_state.ready_for_new_movement_task = false;
            g_state.location_state = LOCATION_STATE_RUNNING;
            src_sdk->setPoseAvailable(true);
        } else if (g_state.slam_state == STATE_SLAM_ERROR) {
            g_state.location_state = LOCATION_STATE_ERROR;
            LOGGER(ERROR, SROS) << "Location error!";
        } else if (g_state.slam_state == STATE_SLAM_IDLE) {
            LOGGER(ERROR, SROS) << "Location error!";
            g_state.sys_state = SYS_STATE_IDLE;
            g_state.ready_for_new_movement_task = false;
            g_state.location_state = LOCATION_STATE_NONE;
            src_sdk->setPoseAvailable(false);

            // 开机自动设置站点编号后，如果定位失败，则清除当前站点编号
            // 其他情况下，如果定位失败，也清除当前站点信息，暂时未发现有副作用
            g_state.station_no = 0;
        } else if (g_state.slam_state == STATE_SLAM_LOCATING_AMCL) {
            if (pre_slam_state == STATE_SLAM_IDLE) {
                LOG(INFO) << "定位初始化";
                g_state.location_state = LOCATION_STATE_INITIALING;
            }
            if (pre_slam_state == STATE_SLAM_SIFT_LOCATING) {
                LOG(INFO) << "SIF过程结束,正在AMCL初始化...";
            } else {
                LOG(INFO) << "定位AMCL过程";
                g_state.location_state = LOCATION_STATE_RELOCATING;
            }
            src_sdk->setPoseAvailable(false);
        } else if (g_state.slam_state == STATE_SLAM_SIFT_LOCATING) {
            LOG(INFO) << "正在执行SIFT过程...";
            g_state.location_state = LOCATION_STATE_INITIALING;
        }
    }

    if (g_state.slam_state == STATE_SLAM_LOCATING) {
        sys_state_before_stop_location_ = sros::core::SYS_STATE_ZERO;
    }

    LOG(INFO) << "slam state updated: location_state: " << g_state.location_state
              << " sys_state: " << g_state.sys_state;
}

/**
 * @brief 站点特征信息识别到站确认状态信息回调
 * @param m 状态消息
 */
void MainModule::onFeatureStateMsg(const core::base_msg_ptr& m) {
    
    LOG(INFO) << "onFeatureStateMsg";
    g_state.enable_station_feature_recognize = false;

    auto task = TaskManager::getInstance()->getMovementTask();
    if (task && !task->isFinished()) {
        if (task->isInCancel()) {
            uint32_t reason = 0;
            if(g_state.isManualControl()){
                reason = sros::core::ERROR_CODE_TASK_FAILED_BY_MANUAL_BTN;
            }
            TaskManager::getInstance()->setMovementFinishCanceled(reason);
        } else {
            
            auto msg = std::dynamic_pointer_cast<CommonStateMsg<bool>>(m);
            LOG(INFO) << "站点特征高精度对接算法返回结果,state：" << msg->state;
            //对接失败
            if(!msg->state) {
                LOG(ERROR) << "站点特征高精度对接失败,failed_code:" << msg->failed_code_;
                if(msg->failed_code_ == FR_ERR_DEPARTURE) {
                    SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_FEATURE_RECOGNIZE_DEPARTURE, "");
                } else if(msg->failed_code_ == FR_ERR_NOT_DISCERN) {
                    SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_FEATURE_RECOGNIZE_NOT_DISCERN, "");
                } else if(msg->failed_code_ == FR_ERR_NOT_MATCH) {
                    SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_FEATURE_RECOGNIZE_NOT_MATCH, "");
                } else {
                    SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_FEATURE_RECOGNIZE_NOT_DISCERN, "");
                }
            } else if (PoseChecker::checkMovementTaskArriveDest()) {  //开启目标位置检测
                TaskManager::getInstance()->setMovementFinishSucceed();
                g_state.playMusic(hmi::MUSIC_ID_MUSIC_ARRIVED);
                LOG(INFO) << "站点特征高精度对接成功";
            } else {
                SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_AGV_NOT_ARRIVED_DEST, "");
                LOG(ERROR) << "站点特征高精度对接成功, 但目标站点位姿校验失败";
            }
        }
    }

    // 重置nav速度限制
    src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);
    LOG(INFO) << "Navigation 彻底结束";
}

/**
 * @brief 导航状态切换
 * @param m 导航状态消息
 */
void MainModule::onNavigationStateMsg(const core::base_msg_ptr &m) {
    auto msg = std::dynamic_pointer_cast<CommonStateMsg<NavigationState>>(m);

    auto &s = sros::core::Settings::getInstance();
    auto nav_slow_speed_level = s.getValue<int>("main.nav_slow_speed_level", 10);

    auto pre_nav_state = g_state.nav_state;

    if (msg->state == STATE_NAV_CONTINUE) {
        // STATE_NAV_CONTINUE 特殊处理，不是一个NavigationState
        LOG(INFO) << "onNavigationStateMsg() : STATE_NAV_CONTINUE";
    } else {
        g_state.nav_state = msg->state;
        LOG(INFO) << "onNavigationStateMsg() navigation state update: " << g_state.nav_state;
    }

    auto task = TaskManager::getInstance()->getMovementTask();

    if (g_state.main_state == STATE_INITIALING) {
        if (msg->state == STATE_NAV_IDLE) {
            LOG(INFO) << "Navigation 初始化完毕";
            is_navigation_okay_ = true;
        }
    } else if (g_state.main_state == STATE_TASK_NEW_MAP) {
        if (msg->state == STATE_NAV_CONVERTING_MAP) {
            LOG(INFO) << "Navigation 开始转换地图:STATE_NAV_CONVERTING_MAP";
        } else if (msg->state == STATE_NAV_IDLE) {
            LOG(INFO) << "Navigation 转换地图任务结束";
            LOG(INFO) << "绘制地图任务彻底结束:STATE_NAV_IDLE";
            g_state.main_state = STATE_IDLE;  // 恢复系统状态
            g_state.sys_state = SYS_STATE_IDLE;
            g_state.ready_for_new_movement_task = false;
            g_state.location_state = LOCATION_STATE_NONE;

            g_state.progress = 0;
        }
    } else if (g_state.main_state == STATE_TASK_NAVIGATING) {
        if (msg->state == STATE_NAV_INITIALIZING) {
            src_sdk->enterPathMode();
            LOG(INFO) << "Navigation 正在初始化 ";
            g_state.sys_state = SYS_STATE_TASK_NAV_INITIALING;
        } else if (msg->state == STATE_NAV_PATH_FINDING) {
            LOG(INFO) << "Navigation 正在寻路";
            g_state.sys_state = SYS_STATE_TASK_NAV_FINDING_PATH;
        } else if (msg->state == STATE_NAV_WAITING_FOR_FINISH) {
            LOG(INFO) << "Navigation 正在等待路径执行结束";
            if (pre_nav_state == STATE_NAV_WAITING_FOR_FINISH_SLOW) {
                // 恢复原速度行驶
                src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);
            } else if (pre_nav_state == STATE_NAV_PATH_PAUSED) {
                // 继续行驶
                src_car.setPauseState(PAUSE_SOURCE_NAV, false);

                // 恢复原速度行驶
                // 由于src的bug，继续运动时需要重新设置speed_level
                src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);
            }

            g_state.sys_state = SYS_STATE_TASK_NAV_WAITING_FINISH;

            if (task && !task->isFinished()) {
                TaskManager::getInstance()->setMovementRunning();
            }
        } else if (msg->state == STATE_NAV_WAITING_FOR_FINISH_SLOW) {
            if (pre_nav_state == STATE_NAV_PATH_ERROR) {
                return;
            }

            LOGGER(INFO, SROS) << "AGV slow down for obstacles detected!";
            if (pre_nav_state == STATE_NAV_PATH_PAUSED) {
                src_car.setPauseState(PAUSE_SOURCE_NAV, false);
            }
            LOG(INFO) << "nav_slow_speed_level: " << nav_slow_speed_level;
            src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, nav_slow_speed_level);

            g_state.sys_state = SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW;
        } else if (msg->state == STATE_NAV_PATH_REFINDING) {
            LOGGER(INFO, SROS) << "AGV re-plan the path due to obstacles detected!";
            g_state.sys_state = SYS_STATE_TASK_NAV_REFINDING_PATH;
        } else if (msg->state == STATE_NAV_PATH_REFINDING_PAUSED) {
            LOGGER(INFO, SROS) << "AGV stop move and re-plan the path due to obstacles detected!";
            src_sdk->stop();  // 暂停运动
            if (pre_nav_state == STATE_NAV_WAITING_FOR_FINISH_SLOW) {
                src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);
            }
            g_state.sys_state = SYS_STATE_TASK_NAV_PAUSED;
        } else if (msg->state == STATE_NAV_PATH_PAUSED) {
            if (pre_nav_state == STATE_NAV_PATH_ERROR) {
                return;
            }

            LOGGER(WARNING, SROS) << "AGV stop move due to obstacles detected!";
            // 暂停运动
            src_car.setPauseState(PAUSE_SOURCE_NAV, true, msg->param_int);
            if (pre_nav_state == STATE_NAV_WAITING_FOR_FINISH_SLOW) {
                src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);
            }
            g_state.sys_state = SYS_STATE_TASK_NAV_PAUSED;
        } else if (msg->state == STATE_NAV_PAUSED) {
            LOG(INFO) << "Navigation paused ...";
            if (g_state.sys_state == SYS_STATE_TASK_MANUAL_PAUSED) {
                LOG(INFO) << "agv already in paused!";
                return;
            }

            src_car.setPauseState(PAUSE_SOURCE_NAV, true, msg->param_int);

            // LOG(INFO)<<"store state to "<<pre_nav_state<<","<<g_state.sys_state;
            // store state !
            nav_state_before_pause_ = pre_nav_state;
            sys_state_before_pause_ = g_state.sys_state;

            g_state.sys_state = SYS_STATE_TASK_MANUAL_PAUSED;

            if (task && !task->isFinished()) {
                TaskManager::getInstance()->setMovementPause();
            }
        } else if (msg->state == STATE_NAV_CONTINUE) {
            LOG(INFO) << "Navigation continue ...";

            if (pre_nav_state != STATE_NAV_PAUSED) {
                LOG(WARNING) << "STATE_NAV_CONTINUE: invalid operation!, pre_nav_state is " << pre_nav_state;
                return;
            }

            // restore state !
            g_state.nav_state = nav_state_before_pause_;
            g_state.sys_state = sys_state_before_pause_;

            if (nav_state_before_pause_ == STATE_NAV_PATH_PAUSED) {
                src_car.setPauseState(PAUSE_SOURCE_NAV, true, msg->param_int);
            } else {
                src_sdk->run();  // 可能会有导航重新规划路径
                src_car.setPauseState(PAUSE_SOURCE_NAV, false);
            }

            // LOG(INFO)<<"restore state to "<<g_state.nav_state<<","<<g_state.sys_state;

            if (task && !task->isFinished()) {
                TaskManager::getInstance()->setMovementRunning();
            }
        } else if (msg->state == STATE_NAV_LOCATING_STATION) {
            LOG(INFO) << "Navigation 正在检测目标站点位置";
        } else if (msg->state == STATE_NAV_PATH_ERROR) {
            LOG(WARNING) << "Navigation 偏离路径！";
            g_state.sys_state = SYS_STATE_TASK_NAV_PATH_ERROR;
        } else if (msg->state == STATE_NAV_IDLE) {
            src_sdk->stop();
            g_state.main_state = STATE_IDLE;

            if (pre_nav_state == STATE_NAV_LOCATING_STATION && !path_list_to_run_.empty()) {
                // 如果路径为空,是"无法到达目标站点"的情况
                LOG(INFO) << "无法检测到目标站点";
                g_state.sys_state = SYS_STATE_TASK_NAV_NO_STATION;
                if (task && !task->isFinished()) {
                    SET_MOVEMENT_EXEC_FAILED(msg->failed_code_, "");
                }

            } else if (path_list_to_run_.empty()) {  // path列表为空,说明无路径可达
                LOG(INFO) << "无法到达目标位置";
                g_state.sys_state = SYS_STATE_TASK_NAV_NO_WAY;

                if (task && !task->isFinished()) {
                    SET_MOVEMENT_EXEC_FAILED(msg->failed_code_, "");
                }
            } else {
                PoseChecker::detectStationWhenMovementTaskFinished();
                LOG(INFO) << "Navigation 彻底结束，目标站点：" << g_state.station_no << " 当前位置："
                          << src_sdk->getCurPose();

                if (!task->remainPoseEmpty() || !task->remainStationEmpty()) {
                    // 任务的目标列表不为空，继续执行下一个目标站点或坐标
                    LOG(INFO) << "Navigation: 任务的目标列表不为空，继续执行下一个目标站点或坐标";
                    TaskManager::getInstance()->setMovementWaitForStart(
                        task);  // 设置等待开始执行路径，不然的话会认为是替换路径
                    startMoveTo(task);
                    return;
                }

                g_state.sys_state = SYS_STATE_IDLE;
                g_state.ready_for_new_movement_task = false;

                if (task && !task->isFinished()) {
                    if (task->isInCancel()) {
                        uint32_t reason = 0;
                        if(g_state.isManualControl()){
                            reason = sros::core::ERROR_CODE_MANUAL_CONTROL_MOVEMENT_RUNNING;
                        }
                        TaskManager::getInstance()->setMovementFinishCanceled(reason);
                    } else {
                        //如果该目标站点是进行特征信息识别
                        if(g_state.enable_station_feature_recognize) {
                            LOG(INFO) << "到站确认，关闭目标站点特征信息识别功能";
                            auto msg = std::make_shared<sros::core::CommonCommandMsg<std::string>>("TOPIC_EXTRACT_COMMAND");
                            msg->command = "STOP_EXTRACTOR";
                            msg->str1 = g_state.feature_mark.sensor_name;
                            sendMsg(msg);

                            //这里直接return, 等待特征信息识别到站确认状态信息回调
                            return;
                        }

                        if (PoseChecker::checkMovementTaskArriveDest()) {
                            TaskManager::getInstance()->setMovementFinishSucceed();
                            g_state.playMusic(hmi::MUSIC_ID_MUSIC_ARRIVED);
                        } else {
                            SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_AGV_NOT_ARRIVED_DEST, "");
                        }
                    }
                }
            }

            // 重置nav速度限制
            src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);
            LOG(INFO) << "Navigation 彻底结束";

            checkChargeStation();
        }
    } else if (g_state.main_state == STATE_TASK_RUN_PATH) {
        if (msg->state == STATE_NAV_INITIALIZING) {
            g_state.sys_state = SYS_STATE_TASK_PATH_NAV_INITIALING;
            src_sdk->enterPathMode();
            LOG(INFO) << "Path nav正在初始化 ";
        } else if (msg->state == STATE_NAV_MANUAL_WAITING_FOR_START) {
            g_state.sys_state = SYS_STATE_TASK_PATH_NAV_INITIALING;
            LOG(INFO) << "Path STATE_NAV_MANUAL_WAITING_FOR_START...";
        } else if (msg->state == STATE_NAV_MANUAL_WAITING_FOR_FINISH) {
            g_state.sys_state = SYS_STATE_TASK_PATH_WAITING_FINISH;

            if (pre_nav_state == STATE_NAV_MANUAL_PAUSED) {
                // 恢复运行
                src_car.setPauseState(PAUSE_SOURCE_NAV, false);
                LOGGER(INFO, SROS) << "The user cancels the pause and the agv continues to move.";
            } else if (pre_nav_state == STATE_NAV_MANUAL_WAITING_FOR_START) {
                src_sdk->run();
                src_car.setPauseState(PAUSE_SOURCE_NAV, false);
                LOGGER(INFO, SROS) << "The obstacle disappears and the AGV start move!";
            } else if (pre_nav_state == STATE_NAV_MANUAL_WAITING_FOR_START_PAUSED) {
                // disable pause state
                src_sdk->run();
                src_car.setPauseState(PAUSE_SOURCE_NAV, false);
                src_car.updateSRCSpeedLevel();
                LOGGER(INFO, SROS) << "The obstacle disappears and the AGV continues to move!";
            } else if (pre_nav_state == STATE_NAV_MANUAL_WAITING_FOR_FINISH_SLOW) {
                src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);
                LOGGER(INFO, SROS) << "The obstacle disappears and the AGV resumes the original speed!";
            }
            src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);

            if (task && !task->isFinished()) {
                TaskManager::getInstance()->setMovementRunning();
            }
        } else if (msg->state == STATE_NAV_MANUAL_WAITING_FOR_START_PAUSED) {
            g_state.sys_state = SYS_STATE_TASK_PATH_PAUSED;
            src_car.setPauseState(PAUSE_SOURCE_NAV, true, msg->param_int);

            // src_sdk->setSpeedLevel(30);
            LOGGER(WARNING, SROS) << "AGV can not start move due to detection of obstacles";
        } else if (msg->state == STATE_NAV_MANUAL_WAITING_FOR_FINISH_SLOW) {
            if (pre_nav_state == STATE_NAV_PATH_ERROR) {
                return;
            }

            if (pre_nav_state == STATE_NAV_MANUAL_PAUSED) {
                src_car.setPauseState(PAUSE_SOURCE_NAV, false);
            }
            src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, nav_slow_speed_level);

            g_state.sys_state = SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW;
            LOGGER(INFO, SROS) << "AGV slow down for obstacles detected!";
        } else if (msg->state == STATE_NAV_MANUAL_PAUSED) {
            if (pre_nav_state == STATE_NAV_PATH_ERROR) {
                return;
            }

            g_state.sys_state = SYS_STATE_TASK_PATH_PAUSED;
            src_car.setPauseState(PAUSE_SOURCE_NAV, true, msg->param_int);
            if (pre_nav_state == STATE_NAV_MANUAL_WAITING_FOR_FINISH_SLOW) {
                src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);
            }
            LOGGER(WARNING, SROS) << "AGV stop move due to obstacles detected!";
        } else if (msg->state == STATE_NAV_PAUSED) {
            LOG(INFO) << "Navigation paused ...";
            if (g_state.sys_state == SYS_STATE_TASK_MANUAL_PAUSED) {
                LOG(INFO) << "agv already in paused!";
                return;
            }

            src_car.setPauseState(PAUSE_SOURCE_NAV, true, msg->param_int);

            // store state !
            nav_state_before_pause_ = pre_nav_state;
            sys_state_before_pause_ = g_state.sys_state;

            g_state.sys_state = SYS_STATE_TASK_MANUAL_PAUSED;

            if (task && !task->isFinished()) {
                TaskManager::getInstance()->setMovementPause();
            }

        } else if (msg->state == STATE_NAV_CONTINUE) {
            LOG(INFO) << "Navigation continue ...";

            if (pre_nav_state != STATE_NAV_PAUSED) {
                LOG(INFO) << "invalid operation!";
                return;
            }

            // restore state !
            g_state.nav_state = nav_state_before_pause_;
            g_state.sys_state = sys_state_before_pause_;

            if (nav_state_before_pause_ == STATE_NAV_MANUAL_PAUSED ||
                nav_state_before_pause_ == STATE_NAV_MANUAL_WAITING_FOR_START_PAUSED) {
                src_car.setPauseState(PAUSE_SOURCE_NAV, true, msg->param_int);
            } else {
                src_sdk->run();  // 可能会有导航重新规划路径
                src_car.setPauseState(PAUSE_SOURCE_NAV, false);
            }

            if (task && !task->isFinished()) {
                if (g_src_state.is_waitting_checkpoint) {
                    TaskManager::getInstance()->setWaitingForCheckpoint();
                } else {
                    TaskManager::getInstance()->setMovementRunning();
                }
            }

        } else if (msg->state == STATE_NAV_PATH_ERROR) {
            LOG(WARNING) << "Path 偏离路径！";
            g_state.sys_state = SYS_STATE_TASK_MANUAL_PATH_ERROR;
        } else if (msg->state == STATE_NAV_IDLE) {
            src_sdk->stop();

            g_state.main_state = STATE_IDLE;

            g_state.sys_state = SYS_STATE_IDLE;
            g_state.ready_for_new_movement_task = false;

            // 重置nav速度限制
            src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_NAV, 100);

            PoseChecker::detectStationWhenMovementTaskFinished();
            LOG(INFO) << "Path 彻底结束，目标站点" << g_state.station_no << " 当前位置：" << src_sdk->getCurPose();

            if (!task) {
                LOG(ERROR) << "UNREACHABLE: task is nullptr!";
                return;
            }

            if (!task->isFinished()) {
                if (task->isInCancel()) {
                    uint32_t reason = 0;
                    if(g_state.isManualControl()){
                        reason = sros::core::ERROR_CODE_TASK_FAILED_BY_MANUAL_BTN;
                    }
                    TaskManager::getInstance()->setMovementFinishCanceled(reason);
                } else {
                    if (PoseChecker::checkMovementTaskArriveDest()) {
                        TaskManager::getInstance()->setMovementFinishSucceed();
                        g_state.playMusic(hmi::MUSIC_ID_MUSIC_ARRIVED);
                    } else {
                        SET_MOVEMENT_EXEC_FAILED(ERROR_CODE_AGV_NOT_ARRIVED_DEST, "");
                    }
                }
            } else {
                LOG(ERROR) << "unreachable!!! task has been finished!";
            }
        }
    }
}

void MainModule::checkChargeStation() {
    auto charge_station_no = Settings::getInstance().getValue("main.charge_station_no", 0);

    if (charge_station_no != 0 && g_state.station_no == charge_station_no) {
        // 到达充电站点，发送启动自动充电命令
        LOG(INFO) << "CHARGE: 到达充电站点，发送启动自动充电命令";
        auto d_msg = make_shared<CommandMsg>(getName());
        d_msg->command = CMD_ENABLE_AUTO_CHARGE;
        sendMsg(d_msg);
    }
}

void MainModule::handleSetCurrentMap(const core::CommandMsg_ptr &msg) {
    auto isMovementRunningFun = []() {
        auto cur_task = sros::core::TaskManager::getInstance()->getMovementTask();

        return cur_task && cur_task->isRunning();
    };
    auto isMissionRunningFun = []() { return MissionManager::getInstance()->isMissionRunning(); };

    if (isMovementRunningFun()) {
        throw EXEC_ERROR(ERROR_CODE_SET_MAP_MOVEMENT_RUNNING, "Movement is running! Can not set current map!");
    }

    if (isMissionRunningFun()) {
        throw EXEC_ERROR(ERROR_CODE_SET_MAP_MISSION_RUNNING, "Mission is running! Can not set current map!");
    }

    if (g_state.main_state != STATE_IDLE) {
        throw EXEC_ERROR(ERROR_CODE_SET_MAP_SYSTEM_NOT_IDLE, "Current main state", g_state.main_state);
    }

    // 仅当没有任务执行且不处于定位状态时才更新当前地图
    LOGGER(INFO, CMD_HANDER) << "Set current map to " << msg->map_name;
    if (!MapManager::getInstance()->setCurrentMap(msg->map_name)) {
        throw EXEC_ERROR(ERROR_CODE_SET_MAP_MAP_LOAD_ERROR, "Map", msg->map_name, "load failed!!!");
    }
    g_state.setCurMapName(msg->map_name);
}

core::ResultState MainModule::handleStartLocation(const core::CommandMsg_ptr &msg) {
    if (g_state.is_map_switching) {
        throw EXEC_ERROR(ERROR_CODE_LOCATION_IN_MAP_SWITCHING, "Map switching! Can not start location!");
    }

    if (g_state.slam_state == STATE_SLAM_LOCATING) {
        return RESPONSE_OK;
    } else if (g_state.slam_state == STATE_SLAM_LOCATING_AMCL ||
               g_state.slam_state == STATE_SLAM_WAITING_INITIAL_POSE ||
               g_state.slam_state == STATE_SLAM_SIFT_LOCATING) {
        return RESPONSE_PROCESSING;
    } else if (g_state.getCurMapName() == NO_MAP) {
        throw EXEC_ERROR(ERROR_CODE_LOCATION_NO_MAP, "NO_MAP!");
    } else if (!MapManager::getInstance()->isLoadMap()) {
        throw EXEC_ERROR(ERROR_CODE_LOCATION_MAP_LOAD_ERROR, "Map load failed!");
    } else if (g_state.slam_state == STATE_SLAM_IDLE) {
        LOGGER(INFO, CMD_HANDER) << "Starting SLAM positioning...";
        if (msg->param0 > 0) {
            // 带有站点信息,说明是使用站点位姿作为初始位姿
            auto station = MapManager::getInstance()->getStation(msg->param0);
            if (station.id == msg->param0) {
                initial_pose_.x() = station.pos.x / 100;
                initial_pose_.y() = station.pos.y / 100;
                initial_pose_.yaw() = station.pos.yaw;
                startSlamLocationManual(g_state.getCurMapName(), msg->param_boolean);
            } else {  // 地图中不存在这个站点
                throw EXEC_ERROR(ERROR_CODE_LOCATION_STATION_NOT_EXIST, "Location by station", msg->param0,
                                 ", but station not exist in map", g_state.getCurMapName());
            }
        } else {
            // 启动SLAM定位
            startSlamLocationManual(g_state.getCurMapName(), msg->param_boolean);
        }
        LOGGER(INFO, CMD_HANDER) << "Already send starting SLAM positioning command";
    } else {
        throw EXEC_ERROR(ERROR_CODE_LOCATION_SLAM_STATE_ERROR, "Can not start location, current slam state is",
                         g_state.slam_state);
    }
    return RESPONSE_PROCESSING;
}

void MainModule::handleNewMovementTask(const core::base_msg_ptr &m) {
    auto msg = dynamic_pointer_cast<CommandMsg>(m);

    auto task = msg->movement_task;

    // 获取当前的Task信息
    auto cur_task = sros::core::TaskManager::getInstance()->getMovementTask();

    uint32_t errCode = 0;

    if ((msg->source != "MissionModule" && msg->source != "ActionController") && MissionManager::getInstance()->isMissionRunning()) {
        LOG(WARNING) << "MOVEMENT_TASK: mission is running, new task is ignored. source module is " << msg->source;

        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_IN_MISSION_RUNNING, "Mission is running, new task is ignored");
    }

    if (cur_task && cur_task->isRunning()) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_PRE_TASK_RUNNING, "previous task is running, new task is ignored");
    }

    // 检测启动移动任务的条件
    g_state.checkStartMovementTaskCondition();

    if (g_state.eac_prohibit_movement_task) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_EAC_PROHIBIT, "EAC prohibit movement task!");
    }

    auto fault = FaultCenter::getInstance()->truckMovementTaskRunningFault();
    if (fault) {
        if (fault->id == FAULT_CODE_SCREEN_TURNED_UP) {
            if (msg->source == "Network") {  // 从网络来的任务需要检测触摸屏是否被翻起来了
                throw EXEC_ERROR(ERROR_CODE_MOVEMENT_SCREEN_TURNED_UP, "screen turned up track movement task running!");
            }
        } else {
            throw EXEC_ERROR(ERROR_CODE_MOVEMENT_FAULT_EXIST, "fault", fault->id, "track movement task running!");
        }
    }

    if (task->getPaths().size() >= 128) {
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_PATHS_IS_TOO_LONG,"path is more than 128");
    }

    if (task->getMovementType() == MOVE_TO_STATION || task->getMovementType() == MOVE_TO_POSE) {
        TaskManager::getInstance()->setMovementWaitForStart(task);
        startMoveTo(task);
    } else if (task->getMovementType() == MOVE_FOLLOW_PATH) {
        startDoRunPath(task, msg->param0);
    } else if (task->getMovementType() == MOVE_MIX2) {
        throw EXEC_ERROR(ERROR_CODE_FUNCTION_ABANDONED, "start movement task while movement type is MOVE_MIX2");
    }
}

void MainModule::handlePathReplace(const core::base_msg_ptr &m) {
    if (!TaskManager::getInstance()->isMovementSlaveRunning()) {
        throw EXEC_ERROR(ERROR_CODE_PATH_REPLACE_MOVEMENT_TASK_NOT_RUNNING,
                         "Path replace failed!, movement task not running!");
    }

    auto movement_task = TaskManager::getInstance()->getMovementTask();
    if (movement_task->getMovementType() != MOVE_FOLLOW_PATH) {
        throw EXEC_ERROR(ERROR_CODE_PATH_REPLACE_MOVEMENT_TASK_TYPE_NOT_MOVE_FOLLOW_PATH,
                         "Path replace failed!, movement task type not MOVE_FOLLOW_PATH!");
    }

    auto msg = dynamic_pointer_cast<CommandMsg>(m);

    NavigationPathi_vector current_paths = movement_task->getPaths();
    const auto &paths_int = msg->paths;

    if (paths_int.front().sx_ != current_paths.front().sx_ || paths_int.front().sy_ != current_paths.front().sy_) {
        throw EXEC_ERROR(ERROR_CODE_PATH_REPLACE_START_POINT_CHANGED,
                         "Path replace failed!, start point not the same!");
    }

    NavigationPath_vector paths_double;
    pathIntToDouble(paths_int, paths_double);

    //    NavigationPath_vector current_paths_double;
    //    pathIntToDouble(current_paths, current_paths_double);
    //    double theta = calculateInnerAngleBetweenTwoLines(paths_double.front(), current_paths_double.front());
    //    if (theta > atan(1.0 / 100.0)) {
    //        throw EXEC_ERROR(ERROR_CODE_PATH_REPLACE_INCONSISTENT_PATHS_SLOP,
    //                         "Path replace failed!, inconsistent path slop! current inner angle is:", theta /
    //                         DEGREE_TO_RAD, "°");
    //    }
    //
    //    auto cur_pose = src_sdk->getCurPose();
    //    LinePath current_to_end_path(cur_pose.x(), cur_pose.y(), paths_double.front().ex_, paths_double.front().ey_);
    //    theta = calculateInnerAngleBetweenTwoLines(current_to_end_path, current_paths_double.front());
    //    if (theta > M_PI_2) {  // 当前位置到终点的位置和之前路线的夹角大于90°，说明目标点已经走过了，需要报错并停下来
    //        TaskManager::getInstance()->setMovementInCancel();
    //        LOGGER(INFO, CMD_HANDER) << "update movement task " << movement_task->getTaskNo()
    //                                 << " to cancel state, due to ERROR_CODE_PATH_REPLACE_DEST_POSE_PAST!";
    //        sendNavCommand(COMMAND_NAV_CANCEL);  // FIXME(pengjiali): 取消需要带上取消原因
    //        throw EXEC_ERROR(ERROR_CODE_PATH_REPLACE_DEST_POSE_PAST, "Path replace failed!, dest pose past!");
    //    }

    PathChecker::checkIfNotPoseContinuous(paths_double);
    PathChecker::checkIfNotAngleContinuous(paths_double);
    PathChecker::checkDisableRotateArea(paths_double);
    PathChecker::checkChassisType(paths_double);

    src_sdk->setPaths(paths_int);

    // NOTE: 下发路径时src那边很有可能就开始结束了，啥时候去检测src是否已经结束了，这个是一个经验值
    boost::this_thread::sleep_for(boost::chrono::milliseconds(5));

    if (!TaskManager::getInstance()->isMovementSlaveRunning()) {
        throw EXEC_ERROR(ERROR_CODE_PATH_REPLACE_MOVEMENT_TASK_NOT_RUNNING,
                         "Path replace failed!, movement task not running!");
    }

    if (paths_int.size() >= 128){
        throw EXEC_ERROR(ERROR_CODE_MOVEMENT_PATHS_IS_TOO_LONG,
                         "Path replace failed!, replace path is too long!");
    }

    movement_task->pathsReplace(paths_int);
    src_sdk->setCheckpoint(paths_int.size());
    g_state.checkpoint_no = paths_int.size();
    auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("NAV_COMMAND");
    mm->command = COMMAND_NAV_SINGLE_APTH_REPLACE;
    mm->paths = paths_double;
    sendMsg(mm);
}

sros::core::ResultState MainModule::handleMapSwitching(core::CommandMsg_ptr msg) {
    if (g_state.is_map_switching) {
        if (target_switching_map_ == msg->map_name) {
            return RESPONSE_PROCESSING;
        } else {
            throw EXEC_ERROR(ERROR_CODE_MAP_SWITCHING_IN_PROCESSING, "Current switching map name is",
                             target_switching_map_, ", new switching map name is", msg->map_name);
        }
    }

    if (g_state.slam_state == STATE_SLAM_LOCATING_AMCL || g_state.slam_state == STATE_SLAM_WAITING_INITIAL_POSE ||
        g_state.slam_state == STATE_SLAM_SIFT_LOCATING) {
        throw EXEC_ERROR(ERROR_CODE_MAP_SWITCHING_IN_START_LOCATION, "Starting location when switching map!");
    }
    if (TaskManager::getInstance()->isMovementTaskRunning()) {
        throw EXEC_ERROR(ERROR_CODE_MAP_SWITCHING_MOVEMENT_RUNNING, "Movement is running! Can not switch map!");
    }

    g_state.is_map_switching = true;
    target_switching_map_ = msg->map_name;

    if (g_state.isLocateSucceed()) {
        LOGGER(INFO, CMD_HANDER) << "stop location";
        stopSlamLocation();
    }

    LOGGER(INFO, CMD_HANDER) << "Set current map to " << msg->map_name;
    if (!MapManager::getInstance()->setCurrentMap(msg->map_name)) {
        g_state.is_map_switching = false;
        throw EXEC_ERROR(ERROR_CODE_MAP_SWITCHING_MAP_LOAD_ERROR, "Map", msg->map_name, "load failed!!!");
    }
    g_state.setCurMapName(msg->map_name);

    LOGGER(INFO, CMD_HANDER) << "Starting SLAM positioning...";
    if (msg->param0 > 0) {
        // 带有站点信息,说明是使用站点位姿作为初始位姿
        auto station = MapManager::getInstance()->getStation(msg->param0);
        if (station.id == msg->param0) {
            initial_pose_.x() = station.pos.x / 100;
            initial_pose_.y() = station.pos.y / 100;
            initial_pose_.yaw() = station.pos.yaw;
        } else {  // 地图中不存在这个站点
            g_state.is_map_switching = false;
            throw EXEC_ERROR(ERROR_CODE_MAP_SWITCHING_STATION_NOT_EXIST, "Location by station", msg->param0,
                             ", but station not exist in map", g_state.getCurMapName());
        }
    } else {
        // 启动SLAM定位
        initial_pose_ = msg->pose;
    }
    std::thread([&]() {
        promise_slam_state_change_ = std::make_shared<std::promise<void>>();
        auto future = promise_slam_state_change_->get_future();
        startSlamLocationManual(g_state.getCurMapName(), msg->param_boolean);
        future.wait();
        promise_slam_state_change_.reset();
        g_state.is_map_switching = false;
    }).detach();

    //add default return value
    return sros::core::ResultState::RESPONSE_NONE;
}

void MainModule::startSlamLocationManual(const string &map_name, bool absolute_location) {
    auto &s = sros::core::Settings::getInstance();
    auto enable_sros_native_debug = (s.getValue<std::string>("debug.enable_sros_native_debug", "False") == "True");
    if (enable_sros_native_debug) {
        // 本地调试的时候，定位某个地方就定位成功
        src_sdk->sendPoseBack(initial_pose_);
        g_state.location_state = LOCATION_STATE_RUNNING;
        return;
    }

    g_state.sys_state = SYS_STATE_START_LOCATING;
    g_state.location_state = LOCATION_STATE_INITIALING;
    LOG(INFO) << "startSlamLocationManual: " << map_name << ", " << initial_pose_ << ", " << std::boolalpha
              << absolute_location;
    auto mm = make_shared<SlamCommandMsg>();
    mm->slam_command = COMMAND_START_LOCATION_MANUAL;
    mm->map_name = map_name;                   // 使用的地图名
    mm->map_path = MapManager::MAP_SAVE_PATH;  // 地图文件存储的路径
    mm->pose = initial_pose_;                  // 初始位姿
    mm->use_curr_pose = absolute_location;     // 是否用绝对定位，绝对定位就直接定在那里
    sendMsg(mm);
}

void MainModule::startSlamRelocationManual(const string &map_name, bool absolute_location) {
    g_state.sys_state = SYS_STATE_START_LOCATING;
    g_state.location_state = LOCATION_STATE_INITIALING;
    LOG(INFO) << "startSlamRelocationManual: " << map_name << ", " << initial_pose_ << ", " << std::boolalpha
              << absolute_location;
    auto mm = make_shared<SlamCommandMsg>();
    mm->slam_command = COMMAND_START_RELOCATION;
    mm->map_name = map_name;                   // 使用的地图名
    mm->map_path = MapManager::MAP_SAVE_PATH;  // 地图文件存储的路径
    mm->pose = initial_pose_;                  // 初始位姿
    mm->use_curr_pose = absolute_location;     // 是否用绝对定位，绝对定位就直接定在那里
    sendMsg(mm);
}

void MainModule::startSlamDrawMap(const string &map_name) {
    auto mm = make_shared<SlamCommandMsg>();
    mm->slam_command = COMMAND_START_DRAW_MAP;
    mm->map_name = map_name;
    cur_drawing_map_name_ = map_name;
    mm->map_path = MapManager::MAP_SAVE_PATH;
    sendMsg(mm);
}

void MainModule::sendSlamCommand(SLAM_COMMAND_TYPE command) {
    if (command == COMMAND_CANCEL_DRAW_MAP) {
        need_convert_map_ = false;
    } else {
        need_convert_map_ = true;
    }

    auto mm = make_shared<SlamCommandMsg>();
    mm->slam_command = command;
    sendMsg(mm);
}

void MainModule::sendNavCommand(NavigationCommand command) {
    auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("NAV_COMMAND");
    mm->command = command;
    sendMsg(mm);
}

void MainModule::setPGVInfoToTask(const core::MovementTask_ptr &task, core::StationNo_t dst_station_no) {
    auto &s = sros::core::Settings::getInstance();
    bool enable_pgv_rectify = (s.getValue<std::string>("main.enable_pgv_rectify", "False") == "True");
    if (enable_pgv_rectify) {
        auto station = MapManager::getInstance()->getStation(dst_station_no);
        if (!station.dmcode_id.empty()) {
            DMCodeOffset offset(station.dmcode_id, station.dmcode_offset.x / 100, station.dmcode_offset.y / 100,
                                station.dmcode_offset.yaw);
            task->setDownCameraOffset(offset);
            g_state.station_camera_offset.set(offset);
        }
    }
}

void MainModule::startDoRunPath(const core::MovementTask_ptr &task, int32_t checkpoint) {
    auto paths_int = task->getPaths();

    // 转换NavigationPathi_vector为NavigationPath_vector
    sros::core::NavigationPath_vector paths_double;
    pathIntToDouble(paths_int, paths_double);
    LOGGER(INFO, CMD_HANDER) << "Received " << paths_int.size() << " manual paths:";
    for (auto i = 0; i < paths_double.size(); ++i) {
        LOGGER(INFO, CMD_HANDER) << i << ": " << paths_double[i];
    }

    PathChecker::checkMoveFollowPathArgsChecker(paths_double);

    TaskManager::getInstance()->setMovementWaitForStart(task);

    // pgv 矫正
    // fms发送路径的时候会将目标站点带进来，这样可以实现pgv矫正。
    // 不能通过到达站点后检查站点编号来做，原因是若两个站点较近，到底是那个站点这个不知道
    auto dst_station_no = task->getCurDstStation();
    if (dst_station_no != 0) {
        LOG(INFO) << "Run path, dst station no is " << dst_station_no;
        setPGVInfoToTask(task, dst_station_no);
    }

    // 启动nav避障（防撞）
    auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("NAV_COMMAND");
    mm->command = COMMAND_NAV_MANUAL_PATH;
    mm->paths = paths_double;
    mm->param0 = dst_station_no;
    mm->str0 = g_state.getCurMapName();
    sendMsg(mm);

    path_list_to_run_ = paths_int;
    g_state.checkpoint_no = 0;
    g_state.main_state = STATE_TASK_RUN_PATH;  // 更新系统状态-

    LOG(INFO) << "checkpoint is " << (int)checkpoint;
    doRunPath(path_list_to_run_, false, checkpoint);
}

void MainModule::startMoveTo(const core::MovementTask_ptr &task) {
    auto dst_station_no = task->getDstStationNo();
    auto dst_pos = task->getDstPose();

    task->setCurStartStation(g_state.station_no);
    task->setCurStartPose(src_sdk->getCurPose());
    task->setCurDstStation(dst_station_no);
    task->setCurDstPose(dst_pos);

    g_state.main_state = STATE_TASK_NAVIGATING;
    g_state.sys_state = SYS_STATE_TASK_NAV_FINDING_PATH;  // 启动移动程序后，将系统状态设置为正在导航

    // pgv 矫正
    if (task->getMovementType() ==
        MOVE_TO_STATION) {  // 若是移动到站点就考虑是否设置了pgv信息, 不考虑同时发送几个站点的情况
        setPGVInfoToTask(task, dst_station_no);
    }

    // 发送导航命令到NavigationModule，等待其生成路径
    auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("NAV_COMMAND");
    mm->command = COMMAND_NAV_NAVIGATE_TO;
    mm->str0 = g_state.getCurMapName();
    mm->pose = dst_pos;
    mm->param0 = dst_station_no;
    mm->param1 = task->getAvoidPolicy();
    mm->param_boolean = task->getForceNavOnMap();
    sendMsg(mm);
}

void MainModule::onSRCState(const SRCState &state) {
    if (!is_src_connected_) {
//        LOGGER(INFO, SROS) << "SRC connected!";
        is_src_resetting_ = false;
    }
    is_src_connected_ = true;
}

void MainModule::initSRCCar() {
    // 必须先设置callback再调用connect
    LOG(INFO) << "Init src!";

    src_sdk->setStateCallback(boost::bind(&MainModule::onSRCState, this, _1));
    src_sdk->setPathFinishCallback(boost::bind(&MainModule::onSRCPathFinish, this));
    src_sdk->setPathAbortedCallback(boost::bind(&MainModule::onSRCPathAborted, this));
    src_sdk->setPathPausedCallback(boost::bind(&MainModule::onSRCPathPaused, this));
    src_sdk->setPoseTimeoutCallback([&]() {
        std::thread([&]() {
            // FIXME(pengjiali): remove me! srotos
            // 由于src报的pose超时问题现在一直没有找到原因，当出现此问题时，我们延迟5s然后更新src的暂停状态
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            src_car.updatePauseState();
        }).detach();
    });

    // UsartDataCallback 在usart_module中设置
    // PoseCallback 在slam中设置回调

    sros::core::Settings &settings = Settings::getInstance();

    string port = settings.getValue<string>("main.src_uart_port", "/dev/ttyTHS2");
    auto baud_rate = settings.getValue<unsigned int>("main.src_uart_baud_rate", 460800);

    if (src_sdk->connect(port, baud_rate)) {
        src_sdk->initState();
        src_sdk->checkSrcParamters();
    }
}

void MainModule::doRunPath(const core::NavigationPathi_vector &paths, bool auto_run, int32_t checkpoint) {
    // 清除用户设置的暂停状态
    src_car.reset();
    g_state.checkpoint_no = 0;
    if (!paths.empty()) {
        LOG(INFO) << "doRunPath :" << paths.size();
        for (auto p : paths) {
            LOG(INFO) << "-> 路径"
                      << ": 类型" << p.type_ << "(sx,sy)= (" << p.sx_ << "," << p.sy_ << "); (ex,ey)= (" << p.ex_ << ","
                      << p.ey_ << ") rotate_angle = " << p.rotate_angle_ << "-> 方向"
                      << ": " << p.direction_;
        }

        auto task = TaskManager::getInstance()->getMovementTask();

        src_sdk->stop();  // 保证src处于路径模式
        src_sdk->enterPathMode();
        src_sdk->setPaths(paths);
        LOG(INFO) << "已向SRC发送" << paths.size() << "条路径";
        if (checkpoint == -1) {  // -1 认为是关卡设在最前面
            g_state.checkpoint_no = paths.size();
            src_sdk->setCheckpoint(0);
        } else if (checkpoint == 0) {  // 0为了兼容，0代表的含义是没有设置CHECKPOINT，也就是讲检查点设置到最后
            src_sdk->setCheckpoint(paths.size());
            g_state.checkpoint_no = paths.size();
        } else {
            src_sdk->setCheckpoint(checkpoint);
            g_state.checkpoint_no = checkpoint;
        }

        LOG(INFO) << "getDownCameraOffset:" << task->getDownCameraOffset();
        auto &s = sros::core::Settings::getInstance();
        bool enable_pgv_rectify = (s.getValue<std::string>("main.enable_pgv_rectify", "False") == "True");

        if (enable_pgv_rectify && task->isSetDownCameraOffset()) {  // 向src设置pgv信息
            auto offset = task->getDownCameraOffset();

            auto imu_ins550 = s.getValue<int>("src.imu_ins550", 0);
            LOG(INFO) << "imu_ins550: " << imu_ins550;
            if (imu_ins550 / 10 % 10 == 2) {
                auto pgv_direction_offset = Settings::getInstance().getValue<double>("main.pgv_direction_offset", 90.0);
                offset.yaw = normalizeYaw0To2Pi(offset.yaw - pgv_direction_offset * DEG_TO_RAD);
            }

            bool ret = src_sdk->setTargetDMCodeOffset(offset.id, offset.x, offset.y, offset.yaw);
            if (!ret) LOG(ERROR) << "设置pgv信息失败！";
        }

        TaskManager::getInstance()
            ->setMovementRunning();  // 给src发送路径了，就算已经开始启动任务了，至于什么时候开始走这个由避障决定

        if (auto_run) {
            if (obstacle_avoid_enable_) {
                if (g_state.main_state == STATE_TASK_NAVIGATING) {
                    //updateNavState(STATE_NAV_PATH_PAUSED);
                } else if (g_state.main_state == STATE_TASK_RUN_PATH) {
                    updateNavState(STATE_NAV_MANUAL_PAUSED);
                }
            }

            src_sdk->run();
            src_car.reset();
            LOG(INFO) << "自动开始执行路径";
        }

    } else {
        LOG(INFO) << "0条路径,执行完毕";
        onSRCPathFinish();
    }
}

void MainModule::onDebugCmdMsg(const core::base_msg_ptr &m) {
    sros::core::ResultState result_state = RESPONSE_OK;

    auto msg = dynamic_pointer_cast<CommandMsg>(m);
    auto session_id = msg->session_id;

    auto isMovementRunningFun = []() {
        auto cur_task = sros::core::TaskManager::getInstance()->getMovementTask();

        return cur_task && cur_task->isRunning();
    };
    auto isActionRunningFun = []() {
        auto cur_task = sros::core::TaskManager::getInstance()->getActionTask();

        return cur_task && cur_task->isRunning();
    };
    auto isMissionRunningFun = []() { return MissionManager::getInstance()->isMissionRunning(); };

    auto getControlMutexFun = [&]() {
        if (!g_state.control_mutex.get(session_id)) {
            auto locker = g_state.control_mutex.getLocker();
            throw EXEC_ERROR(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, "locker:", locker.session_id, "current:", session_id);
        }
    };

    try {
        if (g_state.main_state == STATE_INITIALING && msg->command != CMD_RESET_SROS && msg->command != CMD_SRC_RESET) {
            // 初始化进行中,无法处理部分
            throw EXEC_ERROR(ERROR_CODE_EXECUTE_COMMAND_IN_INITIALIZING, "Current in initialing");
        }

        switch (msg->command) {
            case CMD_NEW_MOVEMENT_TASK: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_NEW_MOVEMENT_TASK command, task no is "
                                         << msg->movement_task->getTaskNo();
                getControlMutexFun();
                
                 if (g_state.isManualControl()) {
                    throw EXEC_ERROR(ERROR_CODE_MANUAL_CONTROL_RUNNING, "vehicle is in manual control mode!");
                }

                handleNewMovementTask(msg);

                if(msg->source == "ActionController") {
                    LOG(INFO) << "msg->source ActionController, just return";
                    return;
                }

                break;
            }
            case CMD_CANCEL_MOVEMENT_TASK: {
                // 取消任务
                bool soft_cancel = msg->param_boolean; // 是否为缓停
                //bool soft_cancel = true; // 是否为缓停
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_CANCEL_MOVEMENT_TASK command， soft_cancel is " << soft_cancel;

                getControlMutexFun();

                auto task = TaskManager::getInstance()->getMovementTask();
                if (!task || task->isFinished()) {
                    throw EXEC_ERROR(ERROR_CODE_CANCEL_MOVEMENT_TASK_NOT_RUNNING,
                                     "Cancel movement task, but task not running!");
                }

                if (task->isWaitForStart() || task->isSlaveRunning()) {
                    if (soft_cancel) {
                        // 先下发暂停指令，然后等待速度为0以后再发生stop
                        src_car.setPauseState(PAUSE_SOURCE_NAV, true, SRC_MAX_PAUSE_LEVEL);
                    }

                    TaskManager::getInstance()->setMovementInCancel();
                    LOGGER(INFO, CMD_HANDER) << "update movement task " << task->getTaskNo() << " to cancel state";

                    if (soft_cancel) {
                        std::thread t([&]() {
                            // this_thread::sleep_for(chrono::milliseconds(100))；
                            // 每隔一段时间检查速度是否降到0，最大超时1000ms
                            for (int i = 0; i < 20; i++) {
                                LOGGER(INFO, SROS) << "Task soft cancel : v = " << g_src_state.cur_v << ", w = " << g_src_state.cur_w;
                                if (g_src_state.cur_v < 0.01 && g_src_state.cur_w < 0.01) {
                                    break;
                                }
                                this_thread::sleep_for(chrono::milliseconds(50));
                            }
                            sendNavCommand(COMMAND_NAV_CANCEL); // 实际发生CANCEL命令
                            LOGGER(INFO, SROS) << "Task soft cancel: sendNavCommand(COMMAND_NAV_CANCEL)";
                        });
                        t.detach();
                    } else {
                        sendNavCommand(COMMAND_NAV_CANCEL);  // FIXME(pengjiali): 取消需要带上取消原因
                    }

                    // 导航结束后会发送Notification
                    result_state = RESPONSE_OK;
                } else {
                    LOGGER(INFO, CMD_HANDER)
                        << "Current movement task state is " << task->getState() << ", can not cancel!";
                }
                break;
            }
            case CMD_SET_CHECKPOINT: {
                auto checkpoint_no = msg->param0;
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SET_CHECKPOINT command, checkpoint no is " << checkpoint_no;

                getControlMutexFun();

                auto task = TaskManager::getInstance()->getMovementTask();
                if (!task || task->isFinished()) {
                    throw EXEC_ERROR(ERROR_CODE_SET_CHECKPOINT_TASK_NOT_RUNNING,
                                     "Set checkpoint failed!, movement task not running!");
                }
                if (!task->movementTypeIsFollowPath()) {
                    throw EXEC_ERROR(ERROR_CODE_SET_CHECKPOINT_TASK_TYPE_NOT_FOLLOW_PATH,
                                     "Set checkpoint failed!, movement task type not follow path!");
                }

                if (checkpoint_no <= g_src_state.cur_checkpoint) {
                    LOG(WARNING) << "Set checkpoint may be fail, new checkpoint must greater than current "
                                 << (int)g_src_state.cur_checkpoint;
                }

                src_sdk->setCheckpoint(checkpoint_no);
                g_state.checkpoint_no = checkpoint_no;

                break;
            }
            case CMD_PATH_REPLACE: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_PATH_REPLACE command";
                getControlMutexFun();

                handlePathReplace(msg);
                break;
            }
            case CMD_START_MISSION:
            case CMD_CONTINUE_MISSION:
            case CMD_CANCEL_MISSION:
            case CMD_REORDER_MISSION: {
                // 在mission module处理,但是如果不返回,在后面会调用responseCommand
                return;
            }
            case CMD_NEW_MAP_START: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_NEW_MAP_START command";
                getControlMutexFun();

                if (isMovementRunningFun()) {
                    throw EXEC_ERROR(ERROR_CODE_NEW_MAP_MOVEMENT_RUNNING,
                                     "movement is running! Can not create new map!");
                }

                if (isMissionRunningFun()) {
                    throw EXEC_ERROR(ERROR_CODE_NEW_MAP_MISSION_RUNNING, "mission is running! Can not create new map!");
                }

                g_state.main_state = STATE_TASK_NEW_MAP;

                if (g_state.slam_state == STATE_SLAM_IDLE) {
                    // 启动SLAM绘图
                    startSlamDrawMap(msg->map_name);
                    LOGGER(INFO, CMD_HANDER) << "Starting SLAM drawing";

                    result_state = RESPONSE_PROCESSING;
                } else if (g_state.slam_state == STATE_SLAM_LOCATING ||
                           g_state.slam_state == STATE_SLAM_LOCATING_AMCL ||
                           g_state.slam_state == STATE_SLAM_SIFT_LOCATING ||
                           g_state.slam_state == STATE_SLAM_WAITING_INITIAL_POSE) {
                    cur_drawing_map_name_ = msg->map_name;
                    is_wait_for_draw_map_ = true;

                    // 停止SLAM定位
                    sendSlamCommand(COMMAND_STOP_LOCATION);
                    LOGGER(INFO, CMD_HANDER) << "Stopping SLAM positioning";
                    result_state = RESPONSE_PROCESSING;

                    // 等待定位结束再开始绘图
                } else {
                    // 正在绘图中
                    throw EXEC_ERROR(ERROR_CODE_NEW_MAP_ALREADY_STARTED, "Create new map already started!");
                }
                break;
            }
            case CMD_NEW_MAP_STOP: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_NEW_MAP_STOP command";
                getControlMutexFun();

                // 保存地图
                sendSlamCommand(COMMAND_STOP_DRAW_MAP);
                result_state = RESPONSE_OK;
                break;
            }
            case CMD_NEW_MAP_CANCEL: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_NEW_MAP_CANCEL command";
                getControlMutexFun();

                sendSlamCommand(COMMAND_CANCEL_DRAW_MAP);

                result_state = RESPONSE_OK;
                break;
            }
            case CMD_COMMON_CANCEL: {
                // 为了兼容 COMMON_CANCEL 指令
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_COMMON_CANCEL command";
                getControlMutexFun();

                if (g_state.main_state == STATE_TASK_NAVIGATING || g_state.main_state == STATE_TASK_RUN_PATH) {
                    msg->command = CMD_CANCEL_MOVEMENT_TASK;
                    sendMsg(msg);

                    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
                    if (action_task && (action_task->isRunning() || action_task->isWaitForStart())) {
                        LOGGER(INFO, CMD_HANDER)
                                << "Action task " << action_task->getTaskNo() << " is running, cancel action task";

                        auto mm = make_shared<sros::core::CommandMsg>(msg->source);
                        mm->user_name = msg->user_name;
                        mm->command = CMD_CANCEL_ACTION_TASK;
                        sendMsg(mm);
                    }

                    return;  // CMD_CANCEL_MOVEMENT_TASK 执行后会发送Response
                } else if (g_state.main_state == STATE_TASK_NEW_MAP) {
                    // 取消任务
                    // 不需要转换地图操作
                    sendSlamCommand(COMMAND_CANCEL_DRAW_MAP);
                } else if (g_state.main_state == STATE_IDLE) {
                    g_state.sys_state = SYS_STATE_IDLE;
                    g_state.ready_for_new_movement_task = false;

                    LOGGER(INFO, CMD_HANDER) << "Set sros state to idle";
                }

                auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
                if (action_task && (action_task->isRunning() || action_task->isWaitForStart())) {
                    LOGGER(INFO, CMD_HANDER)
                        << "Action task " << action_task->getTaskNo() << " is running, cancel action task";

                    msg->command = CMD_CANCEL_ACTION_TASK;
                    sendMsg(msg);
                    return;  // ActionController返回Response
                }

                result_state = RESPONSE_OK;
                break;
            }
            case CMD_LOCK_CONTROL_MUTEX: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_LOCK_CONTROL_MUTEX command";

                getControlMutexFun();

                if (isMovementRunningFun() &&
                    session_id != TaskManager::getInstance()->getMovementTask()->getTaskSessionId()) {
                    throw EXEC_ERROR(ERROR_CODE_LOCK_CONTROL_MUTEX_MOVEMENT_RUNNING,
                                     "Movement task is running! Can not lock control mutex!");
                }
                if (isActionRunningFun() &&
                    session_id != TaskManager::getInstance()->getActionTask()->getTaskSessionId()) {
                    throw EXEC_ERROR(ERROR_CODE_LOCK_CONTROL_MUTEX_ACTION_RUNNING,
                                     "Action task is running! Can not lock control mutex!");
                }
                if (isMissionRunningFun() &&
                    session_id != MissionManager::getInstance()->getCurrentRunningMissionSessionId()) {
                    throw EXEC_ERROR(ERROR_CODE_LOCK_CONTROL_MUTEX_MISSION_RUNNING,
                                     "Mission task is running! Can not lock control mutex!");
                }
                if (msg->locker_nickname.empty()) {
                    throw EXEC_ERROR(ERROR_CODE_LOCK_CONTROL_MUTEX_NONE_NICK_NAME,
                                     "Nick name is empty! Can not lock control mutex!");
                }
                //            if (msg->locker_ip_address.empty()) {
                //                LOG(WARNING) << "ip address is empty! Can not lock control mutex!";
                //                result_state = RESPONSE_FAILED;
                //                result_code = ERROR_CODE_LOCK_CONTROL_MUTEX_NONE_IP_ADDRESS;
                //                break;
                //            }

                // 若能从session_manager获取ip就从其中获取，若获取到的本地的ip,且用户设置了ip，就用用户的ip，解决web-chip不能获取到IP的问题
                auto locker_ip = g_state.network_session_manager.getItem(msg->session_id)->ip_addr;
                if (locker_ip == "127.0.0.1" && !msg->locker_ip_address.empty()) {
                    g_state.control_mutex.lock(msg->session_id, msg->locker_ip_address, msg->user_name,
                                               msg->locker_nickname);
                } else {
                    g_state.control_mutex.lock(msg->session_id, locker_ip, msg->user_name, msg->locker_nickname);
                }
                result_state = RESPONSE_OK;

                break;
            }
            case CMD_UNLOCK_CONTROL_MUTEX: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_UNLOCK_CONTROL_MUTEX command";

                if (!g_state.control_mutex.isLock()) {
                    throw EXEC_ERROR(ERROR_CODE_UNLOCK_CONTROL_MUTEX_CURRENT_IS_UNLOCKED,
                                     "Current control mutex is not locked! Can not lock control mutex!");
                }
                auto locker = g_state.control_mutex.getLocker();
                if (locker.session_id != msg->session_id) {
                    throw EXEC_ERROR(ERROR_CODE_UNLOCK_CONTROL_MUTEX_SESSION_ID_MISMATCH,
                                     "session id is mismatch! Can not unlock control mutex! old:", locker.session_id,
                                     " current:", msg->session_id);
                }

                g_state.control_mutex.unlock();
                result_state = RESPONSE_OK;

                break;
            }
            case CMD_FORCE_UNLOCK_CONTROL_MUTEX: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_FORCE_UNLOCK_CONTROL_MUTEX command";

                auto session = g_state.network_session_manager.getItem(msg->session_id);
                auto &ss = sros::core::UserManager::getInstance();
                auto user_item = ss.getUserItem(session->username);

                if (stoi(user_item.permission) < PERMISSION_ADMIN) {
                    throw EXEC_ERROR(ERROR_CODE_FORCE_UNLOCK_CONTROL_MUTEX_PERMISSION_DENIED,
                                     "Permission denied! Can not force unlock control mutex!");
                }

                if (g_state.fleet_mode == FLEET_MODE_ONLINE) {
                    throw EXEC_ERROR(ERROR_CODE_FORCE_UNLOCK_CONTROL_MUTEX_FLEET_DENIED,
                                     "Fleet denied! Can not force unlock control mutex!");
                }

                g_state.control_mutex.unlock();
                result_state = RESPONSE_OK;

                break;
            }
            case CMD_START_LOCATION: {
                if (msg->param_boolean) {
                    LOGGER(INFO, CMD_HANDER) << "Handling CMD_START_LOCATION command, absolute location!";
                } else {
                    LOGGER(INFO, CMD_HANDER) << "Handling CMD_START_LOCATION command, normal location!";
                }

                getControlMutexFun();

                result_state = handleStartLocation(msg);
                break;
            }
            case CMD_STOP_LOCATION: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_STOP_LOCATION command";
                getControlMutexFun();

                if (isMovementRunningFun()) {
                    if (g_state.sys_state != SYS_STATE_TASK_MANUAL_PAUSED) {  // 暂停任务状态下运行取消定位然后再定位
                        throw EXEC_ERROR(ERROR_CODE_LOCATION_MOVEMENT_RUNNING,
                                         "Movement is running! Can not stop location!");
                    }
                }

                if (g_state.is_map_switching) {
                    throw EXEC_ERROR(ERROR_CODE_STOP_LOCATION_IN_MAP_SWITCHING,
                                     "Map switching! Can not stop location!");
                }

                if (g_state.location_state == LOCATION_STATE_RUNNING ||
                    g_state.location_state == LOCATION_STATE_RELOCATING) {
                    LOGGER(INFO, CMD_HANDER) << "Send stopping SLAM positioning command";

                    // 取消定位前，记录系统状态，当系统重新定位时，恢复到当前记录的状态
                    if (g_state.main_state == STATE_TASK_NAVIGATING || g_state.main_state == STATE_TASK_RUN_PATH) {
                        sys_state_before_stop_location_ = g_state.sys_state;
                    }

                    stopSlamLocation();
                }
                result_state = RESPONSE_OK;
                break;
            }
            case CMD_SET_CUR_MAP: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SET_CUR_MAP command, new map name is " << msg->map_name;
                getControlMutexFun();

                if (g_state.location_state != LOCATION_STATE_NONE) {
                    throw EXEC_ERROR(ERROR_CODE_SET_MAP_LOCATION_NOT_NONE, "Current location state",
                                     g_state.location_state);
                }

                handleSetCurrentMap(msg);

                result_state = RESPONSE_OK;
                break;
            }
            case CMD_SET_LOCATION_INITIAL_POSE: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SET_LOCATION_INITIAL_POSE command, new pose is " << msg->pose;
                getControlMutexFun();

                initial_pose_ = msg->pose;

                result_state = RESPONSE_OK;
                break;
            }
            case CMD_SRC_PAUSE: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SRC_PAUSE command";
                getControlMutexFun();

                // 通过更新NAV状态实现暂停和继续
                src_car.setPauseState(PAUSE_SOURCE_USER, true);
                updateNavState(STATE_NAV_PAUSED, msg->param0);
                sendNavCommand(COMMAND_NAV_PAUSE);
                result_state = RESPONSE_OK;

                // 更新任务状态实现暂停和继续
                updateActionState(m,true);
                break;
            }
            case CMD_SRC_CONTINUE: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SRC_CONTINUE command";
                getControlMutexFun();
                sendNavCommand(COMMAND_NAV_CONTINUE);
                // 允许暂停状态下取消定位，然后再定位，所以当定位未启动不允许继续

                if (TaskManager::getInstance()->isMovementSlaveRunning()) {
                    if (g_state.sys_state != SYS_STATE_TASK_MANUAL_PAUSED) {
                        throw EXEC_ERROR(ERROR_CODE_CONTINUE_IN_NOT_PAUSE, "Can not continue, current is not pause!");
                    }

                    if (!g_state.isLocateSucceed()) {
                        throw EXEC_ERROR(ERROR_CODE_CONTINUE_IN_NOT_LOCATION,
                                         "Can not continue, current is not location!");
                    }
                }

                // 急停触发时忽略此操作
                if (g_state.emergency_state != STATE_EMERGENCY_TRIGER &&
                    g_state.emergency_state != STATE_EMERGENCY_RECOVERABLE) {
                    if (src_car.getPauseState(PAUSE_SOURCE_FAULT)) {
                        throw EXEC_ERROR(ERROR_CODE_CONTINUE_IN_FAULT, "Can not continue, fault is raise!");
                    }

                    // 通过更新NAV状态实现暂停和继续
                    src_car.setPauseState(PAUSE_SOURCE_USER, false);
                    updateNavState(STATE_NAV_CONTINUE);
                    result_state = RESPONSE_OK;

                    // 更新任务状态实现暂停和继续
                    updateActionState(m,false);
                } else {
                    throw EXEC_ERROR(ERROR_CODE_CONTINUE_IN_EMERGENCY, "Can not continue, current is in emergency!");
                }
                break;
            }
            case CMD_SRC_STOP: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SRC_STOP command";
                getControlMutexFun();

                src_sdk->stop();

                if (g_state.sys_state == SYS_STATE_TASK_NAV_NO_WAY) {
                    // 当前状态为NO_WAY且收到stop时，将状态切换为IDLE，仅为了显示友好
                    g_state.sys_state = SYS_STATE_IDLE;
                    g_state.ready_for_new_movement_task = false;
                }
                result_state = RESPONSE_OK;

                break;
            }
            case CMD_START_MANUAL_CONTROL: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_START_MANUAL_CONTROL command";
                getControlMutexFun();

                if (g_state.isBreakSwitchON()) {
                    throw EXEC_ERROR(ERROR_CODE_MANUAL_CONTROL_IN_BREAK_SW,
                                     "start manual control failed! break switch is on!");
                }
                if (g_state.isEmergency()) {
                    throw EXEC_ERROR(ERROR_CODE_MANUAL_CONTROL_IN_EMERGENCY,
                                     "start manual control failed! is emergency!");
                }

                if (isMovementRunningFun()) {
                    throw EXEC_ERROR(ERROR_CODE_MANUAL_CONTROL_MOVEMENT_RUNNING,
                                     "movement is running! Can not start manual control");
                }

                if (isMissionRunningFun()) {
                    throw EXEC_ERROR(ERROR_CODE_MANUAL_CONTROL_MISSION_RUNNING,
                                     "Mission is running! Can not start manual control");
                }

                if (g_state.operation_state != OPERATION_MANUAL) {
                    manual_control_time_record = 0;
                    g_state.operation_state = OPERATION_MANUAL;  // 切换操作模式为手动模式
                    src_sdk->stop();                             // 保证不处于path状态
                    src_sdk->enterVelocityMode();
                    src_sdk->run();
                }

                result_state = RESPONSE_OK;
                break;
            }
            case CMD_STOP_MANUAL_CONTROL: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_STOP_MANUAL_CONTROL command";
                getControlMutexFun();

                if (g_state.operation_state == OPERATION_MANUAL) {
                    src_sdk->stop();
                    g_state.operation_state = OPERATION_AUTO;  // 切换操作模式为自动模式
                } else {
                    LOGGER(WARNING, CMD_HANDER) << "current is in auto control!";
                }

                result_state = RESPONSE_OK;
                break;
            }
            case CMD_SRC_RESET: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SRC_RESET command";
                getControlMutexFun();

                resetSRCCar();
                return;  // 不需要回复
                break;
            }
            case CMD_SRC_SPEED_LEVEL: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SRC_SPEED_LEVEL command, new speed is " << msg->param0;
                getControlMutexFun();

                auto new_user_set_speed_level_global = msg->param0;
                auto &s = sros::core::Settings::getInstance();
                s.setTmpValue("main.user_speed_level_global", std::to_string(new_user_set_speed_level_global));
                // NOTE: 此处为啥要先设进去然后读出来呢？ 原因是可能区域设置了临时参数
                new_user_set_speed_level_global = s.getValue<int>("main.user_speed_level_global", 100);

                src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_USER, new_user_set_speed_level_global);
                result_state = RESPONSE_OK;
                break;
            }
            case CMD_SET_CUR_STATION: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SET_CUR_STATION command, new station is " << msg->param0;
                getControlMutexFun();

                g_state.station_no = (StationNo_t)msg->param0;
                result_state = RESPONSE_OK;
                break;
            }
            case CMD_MAP_SWITCHING: {
                if (msg->param_boolean) {
                    LOGGER(INFO, CMD_HANDER) << "Handling CMD_MAP_SWITCHING command, absolute location!";
                } else {
                    LOGGER(INFO, CMD_HANDER) << "Handling CMD_MAP_SWITCHING command, normal location!";
                }
                getControlMutexFun();

                handleMapSwitching(msg);

                break;
            }
            case CMD_STOP_SROS: {
                LOGGER(WARNING, CMD_HANDER) << "Handling CMD_STOP_SROS command, this command is abandoned!";
                //                getControlMutexFun();

                //                stopAllModules();
                result_state = RESPONSE_OK;
                break;
            }
            case CMD_RESET_SROS: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_RESET_SROS command";  // 重启后报的各种问题都不需要管
                // 由于升级是由HTTP通知sros的，此时命令的session_id一直为0，导致一直无法升级。FIXME：v4.9.0:
                // HTTP解决这个问题
                //            if (!getControlMutexFun()) {
                //                break;
                //            }

                stopMusic();

                // 重启src
                auto reset_msg = std::make_shared<sros::core::CommandMsg>(getName());
                reset_msg->command = sros::core::CMD_SRC_RESET;
                sendMsg(reset_msg);

                //修复软重启不急停问题
                auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
                d_msg->command = sros::core::CMD_TRIGGER_EMERGENCY;
                sros::core::MsgBus::sendMsg(d_msg);

                // 记录系统时间, 防止往回设置日期，然后日期不变的问题
                MetaSetting::getInstance().setValue("sys.time", util::get_timestamp_in_s());

                std::thread t([]() {
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));
                    LOGGER(INFO, SROS) << "SROS shutdown!";
                    // sync(); //  同步磁盘数据,将缓存数据回写到硬盘,以防数据丢失
                    // sleep(2);
                    // reboot(RB_AUTOBOOT);
                    exit(-10);  // 非正常退出当前进程, 等待supervisor重启sros进程
                });
                t.detach();

                result_state = RESPONSE_OK;
                break;
            }
            case CMD_ENABLE_OBSTACLE_AVOID: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_ENABLE_OBSTACLE_AVOID command";
                getControlMutexFun();

                manual_enable_laser_oba_ = true;

                obstacle_avoid_enable_ = true;

                g_state.oba_state = OBA_ENABLED;
                result_state = RESPONSE_OK;

                break;
            }
            case CMD_DISABLE_OBSTACLE_AVOID: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_DISABLE_OBSTACLE_AVOID command";
                getControlMutexFun();

                manual_enable_laser_oba_ = false;

                obstacle_avoid_enable_ = false;

                g_state.oba_state = OBA_DISABLED;
                result_state = RESPONSE_OK;

                break;
            }
            case CMD_SET_SRC_SPEED: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SET_SRC_SPEED command, new speed is (" << msg->param0 << ", "
                                         << msg->param1 << ")";
                getControlMutexFun();

                if (g_state.operation_state == OPERATION_MANUAL && g_state.manual_btn_state == BTN_AUTO) {
                    manual_control_time_record = 0;
                    src_sdk->setVelocity(msg->param0, msg->param1);
                    result_state = RESPONSE_OK;
                } else {
                    throw EXEC_ERROR(ERROR_CODE_SET_SPEED_NOT_IN_MANUAL, "Car is NOT in manual control mode!");
                }
                break;
            }
            case CMD_SET_GPIO_OUTPUT: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_SET_GPIO_OUTPUT command, new output is 0x" << std::hex
                                         << msg->param0;
                getControlMutexFun();

                auto value = (uint8_t)(msg->param0 & 0x000000FF);
                src_sdk->setGPIOOuputBits(0xff, value);
                //src_sdk->setGPIOOuput(0xff00 + value);
                result_state = RESPONSE_OK;
                break;
            }
            case CMD_SET_SPEAKER_VOLUME:
            case CMD_SET_HMI_STATE: {
                // process this in hmi module
                return;
            }
            case CMD_NEW_ACTION_TASK: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_NEW_ACTION_TASK command, task no is "
                                         << msg->action_task->getTaskNo();

                getControlMutexFun();
                if (msg->source != "MissionModule" && MissionManager::getInstance()->isMissionRunning()) {
                    LOG(WARNING) << "MOVEMENT_TASK: mission is running, new task is ignored. source module is "
                                 << msg->source;

                    throw EXEC_ERROR(ERROR_CODE_ACTION_IN_MISSION_RUNNING, "Mission is running, new task is ignored");
                }

                // 检测启动动作任务的条件
                g_state.checkStartActionTaskCondition(
                    msg->action_task->getActionID(),
                    msg->action_task->getActionParam(),
                    msg->action_task->getActionParam1()
                );

                auto cur_task = sros::core::TaskManager::getInstance()->getActionTask();
                if (cur_task) {
                    LOG(INFO) << cur_task << " state is " << cur_task->getState();
                }

                if (cur_task && cur_task->isRunning()) {
                    throw EXEC_ERROR(ERROR_CODE_ACTION_PRE_TASK_RUNNING,
                                     "previous task is running, new task is ignored.");
                }
                
                auto fault = FaultCenter::getInstance()->truckActionTaskRunningFault();
                if (fault) {
                    if (fault->id == FAULT_CODE_SCREEN_TURNED_UP) {
                        if (msg->source == "Network") {
                            throw EXEC_ERROR(ERROR_CODE_ACTION_SCREEN_TURNED_UP,
                                             "screen turned up track action task running!");
                        }
                    } else {
                        throw EXEC_ERROR(ERROR_CODE_ACTION_FAULT_EXIST, "fault", fault->id,
                                         "track action task running!");
                    }
                }

                // 防止多个现场并发，所以此处必须先预启动
                auto action_task = msg->action_task;
                sros::core::TaskManager::getInstance()->setActionWaitForStart(action_task);

                // 通过另外一个topic转发
                auto new_msg = std::make_shared<CommandMsg>(*msg);
                new_msg->topic_ = "ACTION_CMD";
                sendMsg(new_msg);

                return;  // 具体返回在各个action控制器中处理
            }
            case CMD_CANCEL_ACTION_TASK: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_CANCEL_ACTION_TASK command";

                getControlMutexFun();

                auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
                if (!action_task) {
                    throw EXEC_ERROR(ERROR_CODE_UNDEFINED, "action task is NULL");
                }

                // 通过另外一个topic转发
                auto new_msg = std::make_shared<CommandMsg>(*msg);
                new_msg->topic_ = "ACTION_CMD";
                sendMsg(new_msg);
                return;  // 具体返回在各个action控制器中处理
            }
            case CMD_CANCEL_EMERGENCY: {
                auto &s = sros::core::Settings::getInstance();
                if (s.getValue<std::string>("main.vehicle_controller_type", "VC300") == "VC400" &&
                    s.getValue<int>("main.security_unit_type", 1) == 2) {
                    if (g_state.emergency_state != sros::core::STATE_EMERGENCY_RECOVERABLE) {
                        // 如果当前状态不是“可恢复急停”，则不予处理
                        throw EXEC_ERROR(ERROR_CODE_CANCEL_EMERGENCY_CAN_NOT_RECOVER,
                                        "emergency_state is STATE_EMERGENCY_TRIGER, can't recover");
                    }
                    src_sdk->sendCommandMsg(COMMAND_EMERGENCY_RECOVER);
                    break;
                }
                // 其他情况放到vsc中处理
            }
            case CMD_TRIGGER_EMERGENCY: {
                auto &s = sros::core::Settings::getInstance();
                if (s.getValue<std::string>("main.vehicle_controller_type", "VC300") == "VC400" &&
                    s.getValue<int>("main.security_unit_type", 1) == 2) {
                    src_sdk->sendCommandMsg(COMMAND_TRIGGER_EMERGENCY);
                    break;
                }
            }
            case CMD_ENABLE_AUTO_CHARGE:
            case CMD_STOP_CHARGE:
            case CMD_ENTER_POWER_SAVE_MODE:
            case CMD_EXIT_POWER_SAVE_MODE: {
                // handled in VSC module
                return;
            }
            case CMD_RESET_FAULT: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_RESET_FAULT command, fault id :" << msg->param0;
                getControlMutexFun();

                if (msg->param0 != 0) {
                    FaultCenter::getInstance()->recover(msg->param0);
                }
                break;
            }
            case CMD_ADD_PGV_INFO: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_ADD_PGV_INFO command";

                throw EXEC_ERROR(ERROR_CODE_FUNCTION_ABANDONED, "");
                break;
            }
            case CMD_DEL_PGV_INFO: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_DEL_PGV_INFO command";
                throw EXEC_ERROR(ERROR_CODE_FUNCTION_ABANDONED, "");
                break;
            }
            default: {
                return;
                break;
            }
        }
    } catch (const ExecError &e) {
        // 命令处理出错的情况

        //如果是动作中下发移动任务失败  || 或者路网路径下发异常
        if(msg->command == CMD_NEW_MOVEMENT_TASK && msg->source == "ActionController") {
            LOG(INFO) << "msg->source ActionController, new move task fail";

            auto notify_msg = std::make_shared<sros::core::NotificationMsg>("TOPIC_NOTIFY");
            notify_msg->notify_type = sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED;
            msg->movement_task->finishMoveTask(sros::core::TASK_RESULT_FAILED, e.errorCode());
            notify_msg->movement_task = msg->movement_task;
            MsgBus::sendMsg(notify_msg);
            return;
        }

        msg->result_state = RESPONSE_FAILED;
        msg->result_code = e.errorCode();
        responseCommand(msg);
        return;
    }

    msg->result_state = result_state;
    msg->result_code = ERROR_CODE_NONE;
    responseCommand(msg);
}

void MainModule::onMapUpdated(const core::base_msg_ptr &m) {
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);
    const std::string &map_name = msg->str_0_;

    MapManager::getInstance()->mapUpdated(map_name);
}

void MainModule::stopMusic() {
    auto msg = std::make_shared<core::HmiMsg>();
    msg->command = HMI_COMMAND_STOP;
    sendMsg(msg);
}

void MainModule::resetSRCCar() {
    src_sdk->stop();

    src_sdk->reset();
    src_sdk->disconnect();

    is_src_resetting_ = true;
    is_src_connected_ = false;
    LOGGER(INFO, SROS) << "SRC shutdown!";
}

void MainModule::stopSlamLocation() {
    sendSlamCommand(COMMAND_STOP_LOCATION);
    g_state.location_state = LOCATION_STATE_NONE;
}

void MainModule::onSRCPathFinish() {
//    LOG(INFO) << "==============> onSRCPathFinish";
    if (g_state.main_state == STATE_TASK_RUN_PATH) {
//        LOG(INFO) << "TASK_RUN_PATH: 路径执行完成,到达目标点";
        g_src_state.cur_path_no = 0;
        sendNavCommand(COMMAND_NAV_FINISH);

    } else if (g_state.main_state == STATE_TASK_NAVIGATING) {
        if (g_state.nav_state != STATE_NAV_INITIALIZING) {
//            LOG(INFO) << "NAVIGATION: 导航路径执行完成,到达目标点!";
            g_src_state.cur_path_no = 0;
            sendNavCommand(COMMAND_NAV_FINISH);
        }
    } else {
//        LOG(INFO) << "OTHER: 路径执行完成,到达目标点.";
    }
}

void MainModule::onSRCPathAborted() {
    LOG(INFO) << "==============> onSRCPathAborted";

#if 0
    // TODO(nobody): 路径执行中止时，设置当前MovementTask为失败状态
    if (g_state.main_state == STATE_TASK_RUN_PATH) {
        LOG(INFO) << "TASK_RUN_PATH: 路径执行中止";
        g_src_state.cur_path_no = 0;
        sendNavCommand(COMMAND_NAV_FINISH);

    } else if (g_state.main_state == STATE_TASK_NAVIGATING) {
        if (g_state.nav_state != STATE_NAV_INITIALIZING) {
            LOG(INFO) << "NAVIGATION: 导航路径执行中止!";
            g_src_state.cur_path_no = 0;
            sendNavCommand(COMMAND_NAV_FINISH);
        }
    } else {
        LOG(INFO) << "OTHER: 路径执行中止.";
    }
#endif
}

void MainModule::onSRCPathPaused() { LOG(INFO) << "==============> onSRCPathPaused"; }

void MainModule::onTimer_20s(const core::base_msg_ptr &m) {
    // 更新开机时间及里程
    std::thread t([]() { sros::core::RunLogger::getInstance().keepAlive(g_src_state.total_mileage); });
    t.detach();

    // 将tcmalloc缓存中的内存释放到系统，当查内存的时候需要加上，当不查内存的时候需要将其去除
    // MallocExtension::instance()->ReleaseFreeMemory();

    if(g_state.operation_state == OPERATION_MANUAL) {
        if(++manual_control_time_record > 9) {
            LOG(INFO) << "will stop manual control because not receive contorl message recent 3min";
            src_sdk->stop();
            g_state.operation_state = OPERATION_AUTO; //3min左右没有任何用户下发速度，直接
                                                      //退出手动控制，不加锁
        }
    }
}

void MainModule::onTimer_5s(const core::base_msg_ptr &m) {
    // 根据是否处于充电状态，自动切换低功耗模式
    togglePowerMode();

    // 记录系统时间
    MetaSetting::getInstance().setValue("sys.time", util::get_timestamp_in_s());
}

// 检查是否处于充电状态，如果打开了充电自动休眠功能，则自动进入低功耗模式
void MainModule::togglePowerMode() {
    auto &s = Settings::getInstance();
    auto enable_power_save_when_charging =
        s.getValue<std::string>("main.enable_power_save_when_charging", "False") == "True";

    if (!enable_power_save_when_charging) {
        return;
    }

    // 如果处于充电状态，自动进入低功耗模式
    if (pre_battery_state_ == BATTERY_NO_CHARGING && g_state.battery_state == BATTERY_CHARGING &&
        g_state.power_state == POWER_NORMAL) {
        LOG(INFO) << "battery in charging, enter power save mode";

        // 进入低功耗模式
        auto d_msg = make_shared<CommandMsg>(getName());
        d_msg->command = CMD_ENTER_POWER_SAVE_MODE;
        sendMsg(d_msg);
    }

    // 如果停止充电，则退出低功耗模式
    if (pre_battery_state_ == BATTERY_CHARGING && g_state.battery_state == BATTERY_NO_CHARGING &&
        g_state.power_state == POWER_SAVE_MODE) {
        LOG(INFO) << "battery charging finished, exit power save mode";

        // 退出低功耗模式
        auto d_msg = make_shared<CommandMsg>(getName());
        d_msg->command = CMD_EXIT_POWER_SAVE_MODE;
        sendMsg(d_msg);
    }

    pre_battery_state_ = g_state.battery_state;
}

/**
 * 每秒触发一次
 */
void MainModule::onTimer_1s(const core::base_msg_ptr &m) {
    //    static int time_count_recycle_disk=0;
    //    const int RECYCLE_TIME = 30*60*1;  //every 30 minutes to recycle the disk
    //    time_count_recycle_disk++;
    //    if (time_count_recycle_disk % RECYCLE_TIME == 0) {
    //        // 临时禁用FileManager，等待测试ok后再加入系统中
    //        sros::core::FileManager::getInstance().recycleDisk();
    //    }
    if (src_sdk->isReconnSrc()) {
        LOG(INFO) << "reconning src...";
        initSRCCar();
    } else if (is_src_resetting_) {  // 重启后自动连接SRC
        LOG(INFO) << "正在尝试连接到SRC...";
        initSRCCar();
    }

    if (g_state.location_state == LOCATION_STATE_RUNNING) {
        // 保存当前位姿到文件中,启动时读取作为初始位姿
        Pose pose = src_sdk->getCurPose();
        PoseSaver::save(pose, g_state.getCurMapName(), g_state.station_no);
    }

    auto &s = sros::core::Settings::getInstance();
    auto enable_sros_native_debug = (s.getValue<std::string>("debug.enable_sros_native_debug", "False") == "True");
    if (enable_sros_native_debug) {
        return;  // 若是本地调试，就不需要检查这些东西
    }

    updateDownCameraRealtimeInfo();

}

void MainModule::updateDownCameraRealtimeInfo() {
    auto &s = Settings::getInstance();
    bool enable_pgv_rectify = (s.getValue<std::string>("main.enable_pgv_rectify", "False") == "True");
    auto imu_ins550 = s.getValue<int>("main.pgv_version_param", 0);
    bool is_from_src = (imu_ins550 / 10 % 10) != 5;

    // 增加判断数据是否来源于src，如果不是则无需更新
    if (enable_pgv_rectify && is_from_src) {
        auto pgv_device = device::DeviceManager::getInstance()->getDeviceByName(device::DEVICE_PGV_DOWN);
        if (pgv_device && pgv_device->isOk()) {  // PGV 设备好使的时候才获取PGV的值
            auto results = src_sdk->getCurPgvInfo();
            if (results.empty()) {
                //        DLOG(INFO) << "get pgv failed!";
                return;
            }
            auto pgv_direction_offset = Settings::getInstance().getValue<double>("main.pgv_direction_offset", 90.0);
            DMCodeOffset offset(std::to_string(results.at(0)), results.at(1) / 10000.0, results.at(2) / 10000.0,
                                normalizeYaw0To2Pi((results.at(3) / 10.0 + pgv_direction_offset) * DEGREE_TO_RAD));
            g_state.cur_down_camera_offset.set(offset);
        }
    }
}

void MainModule::updateGPIOState() {
    const auto &src_state = src_sdk->getSRCState();
    if (g_state.gpio_input != src_state.gpio_input) {
        LOG(INFO) << "GPIO input changed: 0x" << std::hex << g_state.gpio_input << " -> 0x" << src_state.gpio_input;
        g_state.gpio_input = src_state.gpio_input;
    }
    if (g_state.gpio_output != src_state.gpio_output) {
        LOG(INFO) << "GPIO output changed: 0x" << std::hex << g_state.gpio_output << " -> 0x" << src_state.gpio_output;
        g_state.gpio_output = src_state.gpio_output;
    }
    if(!g_state.gpio_input){    //初始化全0不做处理
        return ;
    }

    //O车不需要做以下处理
    auto &s = sros::core::Settings::getInstance();
    std::string vehicleType = s.getValue<std::string>("main.vehicle_type","");
    std::string type = vehicleType.substr(0,4);
    if(type != "gulf" && type != "Gulf"){   //非叉车不处理以下逻辑
        return ;
    }

    static bool left_forktip_last = true;
    static bool right_forktip_last = true;
    static bool left_load_last = true;
    static bool right_load_last = true;
    static bool left_ps_last = true;
    static bool right_ps_last = true;
    static bool left_press_last = true;
    static bool right_press_last = true;

    bool left_forktip = true;
    bool right_forktip = true;
    bool left_load = true;
    bool right_load = true;
    bool left_ps = true;
    bool right_ps = true;
    bool left_press = true;
    bool right_press = true;

    auto getBit = [] (uint16_t num, int bit) {
        return ((num >> bit) & 0x01);
    };

    auto setState = [] (bool &laste_state, bool state, string state_name) {
        if(laste_state != state) {
            laste_state = state;
            LOGGER(INFO, DEVICE) << state_name<<" "<< (state ? "OFF":"ON");
        }   
    };

    bool enable_1353 = (s.getValue<std::string>("main.enable_1353", "False") == "True");
    if(enable_1353) {
        uint16_t io_1353 = g_state.io_1353;
        if(!io_1353){    //初始化全0不做处理
            return ;
        }
        
        //1353传感器IO解析
        //bit: 0,1 :左右叉尖防撞开关  2,3 :左右货物到位开关  4,5 :左右光电开关  6,7 :左右下压开关  8 ：横移超出范围
        left_forktip = getBit(io_1353,0);
        right_forktip = getBit(io_1353,1);
        left_load = getBit(io_1353,2);
        right_load = getBit(io_1353,3);
        left_ps = getBit(io_1353,4);
        right_ps = getBit(io_1353,5);
        left_press = getBit(io_1353,6);
        right_press = getBit(io_1353,7);

    }else{  //第一版的电气接线
        uint8_t gpio_output = g_state.gpio_input;

        left_forktip = getBit(gpio_output,1);
        right_forktip = getBit(gpio_output,2);
        left_load = getBit(gpio_output,3);
        right_load = getBit(gpio_output,4);
        left_ps = getBit(gpio_output,5);
        right_ps = getBit(gpio_output,6);
    }

    //打印具体的IO变化
    setState(left_forktip_last, left_forktip, "left_forktip");
    setState(right_forktip_last, right_forktip, "right_forktip");
    //src有上传该打印，不重复打印
    setState(left_load_last, left_load, "left_load");
    setState(right_load_last, right_load, "right_load");
    setState(left_ps_last, left_ps, "left_ps");
    setState(right_ps_last, right_ps, "right_ps");
    setState(left_press_last, left_press, "left_press");
    setState(right_press_last, right_press, "right_press");

    //检查叉臂防撞开关
    auto check_forktip_collision = [&] {
        if(!left_forktip){   //常闭，平时为1，触发为0
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_EMERGENCY_LEFT_TRIGGER_FORKTIP);
        }
        if(!right_forktip){
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_EMERGENCY_RIGHT_TRIGGER_FORKTIP);
        }
    };

    auto enable_ft_switch = ( s.getValue<std::string>(
                                "obstacle.enable_forktip_collision", "False") == "True");
    if(enable_ft_switch){    //叉车系列且打开了配置
        check_forktip_collision();
    }
}

void MainModule::onTimer_50ms(const core::base_msg_ptr &m) {
    // 更新系统状态: SYS_STATE_IDLE_USELESS <-> SYS_STATE_IDLE_READY
    auto check_ready_fun = [&] {
        auto movement_task = TaskManager::getInstance()->getMovementTask();
        return (!g_state.isEmergency() && !g_state.isBreakSwitchON() && !g_state.isPowerSaveMode() &&
                g_state.isLocateSucceed() && !g_state.isManualControl() &&
                (!movement_task || movement_task->isFinished()) && !g_state.isLaserError() && !g_state.isSrcError() &&
                !g_state.isMotor1Error() && !g_state.isMotor2Error() && !g_state.isVscError());
    };
    if (g_state.sys_state == SYS_STATE_IDLE) {
        if (check_ready_fun()) {
            g_state.ready_for_new_movement_task = true;
        } else {
            g_state.ready_for_new_movement_task = false;
        }
    }

    updateGPIOState();

    updateIOPauseState();
}

void MainModule::performAreaOperation() {
    core::Pose cur_pose = src_sdk->getCurPose();
    auto new_area_list = core::MapManager::getInstance()->getInsideArea(cur_pose.x() * 100, cur_pose.y() * 100);
    std::stable_sort(new_area_list.begin(), new_area_list.end(),
                     [](const sros::map::AreaMark &a, const sros::map::AreaMark &b) { return a.z < b.z; });

    if (resetting_area_info_) {
        resetting_area_info_ = false;
    } else {
        if (new_area_list == area_list_) {
            return;
        }
    }
    LOG(INFO) << "current area changed! area size is " << new_area_list.size() << " and current pose is " << cur_pose;

    area_list_ = std::move(new_area_list);

    uint8_t area_set_speed_level = 100;  // 不在任何限速区域内时不限速
    auto area_disable_oba = false;       // enable oba if it is NOT in DISABLE_OBA area
    auto area_no_entry = false;          // 是否进入了禁止进入区， 进入了就强制停车

    auto getAreaInfoFun = [&](uint16_t type) {
        if (type == AreaMark::AREA_TYPE_SPEED_LEVEL_20) {
            area_set_speed_level = std::min(area_set_speed_level, (uint8_t)20);
        } else if (type == AreaMark::AREA_TYPE_SPEED_LEVEL_40) {
            area_set_speed_level = std::min(area_set_speed_level, (uint8_t)40);
        } else if (type == AreaMark::AREA_TYPE_SPEED_LEVEL_60) {
            area_set_speed_level = std::min(area_set_speed_level, (uint8_t)60);
        } else if (type == AreaMark::AREA_TYPE_SPEED_LEVEL_80) {
            area_set_speed_level = std::min(area_set_speed_level, (uint8_t)80);
        }

        else if (type == AreaMark::AREA_TYPE_DISABLE_OBA) {
            area_disable_oba = true;
        } else if (type == AreaMark::AREA_TYPE_NO_ENTER) {
            area_no_entry = true;
        }
    };

    std::map<std::string, std::string> tmp_settings;
    auto &s = sros::core::Settings::getInstance();
    for (const auto &area : area_list_) {
        LOG(INFO) << "area id: " << area.id << " index: " << area.z << " type: " << area.type 
        << " cur_pose:("<<cur_pose.x() * 100 <<", "<<cur_pose.y() * 100
        << ") A(" << area.pa.x << "," << area.pa.y << ") B(" << area.pb.x << "," << area.pb.y 
        << ") C(" << area.pc.x << "," << area.pc.y << ") D(" << area.pd.x << "," << area.pd.y << ")";
        if (area.type == AreaMark::AREA_TYPE_MULTIPLE_TYPE) {
            for (auto type : area.user_define_type_list) {
                getAreaInfoFun(type);
            }

            for (const std::pair<std::string, std::string> &param : area.user_define_param) {
                LOG(INFO) << "SET (" << param.first << ", " << param.second << ")";
                tmp_settings[param.first] = param.second;
            }
        } else {
            getAreaInfoFun(area.type);
        }
    }

    auto user_speed_level_global = s.getValue<int>("main.user_speed_level_global", 100);

    s.setTmpValues(std::move(tmp_settings));

    auto area_user_set_speed_level = s.getValue<int>("main.user_speed_level", 100);

    s.setTmpValue("main.user_speed_level_global", std::to_string(user_speed_level_global));

    int new_area_set_speed_level = std::min((int)area_set_speed_level, area_user_set_speed_level);

    // LOG(INFO) << "area_set_speed_level: " << new_area_set_speed_level << ", new_user_set_speed_level: " <<
    // user_speed_level_global;
    src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_AREA, new_area_set_speed_level);
    src_car.setSpeedLevel(SPEED_LIMIT_SOURCE_USER, user_speed_level_global);

    if (pre_area_disable_oba_ != area_disable_oba) {
        LOG(INFO) << " -> AREA_TYPE_DISABLE_OBA " << area_disable_oba;
        // 构造DebugCmdMsg处理
        auto d_msg = std::make_shared<CommandMsg>(getName());

        if (area_disable_oba) {
            d_msg->command = CMD_DISABLE_OBSTACLE_AVOID;
        } else {
            d_msg->command = CMD_ENABLE_OBSTACLE_AVOID;
        }

        pre_area_disable_oba_ = area_disable_oba;

        sendMsg(d_msg);
    }

    auto fault_center = sros::core::FaultCenter::getInstance();
    if (pre_area_no_enter_ != area_no_entry) {
        LOG(INFO) << " -> AREA_TYPE_NO_ENTRY " << area_no_entry;

        if (area_no_entry) {
            if (TaskManager::getInstance()->isMovementSlaveRunning()) {
                auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
                d_msg->command = CMD_SRC_PAUSE;
                d_msg->param0 = 0;
                //移动任务进入禁止进入区域才报错
                fault_center->track(sros::core::FAULT_CODE_ENTER_NO_ENTRY_AREA, [&]() { return area_no_entry; }, nullptr, nullptr, false);
                sendMsg(d_msg);
                LOGGER(ERROR, SROS) << "Agv enter no entry area! force pause the agv!!!";
            }
        } else {
            //只要是离开禁止进入区域都消除错误
            fault_center->track(sros::core::FAULT_CODE_ENTER_NO_ENTRY_AREA, [&]() { return area_no_entry; }, nullptr, nullptr, false);
        }
        pre_area_no_enter_ = area_no_entry;
    }
}

void MainModule::updateParameterByLoadState() {
    // 与上一次状态不同时，才进行参数配置
    if (set_load_state_ != g_state.load_state) {
        double vehicle_width;
        double vehicle_length;
        std::string str_w("nav.vehicle_width");
        std::string str_l("nav.vehicle_length");

        // 目前navigation_module已经自己处理了载货和无货之间避障尺寸的切换，所以此处理论上已经没有用了
        // 为了避免未知问题，故此处暂时保留
        if (g_state.load_state == sros::core::LOAD_FULL) {
            // LOG(INFO)<<"Loading full! change the size of vehicle!";
            vehicle_width = Settings::getInstance().getValue<int>("rack.max_contour_width", 600) / 1000.0;
            vehicle_length = Settings::getInstance().getValue<int>("rack.max_contour_length", 1060) / 1000.0;
        } else {
            // LOG(INFO)<<"Loading none! change the size of vehicle!";
            vehicle_width = Settings::getInstance().getValue<double>("nav.vehicle_width", 0.5);
            vehicle_length = Settings::getInstance().getValue<double>("nav.vehicle_length", 0.7);
        }

        auto msg1 = std::make_shared<ParameterMsg>("NAV_PARAMETER");
        msg1->name = str_w;
        msg1->value = std::to_string(vehicle_width);
        sendMsg(msg1);

        auto msg2 = std::make_shared<ParameterMsg>("NAV_PARAMETER");
        msg2->name = str_l;
        msg2->value = std::to_string(vehicle_length);
        sendMsg(msg2);

        set_load_state_ = g_state.load_state;
    }
}

void MainModule::updateMovementTaskProcess() {
    auto src_state = src_sdk->getSRCState();

    auto move_task = sros::core::TaskManager::getInstance()->getMovementTask();
    if (move_task) {
        move_task->updateProcessInfo(src_state.path_remain_time, src_state.path_remain_distance,
                                     src_state.path_total_distance);
    }
}

/**
 * 每100ms触发一次
 */
void MainModule::onTimer_100ms(const core::base_msg_ptr &m) {
    // 检查全部的device，查看是否有超时的设备
    auto dm = sros::device::DeviceManager::getInstance();
    auto device_map = dm->getDeviceList();
    for (auto &device : *device_map) {
        device.second->checkAlive();
    }

    auto &s = sros::core::Settings::getInstance();
    auto enable_sros_native_debug = (s.getValue<std::string>("debug.enable_sros_native_debug", "False") == "True");
    if (enable_sros_native_debug) {
        return;  // 若是本地调试，就不需要检查这些东西
    }

    FaultCenter::getInstance()->track(FAULT_CODE_LOCATION_ERROR, [&]() -> bool { return g_state.isLocationError(); });

    // 根据当前所在区域执行对应的操作
    performAreaOperation();

    // 根据LoadState配置车体参数
    updateParameterByLoadState();

    // 更新当前MovementTask的进度状态
    updateMovementTaskProcess();

    updateIOCancelEmergencyState();

    updateIOScreenTurnedUpState();

    PoseChecker::autoDetectStation();

    updateTrafficControlState();

    checkUpSvc100ScanState();

    //载货状态下检测栈板到位信号
    loadingCheckPalletSignal();
}

void MainModule::loadingCheckPalletSignal() {

    if(g_state.is_loading_check_pallet_signal_) {

        // 检测到位信号是否触发
        if(g_state.load_state != sros::core::LOAD_FULL) {
            LOG(ERROR) << "loading check pallet signal: false";
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);

        }
    }
}

void MainModule::checkUpSvc100ScanState() {
    auto &s = sros::core::Settings::getInstance();
    bool enable_rotary_jack_keep_scan = (s.getValue<std::string>("inspection.enable_rotary_jack_keep_scan", "False") == "True");

    if(!enable_rotary_jack_keep_scan) {
        return;
    }

    static int CALC_COUNT = 8;

    if(g_state.is_check_up_svc100_scan_state
        && (g_state.load_state == sros::core::LOAD_FULL)) {

        if(!is_last_sliding_window_calc_) {
            is_last_sliding_window_calc_ = true;
            lst_sliding_window_calc_.clear();
        }

        std::string up_camera_offset_info = g_state.cur_up_camera_offset.get().no;
        int result = (up_camera_offset_info.empty()) ? 0 : 1;

        //检测连续的8帧数据
        lst_sliding_window_calc_.push_back(result);
        int lst_size = lst_sliding_window_calc_.size();

        if (lst_size < CALC_COUNT) {
            return;
        } else if (lst_size > CALC_COUNT) {
            lst_sliding_window_calc_.pop_front();
        }

        int sum = 0;
        for (auto val : lst_sliding_window_calc_){
            sum += val;
        }

        //LOG(INFO) << "lst_size: " << lst_sliding_window_calc_.size() << ", sum: " << sum;

        //连续8次中，有6次以上没有检测到码就触发急停
        if(sum <= 2) {
            //如果在410动作执行过程中检测到触发逻辑，则不触发急停
            auto cur_task = sros::core::TaskManager::getInstance()->getActionTask();
            if (cur_task->getActionID() == 4 && cur_task->getActionParam1() == 1 && cur_task->isRunning()){
                LOGGER(INFO,SROS) << "action 410 is running. disable the rotary_jack_keep_scan";
                return;
            }

            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_UPSVC100_SCAN_FAIL);
            g_state.sros_emergency_src = g_state.sros_emergency_src | sros::core::SROS_EMERGENCY_UPSVC100_SCAN;
            std::shared_ptr<sros::core::CommonCommandMsg<std::string>> cmd_msg(new sros::core::CommonCommandMsg<std::string>("SAVE_IMG"));
            LOG(INFO) << "lst_size: " << lst_sliding_window_calc_.size() << ", sum: " << sum;
            cmd_msg->seq = 4096;
            cmd_msg->command = "SAVE";
            cmd_msg->str0 = sros::device::DEVICE_SVC100_UP;
            sendMsg(cmd_msg);
        }

        //连续8次中，有6次以上检测到码就解除急停
        if(sum >= 6) {
            sros::core::FaultCenter::getInstance()->removeFault(sros::core::FAULT_CODE_UPSVC100_SCAN_FAIL);
            g_state.sros_emergency_src = g_state.sros_emergency_src & (~sros::core::SROS_EMERGENCY_UPSVC100_SCAN);
        }

    } else {
        is_last_sliding_window_calc_ = false;
    }
}

void MainModule::updateIOPauseState() {
    sros::core::Settings &settings = Settings::getInstance();
    auto enable_io_pause = settings.getValue<string>("io.enable_pause", "False") == "True";

    if (!enable_io_pause ||
        (g_state.main_state != STATE_TASK_NAVIGATING && g_state.main_state != STATE_TASK_RUN_PATH)) {
        // 如果没有启用IO减速或没有在执行路径, 则不处理检测结果, 直接返回
        return;
    }

    int io_bits = settings.getValue("io.pause_input_bits", 0);
    uint8_t io_mask = (uint8_t)(1 << io_bits);

    auto io_set_paused = src_car.getPauseState(PAUSE_SOURCE_IO);
    auto pre_io_set_paused = io_set_paused;

    int io_value = ((g_state.gpio_input & io_mask) == 0) ? 0 : 1;
    int io_active_value = settings.getValue("io.pause_input_active_value", 0);

    if (io_value == io_active_value && io_value != pre_io_pause_input_value_) {
        // IO信号有效，且与上一次检测到的信号值不同（防止一直按下按钮的情况）

        LOG(INFO) << "IO input." << io_bits << " trigger";

        if (io_set_paused) {
            // 如果当前处于暂停状态，那么取消暂停
            io_set_paused = false;

            if (pre_io_set_paused != io_set_paused) {
                LOG(INFO) << "IO触发暂停取消，恢复正常速度执行";
                g_state.sys_state = SYS_STATE_TASK_NAV_WAITING_FINISH;
                src_car.setPauseState(PAUSE_SOURCE_IO, false);
            }
        } else {
            // 否则，暂停
            io_set_paused = true;

            if (pre_io_set_paused != io_set_paused) {
                LOG(INFO) << "IO触发暂停";
                g_state.sys_state = SYS_STATE_TASK_NAV_PAUSED;
                src_car.setPauseState(PAUSE_SOURCE_IO, true);
            }
        }

        auto enable_pause_response = settings.getValue<string>("io.enable_pause_response", "False") == "True";

        if (enable_pause_response) {
            int pause_response_bits = settings.getValue("io.pause_response_bits", 0);
            int pause_response_active_value = settings.getValue("io.pause_response_active_value", 0);

            // // 计算需要设置的bit值
            // auto set_value = (uint8_t)(pause_response_active_value << pause_response_bits);

            // // 设置IO output值
            // auto new_output_value = io_set_paused ? (uint8_t)(g_state.gpio_output | set_value)
            //                                       : (uint8_t)(g_state.gpio_output & ~set_value);

            // LOG(INFO) << "g_state.gpio_output = " << static_cast<int>(g_state.gpio_output);
            // LOG(INFO) << "set_value = " << static_cast<int>(set_value);
            // LOG(INFO) << "new_output_value = " << static_cast<int>(new_output_value);

            // src_sdk->setGPIOOuput(new_output_value);

            uint8_t value = 1 << pause_response_bits;
            if(pause_response_active_value == 0 && io_set_paused
                || pause_response_active_value != 0 && !io_set_paused) {
                src_sdk->setGPIOOuputBits(value, 0);
            } else {
                src_sdk->setGPIOOuputBits(0, value);
            }
        }
    }

    pre_io_pause_input_value_ = io_value;
}

void MainModule::updateIOCancelEmergencyState() {
    sros::core::Settings &settings = Settings::getInstance();
    auto enable = settings.getValue<string>("main.enable_io_cancel_emergency", "False");

    if (enable == "True" && g_state.emergency_state == STATE_EMERGENCY_RECOVERABLE) {
        // 如果没有启用IO解除急停且目前不处于急停可恢复状态, 则不处理检测结果, 直接返回

        int io_bits = settings.getValue("main.io_cancel_emergency_bits", 0);
        uint8_t io_mask = (uint8_t)(1 << io_bits);

        int io_value = ((g_state.gpio_input & io_mask) == 0) ? 0 : 1;
        int io_active_value = settings.getValue("main.io_cancel_emergency_active_value", 0);

        if (io_value == io_active_value) {  // IO信号有效
            LOG(INFO) << "IO cancel emergency signal triggered!!!!";

            auto mm = make_shared<CommandMsg>(getName());
            mm->command = CMD_CANCEL_EMERGENCY;

            sendMsg(mm);
        }
    }
}

void MainModule::updateIOScreenTurnedUpState() {
    sros::core::Settings &settings = Settings::getInstance();
    auto enable = settings.getValue<string>("io.enable_screen_turned_up_check", "False");

    if (enable == "True") {
        // 如果没有启用IO解除急停且目前不处于急停可恢复状态, 则不处理检测结果, 直接返回

        int io_bits = settings.getValue("io.screen_turned_up_input_bits", 3);
        uint8_t io_mask = (uint8_t)(1 << io_bits);

        int io_value = ((g_state.gpio_input & io_mask) == 0) ? 0 : 1;
        int io_active_value = settings.getValue("io.screen_turned_up_input_active_value", 1);

        if (io_value == io_active_value) {  // IO信号有效
            g_state.screen_turned_up_ = true;
            FaultCenter::getInstance()->addFault(FAULT_CODE_SCREEN_TURNED_UP);
        } else {
            g_state.screen_turned_up_ = false;
            FaultCenter::getInstance()->removeFault(FAULT_CODE_SCREEN_TURNED_UP);
        }
    }
}

void MainModule::sendSrcForkParams() {
    auto &s = sros::core::Settings::getInstance();
    int fork_up_speed_rate = s.getValue<int>("inspection.fork_up_speed_rate", 100);
    int fork_down_speed_rate = s.getValue<int>("inspection.fork_down_speed_rate", 100);
    int proportional_critical_value = s.getValue<int>("inspection.proportional_critical_value", 100);
    int fork_feed_speed = s.getValue<float>("inspection.fork_feed_speed", 0.02) * 1000;
    int forklift_type = s.getValue<int>("inspection.forklift_type", 1);
    int fork_encoder_resolution = s.getValue<int>("inspection.fork_encoder_resolution", 16384);
    int fork_coupling_speed_threshold = s.getValue<float>("inspection.fork_coupling_speed_threshold", 0.2) * 1000;

    bool ret = src_sdk->setForkTruckParams(fork_up_speed_rate, fork_down_speed_rate, proportional_critical_value, 
                                fork_feed_speed,forklift_type,fork_encoder_resolution,fork_coupling_speed_threshold);
    if(!ret){
        LOG(ERROR) << "src_sdk->setForkTruckParams failed!";
    }
}


void MainModule::sendModuleParameters() {
    auto settings_list = sros::core::Settings::getInstance().getItemInfoLists();

    std::map<string, string> section_topic;
    section_topic["posefilter"] = "POSEFILTER_PARAMETER";
    section_topic["slam"] = "SLAM_PARAMETER";
    section_topic["nav"] = "NAV_PARAMETER";
    section_topic["main"] = "MAIN_PARAMETER";
    section_topic["station_recog"] = "STATION_RECOG_PARAMETER";

    ParameterMsg_ptr msg;

    for (const auto &item : settings_list) {
        auto section_name = sros::core::Settings::getSectionName(item.key);

        if (section_topic.count(section_name) > 0) {
            string topic_name = section_topic.find(section_name)->second;
            msg.reset(new ParameterMsg(topic_name));
            msg->name = item.key;
            msg->value = item.value;
            if (section_name == "main") {
            } else {
                sendMsg(msg);  // 发布msg
            }
        }
    }
}

void MainModule::stopAllModules() {
    LOG(INFO) << "MainModule::stopAllModules()";

    // 发送start Msg来启动Timer等module
    auto msg = make_shared<core::StrMsg>("COMMAND");
    msg->data = "stop";
    sendMsg(msg);  // 第一次消息设置state

    LOG(INFO) << "MainModule::stopAllModules() stop -> 1";

    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    sendMsg(msg);  // 第二次消息停止module
    LOG(INFO) << "MainModule::stopAllModules() stop -> 2";
}

void MainModule::onDMCodeInfoMsg(const sros::core::base_msg_ptr &m) {
    auto msg = std::dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(m);

    // 如果开启了下视摄像头矫正就将实时偏差数据上传
    if (msg->camera_name_ == sros::device::DEVICE_SVC100_DOWN) {
        auto &s = sros::core::Settings::getInstance();
        bool enable_pgv_rectify = (s.getValue<std::string>("main.enable_pgv_rectify", "False") == "True");
        auto imu_ins550 = s.getValue<int>("main.pgv_version_param", 0);
        bool is_from_vision = (imu_ins550 / 10 % 10) == 5;

        // 增加判断数据是否来源于视觉，如果不是则无需更新
        if (enable_pgv_rectify && is_from_vision) {
            if (msg->state_ == DM_CODE_DETECTED) {
                DMCodeOffset offset(msg->code_str_, msg->x_, msg->y_, msg->angle_);
                g_state.cur_down_camera_offset.set(offset);
            } else {
                DMCodeOffset offset;
                g_state.cur_down_camera_offset.set(offset);
            }
        }
        
    } else if (msg->camera_name_ == sros::device::DEVICE_SVC100_UP) {
        if (msg->state_ == DM_CODE_DETECTED) {
            DMCodeOffset offset(msg->code_str_, msg->x_, msg->y_, msg->angle_);
            g_state.cur_up_camera_offset.set(offset);
        } else {
            DMCodeOffset offset;

            g_state.cur_up_camera_offset.set(offset);
        }
    }
}

void MainModule::updateNavState(sros::core::NavigationState state, int param_int) {
    auto m = make_shared<CommonStateMsg<sros::core::NavigationState>>("NAV_STATE");
    m->state = state;
    m->param_int = param_int;
    sendMsg(m);
}

void MainModule::responseCommand(const sros::core::CommandMsg_ptr &msg) {
    auto response_msg = std::make_shared<CommandMsg>(msg->source, "TOPIC_CMD_RESPONSE");
    response_msg->command = msg->command;
    response_msg->session_id = msg->session_id;
    response_msg->req_seq = msg->req_seq;
    response_msg->result_state = msg->result_state;
    response_msg->result_code = msg->result_code;
    sendMsg(response_msg);
}

void MainModule::updateTrafficControlState() const {
    if (TaskManager::getInstance()->isMovementSlaveRunning()) {
        if (g_src_state.is_waitting_checkpoint) {
            if (!TaskManager::getInstance()->isWaitingForCheckpoint()) {
                TaskManager::getInstance()->setWaitingForCheckpoint();
            }
        } else {  // 新状态不是交通管制
            if (TaskManager::getInstance()->isWaitingForCheckpoint()) {
                switch (g_state.sys_state) {
                    case SYS_STATE_TASK_PATH_NAV_INITIALING: {
                        TaskManager::getInstance()->setMovementStart();
                        break;
                    }
                    case SYS_STATE_TASK_PATH_WAITING_FINISH:
                    case SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW:
                    case SYS_STATE_TASK_PATH_PAUSED: {
                        TaskManager::getInstance()->setMovementRunning();
                        break;
                    }
                    case SYS_STATE_TASK_MANUAL_PAUSED: {
                        TaskManager::getInstance()->setMovementPause();
                        break;
                    }
                    default: {
                        LOG(ERROR) << "UNREACHABLE! " << g_state.sys_state;
                    }
                }
            }
        }
    }
}

// 更新任务状态（暂停或继续）
void MainModule::updateActionState(const core::base_msg_ptr &msg,bool _bPauseOrConinue) 
{
    auto cmd_msg = dynamic_pointer_cast<CommandMsg>(msg);
    auto action_task = core::TaskManager::getInstance()->getActionTask();
    if (action_task && action_task->isSlaveRunning()) {
        int action_id = action_task->getActionID();
        auto ac_type = ac::ActionManager::getInstance()->GetActionType(action_id);
        switch (ac_type)
        {
        case ACT_TYPE_SROS:
        case ACT_TYPE_SRC:{
            src_car.setPauseState(PAUSE_SOURCE_NAV, _bPauseOrConinue);
            break;
        }
        case ACT_TYPE_EAC:{
            auto new_msg = std::make_shared<CommandMsg>(*cmd_msg);
            new_msg->topic_ = "ACTION_CMD";
            sendMsg(new_msg);
            break;
        }
        default:
            break;
        }
    }
}

}  // namespace sros
