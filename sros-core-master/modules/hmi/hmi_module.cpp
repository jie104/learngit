//
// Created by penglei on 11/15/17.
//

#include "hmi_module.h"

#include <boost/utility.hpp>
#include <functional>

#include "core/device/can_interface.h"
#include "core/device/device_manager.h"
#include "core/exec_error.hpp"
#include "core/fault_center.h"
#include "core/msg/can_data_msg.hpp"
#include "core/msg/command_msg.hpp"
#include "core/msg/common_msg.hpp"
#include "core/msg/hmi_msg.hpp"
#include "core/msg/notification_msg.hpp"
#include "core/msg/str_msg.hpp"
#include "core/settings.h"
#include "core/src.h"
#include "core/state.h"
#include "core/task/task_manager.h"
#include "core/user_manager.h"

using namespace sros::core;
using namespace sros::device;

namespace hmi {

HMIModule::HMIModule()
    : Module("HMIModule"),
      speaker_music_vol_(0),
      cur_play_music_id_(MUSIC_ID_MUSIC_NONE),
      next_play_music_id_(MUSIC_ID_MUSIC_NONE),
      is_playing_notify_music_(false),
      is_speaker_mute_(false) {}

HMIModule::~HMIModule() {}

void HMIModule::run() {
    LOG(INFO) << "HMIModule module start running";
    auto &s = sros::core::Settings::getInstance();
    enable_hmi_ = (s.getValue<std::string>("main.enable_hmi", "False") == "True");

    if (!enable_hmi_) {
        LOG(ERROR) << "HMI module disabled";
        stop();
        return;
    }

    auto enable_speaker = s.getValue<std::string>("hmi.enable_speaker", "True") == "True";
    low_battery_threshold_ = s.getValue<int>("main.low_battery_threshold", 15);
    auto speaker_can_id = std::stoi(s.getValue<string>("hmi.speaker_can_id", "202"), nullptr, 16);

    //电池温度阈值
    battery_emergency_temperature_ = sros::core::Settings::getInstance().getValue<int>("battery.emergency_temperature",60);
    battery_warning_temperature_ = sros::core::Settings::getInstance().getValue<int>("battery.warning_temperature",50);

    auto default_volume = s.getValue<int>("hmi.speaker_volume", 50);
    speaker_music_vol_ = default_volume;
    g_state.cur_volume = default_volume;

    if (enable_speaker) {
        speaker_ptr_ = createDevice<Speaker>(DEVICE_SPEAKER, DEVICE_ID_SPEAKER, DEVICE_COMM_INTERFACE_TYPE_CAN_1,
                                             std::make_shared<CanInterface>(speaker_can_id, speaker_can_id + 1));
        CHECK(speaker_ptr_);
        speaker_ptr_->setModelNo("speaker");
        speaker_ptr_->setSerialNo("-");
        speaker_ptr_->setVersionNo("0.0.0");
    }

	int can_id = s.getValue<int>("hmi.lightcontrl_can_id", 0x302);
	LOG(INFO) << "!!!!!Light Control Device CAN ID is " << can_id;
    // 是否开启由sros直接控制lc100
    auto is_hmi_lc100_for_sros = (s.getValue<std::string>("hmi.is_hmi_lc100_for_sros", "False") == "True");
    if (is_hmi_lc100_for_sros) {
        lc100_ptr_agv_ = createDevice<LC100>(DEVICE_LC100, DEVICE_ID_LC100, DEVICE_COMM_INTERFACE_TYPE_CAN_1,
                                             std::make_shared<CanInterface>(can_id));
        CHECK(lc100_ptr_agv_);

        lc100_ptr_agv_->setModelNo(DEVICE_LC100);
        lc100_ptr_agv_->setSerialNo("-");
        lc100_ptr_agv_->setVersionNo("0.0.0");

        auto type = s.getValue<std::string>("hmi.middle_led_type", "tricolor");
        if (type == "extension") {
            the_middle_led_type_ = EXTENSION;
        } else if (type == "tricolor-two_wire") {
            the_middle_led_type_ = TRICOLOR_TWO_WIRE;
        } else if (type == "AMR600") {
            the_middle_led_type_ = AMR600;
        } else if (type == "business") {
            the_middle_led_type_ = BUSINESS;
        } else {
            the_middle_led_type_ = TRICOLOR;
        }

        // 通知 vsc模块，不要在控制LC100了
        auto msg = make_shared<sros::core::StrMsg>("TOPIC_VSC_COMMAND");
        msg->data = "DISABLE_VSC_HMI";
        sendMsg(msg);
        LOG(INFO) << "HMIModule send DISABLE_VSC_HMI";
    }

    subscribeTopic("TOPIC_MUSIC", CALLBACK(&HMIModule::onMusicMsg));
    subscribeTopic("TIMER_1S", CALLBACK(&HMIModule::onTimer_1s));
    subscribeTopic("TIMER_200MS", CALLBACK(&HMIModule::onTimer_200ms));
    subscribeTopic("DEBUG_CMD", CALLBACK(&HMIModule::onDebugCmdMsg));
    subscribeTopic("TOPIC_NOTIFY_CONNECTED_CLIENT_COUNT", CALLBACK(&HMIModule::onNotifyConnectedClinetCountMsg));
    subscribeTopic("TOPIC_NOTIFY", CALLBACK(&HMIModule::onNotifyTaskResultMsg));


    // boost::thread(boost::bind(&HMIModule::test, this));
    // wait speaker start!!
    boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));

    LOG(INFO) << "HMI module start working now";
    dispatch();
}

void HMIModule::onDebugCmdMsg(sros::core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<sros::core::CommandMsg>(msg);
    auto session_id = mm->session_id;
    auto getControlMutexFun = [&]() {
        if (!g_state.control_mutex.get(session_id)) {
            throw EXEC_ERROR(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, "locker:", g_state.control_mutex.getLockerSessionId(),
                             "current:", session_id);
        }
    };
    try {
        switch (mm->command) {
            case sros::core::CMD_SET_SPEAKER_VOLUME: {
                LOGGER(INFO, CMD_HANDER) << "Handing CMD_SET_SPEAKER_VOLUME command, new volume is " << mm->param0;
                getControlMutexFun();

                std::vector<uint8_t> data;
                if (mm->param0 < 0 || mm->param0 > 100) {
                    throw EXEC_ERROR(ERROR_CODE_SET_SPEAKER_VOLUME_PARAM_ERROR, "set volume error: ", mm->param0);
                }

                auto new_volume = mm->param0;

                // 将新音量保存到配置中
                sros::core::Settings::getInstance().setValue("hmi.speaker_volume", new_volume);

                speaker_music_vol_ = new_volume;
                g_state.cur_volume = (uint8_t)new_volume;

                is_speaker_mute_ = false;
                playMusicItem(cur_play_music_id_);

                mm->result_state = RESPONSE_OK;

                LOGGER(INFO, CMD_HANDER) << "set new volume to " << new_volume;
                break;
            }
            case sros::core::CMD_SET_HMI_STATE: {
                LOGGER(INFO, CMD_HANDER) << "Handing CMD_SET_HMI_STATE command, state " << mm->param0 << " continue "
                                         << mm->param1 << "ms";
                getControlMutexFun();

                if (mm->param0 < (int)UserSetState::WAITING_FOR_ELEVATOR ||
                    mm->param0 > (int)UserSetState::WHERE_AM_I) {
                    throw EXEC_ERROR(ERROR_CODE_SET_HMI_STATE_PARAM_ERROR, "user set state param0 error!");
                }
                auto duration = std::max(std::min(mm->param1, 5000), 1000);
                user_state_duration_time_ = duration;
                user_set_state_ = (UserSetState)mm->param0;

                mm->result_state = RESPONSE_OK;

                break;
            }
            case sros::core::CMD_SET_HMI_CUSTOM_STATE: {
                LOGGER(INFO, CMD_HANDER) << "Handing CMD_SET_HMI_CUSTOM_STATE command, state " << mm->param0 << " continue "
                                         << mm->param1 << "ms";
                getControlMutexFun();

                auto duration = std::max(std::min(mm->param1, 5000), 1000);
                custom_state_duration_time_ = duration;
                custom_state = (CustomState)mm->param0;

                mm->result_state = RESPONSE_OK;
            }
            default: {
                return;  // 不是由此处处理，也不会消息
            }
        }
    } catch (const ExecError &e) {
        // 命令处理出错的情况
        mm->result_state = RESPONSE_FAILED;
        mm->result_code = e.errorCode();
        responseCommand(mm);
        return;
    }

    responseCommand(mm);
}

void HMIModule::onNotifyConnectedClinetCountMsg(sros::core::base_msg_ptr msg) {
    auto m = dynamic_pointer_cast<sros::core::CommonMsg>(msg);

    if (m->int_0_ > 0) {
        connect_state_ = ConnectState::CONNECTED;
    } else {
        connect_state_ = ConnectState::DISCONNECTED;
    }
}

void HMIModule::onTimer_1s(sros::core::base_msg_ptr m) {
    // 记录单曲播放的时间，若时间超过3秒要重发一次，重发一次不会重新播放，但是会返回结果，这样就可以根据结果来让喇叭设备keepalive
    static uint32_t i = 0;

    // 根据当前状态获取要播放的音乐id
    if (next_play_music_id_ != cur_play_music_id_) {
        cur_play_music_id_ = next_play_music_id_;
        playMusicItem(cur_play_music_id_);
        i = 0;
    } else {
        ++i;
        if (i > 3) {
            playMusicItem(cur_play_music_id_);
        }
    }
}

void HMIModule::onTimer_200ms(sros::core::base_msg_ptr m) {

    if (exec_result_duration_time_ > 0) {
        exec_result_duration_time_ -= 200;
    }
    if (user_state_duration_time_ > 0) {
        user_state_duration_time_ -= 200;
    }
    if (custom_state_duration_time_ > 0)
    {
        custom_state_duration_time_ -= 200;
    }

    updateState();  // 更新状态

    setHMIState();
}

void HMIModule::onMusicMsg(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<sros::core::HmiMsg>(m);

    switch (msg->command) {
        case HMI_COMMAND_STOP: {
            is_speaker_mute_ = true;
            playMusicItem(cur_play_music_id_);

            break;
        }
        case HMI_COMMAND_MUTE: {
            LOG(INFO) << "- - - - - > MUTE MUSIC";

            is_speaker_mute_ = true;
            playMusicItem(cur_play_music_id_);
            break;
        }
        case HMI_COMMAND_MUTE_CANCEL: {
            LOG(INFO) << "- - - - - > MUTE_CANCEL MUSIC";

            is_speaker_mute_ = false;
            playMusicItem(cur_play_music_id_);
            break;
        }
        case HMI_COMMAND_SET_SUCCEED_MUSIC: {
            exec_result_duration_time_ = 5000;  // 动作反馈执行5s
            error_music_id_ = msg->int_0;
            result_state_ = ResultState::SUCCEED;
            break;
        }
        case HMI_COMMAND_SET_ERROR_MUSIC: {
            exec_result_duration_time_ = 5000;  // 动作反馈执行5s
            error_music_id_ = msg->int_0;
            result_state_ = ResultState::FAILED;
            break;
        }
        default: {
            LOG(ERROR) << "unreachable! command is " << msg->command;
            break;
        }
    }
}

void HMIModule::playMusicItem(int id) {
    if (id < MUSIC_ID_MUSIC_NONE) {
        LOG(ERROR) << "INVALID MUSIC ID: " << id;
        return;
    }

    if (cur_play_music_id_ != id) {
        LOG(INFO) << "Music change to " << id;
        //    } else {
        //        return;
    }

    cur_play_music_id_ = id;

    int volume = 0;
    if (!is_speaker_mute_) {
        volume = (int)((speaker_music_vol_ / 100.0) * 28);
    }

    if (speaker_ptr_) {
        speaker_ptr_->syncPlay(static_cast<uint8_t>(id), static_cast<uint8_t>(volume));
    }

    last_sent_time_ = sros::core::util::get_time_in_ms();
}

void HMIModule::updateState() {
    auto action_task = TaskManager::getInstance()->getActionTask();
    auto movement_task = TaskManager::getInstance()->getMovementTask();

    auto fault_center = FaultCenter::getInstance();
    const Fault * fault = fault_center->getFirstFault();
    if (g_state.battery_percentage < very_low_battery_threshold_) {
        system_state_ = SeriousSystemState::BATTERY_TOO_LOW;
    } else if (g_state.sys_state == SYS_STATE_INITIALING || g_state.sys_state == SYS_STATE_ZERO) {
        system_state_ = SeriousSystemState::INITIALIZING;
    } else if (fault_center->findFaultByCode(FAULT_CODE_POWER_SAVE_MODE)) {
        system_state_ = SeriousSystemState::POWER_SAVE_MODE;
    } else if (fault != nullptr && fault->level > WARNING) {
        if (g_state.battery_state == BATTERY_CHARGING) {
            if (fault->id == FAULT_CODE_EMERGENCY_TRIGGER_BUTTON || 
                fault->id == FAULT_CODE_EMERGENCY_TRIGGER_EDGE ||
                fault->id == FAULT_CODE_EMERGENCY_TRIGGER_SOFTWARE ||
                fault->id == FAULT_CODE_EMERGENCY_TRIGGER_BATTERY_DOOR_OPEN ||
                fault->id == FAULT_CODE_EMERGENCY_TRIGGER_UNKNOWN) {
                system_state_ = SeriousSystemState::FAULT;
                fault_music_id_ = fault->music;
            } else {
                system_state_ = SeriousSystemState::CHARGING;
            }
        } else if (fault->id == FAULT_CODE_BREAK_SWITCH_ON) {
            system_state_ = SeriousSystemState::BREAk_SW_NO;
        } else if (g_state.isManualControl()) {
            if (fault->id == FAULT_CODE_EMERGENCY_TRIGGER_BUTTON || 
                fault->id == FAULT_CODE_EMERGENCY_TRIGGER_EDGE ||
                fault->id == FAULT_CODE_EMERGENCY_TRIGGER_SOFTWARE ||
                fault->id == FAULT_CODE_EMERGENCY_TRIGGER_BATTERY_DOOR_OPEN || 
                fault->id == FAULT_CODE_BREAK_SWITCH_ON ||
                fault->id == FAULT_CODE_EMERGENCY_TRIGGER_UNKNOWN) {
                system_state_ = SeriousSystemState::FAULT;
                fault_music_id_ = fault->music;
            } else {
                system_state_ = SeriousSystemState::MANUAL_CONTROL;
            }
        } else if (fault->id == FAULT_CODE_LOCATION_ERROR) {
            system_state_ = SeriousSystemState::LOCATE_ERROR;
        } else if (fault->id == FAULT_CODE_FMS_DISCONNECT) {
            system_state_ = SeriousSystemState::FMS_DISCONNECT;
        } else {
            if (movement_task && movement_task->isRunning() && !fault_center->truckMovementTaskRunningFault()) {
                system_state_ = SeriousSystemState::NONE;
            } else if (action_task && action_task->isRunning() && !fault_center->truckActionTaskRunningFault()) {
                system_state_ = SeriousSystemState::NONE;
            } else {
                system_state_ = SeriousSystemState::FAULT;
                fault_music_id_ = fault->music;
            }
        }
    } else if (g_state.battery_state == BATTERY_CHARGING) {
        system_state_ = SeriousSystemState::CHARGING;
    } else if (g_state.isManualControl()) {
        system_state_ = SeriousSystemState::MANUAL_CONTROL;
    } else if ((movement_task && movement_task->isPaused()) || (action_task && action_task->isPaused())) {
        system_state_ = SeriousSystemState::PAUSE;
    } else {
        system_state_ = SeriousSystemState::NONE;
    }
    //pmu temperature monitor
    if (g_state.battery_temperature > battery_emergency_temperature_ && g_state.battery_percentage < 101){
        system_state_ = SeriousSystemState::EMERGENCY_TEMPERATURE;
    } else if (g_state.battery_temperature > battery_warning_temperature_ && g_state.battery_percentage < 101){
        system_state_ = SeriousSystemState::WARNING_TEMPERATURE;
    }
    //    LOG(INFO) << "system_state_: " << (int)system_state_ << " music: " << fault_music_id_;

    // 根据src上传的movement_state得到运动状态
    if ((!movement_task || !movement_task->isSlaveRunning())&&!g_state.in_manual_obstacle_paused) {
        motion_state_ = MotionState::NONE;
    } else if (g_src_state.is_waitting_checkpoint) {
        motion_state_ = MotionState::TRAFFIC_CONTROL;
    } else if (g_state.sys_state == SYS_STATE_TASK_PATH_PAUSED || g_state.sys_state == SYS_STATE_TASK_NAV_PAUSED ||
               g_state.in_manual_obstacle_paused) {
        switch (g_state.obstacle_direction) {
            case OBSTACLE_DIRECTION_UNKNOWN: {
                motion_state_ = MotionState::BLOCKED_DIRECTION_UNKNOWN;
                break;
            }
            case OBSTACLE_DIRECTION_FORWARD: {
                motion_state_ = MotionState::BLOCKED_DIRECTION_FORWARD;
                break;
            }
            case OBSTACLE_DIRECTION_BACKWARD: {
                motion_state_ = MotionState::BLOCKED_DIRECTION_BACKWARD;
                break;
            }
            case OBSTACLE_DIRECTION_LEFT: {
                motion_state_ = MotionState::BLOCKED_DIRECTION_LEFT;
                break;
            }
            case OBSTACLE_DIRECTION_RIGHT: {
                motion_state_ = MotionState::BLOCKED_DIRECTION_RIGHT;
                break;
            }
            case OBSTACLE_DIRECTION_LEFT_FORWARD: {
                motion_state_ = MotionState::BLOCKED_DIRECTION_LEFT_FORWARD;
                break;
            }
            case OBSTACLE_DIRECTION_RIGHT_FORWARD: {
                motion_state_ = MotionState::BLOCKED_DIRECTION_RIGHT_FORWARD;
                break;
            }
            case OBSTACLE_DIRECTION_LEFT_BACKWARD: {
                motion_state_ = MotionState::BLOCKED_DIRECTION_LEFT_BACKWARD;
                break;
            }
            case OBSTACLE_DIRECTION_RIGHT_BACKWARD: {
                motion_state_ = MotionState::BLOCKED_DIRECTION_RIGHT_BACKWARD;
                break;
            }
            default: {
                LOG(ERROR) << "unreachable!";
                break;
            }
        }
    } else if (g_state.sys_state == SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW ||
               g_state.sys_state == SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW) {
        switch (g_state.obstacle_direction) {
            case OBSTACLE_DIRECTION_UNKNOWN: {
                motion_state_ = MotionState::SLOWDOWN_DIRECTION_UNKNOWN;
                break;
            }
            case OBSTACLE_DIRECTION_FORWARD: {
                motion_state_ = MotionState::SLOWDOWN_DIRECTION_FORWARD;
                break;
            }
            case OBSTACLE_DIRECTION_BACKWARD: {
                motion_state_ = MotionState::SLOWDOWN_DIRECTION_BACKWARD;
                break;
            }
            case OBSTACLE_DIRECTION_LEFT: {
                motion_state_ = MotionState::SLOWDOWN_DIRECTION_LEFT;
                break;
            }
            case OBSTACLE_DIRECTION_RIGHT: {
                motion_state_ = MotionState::SLOWDOWN_DIRECTION_RIGHT;
                break;
            }
            case OBSTACLE_DIRECTION_LEFT_FORWARD: {
                motion_state_ = MotionState::SLOWDOWN_DIRECTION_LEFT_FORWARD;
                break;
            }
            case OBSTACLE_DIRECTION_RIGHT_FORWARD: {
                motion_state_ = MotionState::SLOWDOWN_DIRECTION_RIGHT_FORWARD;
                break;
            }
            case OBSTACLE_DIRECTION_LEFT_BACKWARD: {
                motion_state_ = MotionState::SLOWDOWN_DIRECTION_LEFT_BACKWARD;
                break;
            }
            case OBSTACLE_DIRECTION_RIGHT_BACKWARD: {
                motion_state_ = MotionState::SLOWDOWN_DIRECTION_RIGHT_BACKWARD;
                break;
            }
            default: {
                LOG(ERROR) << "unreachable!";
                break;
            }
        }
    } else {
        switch (g_src_state.movement_state) {
            case MOVEMENT_STILL: {
                motion_state_ = MotionState::STILL;
                break;
            }
            case MOVEMENT_GO_FORWARD: {
                motion_state_ = MotionState::FORWARD;
                movement_forward_music_id_ = movement_task->getMusicId();
                break;
            }
            case MOVEMENT_GO_BACKWARD: {
                motion_state_ = MotionState::BACKWARD;
                break;
            }
            case MOVEMENT_TURN_LEFT: {
                motion_state_ = MotionState::LEFTWARD;
                break;
            }
            case MOVEMENT_TURN_RIGHT: {
                motion_state_ = MotionState::RIGHTWARD;
                break;
            }
            default: {
                LOG(ERROR) << "HMIModule UNREACHABLE! motion_state_: " << (int)g_src_state.movement_state;
                break;
            }
        }
    }

    action_state_ = ActionState::NONE;
    if (action_task && action_task->isSlaveRunning()) {
        // 比如旋转顶升等动作执行过程中agv也会移动，所以要显示灯光。
        switch (g_src_state.movement_state) {
            case MOVEMENT_GO_FORWARD: {
                action_state_ = ActionState::FORWARD;
                break;
            }
            case MOVEMENT_GO_BACKWARD: {
                action_state_ = ActionState::BACKWARD;
                break;
            }
            case MOVEMENT_TURN_LEFT: {
                action_state_ = ActionState::LEFTWARD;
                break;
            }
            case MOVEMENT_TURN_RIGHT: {
                action_state_ = ActionState::RIGHTWARD;
                break;
            }
            default: {
                action_state_ = ActionState::RUNNING;
                break;
            }
        }

        int music_id = action_task->getMusicId();
        action_music_id_ = music_id;
    }

    // 去除通知的状态
    if (result_state_ != ResultState::NONE && exec_result_duration_time_ <= 0) {
        result_state_ = ResultState::NONE;
    }

    if (user_set_state_ != UserSetState::NONE && user_state_duration_time_ <= 0) {
        user_set_state_ = UserSetState::NONE;
    }

    if (custom_state != CustomState::NONE && custom_state_duration_time_ <= 0) {
        custom_state = CustomState::NONE;
    }

    if (g_state.is_map_switching) {
        normal_system_state_ = NormalSystemState::MAP_SWITCHING;
    } else {
        normal_system_state_ = NormalSystemState::NONE;
    }

    if (g_state.battery_percentage < low_battery_threshold_) {
        battery_state_ = BatteryState::LOW_BATTERY;
    } else {
        battery_state_ = BatteryState::NONE;
    }

    fault_center->track(sros::core::FAULT_CODE_BATTERY_TEMPERATURE_EMERGENCY,
                                [&]() { return system_state_ == SeriousSystemState::EMERGENCY_TEMPERATURE; });
    fault_center->track(sros::core::FAULT_CODE_BATTERY_TEMPERATURE_WARNING,
                                [&]() { return system_state_ == SeriousSystemState::WARNING_TEMPERATURE; });
}

void HMIModule::setHMIState() {
    next_play_music_id_ = MUSIC_ID_MUSIC_NONE;
    LC100::LedState left_led_state;
    //中间灯
    LC100::LedState middle_led_state_extension;  // 中间的圆形扩展灯
    LC100::LedState middle_led_state_tricolor;  // 中间的三色灯，蓝色被映射成了黄色，运行为绿色，故障为红色、待机为黄色
    LC100::LedState middle_led_state_tricolor_two_wire;  // 中间的三色灯，两根线，黄色由绿色和红色合成，运行为绿色，故障为红色、待机为黄色
    LC100::LedState middle_led_state_amr600;    // arm600的灯
    LC100::LedState middle_led_state_business;  // 商用灯

    LC100::LedState right_led_state;

    auto updateLC100StateFun = [&]() {
        if (lc100_ptr_agv_) {
            lc100_ptr_agv_->setLed(LED_LEFT, left_led_state);
            switch (the_middle_led_type_) {
                case TRICOLOR:
                    lc100_ptr_agv_->setLed(LED_MIDDLE, middle_led_state_tricolor);
                    break;
                case EXTENSION:
                    lc100_ptr_agv_->setLed(LED_MIDDLE, middle_led_state_extension);
                    break;
                case TRICOLOR_TWO_WIRE:
                    lc100_ptr_agv_->setLed(LED_MIDDLE, middle_led_state_tricolor_two_wire);
                    break;
                case AMR600:
                    lc100_ptr_agv_->setLed(LED_MIDDLE, middle_led_state_amr600);
                    break;
                case BUSINESS:
                    lc100_ptr_agv_->setLed(LED_MIDDLE, middle_led_state_business);
                    break;
                default:
                    break;
            }
            lc100_ptr_agv_->setLed(LED_RIGHT, right_led_state);
        }
    };
    middle_led_state_tricolor.setState(LED_BLUE, LED_STATIC);
    middle_led_state_tricolor_two_wire.setState(LED_YELLOW, LED_STATIC);
    if (user_set_state_ == UserSetState::WHERE_AM_I) {
        VINFO_EVERY_N(20) << "WHERE_AM_I";

        left_led_state.setState(LED_WHITE, LED_BLINK, 250);
        middle_led_state_extension.setState(LED_WHITE, LED_BLINK, 250);
        middle_led_state_business.setState(LED_WHITE, LED_BLINK, 250);
        middle_led_state_amr600.setState(LED_WHITE, LED_BLINK, 250);
        right_led_state.setState(LED_WHITE, LED_BLINK, 250);

        next_play_music_id_ = MUSIC_ID_MUSIC_FIND_AGV;
        updateLC100StateFun();
        return;
    }

    if (system_state_ != SeriousSystemState::NONE) {
        middle_led_state_extension.setState(LED_BLUE, LED_BLINK);
        middle_led_state_business.setState(LED_GREEN, LED_STATIC);
        middle_led_state_tricolor.setState(LED_RED, LED_STATIC);
        middle_led_state_tricolor_two_wire.setState(LED_RED, LED_STATIC);

        switch (system_state_) {
            case SeriousSystemState::INITIALIZING: {
                VINFO_EVERY_N(20) << "InitialisState::INITIALIZING";

                left_led_state.setState(LED_BLUE, LED_BLINK);
                middle_led_state_extension.setState(LED_BLUE, LED_BLINK);
                middle_led_state_business.setState(LED_RED, LED_STATIC);
                middle_led_state_amr600.setState(LED_BLUE, LED_BLINK);
                right_led_state.setState(LED_BLUE, LED_BLINK);
                break;
            }
            case SeriousSystemState::CHARGING: {
                VINFO_EVERY_N(20) << "CHARGING";

                left_led_state.setState(LED_GREEN, LED_BREATH, 4000);
                middle_led_state_extension.setState(LED_GREEN, LED_BREATH, 4000);
                middle_led_state_business.setState(LED_GREEN, LED_STATIC);
                right_led_state.setState(LED_GREEN, LED_BREATH, 4000);
                break;
            }
            case SeriousSystemState::POWER_SAVE_MODE: {
                VINFO_EVERY_N(20) << "Power Save";
                left_led_state.setState(LED_RED, LED_BREATH, 4000);
                middle_led_state_extension.setState(LED_RED, LED_BREATH, 4000);
                middle_led_state_business.setState(LED_RED, LED_BREATH);
                right_led_state.setState(LED_RED, LED_BREATH, 4000);
                break;
            }
            case SeriousSystemState::FAULT: {
                VINFO_EVERY_N(20) << "FAULT";

                next_play_music_id_ = fault_music_id_;

                left_led_state.setState(LED_RED, LED_STATIC);
                middle_led_state_extension.setState(LED_RED, LED_STATIC);
                middle_led_state_business.setState(LED_RED, LED_STATIC);
                middle_led_state_amr600.setState(LED_RED, LED_STATIC);
                right_led_state.setState(LED_RED, LED_STATIC);
                break;
            }
            case SeriousSystemState::BREAk_SW_NO: {
                VINFO_EVERY_N(20) << "BREAk_SW_NO";
                next_play_music_id_ = MUSIC_ID_MUSIC_BREAK_SWITCH_ON;

                left_led_state.setState(LED_WHITE, LED_BLINK, 1000);
                middle_led_state_extension.setState(LED_WHITE, LED_BLINK, 1000);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_amr600.setState(LED_WHITE, LED_BLINK, 1000);
                right_led_state.setState(LED_WHITE, LED_BLINK, 1000);
                break;
            }
            case SeriousSystemState::MANUAL_CONTROL: {
                VINFO_EVERY_N(20) << "MANUAL_CONTROL";

                left_led_state.setState(LED_WHITE, LED_STATIC);
                middle_led_state_extension.setState(LED_WHITE, LED_STATIC);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_amr600.setState(LED_WHITE, LED_STATIC);
                right_led_state.setState(LED_WHITE, LED_STATIC);
                break;
            }
            case SeriousSystemState::LOCATE_ERROR: {
                next_play_music_id_ = MUSIC_ID_MUSIC_LOCATION_ERROR;

                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                if (connect_state_ == ConnectState::DISCONNECTED) {
                    VINFO_EVERY_N(20) << "LOCATE DISCONNECTED";

                    left_led_state.setState(LED_BLUE, LED_BLINK);
                    middle_led_state_extension.setState(LED_BLUE, LED_BLINK);
                    right_led_state.setState(LED_BLUE, LED_BLINK);
                } else {
                    VINFO_EVERY_N(20) << "LOCATE CONNECTED";

                    left_led_state.setState(LED_CYAN, LED_BLINK);
                    middle_led_state_extension.setState(LED_CYAN, LED_BLINK);
                    right_led_state.setState(LED_CYAN, LED_BLINK);
                }
                break;
            }
            case SeriousSystemState::FMS_DISCONNECT: {
                VINFO_EVERY_N(20) << "FMS_DISCONNECT";

                left_led_state.setState(LED_MAGENTA, LED_STATIC);
                middle_led_state_extension.setState(LED_MAGENTA, LED_STATIC);
                middle_led_state_business.setState(LED_RED, LED_STATIC);
                right_led_state.setState(LED_MAGENTA, LED_STATIC);
                next_play_music_id_ = MUSIC_ID_MUSIC_FMS_RECONNECTING;
                break;
            }
            case SeriousSystemState::PAUSE: {
                VINFO_EVERY_N(20) << "PAUSE";

                left_led_state.setState(LED_GREEN, LED_BLINK, 250);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_GREEN, LED_BLINK, 250);
                next_play_music_id_ = MUSIC_ID_MUSIC_MISSION_PAUSED;
                break;
            }
            case SeriousSystemState::EMERGENCY_TEMPERATURE: {
                LOG(INFO) << "TEST CODE : battery emergency";
                next_play_music_id_ = MUSIC_ID_MUSIC_NONE;
                left_led_state.setState(LED_RED, LED_BLINK,250);
                middle_led_state_extension.setState(LED_RED, LED_BLINK,250);
                middle_led_state_business.setState(LED_RED, LED_BLINK,250);
                middle_led_state_amr600.setState(LED_RED, LED_BLINK,250);
                right_led_state.setState(LED_RED, LED_BLINK,250);
                break;
            }
            case SeriousSystemState::WARNING_TEMPERATURE: {
                LOG(INFO) << "TEST CODE : battery warning";
                next_play_music_id_ = MUSIC_ID_MUSIC_NONE;
                left_led_state.setState(LED_YELLOW, LED_BLINK,250);
                middle_led_state_extension.setState(LED_YELLOW, LED_BLINK,250);
                middle_led_state_business.setState(LED_YELLOW, LED_BLINK,250);
                middle_led_state_amr600.setState(LED_YELLOW, LED_BLINK,250);
                right_led_state.setState(LED_YELLOW, LED_BLINK,250);
                break;
            }

            default: {
                if (user_set_state_ == UserSetState::FMS_DISCONNECTED) {
                    VINFO_EVERY_N(20) << "FMS_DISCONNECTED";
                    left_led_state.setState(LED_MAGENTA, LED_STATIC);
                    middle_led_state_extension.setState(LED_MAGENTA, LED_STATIC);
                    middle_led_state_business.setState(LED_RED, LED_STATIC);
                    right_led_state.setState(LED_MAGENTA, LED_STATIC);
                    next_play_music_id_ = MUSIC_ID_MUSIC_FMS_RECONNECTING;
                }

                break;
            }
        }
        updateLC100StateFun();
        //如果手动状态下，发生了避障，是需要继续往下执行的，不能直接跳过
        if (!(system_state_ == SeriousSystemState::MANUAL_CONTROL && g_state.in_manual_obstacle_paused)) {
            return;
        }
    }

    if (custom_state != CustomState::NONE)
    {
        if (custom_state == CustomState::WAIT_FOR_MATCH_VSC_INFO)
        {
            left_led_state.setState(LED_RED, LED_STATIC);
            middle_led_state_extension.setState(LED_RED, LED_STATIC);
            middle_led_state_business.setState(LED_RED, LED_STATIC);
            middle_led_state_amr600.setState(LED_RED, LED_STATIC);
            right_led_state.setState(LED_RED, LED_STATIC);
            next_play_music_id_ = MUSIC_ID_MUSIC_RACK_DM_CODE_DETECTED;
            updateLC100StateFun();
        }
        return;
    }

    if (user_set_state_ == UserSetState::DISINGECTING) {
        if (motion_state_ < MotionState::BLOCKED_DIRECTION_UNKNOWN) {
            VINFO_EVERY_N(20) << "DISINGECTING";
            left_led_state.setState(LED_YELLOW, LED_BREATH, 4000);
            middle_led_state_extension.setState(LED_YELLOW, LED_STATIC);
            middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
            right_led_state.setState(LED_YELLOW, LED_BREATH, 4000);
            next_play_music_id_ = MUSIC_ID_MUSIC_DISINGECTING;
            updateLC100StateFun();
            return;
        }
    }

    if (motion_state_ != MotionState::NONE) {
        // LOG(INFO) << "motion state:" << (int)motion_state_;
        middle_led_state_extension.setState(LED_GREEN, LED_STATIC);
        middle_led_state_tricolor.setState(LED_GREEN, LED_STATIC);
        middle_led_state_business.setState(LED_GREEN, LED_STATIC);
        middle_led_state_tricolor_two_wire.setState(LED_GREEN, LED_STATIC);

        switch (motion_state_) {
            case MotionState::STILL: {
                VINFO_EVERY_N(20) << "STILL";
                VINFO_EVERY_N(20) << "system state: " << g_state.sys_state;

                left_led_state.setState(LED_GREEN, LED_BLINK, 250);
                middle_led_state_amr600.setState(LED_GREEN, LED_BLINK, 250);
                right_led_state.setState(LED_GREEN, LED_BLINK, 250);
                break;
            }
            case MotionState::FORWARD: {
                VINFO_EVERY_N(20) << "FORWARD";

                next_play_music_id_ = movement_forward_music_id_;
                left_led_state.setState(LED_GREEN, LED_STATIC);
                middle_led_state_amr600.setState(LED_GREEN, LED_STATIC);
                right_led_state.setState(LED_GREEN, LED_STATIC);
                break;
            }
            case MotionState::BACKWARD: {
                VINFO_EVERY_N(20) << "BACKWARD";

                next_play_music_id_ = MUSIC_ID_MUSIC_BACK_UP;
                left_led_state.setState(LED_GREEN, LED_BLINK);
                middle_led_state_amr600.setState(LED_GREEN, LED_BLINK);
                right_led_state.setState(LED_GREEN, LED_BLINK);
                break;
            }
            case MotionState::LEFTWARD: {
                VINFO_EVERY_N(20) << "LEFTWARD";

                next_play_music_id_ = MUSIC_ID_MUSIC_TURN_LEFT;
                left_led_state.setState(LED_YELLOW, LED_BLINK);
                middle_led_state_amr600.setState(LED_OFF, LED_BLINK);
                right_led_state.setState(LED_OFF, LED_BLINK);
                break;
            }
            case MotionState::RIGHTWARD: {
                VINFO_EVERY_N(20) << "RIGHTWARD";

                next_play_music_id_ = MUSIC_ID_MUSIC_TURN_RIGHT;
                left_led_state.setState(LED_OFF, LED_BLINK);
                middle_led_state_amr600.setState(LED_OFF, LED_BLINK);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_BLINK);
                break;
            }
            case MotionState::SLOWDOWN_DIRECTION_UNKNOWN: {
            }
            case MotionState::SLOWDOWN_DIRECTION_FORWARD: {
                VINFO_EVERY_N(20) << "SLOWDOWN";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_AHEAD;
                left_led_state.setState(LED_YELLOW, LED_BREATH);
                middle_led_state_amr600.setState(LED_YELLOW, LED_BREATH);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_BREATH);
                break;
            }
            case MotionState::SLOWDOWN_DIRECTION_BACKWARD: {
                VINFO_EVERY_N(20) << "SLOWDOWN";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_BEHIND;
                left_led_state.setState(LED_YELLOW, LED_BREATH);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_BREATH);
                break;
            }
            case MotionState::SLOWDOWN_DIRECTION_LEFT: {
                VINFO_EVERY_N(20) << "SLOWDOWN";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_LEFT;
                left_led_state.setState(LED_YELLOW, LED_BREATH);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_BREATH);
                break;
            }
            case MotionState::SLOWDOWN_DIRECTION_RIGHT: {
                VINFO_EVERY_N(20) << "SLOWDOWN";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_RIGHT;
                left_led_state.setState(LED_YELLOW, LED_BREATH);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_BREATH);
                break;
            }
            case MotionState::SLOWDOWN_DIRECTION_LEFT_FORWARD: {
                VINFO_EVERY_N(20) << "SLOWDOWN";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_LEFT_FRONT;
                left_led_state.setState(LED_YELLOW, LED_BREATH);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_BREATH);
                break;
            }
            case MotionState::SLOWDOWN_DIRECTION_RIGHT_FORWARD: {
                VINFO_EVERY_N(20) << "SLOWDOWN";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_RIGHT_FRONT;
                left_led_state.setState(LED_YELLOW, LED_BREATH);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_BREATH);
                break;
            }
            case MotionState::SLOWDOWN_DIRECTION_LEFT_BACKWARD: {
                VINFO_EVERY_N(20) << "SLOWDOWN";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_LEFT_REAR;
                left_led_state.setState(LED_YELLOW, LED_BREATH);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_BREATH);
                break;
            }
            case MotionState::SLOWDOWN_DIRECTION_RIGHT_BACKWARD: {
                VINFO_EVERY_N(20) << "SLOWDOWN";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_RIGHT_REAR;
                left_led_state.setState(LED_YELLOW, LED_BREATH);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_BREATH);
                break;
            }
            case MotionState::BLOCKED_DIRECTION_UNKNOWN: {
            }
            case MotionState::BLOCKED_DIRECTION_FORWARD: {
                VINFO_EVERY_N(20) << "BLOCKED";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_AHEAD;
                left_led_state.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_STATIC);
                break;
            }
            case MotionState::BLOCKED_DIRECTION_BACKWARD: {
                VINFO_EVERY_N(20) << "BLOCKED";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_BEHIND;
                left_led_state.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_STATIC);
                break;
            }
            case MotionState::BLOCKED_DIRECTION_LEFT: {
                VINFO_EVERY_N(20) << "BLOCKED";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_LEFT;
                left_led_state.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_STATIC);
                break;
            }
            case MotionState::BLOCKED_DIRECTION_RIGHT: {
                VINFO_EVERY_N(20) << "BLOCKED";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_RIGHT;
                left_led_state.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_STATIC);
                break;
            }
            case MotionState::BLOCKED_DIRECTION_LEFT_FORWARD: {
                VINFO_EVERY_N(20) << "BLOCKED";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_LEFT_FRONT;
                left_led_state.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_STATIC);
                break;
            }
            case MotionState::BLOCKED_DIRECTION_RIGHT_FORWARD: {
                VINFO_EVERY_N(20) << "BLOCKED";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_RIGHT_FRONT;
                left_led_state.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_STATIC);
                break;
            }
            case MotionState::BLOCKED_DIRECTION_LEFT_BACKWARD: {
                VINFO_EVERY_N(20) << "BLOCKED";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_LEFT_REAR;
                left_led_state.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_STATIC);
                break;
            }
            case MotionState::BLOCKED_DIRECTION_RIGHT_BACKWARD: {
                VINFO_EVERY_N(20) << "BLOCKED";

                next_play_music_id_ = MUSIC_ID_MUSIC_OBSTACLE_ON_THE_RIGHT_REAR;
                left_led_state.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_amr600.setState(LED_YELLOW, LED_STATIC);
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                right_led_state.setState(LED_YELLOW, LED_STATIC);
                break;
            }
            case MotionState::TRAFFIC_CONTROL: {
                VINFO_EVERY_N(20) << "TRAFFIC_CONTROL";
                middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
                if (connect_state_ == ConnectState::DISCONNECTED) {
                    left_led_state.setState(LED_MAGENTA, LED_BLINK,250);
                    middle_led_state_amr600.setState(LED_MAGENTA, LED_BLINK,250);
                    right_led_state.setState(LED_MAGENTA, LED_BLINK,250);
                }
                else {
                    left_led_state.setState(LED_MAGENTA, LED_STATIC);
                    middle_led_state_amr600.setState(LED_MAGENTA, LED_STATIC);
                    right_led_state.setState(LED_MAGENTA, LED_STATIC);
                }
                break;
            }
            default: {
                LOG(ERROR) << "HMIModule UNREACHABLE!";

                break;
            }
        }
        updateLC100StateFun();

        return;
    }

    if(action_or_motion_task_result_ == TaskResultSate::AGV_TASK_ERROR) {
        left_led_state.setState(LED_YELLOW, LED_BLINK, 250);
        middle_led_state_extension.setState(LED_YELLOW, LED_STATIC);
        middle_led_state_business.setState(LED_YELLOW, LED_STATIC);
        right_led_state.setState(LED_YELLOW, LED_BLINK, 250);
        updateLC100StateFun();
        return;
    }

    if (action_state_ != ActionState::NONE) {
        switch (action_state_) {
            case ActionState::FORWARD: {
                VINFO_EVERY_N(20) << "FORWARD";

                left_led_state.setState(LED_GREEN, LED_STATIC);
                right_led_state.setState(LED_GREEN, LED_STATIC);
                break;
            }
            case ActionState::BACKWARD: {
                VINFO_EVERY_N(20) << "BACKWARD";

                left_led_state.setState(LED_GREEN, LED_BLINK);
                right_led_state.setState(LED_GREEN, LED_BLINK);
                break;
            }
            case ActionState::LEFTWARD: {
                VINFO_EVERY_N(20) << "LEFTWARD";

                left_led_state.setState(LED_YELLOW, LED_BLINK);
                right_led_state.setState(LED_OFF, LED_BLINK);
                break;
            }
            case ActionState::RIGHTWARD: {
                VINFO_EVERY_N(20) << "RIGHTWARD";

                left_led_state.setState(LED_OFF, LED_BLINK);
                right_led_state.setState(LED_YELLOW, LED_BLINK);
                break;
            }
            case ActionState::RUNNING: {
                VINFO_EVERY_N(20) << "ActionState::RUNNING";

                left_led_state.setState(LED_GREEN, LED_STATIC);
                right_led_state.setState(LED_GREEN, LED_STATIC);
                middle_led_state_extension.setState(LED_GREEN, LED_STATIC);
                middle_led_state_tricolor.setState(LED_GREEN, LED_STATIC);
                middle_led_state_tricolor_two_wire.setState(LED_GREEN, LED_STATIC);
                break;
            }
            default: {
                LOG(ERROR) << "HMIModule UNREACHABLE! action_state_ is " << (int)action_state_;
            }
        }

        next_play_music_id_ = action_music_id_;
        updateLC100StateFun();
        return;
    }

    if (result_state_ != ResultState::NONE) {
        // 根据最新灯语定义修改
        middle_led_state_tricolor.setState(LED_BLUE, LED_STATIC);
        middle_led_state_tricolor_two_wire.setState(LED_YELLOW, LED_STATIC);

        if (result_state_ < ResultState::FAILED) {  // 成功的情况
            middle_led_state_business.setState(LED_GREEN, LED_STATIC);
            if (connect_state_ == ConnectState::DISCONNECTED) {
                left_led_state.setState(LED_BLUE, LED_STATIC);
                middle_led_state_extension.setState(LED_BLUE, LED_STATIC);
                middle_led_state_amr600.setState(LED_BLUE, LED_STATIC);
                right_led_state.setState(LED_BLUE, LED_STATIC);
            } else {
                left_led_state.setState(LED_CYAN, LED_STATIC);
                middle_led_state_extension.setState(LED_CYAN, LED_STATIC);
                middle_led_state_amr600.setState(LED_CYAN, LED_STATIC);
                right_led_state.setState(LED_CYAN, LED_STATIC);
            }
        } else {  // 失败的情况
            left_led_state.setState(LED_RED, LED_BLINK);
            middle_led_state_extension.setState(LED_RED, LED_BLINK);
            middle_led_state_business.setState(LED_RED, LED_BLINK);
            middle_led_state_amr600.setState(LED_RED, LED_BLINK);
            right_led_state.setState(LED_RED, LED_BLINK);
        }
        next_play_music_id_ = error_music_id_;
        updateLC100StateFun();

        return;
    }

    if (user_set_state_ == UserSetState::WAITING_FOR_ELEVATOR) {
        VINFO_EVERY_N(20) << "WAITING_FOR_ELEVATOR";
        left_led_state.setState(LED_CYAN, LED_STATIC);
        middle_led_state_extension.setState(LED_CYAN, LED_STATIC);
        middle_led_state_business.setState(LED_GREEN, LED_STATIC);
        right_led_state.setState(LED_CYAN, LED_STATIC);
        next_play_music_id_ = MUSIC_ID_MUSIC_WAITING_ELEVATOR;
        updateLC100StateFun();
        return;
    }

    if (normal_system_state_ != NormalSystemState::NONE) {
        switch (normal_system_state_) {
            case NormalSystemState::MAP_SWITCHING: {
                VINFO_EVERY_N(20) << "MAP_SWITCHING";

                next_play_music_id_ = MUSIC_ID_MUSIC_SWITCHING_MAP;
                // 根据最新灯语定义修改
                middle_led_state_business.setState(LED_GREEN, LED_STATIC);
                middle_led_state_tricolor.setState(LED_BLUE, LED_STATIC);
                middle_led_state_tricolor_two_wire.setState(LED_YELLOW, LED_STATIC);
                if (connect_state_ == ConnectState::DISCONNECTED) {
                    left_led_state.setState(LED_BLUE, LED_STATIC);
                    middle_led_state_extension.setState(LED_BLUE, LED_STATIC);
                    middle_led_state_amr600.setState(LED_BLUE, LED_STATIC);
                    right_led_state.setState(LED_BLUE, LED_STATIC);
                } else {
                    left_led_state.setState(LED_CYAN, LED_STATIC);
                    middle_led_state_extension.setState(LED_CYAN, LED_STATIC);
                    middle_led_state_amr600.setState(LED_CYAN, LED_STATIC);
                    right_led_state.setState(LED_CYAN, LED_STATIC);
                }
                break;
            }
            default: {
                LOG(ERROR) << "HMIModule UNREACHABLE! normal_system_state_: " << (int)normal_system_state_;

                break;
            }
        }
        updateLC100StateFun();

        return;
    }

    if (battery_state_ != BatteryState::NONE) {
        switch (battery_state_) {
            case BatteryState::LOW_BATTERY: {
                VINFO_EVERY_N(20) << "LOW_BATTERY";

                next_play_music_id_ = MUSIC_ID_MUSIC_LOW_BATTERY;
                left_led_state.setState(LED_YELLOW, LED_BLINK, 250);
                middle_led_state_extension.setState(LED_YELLOW, LED_BLINK, 250);
                middle_led_state_business.setState(LED_RED, LED_STATIC);
                middle_led_state_amr600.setState(LED_YELLOW, LED_BLINK, 250);
                right_led_state.setState(LED_YELLOW, LED_BLINK, 250);
                break;
            }
            default: {
                LOG(ERROR) << "HMIModule UNREACHABLE!";

                break;
            }
        }
        updateLC100StateFun();

        return;
    }

    // 根据最新灯语定义修改
    middle_led_state_tricolor.setState(LED_BLUE, LED_STATIC);
    middle_led_state_tricolor_two_wire.setState(LED_YELLOW, LED_STATIC);
    middle_led_state_business.setState(LED_GREEN, LED_STATIC);
    if (connect_state_ == ConnectState::DISCONNECTED) {
        VINFO_EVERY_N(20) << "DISCONNECTED";

        left_led_state.setState(LED_BLUE, LED_STATIC);
        middle_led_state_extension.setState(LED_BLUE, LED_STATIC);
        middle_led_state_amr600.setState(LED_BLUE, LED_STATIC);
        right_led_state.setState(LED_BLUE, LED_STATIC);
    } else {
        VINFO_EVERY_N(20) << "CONNECTED";

        left_led_state.setState(LED_CYAN, LED_STATIC);
        middle_led_state_extension.setState(LED_CYAN, LED_STATIC);
        middle_led_state_amr600.setState(LED_CYAN, LED_STATIC);
        right_led_state.setState(LED_CYAN, LED_STATIC);
    }

    updateLC100StateFun();
}

bool HMIModule::sendCanMsg(uint32_t id, const std::vector<uint8_t> &data) {
    auto msg = std::make_shared<sros::core::CanDataMsg>("TOPIC_MSG_TO_CAN");
    msg->can_id = id;
    //    msg->data = std::move(data); // NOTE:
    //    此函数以前崩溃过，怀疑是std::move的原因，此处将move改为copy看是否还会报错
    msg->data = data;
    sendMsg(msg);

    return true;
}

void HMIModule::responseCommand(const sros::core::CommandMsg_ptr &msg) {
    auto response_msg = std::make_shared<CommandMsg>(msg->source, "TOPIC_CMD_RESPONSE");
    response_msg->command = msg->command;
    response_msg->session_id = msg->session_id;
    response_msg->req_seq = msg->req_seq;
    response_msg->result_state = msg->result_state;
    response_msg->result_code = msg->result_code;
    LOG(INFO) << "Response command : command = " << response_msg->command
              << " result state = " << response_msg->result_state << " result code = " << response_msg->result_code
              << " seq = " << response_msg->req_seq << " session id = " << response_msg->session_id;
    sendMsg(response_msg);
}

void HMIModule::onNotifyTaskResultMsg(sros::core::base_msg_ptr msg) {
    auto notify_msg = dynamic_pointer_cast<sros::core::NotificationMsg>(msg);
    if (notify_msg->notify_type == sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED) {
        if (notify_msg->movement_task->getTaskResult() == TaskResult::TASK_RESULT_FAILED) {
            action_or_motion_task_result_ = TaskResultSate::AGV_TASK_ERROR;
        } else {
            action_or_motion_task_result_ = TaskResultSate::NONE;
        }
    } else if (notify_msg->notify_type == sros::core::NotificationMsg::NOTIFY_ACTION_TASK_FINISHED) {
        if (notify_msg->action_task->getTaskResult() == TaskResult::TASK_RESULT_FAILED) {
            action_or_motion_task_result_ = TaskResultSate::AGV_TASK_ERROR;
        } else {
            action_or_motion_task_result_ = TaskResultSate::NONE;
        }
    }
}

} /* namespace hmi */
