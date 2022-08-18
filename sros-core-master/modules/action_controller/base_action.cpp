//
// Created by caoyan on 1/9/21.
//

#include "base_action.h"
#include "core/exec_error.hpp"
#include "core/task/task_manager.h"
#include "core/msg/common_poses_info_msg.hpp"


using namespace sros::core;

namespace ac {

BaseAction::BaseAction(int reg_act_id) {
    REG_ACT_ID_ = reg_act_id;
}

BaseAction::~BaseAction() {}

bool BaseAction::initAction() {
    return doInit();
}

void BaseAction::startAction(ActionTask_ptr action_task) {
    if(action_task == nullptr) {
        LOG(ERROR) << "action_task == nullptr, cannot startAction";
        return;
    }

    if(REG_ACT_ID_ != action_task->getActionID()
        && REG_ACT_ID_ != ACTION_ID_COMMON_SRC
        && REG_ACT_ID_ != ACTION_ID_COMMON_EAC) {
        LOG(ERROR) << "REG_ACT_ID_(" << REG_ACT_ID_ << ") != action_id(" << action_task->getActionID() << ")";
        return;
    }

    action_no_ = action_task->getTaskNo();
    action_id_ = action_task->getActionID();
    action_param0_ = action_task->getActionParam();
    action_param1_ = action_task->getActionParam1();
    action_param2_ = action_task->getActionParam2();
    action_param_str_ = action_task->getActionParamStr();

    action_task_ = action_task;

    LOGGER(INFO, ACTION_TASK)   << "startAction(): "
                                << "no " << action_no_ << ", "
                                << "id " << action_id_ << ", "
                                << "p0 " << action_param0_ << ", "
                                << "p1 " << action_param1_ << ", "
                                << "p2 " << action_param2_ << ", "
                                << "pstr " << action_param_str_;

    //start action
    TaskManager::getInstance()->setActionStart();

    doStart();
}

void BaseAction::cancelAction(int result_value) {
    doCancel();

    if(doCancelToInCancel()) {
        TaskManager::getInstance()->setActionInCancel();
    } else if(REG_ACT_ID_ >= 0 && REG_ACT_ID_ <= 0x7F){
        src_sdk->cancelAction(action_no_);
        TaskManager::getInstance()->setActionFinishCanceled(result_value);
    } else if(REG_ACT_ID_ >= 0xC0 && REG_ACT_ID_ <= 0xFF) {
        cancelEacActionTask(action_no_);
        //TaskManager::getInstance()->setActionFinishCanceled(result_value);
    } else {
        TaskManager::getInstance()->setActionFinishCanceled(result_value);
    }
}

void BaseAction::doActionFinishSucceed(int result_value) {
    TaskManager::getInstance()->setActionFinishSucceed(result_value);
}

void BaseAction::doActionFinishFailed(int result_value) {
    SET_ACTION_EXEC_FAILED(result_value, "action failed");
}

void BaseAction::doActionFinishCanceled(int result_value) {
    TaskManager::getInstance()->setActionFinishCanceled(result_value);
}


void BaseAction::onTimer50ms(uint64_t cur_time) {

    if (action_task_ == nullptr) {
        return;
    }

    if(action_task_->isInCancel()) {
        LOG(INFO) << "action incancel!";
        doInCancel();
        TaskManager::getInstance()->setActionFinishCanceled();
    }

    if(action_task_->isSlaveRunning()) {
        doTimerEvent(cur_time);
    }
}

//功能函数
void BaseAction::sendEacActionTask() {

    sendEacActionTask(action_no_, action_id_, action_param0_, action_param1_);
}

void BaseAction::sendEacActionTask(int action_no, int action_id, int param0, int param1) {
    static std::string msg_source = "ActionController";

    auto new_task = std::make_shared<sros::core::ActionTask>(action_no, msg_source, action_id, param0, param1);

    auto mm = make_shared<sros::core::CommandMsg>(msg_source, "EAC_ACTION_CMD");
    mm->command = sros::core::CMD_NEW_ACTION_TASK;
    mm->param0 = action_no;
    mm->action_task = new_task;

    sros::core::MsgBus::sendMsg(mm);
}

void BaseAction::cancelEacActionTask(int action_no) {
    static std::string msg_source = "ActionController";

    auto mm = make_shared<sros::core::CommandMsg>(msg_source, "EAC_ACTION_CMD");
    mm->command = sros::core::CMD_CANCEL_ACTION_TASK;
    mm->param0 = action_no;

    sros::core::MsgBus::sendMsg(mm);
}

void BaseAction::sendMoveTask(const sros::core::NavigationPath_vector& dst_paths) {
    sros::core::MovementTask_ptr new_task;

    sros::core::NavigationPathi_vector dst_paths_int;
    pathDoubleToInt(dst_paths, dst_paths_int);

    static std::string msg_source = "ActionController";

    new_task = std::make_shared<sros::core::MovementTask>(action_no_, msg_source, dst_paths_int);

    new_task->setTaskSeq(action_no_);

    auto mm = make_shared<sros::core::CommandMsg>(msg_source);
    mm->req_seq = action_no_;
    mm->command = sros::core::CMD_NEW_MOVEMENT_TASK;
    mm->movement_task = new_task;
    mm->param0 = 0;

    sros::core::MsgBus::sendMsg(mm);
}


void BaseAction::sendReplaceTask(const sros::core::NavigationPath_vector& dst_paths) {

    sros::core::NavigationPathi_vector dst_paths_int;
    pathDoubleToInt(dst_paths, dst_paths_int);

    static std::string msg_source = "ActionController";

    auto mm = make_shared<sros::core::CommandMsg>(msg_source);
    mm->req_seq = action_no_;
    mm->command = sros::core::CMD_PATH_REPLACE;
    mm->paths = dst_paths_int;
    mm->param0 = 0;

    sros::core::MsgBus::sendMsg(mm);
}

void BaseAction::cancelMoveTask() {
    static std::string msg_source = "ActionController";
    auto mm = make_shared<sros::core::CommandMsg>(msg_source);
    mm->command = CMD_CANCEL_MOVEMENT_TASK;
    sros::core::MsgBus::sendMsg(mm);
}

bool BaseAction::checkCurPoseArriveDestStation(sros::map::StationMark& dst_station) {
    auto& s = sros::core::Settings::getInstance();
    auto point_threshold = s.getValue<double>("main.movement_task_arrive_point_check_threshold", 100) / 1000;
    auto angle_threshold = s.getValue<double>("main.movement_task_arrive_angle_check_threshold", 50) / 10.0 * DEGREE_TO_RAD;  // 弧度

    auto cur_pose = src_sdk->getCurPose();
    LOG(INFO) << "Station " << dst_station.id << "'s pose is " << dst_station.pos;

    auto distance = get2PointDistance(dst_station.pos.x / 100, dst_station.pos.y / 100, cur_pose.x(), cur_pose.y());
    if (distance > point_threshold) {
        LOG(INFO) << "agv not arrive at station " << dst_station.id
                  << ", distance offset: " << distance << ", threshold: " << point_threshold;
        return false;
    } else {
        auto offset = std::abs(normalizeYaw(cur_pose.yaw() - dst_station.pos.yaw));
        if (offset > angle_threshold) {
            LOG(INFO) << "agv not arrive at station " << dst_station.id
                      << ", angle offset: " << offset << ", threshold: " << angle_threshold;
            return false;
        }

        LOG(INFO) << "agv arrive at station " << dst_station.id << ", offset: " << distance
                  << ", threshold: " << point_threshold;

        return true;
    }

}

bool BaseAction::isEnableEac() {
    auto& s = sros::core::Settings::getInstance();
    bool is_enable_eac = (s.getValue<std::string>("device.enable_eac", "False") == "True");
    return is_enable_eac;
}

bool BaseAction::isForkControlTypeSrc() {
    auto& s = sros::core::Settings::getInstance();
    bool is_fork_control_type_src = (s.getValue<string>("forklift.fork_control_type", "SRC") == "SRC");
    return is_fork_control_type_src;
}

bool BaseAction::checkPalletInPlaceSignal() {
    if(g_state.load_state == sros::core::LOAD_FULL) {
        return true;
    } else {
        return false;
    }
}

void BaseAction::sendPerceptionCmd(uint32_t seq, PerceptionCommandMsg::Command detect_type, int goal_id, const Pose& pose) {
        auto mm = std::make_shared<sros::core::PerceptionCommandMsg>("DETECT_COMMAND");
        mm->seq = seq;
        mm->command = detect_type;
        mm->goal_id = goal_id;
        mm->theory_pose.x() = pose.x();
        mm->theory_pose.y() = pose.y();
        mm->theory_pose.yaw() = pose.yaw();

        LOG(INFO) << "sendDetect cmd => " << detect_type.detect_stage*10+detect_type.object_type
                  << ", goal_id" << goal_id << ", detect location coordinate x:" << mm->theory_pose.x()
                  << ", y:" << mm->theory_pose.y() << ", yaw:" << mm->theory_pose.yaw();

        sros::core::MsgBus::sendMsg(mm);
}

void BaseAction::sendPerceptionCmd(uint32_t seq,
                       PerceptionCommandMsg::Command detect_type,
                       int goal_id,
                       const Pose& pose,
                       int obj_self_height,
                       int obj_center_ground_clearance,
                       int current_camera_height) {

    auto mm = std::make_shared<sros::core::PerceptionCommandMsg>("DETECT_COMMAND");
    mm->seq = seq;
    mm->command = detect_type;
    mm->goal_id = goal_id;
    mm->theory_pose.x() = pose.x();
    mm->theory_pose.y() = pose.y();
    mm->theory_pose.yaw() = pose.yaw();
    mm->obj_center_ground_clearance = obj_center_ground_clearance;
    mm->obj_self_height = obj_self_height;
    mm->current_camera_height = current_camera_height;

    LOG(INFO) << "sendDetect cmd => " << detect_type.detect_stage*10+detect_type.object_type
              << ", goal_id" << goal_id << ", detect location coordinate x:" << mm->theory_pose.x()
              << ", y:" << mm->theory_pose.y() << ", yaw:" << mm->theory_pose.yaw()
              << ", obj_center_ground_clearance" << mm->obj_center_ground_clearance
              << ", obj_self_height" << mm->obj_self_height
              << ", current_camera_height" << mm->current_camera_height;


    sros::core::MsgBus::sendMsg(mm);
}

void BaseAction::sendPostureCorrectCmd(uint32_t seq,
                                       PostureCorrectCommandMsg::EnumCorrectCmd correct_cmd,
                                       const std::string sensor_name,
                                       PostureCorrectCommandMsg::EnumCorrectDir correct_dir,
                                       bool first_send_cmd) {
    auto mm = std::make_shared<sros::core::PostureCorrectCommandMsg>("TOPIC_POSTURE_CORRECT_CMD");
    mm->seq = seq;
    mm->command.correct_cmd = correct_cmd;
    mm->command.sensor_name = sensor_name;
    mm->command.correct_dir = correct_dir;
    mm->command.first_send_cmd = first_send_cmd;

    LOG(INFO) << "sendPostureCorrectCmd => correct_cmd: " << correct_cmd
              << ", sensor_name: " << sensor_name << ", correct_dir: " << correct_dir;

    sros::core::MsgBus::sendMsg(mm);
}


void BaseAction::enableSVC100Camera(bool enable, const std::string &which_camera) {
    LOG(INFO) << "enableSVC100Camera() => " << enable << " " << which_camera;

    auto msg = std::make_shared<sros::core::CommonMsg>("TOPIC_SVC100_ENABLE_PUBLISH");
    msg->flag = enable;
    msg->str_0_ = which_camera;
    sros::core::MsgBus::sendMsg(msg);
}

void BaseAction::enableBackLidar(bool enable) {
    LOG(INFO) << "enableBackLidar() => " << enable;
    g_state.enable_back_main_laser_oba = enable;
    // auto msg = std::make_shared<sros::core::CommonMsg>("TOPIC_BACK_LASER_ENABLE_PUBLISH");
    // msg->flag = enable;
    // msg->str_0_ = sros::device::DEVICE_LIDAR;
    // sros::core::MsgBus::sendMsg(msg);

    // auto msg2 = std::make_shared<sros::core::CommonMsg>("TOPIC_BACK_LASER_ENABLE_PUBLISH");
    // msg2->flag = enable;
    // msg2->str_0_ = sros::device::DEVICE_UST_LIDAR_BACK;
    // sros::core::MsgBus::sendMsg(msg2);
}

void BaseAction::sendCommonPosesInfo(const Location_Vector& loc_vec, std::string sensor_name) {
    Polygons polygons;

    Polygon polygon;
    for (const auto &location : loc_vec) {
        polygon.emplace_back(Geometry_Point(location.x() * 1000, location.y() * 1000));
    }

    polygons.push_back(polygon);

    auto common_poses_info_msg = make_shared<sros::core::CommonPosesInfoMsg>("TOPIC_TIMER_SEND_COMMON_POSES_INFO");
    common_poses_info_msg->type = CommonPosesInfoMsg::Type::NONE;
    common_poses_info_msg->car_simulate_poses = polygons;
    common_poses_info_msg->sensor_name = sensor_name;
    LOG(INFO)<<"sendCommonPosesInfo sensor_name: " << sensor_name << ", posesize:" << polygon.size();
    sros::core::MsgBus::sendMsg(common_poses_info_msg);

    //
    g_state.is_send_forklift_common_poses = true;
}

void BaseAction::sendAvdObaCmd(uint32_t seq,  AvdObaCommandMsg::Command command) {
        auto mm = std::make_shared<sros::core::AvdObaCommandMsg>("GULF_GOODS_AVDOBA_CMD");
        mm->seq = seq;
        mm->command = command;

        LOG(INFO) << "send AvdOba cmd => " << command.avdoba_state;

        sros::core::MsgBus::sendMsg(mm);
}


bool BaseAction::checkForkHeightFautl(float dst_fork_height) {

    auto& s = sros::core::Settings::getInstance();
    
    bool enable_detect_fork_height = (s.getValue<std::string>("forklift.enable_detect_fork_height", "False") == "True");
    if (!enable_detect_fork_height) {
        return false;
    }

    double deviate_fork_height = s.getValue<double>("forklift.deviate_fork_height", 0.03);
    // src_sdk->getForkHeightEncoder();

    auto deviate_value = std::fabs(g_state.fork_height_encoder - dst_fork_height);

    LOG(INFO) << "g_state.fork_height_encoder: " << g_state.fork_height_encoder 
                  << ", dst_fork_height: " << dst_fork_height 
                  << ", deviate_fork_height: " << deviate_fork_height; 

    if (deviate_value > deviate_fork_height) {
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_HEIGHT_DEVIATE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_HEIGHT_DEVIATE);

        return true;
    } 

    return false;
}


}
