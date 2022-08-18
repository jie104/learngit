//
// Created by caoyan on 7/1/21.
//

#include "action_135.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "core/fault_center.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action135::doStart() {
    auto& s = sros::core::Settings::getInstance();
    auto bolt_relative_agv_x_offset = s.getValue<float>("perception.bolt_relative_agv_x_offset", -600); //mm

    // 载货模型变更使能
    enable_change_avd_model = (s.getValue<std::string>("obstacle.enable_change_avd_model", "false") == "True");

    tractor_docking_deviate_distance_ = bolt_relative_agv_x_offset / 1000.0; //m

    //清空货物的识别坐标
    Pose pose;
    g_state.goods_pose = pose;

    if( action_param0_ == 0x01){
        // 1.通过目标站点的id去获取目标点的位姿
        auto dst_station_id = action_param1_;
        auto dst_station = MapManager::getInstance()->getStation(dst_station_id);

        if (dst_station_id != 0 && dst_station.id == 0) {
            LOG(ERROR) << "not exist station id: " << dst_station_id;
            doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
            return;
        }
        
        dst_pose_.x() = dst_station.pos.x / 100.0;
        dst_pose_.y() = dst_station.pos.y / 100.0;
        dst_pose_.yaw() = normalizeYaw(dst_station.pos.yaw);
        LOG(INFO) << "dst_pose, [x: " << dst_pose_.x() << "][y: " << dst_pose_.y() << "][yaw: " << dst_pose_.yaw() << "]";

        
        //脱钩后马上执行挂钩，挂钩会改变DO5，然后gpio_output刷新慢，src改变了，我使用了一个旧值覆盖
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        bits_ = 7;
        // uint16_t ori_output_value = g_state.gpio_output;  // 保存当前gpio output值
        // uint16_t new_output_value = setBit(ori_output_value, bits_);

        // LOG(INFO) << "ori_output_value = " << std::hex << ori_output_value;
        // LOG(INFO) << "bits_mask = " << bits_;
        // LOG(INFO) << "new_output_value = " << std::hex << new_output_value;

        // src_sdk->setGPIOOuput(new_output_value);
        src_sdk->setGPIOOuputBits(0, (uint8_t)(1 << bits_));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));    //打光后延迟一下下
    }
    else{
        src_sdk->executeAction(action_no_, 26, 1, 0);
        LOG(INFO) << "src_ac: no " << action_no_ << ", id 26, p0 1, p1 0";
    }

    enableAlgTractorDocking(true);
    is_src_tractor_docking_running_ = true;
}

void Action135::doCancel() {
    if(is_src_tractor_docking_running_) {
        is_src_tractor_docking_running_ = false;
        enableAlgTractorDocking(false);
        src_sdk->cancelAction(action_no_);
    }
}

void Action135::onSrcAcFinishSucceed(int result_value) {
    is_src_tractor_docking_running_ = false;
    enableAlgTractorDocking(false);
    doActionFinishSucceed();

    // 只有對接成功才下发更改避障模型指令
    if(enable_change_avd_model){
        sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType::CHANGE_MDOEL_AVD, code_str, dst_pose_); 
    }

}

void Action135::onSrcAcFinishFailed(int result_value) {
    LOG(ERROR) << "action failed, result_value:"<<result_value;
    is_src_tractor_docking_running_ = false;
    enableAlgTractorDocking(false);
    doActionFinishFailed(sros::core::ERROR_CODE_TRACTOR_DOCKING_FAILED);
}

void Action135::onSrcAcFinishCanceled(int result_value) {
    is_src_tractor_docking_running_ = false;
    enableAlgTractorDocking(false);
    doActionFinishFailed(sros::core::ERROR_CODE_TRACTOR_DOCKING_FAILED);
}

void Action135::enableAlgTractorDocking(bool enable) {

    PerceptionCommandMsg::Command cmd;
    cmd.detect_stage = PerceptionCommandMsg::DetectStage::DETECT_STAGE_LOAD;
    cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_QRCODE;
    if(action_param0_ == 0x01){
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_QRCODE_GLOBAL;
    }else{
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_QRCODE;
    }
    cmd.is_enable = enable;

    sendPerceptionCmd(action_no_, cmd, g_state.cur_goal_id, dst_pose_);
}

void Action135::sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type, const std::string& code_str, Pose& pose) {
    AvdObaCommandMsg::Command command;
    command.oba_model.is_change_oba_model = true;
    command.oba_model.forklif_type = AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_3000;
    command.oba_model.restore_oba_model = false;
    command.oba_model.QR_Code_id = code_str;
    command.avdoba_type = avdoba_type;
    command.goal_position = pose;

    sendAvdObaCmd(action_no_, command);
}

void Action135::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {
    // uint16_t new_output_value = resetBit(g_state.gpio_output, bits_);
    // src_sdk->setGPIOOuput(new_output_value);
    src_sdk->setGPIOOuputBits((uint8_t)(1 << bits_), 0);

    if (!is_src_tractor_docking_running_) {
        LOGGER(INFO, ACTION_TASK) << "is_src_tractor_docking_running_ = false ";
        
        return;
    }

    auto mm = dynamic_pointer_cast<PerceptionStateMsg>(msg);

    int detect_result = mm->detect_result;

    bool result = (detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS);
    LOG(INFO) <<"result:"<<result;

    //拿到结果之后
    if (!result) {
        //未检测到货物
        doCancel();
        saveCameraImg(true);
        doActionFinishFailed(PerceptionStateMsg::errCodeConvert(mm->error_code));
        return;
    }

    double x = mm->goal_in_global_pose.x();
    double y = mm->goal_in_global_pose.y();
    double yaw = mm->goal_in_global_pose.yaw();
    code_str = mm->code_str_;

    LOG(INFO) <<"Action_135 Send location(x,y,yaw):"<<x<<", "<<y<<", "<< yaw ;

    g_state.goods_pose = mm->goal_in_agv_pose;

    if(action_param0_ == 0x01){
        if(src_sdk->setTractorDockingDMCodeOffset(x, y, yaw, 0)){
            src_sdk->executeAction(action_no_, 27, 1, 0);
            LOG(INFO) << "src_ac: no " << action_no_ << ", id 27, p0 1, p1 0";  
        }
    }else{
        src_sdk->setTractorDockingDMCodeOffset(x, y, yaw, tractor_docking_deviate_distance_);
    }
}

uint16_t Action135::setBit(uint16_t val, int bits) {
    uint16_t tmp = 0x1 << bits;
    return (tmp | val);           
};

uint16_t Action135::resetBit(uint16_t val, int bits) {
    uint16_t tmp = ~(0x1 << bits);
    return val & tmp;          
};

void Action135::saveCameraImg(bool flag) {
    // LOG(INFO) << "saveCameraImg() => " << flag;
    auto msg = std::make_shared<sros::core::CommonMsg>("SAVE_FAILURE_IMG");
    msg->flag = flag; //false：(sucess 不存图) true:(fail 保存IMG)
    sros::core::MsgBus::sendMsg(msg);
}

}
