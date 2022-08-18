//
// Created by lfc on 19-5-28.
//

#ifndef SROS_EU100_TIM320_SENSOR_MANAGER_HPP
#define SROS_EU100_TIM320_SENSOR_MANAGER_HPP

#include <core/msg/common_command_msg.hpp>
#include "base_sensor_manager.hpp"
#include "core/device/can_interface.h"
#include "core/hardware/EU100.h"
#include "functional"

namespace sensor {
class Eu100Tim320SensorManager : public BaseSensorManager {
 public:
    Eu100Tim320SensorManager(oba::ObstacleModulePara_Ptr para, ObaMsgCallback msg)
        : BaseSensorManager(para, msg, TYPE_EU100_TIM320_SENSOR) {
        oba_name = "TIM320_OBA";
        oba_state_ = (sros::core::ObstacleMsg::ObaState)(para->eu100_tim320_stop_state);
        tim320_region_id_.resize(4);
        tim320_err_count_.resize(4);
        tim320_region_id_[0] = 0;
        if (para->enable_left_tim320) {
            eu100_ptr_agv_left_ = sros::device::createDevice<sros::device::EU100>(
                sros::device::DEVICE_EU100_TIM312_1, sros::device::DEVICE_ID_EU100_TIM312_1,
                sros::device::DEVICE_COMM_INTERFACE_TYPE_CAN_1,
                std::make_shared<sros::device::CanInterface>(para->left_tim320_id));
            eu100_ptr_agv_left_->setIODataCallback(std::bind(&Eu100Tim320SensorManager::onDataRecieve, this, 1,
                                                             std::placeholders::_1, std::placeholders::_2));
            tim320_region_id_[1] = 0;
            tim320_err_count_[1] = 0;
        }
        if (para->enable_right_tim320) {
            eu100_ptr_agv_right_ = sros::device::createDevice<sros::device::EU100>(
                sros::device::DEVICE_EU100_TIM312_2, sros::device::DEVICE_ID_EU100_TIM312_2,
                sros::device::DEVICE_COMM_INTERFACE_TYPE_CAN_1,
                std::make_shared<sros::device::CanInterface>(para->right_tim320_id));
            eu100_ptr_agv_right_->setIODataCallback(std::bind(&Eu100Tim320SensorManager::onDataRecieve, this, 2,
                                                              std::placeholders::_1, std::placeholders::_2));
            tim320_region_id_[2] = 0;
            tim320_err_count_[2] = 0;
        }
        if (para->enable_back_tim320) {
            tim320_region_id_[3] = 0;
            eu100_ptr_agv_back_ = sros::device::createDevice<sros::device::EU100>(
                sros::device::DEVICE_EU100_TIM312_BACK, sros::device::DEVICE_ID_EU100_TIM312_3,
                sros::device::DEVICE_COMM_INTERFACE_TYPE_CAN_1,
                std::make_shared<sros::device::CanInterface>(para->back_tim320_id));
            eu100_ptr_agv_back_->setIODataCallback(std::bind(&Eu100Tim320SensorManager::onDataRecieve, this, 3,
                                                             std::placeholders::_1, std::placeholders::_2));
            tim320_region_id_[3] = 0;
            tim320_err_count_[3] = 0;
        }

        LOG(INFO) << "successfully to initialize TIM320 module!";
    }

    /*
    virtual void processMsg(sros::core::base_msg_ptr& m) {
        if (para->enable_switch_region_avoid_obstacle) {
            std::shared_ptr<sros::core::CommonCommandMsg<std::string>> region_cmd_msg =
                std::dynamic_pointer_cast<sros::core::CommonCommandMsg<std::string>>(m);
            const auto& msg_cmd = region_cmd_msg->command;
            std::string cmd;
            if (msg_cmd == "LINE FORWARD") {
                if (region_cmd_msg->param0 == 1) {
                    cmd = "GO STRAIGHT FULL";
                } else {
                    cmd = "GO STRAIGHT EMPTY";
                }
            } else if (msg_cmd == "LINE BACKWARD") {
                if (region_cmd_msg->param0 == 1) {
                    cmd = "GO BACK FULL";
                } else {
                    cmd = "GO BACK EMPTY";
                }
            } else if (msg_cmd == "TURN RIGHT FORWARD") {
                cmd = msg_cmd;
            } else if (msg_cmd == "TURN LEFT FORWARD") {
                cmd = msg_cmd;
            } else if (msg_cmd == "TURN RIGHT BACKWARD") {
                cmd = msg_cmd;
            } else if (msg_cmd == "TURN LEFT BACKWARD") {
                cmd = msg_cmd;
            } else if (msg_cmd == "ROTATE RIGHT") {
                cmd = "ROTATE RIGHT";
            } else if (msg_cmd == "ROTATE LEFT") {
                cmd = "ROTATE LEFT";
            } else if (msg_cmd == "DEFAULT") {
                cmd = "DEFAULT";
            }
            if (region_cmd_msg->param1 == 1) {
                LOG(INFO) << "进叉";
                cmd = "ENTER FORK";
            } else if (region_cmd_msg->param1 == 2) {
                LOG(INFO) << "退叉";
                cmd = "BACK FORK";
            }
            if (para->enable_photoelectric_switch_avoid_obstacle) {
                processPSObstacles(cmd, region_cmd_msg);
            }

            if (last_cmd_ == cmd && (!need_second_set_region_)) {
                return;
            } else {
                last_cmd_ = cmd;
            }
            if (need_second_set_region_) {
                need_second_set_region_ = false;
            }
            LOG(INFO) << "cmd:" << cmd << "," << last_cmd_;
            const uint8_t mask = 0xff;
            int8_t left_value = 0x01;
            int8_t right_value = 0x01;
            int8_t back_value = 0x01;
            if (cmd == "GO STRAIGHT FULL") {  //直线空载前进
                left_value = 0x01;
                right_value = 0x01;
                back_value = 0x01;
            } else if (cmd == "GO STRAIGHT EMPTY") {  //直线载货前进
                left_value = 0x02;
                right_value = 0x02;
                back_value = 0x02;
            } else if (cmd == "GO BACK FULL") {  //直线载货后退
                left_value = 0x05;
                right_value = 0x05;
                back_value = 0x03;
            } else if (cmd == "GO BACK EMPTY") {  //直线空载后退
                left_value = 0x06;
                right_value = 0x06;
                back_value = 0x04;
            } else if (cmd == "TURN RIGHT FORWARD") {  //右转前进
                left_value = 0x03;
                right_value = 0x03;
                back_value = 0x01;
            } else if (cmd == "TURN LEFT FORWARD") {  //左转前进
                left_value = 0x04;
                right_value = 0x04;
                back_value = 0x01;
            } else if (cmd == "TURN RIGHT BACKWARD") {  //右转后退
                left_value = 0x05;
                right_value = 0x05;
                back_value = 0x05;
            } else if (cmd == "TURN LEFT BACKWARD") {  //左转后退
                left_value = 0x05;
                right_value = 0x06;
                back_value = 0x05;
            } else if (cmd == "ROTATE LEFT") {
                left_value = 0x07;
                right_value = 0x07;
                back_value = 0x08;
            } else if (cmd == "ROTATE RIGHT") {
                left_value = 0x07;
                right_value = 0x07;
                back_value = 0x08;
            } else if (cmd == "LIFT FORK") {  //升叉

            } else if (cmd == "DROP FORK") {  //降叉

            } else if (cmd == "ENTER FORK") {  //进叉
                left_value = 0x06;             //进叉按照直线空载后退处理
                right_value = 0x06;
                back_value = 0x07;
            } else if (cmd == "BACK FORK") {  //退叉
                left_value = 0x02;            //退叉按照直线空载前进处理
                right_value = 0x02;
                back_value = 0x07;
            } else if (cmd == "DEFAULT") {
                left_value = 0x08;  //退叉按照直线空载前进处理
                right_value = 0x08;
                back_value = 0x07;
            } else {
                LOG(INFO) << "err cmd:" << cmd;
            }
            left_value--;
            right_value--;
            back_value--;
            if(eu100_ptr_agv_left_){
                eu100_ptr_agv_left_->setIO(mask, left_value);
            }
            if(eu100_ptr_agv_right_){
                eu100_ptr_agv_right_->setIO(mask, right_value);
            }
            if(eu100_ptr_agv_back_){
                eu100_ptr_agv_back_->setIO(mask, back_value);
            }
            tim320_region_id_[1] = left_value;
            tim320_region_id_[2] = right_value;
            tim320_region_id_[3] = back_value;
        }
    }
    */

    virtual void processMsg(sros::core::base_msg_ptr& m) {
        if (para->enable_switch_region_avoid_obstacle) {
            std::shared_ptr<sros::core::CommonCommandMsg<std::string>> region_cmd_msg =
                std::dynamic_pointer_cast<sros::core::CommonCommandMsg<std::string>>(m);
            const auto& msg_cmd = region_cmd_msg->command;
            std::string cmd;
            if (msg_cmd == "LINE FORWARD") {
                if (region_cmd_msg->param0 == 1) {
                    cmd = "GO STRAIGHT FULL";
                } else {
                    cmd = "GO STRAIGHT EMPTY";
                }
            } else if (msg_cmd == "LINE BACKWARD") {
                if (region_cmd_msg->param0 == 1) {
                    cmd = "GO BACK FULL";
                } else {
                    cmd = "GO BACK EMPTY";
                }
            } else if (msg_cmd == "TURN RIGHT FORWARD") {
                cmd = msg_cmd;
            } else if (msg_cmd == "TURN LEFT FORWARD") {
                cmd = msg_cmd;
            } else if (msg_cmd == "TURN RIGHT BACKWARD") {
                cmd = msg_cmd;
            } else if (msg_cmd == "TURN LEFT BACKWARD") {
                cmd = msg_cmd;
            } else if (msg_cmd == "ROTATE RIGHT") {
                cmd = "ROTATE RIGHT";
            } else if (msg_cmd == "ROTATE LEFT") {
                cmd = "ROTATE LEFT";
            } else if (msg_cmd == "DEFAULT") {
                cmd = "DEFAULT";
            }
            if (region_cmd_msg->param1 == 1) {
                LOG(INFO) << "进叉";
                cmd = "ENTER FORK";
            } else if (region_cmd_msg->param1 == 2) {
                LOG(INFO) << "退叉";
                cmd = "BACK FORK";
            }

            if (para->enable_photoelectric_switch_avoid_obstacle) {
                processPSObstacles(cmd, region_cmd_msg);
            }

            if (last_cmd_ == cmd && (!need_second_set_region_)) {
                return;
            } else {
                last_cmd_ = cmd;
            }
            if (need_second_set_region_) {
                need_second_set_region_ = false;
            }
            LOG(INFO) << "cmd:" << cmd << " last_cmd:" << last_cmd_;
            const uint8_t mask = 0xff;
            int8_t forward_down_value = 0x01;   // 对应雷达1
            int8_t forward_up_value = 0x01;     // 对应雷达2
            int8_t back_value = 0x01;           // 对应雷达3

            if (cmd == "GO STRAIGHT FULL") {  //直线空载前进
                forward_down_value = 0x01;
                forward_up_value = 0x01;
                back_value = 0x01;
                LOG(INFO) << "直线空载前进 1 1 1";
            } else if (cmd == "GO STRAIGHT EMPTY") {  //直线载货前进
                forward_down_value = 0x01;
                forward_up_value = 0x01;
                back_value = 0x01;
                LOG(INFO) << "直线空载前进, 1 1 1";
            } else if (cmd == "GO BACK FULL") {  //直线载货后退
                forward_down_value = 0x02;
                forward_up_value = 0x02;
                back_value = 0x02;
                LOG(INFO) << "直线载货后退 2 2 2";
            } else if (cmd == "GO BACK EMPTY") {  //直线空载后退
                forward_down_value = 0x02;
                forward_up_value = 0x02;
                back_value = 0x02;
                LOG(INFO) << "直线空载后退 2 2 2";
            } else if (cmd == "TURN RIGHT FORWARD") {  //右转前进
                forward_down_value = 0x04;
                forward_up_value = 0x04;
                back_value = 0x01;
                LOG(INFO) << "右转前进 4 4 1";
            } else if (cmd == "TURN LEFT FORWARD") {  //左转前进
                forward_down_value = 0x03;
                forward_up_value = 0x03;
                back_value = 0x01;
                LOG(INFO) << "左转前进 3 3 1";
            } else if (cmd == "TURN RIGHT BACKWARD") {  //右转后退
                forward_down_value = 0x02;
                forward_up_value = 0x02;
                back_value = 0x04;
                LOG(INFO) << "右转后退 2 2 4";
            } else if (cmd == "TURN LEFT BACKWARD") {  //左转后退
                forward_down_value = 0x02;
                forward_up_value = 0x02;
                back_value = 0x03;
                LOG(INFO) << "左转后退 2 2 3";
            } else if (cmd == "ROTATE LEFT") {  //原地左转
                forward_down_value = 0x05;
                forward_up_value = 0x05;
                back_value = 0x05;
                LOG(INFO) << "原地左转 5 5 5";
            } else if (cmd == "ROTATE RIGHT") {  //原地右转
                forward_down_value = 0x05;
                forward_up_value = 0x05;
                back_value = 0x05;
                LOG(INFO) << "原地右转 5 5 5";
            } else {
                LOG(INFO) << "err cmd:" << cmd;
            }
            forward_down_value--;
            forward_up_value--;
            back_value--;

            if(eu100_ptr_agv_left_){
                eu100_ptr_agv_left_->setIO(mask, forward_down_value);
            }
            if(eu100_ptr_agv_right_){
                eu100_ptr_agv_right_->setIO(mask, forward_up_value);
            }
            if(eu100_ptr_agv_back_){
                eu100_ptr_agv_back_->setIO(mask, back_value);
            }
            tim320_region_id_[1] = forward_down_value;
            tim320_region_id_[2] = forward_up_value;
            tim320_region_id_[3] = back_value;
            LOG(INFO) << "forward_up_value=" << static_cast<int>(forward_up_value)
                      << " forward_down_value=" << static_cast<int>(forward_down_value)
                      << " back_value=" << static_cast<int>(back_value);
        }
    }

 private:
    void processPSObstacles(const std::string cmd,
                            const std::shared_ptr<sros::core::CommonCommandMsg<std::string>> msg) {
        sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("OBSTACLES"));
        oba_msg->oba_name = "PS_OBA";
        oba_msg->time_ = sros::core::util::get_time_in_us();
        oba_msg->is_region_oba = true;
        oba_msg->oba_state = sros::core::ObstacleMsg::STATE_OBA_FREE;
        if ((cmd == "ENTER FORK") || (cmd == "TURN RIGHT BACKWARD") || (cmd == "TURN LEFT BACKWARD") ||
            (cmd == "GO BACK FULL") || (cmd == "GO BACK EMPTY")) {
            bool left_on = false;
            bool right_on = false;
            if (msg->str0 == "PS LEFT ON") {
                left_on = true;
            }
            if (msg->str1 == "PS RIGHT ON") {
                right_on = true;
            }
            if (left_on | right_on) {
                oba_msg->oba_state = oba_state_;
            }
        }
        if (sendObaMsg) {
            sendObaMsg(oba_msg);
        }
    }
    // eu100_id: EU100编号
    // io_in: EU100输入给程序的区域io信号.
    // io_out: EU100输入给Tim320设备的通道信号.
    // tim320_region_id_[eu100_id]: 程序下发给EU100_ID的通道信号.
    void onDataRecieve(int eu100_id, uint8_t io_in, uint8_t io_out) {
//        LOG(INFO) << "id:" << eu100_id << " io_in:" << (int)io_in << " io_out:" << (int)io_out
//                  << "," << oba_name + std::to_string(eu100_id);
        if ((tim320_region_id_[eu100_id] != io_out) && (tim320_region_id_[eu100_id] != 0)) {
            LOG(INFO) << "eu100_id region switch is wrong!:id:" << eu100_id << " tim320_region_id_[eu100_id]:"
                      <<static_cast<int>(tim320_region_id_[eu100_id]) <<", io_out:" << static_cast<int>(io_out)
                      <<", io_in=" << static_cast<int>(io_in);
            need_second_set_region_ = true;  //不能在回调函数里设置tim320的IO
        }
        if (io_in & bit4_mask) {
            uint8_t bit2_state = io_in & bit2_mask;  //外层区域
            uint8_t bit1_state = io_in & bit1_mask;  //内层区域

            sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("OBSTACLES"));
            oba_msg->oba_name = oba_name + std::to_string(eu100_id);
            oba_msg->time_ = sros::core::util::get_time_in_us();
            oba_msg->is_region_oba = true;
            oba_msg->oba_state = sros::core::ObstacleMsg::STATE_OBA_FREE;
            if (!bit2_state) {  // off状态
                oba_msg->oba_state = sros::core::ObstacleMsg::STATE_OBA_SLOW;
                //                LOG(INFO) << "slow!" << eu100_id << "," << (int)io_in;
                if (!bit1_state) {
                    oba_msg->oba_state = oba_state_;
                    //LOG(INFO) << "stop! tim320 id:" << eu100_id << "," <<
                    //(int)oba_state_ << ",io in:" << (int)io_in;
                } else {
                    //LOG(INFO) << "slow! tim320 id:" << eu100_id << "," << (int)oba_state_ << ",io
                    //in:" << (int)io_in;
                }
            }
            if (sendObaMsg) {
                sendObaMsg(oba_msg);
            }
            tim320_err_count_[eu100_id] = 0;
        }else{
            tim320_err_count_[eu100_id]++;
            if (tim320_err_count_[eu100_id] >= 3) {
                LOG(INFO) << "tim 320 err:" << eu100_id << ","
                          << "will stop car!";
                sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("OBSTACLES"));
                oba_msg->oba_name = oba_name + std::to_string(eu100_id);
                oba_msg->time_ = sros::core::util::get_time_in_us();
                oba_msg->is_region_oba = true;
                oba_msg->oba_state = oba_state_;
                if (sendObaMsg) {
                    sendObaMsg(oba_msg);
                }
            }
        }
    }

    std::string oba_name;
    sros::device::EU100_ptr eu100_ptr_agv_left_;
    sros::device::EU100_ptr eu100_ptr_agv_right_;
    sros::device::EU100_ptr eu100_ptr_agv_back_;
    sros::core::ObstacleMsg::ObaState oba_state_;
    std::string last_cmd_;

    std::vector<uint8_t> tim320_region_id_;
    std::vector<int> tim320_err_count_;
    bool need_second_set_region_ = false;
    const int can_id_1 = 0x331;
    const int can_id_2 = 0x332;

    const uint8_t bit4_mask = 0x08;
    const uint8_t bit2_mask = 0x04;
    const uint8_t bit1_mask = 0x02;
};
}  // namespace sensor

#endif  // SROS_EU100_TIM320_SENSOR_MANAGER_HPP
