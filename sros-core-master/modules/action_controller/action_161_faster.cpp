//
// Created by lbx on 2022/1/14
//
#include "action_161_faster.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "rfid_manager.h"
#include "core/fault_center.h"
#include "path_utils.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action161Faster::doStart() {

    //action_param0_: 货位点id
    auto dst_station_id = action_param0_;
    auto dst_station = MapManager::getInstance()->getStation(dst_station_id);

    LOG(ERROR) << "dst_station.id id: " << dst_station_id;
    if (dst_station.id == 0) {
        LOG(ERROR) << "not exist station id: " << dst_station_id;
        doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
        return;
    }

    dst_pose_.x() = dst_station.pos.x / 100.0;
    dst_pose_.y() = dst_station.pos.y / 100.0;
    dst_pose_.yaw() = normalizeYaw(dst_station.pos.yaw);

    LOG(INFO) << "dst_pose, [x: " << dst_pose_.x() << "][y: " << dst_pose_.y() << "][yaw: " << dst_pose_.yaw() << "]";
    
    //导航中不能低于行驶高度
    if(action_param1_ <= 300 ){
        doActionFinishSucceed();
        return ;
    }

    //计算抬升这么搞需要提前多少距离开始
    //（叉臂高度差 / 叉臂速度） * 车速 = 提前抬臂的距离； 由于叉臂速度和车速都是变量，所以使用阈值参数代替，该参数为（车速/叉臂抬升速度）
    auto& s = sros::core::Settings::getInstance();
    int limit_height = s.getValue<int>("forklift.nav_forkarm_height_limit", 1200);
    if(action_param1_ > limit_height)
        height_ = limit_height;
    else
        height_ = action_param1_;

    int offset_height = std::fabs(height_ - g_state.fork_height_encoder * 1000);
    double fork_speed = s.getValue<int>("forklift.adjust_forkarm_advance_threshold", 10); //大致值为：（车速/叉臂抬升速度）
    action_start_distance_ = (offset_height/fork_speed) * 200;
    std::thread thread2(&Action161Faster::waitActionStart, this);
    thread2.detach();
}

//当前位置检测，改为定时器事件
void Action161Faster::waitActionStart(){
    //
    while(step_ < FINISH){
        
        //到达C1点就可以退出检测了
        double distance = distanceTwoPose(src_sdk->getCurPose(),dst_pose_)*1000;
        LOG(INFO) << "distance " << distance;

        //从配置获取到点精度
        double rearch_threshold = 50+20; //到点检测阈值  
        if(distance < rearch_threshold){  //到达站点，若未到位结束动作
            if(step_ == AC_RUNING){
                doCancel();
            }else {
                LOG(INFO) << "doActionFinishSucceed ";
                doActionFinishSucceed();
            }
            break;
        } else if(distance < action_start_distance_ && step_ == WAITING){
            // 向SRC发送动作指令
            // if(action_param1_ > limit_height)
            src_sdk->executeAction(action_no_, 24, 10, action_param1_);
            LOG(INFO) << "src_ac: no " << action_no_ << ", "
                    << "id 24, p0 10, p1 " << action_param1_;
            step_ = AC_RUNING;

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Action161Faster::doCancel() {

    if(step_ == AC_RUNING) {
        if(isForkControlTypeSrc()) {
            src_sdk->cancelAction(action_no_);
        } else {
            cancelEacActionTask(action_no_);
        }
    }
    
}

void Action161Faster::onSrcAcFinishSucceed(int result_value) {
    step_ = FINISH;
    LOG(INFO) << "onSrcAcFinishSucceed";
    doActionFinishSucceed();
}

void Action161Faster::onSrcAcFinishFailed(int result_value) {
    step_ = FINISH;
    LOG(INFO) << "onSrcAcFinishFailed,but doActionFinishSucceed ";
    doActionFinishSucceed();

}

void Action161Faster::onSrcAcFinishCanceled(int result_value) {
    step_ = FINISH;
    LOG(INFO) << "onSrcAcFinishCanceled,but doActionFinishSucceed ";
    doActionFinishSucceed();
}



}