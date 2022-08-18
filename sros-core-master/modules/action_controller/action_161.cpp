//
// Created by caoyan on 4/1/21.
//

#include "action_161.h"
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

void Action161::doStart() {

    g_state.is_fork_pose_adjust = false;

    is_forklift_running_ = false;
    //is_get_rfid_running_ = false;
    is_paths_running_ = false;
    is_keep_recv_src_ret_ = true;

    if(isForkControlTypeSrc()) {
        // 向SRC发送动作指令
        src_sdk->executeAction(action_no_, 24, 3, action_param1_);
        LOG(INFO) << "src_ac: no " << action_no_ << ", "
                  << "id 24, p0 3, p1 " << action_param1_;
    } else {
        //日本叉车走eac控制货叉升降指令， 207.1.H   货叉升降指令
        if(!isEnableEac()) {
            doActionFinishFailed(ERROR_CODE_ACTION_EAC_DISABLED);
            return;
        }
        sendEacActionTask(action_no_, 207, 1, action_param1_);
        LOG(INFO) << "eac_ac: no " << action_no_ << ", "
                  << "id 207, p0 1, p1 " << action_param1_;
    }

    is_forklift_running_ = true;
}

void Action161::doCancel() {

    if(is_forklift_running_) {
        if(isForkControlTypeSrc()) {
            src_sdk->cancelAction(action_no_);
        } else {
            cancelEacActionTask(action_no_);
        }
    }

    if(is_paths_running_) {
        src_sdk->cancelAction(action_no_);   
    }

    //取消时恢复
    enableBackLidar(true);
    
}

bool Action161::onSubSrcActionCheck(uint32_t& sub_src_action_no) {

    if (is_keep_recv_src_ret_) {
        return false;
    } else {
        sub_src_action_no = -1;
        return true;
    }
}

void Action161::onSrcAcFinishSucceed(int result_value) {

    if(!is_keep_recv_src_ret_) {
        return;
    }

    if (checkForkHeightFautl(action_param1_ / 1000.0)) {
        return;
    }

    is_forklift_running_ = false;

    is_keep_recv_src_ret_ = false;

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

    //是否调整检测距离
    auto& s = sros::core::Settings::getInstance();
    bool enable_adjust_detect_distance = (s.getValue<string>("forklift.enable_adjust_detect_distance", "False") == "True");
    if (!enable_adjust_detect_distance) {
        LOG(INFO) << "dont't adjust detect distance.";
        doActionFinishSucceed();
        return ;

    }


    //生成调整路径
    sros::core::NavigationPath_vector dst_paths;
    if (genGoToDockingPosePath(dst_paths)) {

        if (dst_paths.size() > 0 ) {

            //调试
            debugOuputPaths(dst_paths);

            //关闭后侧避障雷达，打开R200滤波
            enableBackLidar(false);

            is_paths_running_ = true;

            //向SRC发送移动对接路径
            sendMoveTask(dst_paths);

        } else {
            LOG(INFO) << "dst_paths empty";
            doActionFinishSucceed();
            enableBackLidar(true);
        }
        
    } else {

        LOG(INFO) << "离货物距离过近会碰撞";
        sros::core::FaultCenter::getInstance()->addFault(sros::core::ERROR_CODE_ACTION_FORK_MOVE_LEN_NOT_ENOUGH);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_MOVE_LEN_NOT_ENOUGH);
        
    }

}

void Action161::onSrcAcFinishFailed(int result_value) {
    //
    is_forklift_running_ = false;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);

}

void Action161::onSrcAcFinishCanceled(int result_value) {
    is_forklift_running_ = false;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
}


bool Action161::genGoToDockingPosePath(sros::core::NavigationPath_vector& dst_paths) {
    // 按照最大货物的长度推算得出货物前沿位置
    double goods_len = calcGoodsLength(-1);
    Pose guess_goods_front_pose;
    calcDstPose(dst_pose_, guess_goods_front_pose, goods_len/2);
    LOG(INFO) << "guess_goods_front_pose, [x: " << guess_goods_front_pose.x() 
              << "][y: " << guess_goods_front_pose.y() 
              << "][yaw: " << guess_goods_front_pose.yaw() << "]";

    // 推算调整的最佳检测位置
    double detect_distance = calcDetectDistance();
    Pose detect_pose;
    calcDstPose(guess_goods_front_pose, detect_pose, detect_distance);
    LOG(INFO) << "detect_pose, [x: " << detect_pose.x() 
              << "][y: " << detect_pose.y() 
              << "][yaw: " << detect_pose.yaw() << "]";
              
    //简单推算距离是否可以自适应调整          
    auto curr_pose = src_sdk->getCurPose();
    LOG(INFO) << "curr_pose, [x: " << curr_pose.x() << "][y: " << curr_pose.y() << "][yaw: " << curr_pose.yaw() << "]";

    auto distance_two_pose = [](const Pose &p1,const Pose &p2) {
        return sqrt((p1.x()- p2.x()) * (p1.x()- p2.x()) + (p1.y()- p2.y()) * (p1.y()- p2.y()));
    };

    double curr_pose_to_dst = distance_two_pose(curr_pose, dst_pose_);
    double detect_pose_to_dst = distance_two_pose(detect_pose, dst_pose_);

    //double len_useful = distance_two_pose(detect_pose, curr_pose);
    double len_useful = curr_pose_to_dst - detect_pose_to_dst;

    auto& s = sros::core::Settings::getInstance();
    double auto_adjust_need_distance = s.getValue<double>("forklift.auto_adjust_need_distance", 5.0);
    double bezier_adjust_distance = s.getValue<double>("forklift.bezier_adjust_distance", 0.3);
    double docking_need_distance = s.getValue<double>("forklift.docking_need_distance", 0.1);

    LOG(INFO) << "[curr_pose_to_dst: " << curr_pose_to_dst 
              << "][detect_pose_to_dstj: " << detect_pose_to_dst
              << "][len_useful: " << len_useful << "]"; 

    LOG(INFO) << "[auto_adjust_need_distance: " << auto_adjust_need_distance 
              << "][bezier_adjust_distance: " << bezier_adjust_distance
              << "][docking_need_distance: " << docking_need_distance << "]";   

    // 推算检测点的距离
    double docking_pose_distance;

    if (len_useful < 0) {
        LOG(INFO) << "fault(len_useful < 0) canot do action";
        return false;

    } else if (len_useful < docking_need_distance) {
        LOG(INFO) << "docking_need_distance not enough";
        return true;

    } else if (len_useful > (auto_adjust_need_distance + bezier_adjust_distance)) {

        double len_left = len_useful - auto_adjust_need_distance - bezier_adjust_distance;
        LOG(INFO) << "[len_left: " << len_left << "]";

        docking_pose_distance = auto_adjust_need_distance + bezier_adjust_distance;

        g_state.is_fork_pose_adjust = true;

        //推算检测点
        Pose detect_docking_pose;
        calcDstPose(detect_pose, detect_docking_pose, docking_pose_distance);
        LOG(INFO) << "detect_docking_pose, [x: " << detect_docking_pose.x()
                << "][y: " << detect_docking_pose.y() 
                << "][yaw: " << detect_docking_pose.yaw() << "]";

        //生成路径
        LinePath p;
        p = makeLine(curr_pose.x(), curr_pose.y(), detect_docking_pose.x(), detect_docking_pose.y(), PATH_BACKWARD);
        dst_paths.push_back(p);

    } else {

        docking_pose_distance = len_useful;

        g_state.is_fork_pose_adjust = false;

        //生成路径
        LinePath p;
        p = makeLine(curr_pose.x(), curr_pose.y(), detect_pose.x(), detect_pose.y(), PATH_BACKWARD);
        dst_paths.push_back(p);

    }

    LOG(INFO) << "[g_state.is_fork_pose_adjust: " << g_state.is_fork_pose_adjust << "]";

    LOG(INFO) << "[docking_pose_distance: " << docking_pose_distance << "]";

    genStartRotatePath(dst_paths, curr_pose.yaw());
    genEndRotatePath(dst_paths, detect_pose.yaw());

    return true;
}


void Action161::onSrcMcFinishSucceed(int result_value) {
    is_paths_running_ = false;
    doActionFinishSucceed();

    enableBackLidar(true);
}

void Action161::onSrcMcFinishFailed(int result_value) {
    LOG(ERROR) << "货叉取货路径前往对接失败, result_value" << result_value;
    is_paths_running_ = false;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);

    enableBackLidar(true);
}


// void Action161::getGoodsRfid() {
//     rfid_data_.clear();

//     if (!RfidManager::getInstance()->isConnected()) {
//         LOG(ERROR) << "RFID not enable!";
//         doActionFinishFailed(sros::core::ERROR_CODE_RFID_NOT_ENABLED);
//     } else {
//         auto getRFIDDataFun = [&](int tryTimes) {
//           // 100毫秒尝试一次
//           for (int i = 0; i < tryTimes; ++i) {

//               auto data = RfidManager::getInstance()->syncGetRfid();
//               if (data.empty()) {
//                   std::this_thread::sleep_for(std::chrono::milliseconds(100));
//               } else {
//                   action_task_->setResultValueStr(data);
//                   doActionFinishSucceed();
//                   return;
//               }
//           }
//           LOG(ERROR) << "RFID get none!";
//           doActionFinishFailed(sros::core::ERROR_CODE_RFID_GET_NONE);
//         };

//         std::thread thread1(getRFIDDataFun, 10);
//         thread1.detach();
//     }
// }

}