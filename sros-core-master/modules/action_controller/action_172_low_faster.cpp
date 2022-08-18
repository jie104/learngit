
#include "action_172_low_faster.h"
#include "core/logger.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "path_utils.h"
#include "rfid_manager.h"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/path_msg.hpp"
#include "modules/main/path_checker.h"
#include "core/navigation_path.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action172LowFaster::doStart() {

    src_action_no_ = CalcSrcActionNo();
    //参数读取
    auto& s = sros::core::Settings::getInstance();

    //action_param0_: 货位点id
    auto dst_station_id = action_param0_;
    auto dst_station = MapManager::getInstance()->getStation(dst_station_id);
    LOG(INFO) << "dst_station.id id: " << dst_station_id;
    if (dst_station.id == 0) {
        LOG(ERROR) << "not exist station id: " << dst_station_id;
        doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
        return;
    }

    dst_pose_.x() = dst_station.pos.x / 100.0;
    dst_pose_.y() = dst_station.pos.y / 100.0;
    dst_pose_.yaw() = normalizeYaw(dst_station.pos.yaw);

    LOG(INFO) << "dst_pose, [x: " << dst_pose_.x() << "][y: " << dst_pose_.y() << "][yaw: " << dst_pose_.yaw() << "]";

    sendGeneratePathsCmd();
}

void Action172LowFaster::sendGeneratePathsCmd(){
    //发送导航命令到NavigationModule，获取到达物料点的路径
    Pose tmp_pose;  //空值
    auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("NAV_PATH_BUILD");
    //mm->command = COMMAND_NAV_NAVIGATE_TO;
    mm->str0 = g_state.getCurMapName();
    mm->pose = tmp_pose;
    mm->param0 = action_param0_; //站点id
    mm->param1 = ObstacleAvoidPolicy::OBSTACLE_AVOID_WAIT;
    mm->param_boolean = false;
    sros::core::MsgBus::sendMsg(mm);
}
void Action172LowFaster::onNavResultCallback(const sros::core::base_msg_ptr& msg) {

    auto mm = dynamic_pointer_cast<PathMsg>(msg);
    //路径siz小于2都是错误的
    if(mm->paths.size() < 2){
        LOG(ERROR) << "NavResult path is wrong, paths size:"<<mm->paths.size();
        //生成到目标点的路径异常errcode
        doActionFinishFailed(sros::core::ERROR_CODE_PATH_NOT_EXPECTED);
        return ;
    }

    //路网生成的起始路径可能是旋转+bezier  或者  bezier，获取bezier的终点作为检测开始点
    for(unsigned int i = 0; i < mm->paths.size(); ++i)
	{
		NavigationPath<double> path = mm->paths[i];
        dst_paths_.push_back(path);
        if(path.type_ == PATH_BEZIER){  //获取bezier路径的终点作为检测开始点
            bezier_end_pose_ = Pose(path.ex_,path.ey_,path.e_facing_);
            LOG(INFO) << "bezier_end_pose_ [x: " << bezier_end_pose_.x() << "][y: " << bezier_end_pose_.y() 
                        << "][yaw: " << bezier_end_pose_.yaw() << "]";
                        
            const double eps=1e-6;//1的负6次方	path
            if(fabs(bezier_end_pose_.yaw() - dst_pose_.yaw()) < eps){   //判断是否相等，不相等会影响识别
                LOG(WARNING) << "yaw is not equal！ bezier_end_pose_.yaw:" << path.e_facing_ 
                    <<", dst_pose_.yaw:"<< dst_pose_.yaw();
            }
            //距离校验
            calculatePoint();

            return ;
        }
        if(i == 1){ //如果前两段都没有bezier则报错结束
            LOG(ERROR) << "NavResult path is wrong, not found bezier path!" ;
            // 生成到目标点的路径异常errcode
            doActionFinishFailed(sros::core::ERROR_CODE_PATH_NOT_EXPECTED);
            return ;
        }
	}
    
}

//点位推算及距离校验
void Action172LowFaster::calculatePoint(){

    //求货物前沿点
    double goods_len = calcGoodsLength(-1);
    Pose guess_goods_front_pose;
    calcDstPose(dst_pose_, guess_goods_front_pose, goods_len/2);
    LOG(INFO) << "guess_goods_front_pose, [x: " << guess_goods_front_pose.x() 
              << "][y: " << guess_goods_front_pose.y() 
              << "][yaw: " << guess_goods_front_pose.yaw() << "]";

    //计算叉车bezier_end_pose_时叉尖和货物前沿的距离
    double distance = distanceTwoPose(bezier_end_pose_,guess_goods_front_pose) * 1000;
    //检测终点叉尖和货物前沿的距离不能低于40CM
    double detect_end_distance = 0.4;
    //叉尖到agv中心的距离
    auto& s = sros::core::Settings::getInstance();
    double fork_arm_length = s.getValue<double>("forklift.fork_arm_length", 1.0);
    string fork_end_coordinate_str = s.getValue<string>("forklift.fork_end_coordinate", "1;0;0.1;");
    std::vector<string> vec;
    string_split(fork_end_coordinate_str, ";", vec);
    double fork_end_coordinate_x = std::stod(vec[0]);

    double forktip_agv_distance = fork_arm_length - fork_end_coordinate_x; 

    //校验完距离足够再推位置
    if((distance - forktip_agv_distance) < detect_end_distance) {
        // //检测终点离货物过近
        // LOG(ERROR) << "detect distance too big!";
        // // 检测距离设置太大，推算的检测结束点和货物之间的距离过近
        doActionFinishFailed(sros::core::ERROR_CODE_FASTAER_LOAD_DETECT_DISTANCE_TOO_BIG);
        return ;
    }

    Pose detect_pose;
    // 检测终点;
    calcDstPose(guess_goods_front_pose, detect_pose, detect_end_distance + forktip_agv_distance);
    LOG(INFO) << "detect_pose, [x: " << detect_pose.x() 
            << "][y: " << detect_pose.y() 
            << "][yaw: " << detect_pose.yaw() << "]";   

    LinePath path;
    path = makeLine(bezier_end_pose_.x(), bezier_end_pose_.y(), detect_pose.x(), detect_pose.y(), PATH_BACKWARD);
    dst_paths_.push_back(path);

    starFirstMoveTask(dst_paths_);  

}

void Action172LowFaster::starFirstMoveTask(const sros::core::NavigationPath_vector& paths) {
    //关闭后侧避障雷达，打开R200滤波
    enableBackLidar(false);
    //调试
    debugOuputPaths(paths);
    //向SRC发送移动对接路径
    step_ = BEZIER;
    sendMoveTask(paths);
}

void Action172LowFaster::doCancel() {
    if(step_ == BEZIER){
        src_sdk->cancelAction(action_no_);
    }
    else if(step_ == AC_RUNING || step_ == GOBACK){
        src_sdk->cancelAction(src_action_no_);
    }
}

void Action172LowFaster::onSrcAcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcAcFinishSucceed" ;
    // //防止多个AC动作时，某个结束后一直触发此函数
    src_action_no_ = CalcSrcActionNo();
    if(step_ == Step::AC_RUNING){

        auto& s = sros::core::Settings::getInstance();
        bool enable_unload_detect = (s.getValue<string>("main.enable_unload_detect", "True") == "True");
        if(enable_unload_detect) {
            // 2.取货目标点位姿检测
            detectGoodsRacks();
            step_ = DETECTING;
        } else {
             step_ = GOBACK;

            //3.生成移动下货路径
            sros::core::NavigationPath_vector dst_paths;
            genLinePath(dst_paths);
            debugOuputPaths(dst_paths);
            sendMoveTask(dst_paths);
        }

        return ;
    }

    // // 检测到位信号是否触发，判断进叉是否到位
    // if(!checkPalletInPlaceSignal()) {
    //     //防止状态未及时更新到retry
    //     boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    //     if(!checkPalletInPlaceSignal()) {
    //         LOG(ERROR) << "checkPalletInPlaceSignal: false";
    //         sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
    //         doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
    //         enableBackLidar(true);
    //         return;
    //     }
    // }
    //成功返回
    // doActionFinishSucceed();
}

void Action172LowFaster::onSrcAcFinishFailed(int result_value) {
    if(step_ == AC_RUNING){
        //叉臂升降失败
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    }else if(step_ == GOBACK){
        if(result_value == 2498){
            //到位开关未触发
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
            doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
        }else{
            //路径执行失败
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
            doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);
        }
    }else{
        //nothing
        LOG(ERROR) << "onSrcAcFinishFailed,but step = " << step_ ;
    }
    step_ = FAILED;
}

void Action172LowFaster::onSrcAcFinishCanceled(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    step_ = FAILED;
}

void Action172LowFaster::onSrcMcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcMcFinishSucceed";
    LOG(INFO) << "step_:"<<step_;
    if(step_ == BEZIER){
        step_ = AC_RUNING;
        src_sdk->executeAction(src_action_no_, 24, 3, action_param1_);
        LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
                    << "id 24, p0 3, p1 " << action_param1_;
    }else {
        doActionFinishSucceed();
    }
}

void Action172LowFaster::onSrcMcFinishFailed(int result_value) {
    LOG(ERROR) << "货叉取货路径前往对接失败, result_value" << result_value;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);
    step_ = FAILED;
    enableBackLidar(true);
}

void Action172LowFaster::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {

    auto mm = dynamic_pointer_cast<PerceptionStateMsg>(msg);
    int detect_result = mm->detect_result;
    LOG(INFO) << "unload detect_result :" << detect_result;

    bool result = (detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS);

    //上传卸货检测的调试区域
    sendCommonPosesInfo(mm->put_detect_region, (result ? STR_UNLOAD_LASER_NONE : STR_UNLOAD_LASER_HAVE));

    if (!result) {
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_DETECTED);
        return;
    }

    //继续
    // fork_step_ = MC_RUNING;
    step_ = DETECTING;
    //向SRC发送移动对接路径
    sros::core::NavigationPath_vector dst_paths;
    genLinePath(dst_paths);
    debugOuputPaths(dst_paths);
    sendMoveTask(dst_paths);
}

void Action172LowFaster::genLinePath(sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "genLinePath";

    double goods_len = calcGoodsLength(goal_id_);
    double fork_end_coordinate = calcForkEndCoordinateLength();

    //计算最终移动点偏移
    double deviation_len = (goods_len / 2) - fork_end_coordinate;

    //计算最终移动点位姿
    Pose fin_pose;
    calcDstPose(dst_pose_, fin_pose, deviation_len);
    LOG(INFO) << "fin_pose, [x: " << fin_pose.x() << "][y: " << fin_pose.y() << "][yaw: " << fin_pose.yaw() << "]";

    auto curr_pose = src_sdk->getCurPose();
    LinePath p;
    p = makeLine(curr_pose.x(), curr_pose.y(), fin_pose.x(), fin_pose.y(), PATH_BACKWARD);
    dst_paths.push_back(p);
    genStartRotatePath(dst_paths, curr_pose.yaw());
}

void Action172LowFaster::detectGoodsRacks() {
    LOG(INFO) << "detectGoodsRacks";
    Pose pose;
    pose.x() = dst_pose_.x();
    pose.y() = dst_pose_.y();
    pose.yaw() = dst_pose_.yaw();

    auto& s = sros::core::Settings::getInstance();
    auto fork_goods_type = s.getValue<string>("perception.detect_goods_type", "CIRCLE");

    PerceptionCommandMsg::Command cmd;
    cmd.detect_stage = PerceptionCommandMsg::DetectStage::DETECT_STAGE_UNLOAD;
    if (fork_goods_type == "CARD") {
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CARD;
    } else if (fork_goods_type == "CIRCLE") {
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CIRCLE;
    } else {
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_HANDCART;
    }

    sendPerceptionCmd(action_no_, cmd, g_state.cur_goal_id, pose);
}

bool Action172LowFaster::onSubSrcActionCheck(uint32_t& sub_src_action_no) {
    sub_src_action_no = src_action_no_;
    return true;
}
//第一次调用返回action_no_
uint32_t Action172LowFaster::CalcSrcActionNo() {
    int no = action_no_ - count_;
    count_++;
    return no ;
}

}
