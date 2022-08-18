
#include "action_162_low_faster.h"
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

void Action162LowFaster::doStart() {

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

void Action162LowFaster::sendGeneratePathsCmd(){
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
void Action162LowFaster::onNavResultCallback(const sros::core::base_msg_ptr& msg) {

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
void Action162LowFaster::calculatePoint(){

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

void Action162LowFaster::starFirstMoveTask(const sros::core::NavigationPath_vector& paths) {
    //关闭后侧避障雷达，打开R200滤波
    enableBackLidar(false);
    //调试
    debugOuputPaths(paths);
    //向SRC发送移动对接路径
    step_ = BEZIER;
    sendMoveTask(paths);
}

void Action162LowFaster::doCancel() {
    if(step_ == BEZIER){
        src_sdk->cancelAction(action_no_);
    }
    else if(step_ == AC_RUNING || step_ == GOBACK){
        src_sdk->cancelAction(src_action_no_);
    }
}

void Action162LowFaster::onSrcAcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcAcFinishSucceed" ;
    // //防止多个AC动作时，某个结束后一直触发此函数
    src_action_no_ = CalcSrcActionNo();
    if(step_ == Step::AC_RUNING){

        auto& s = sros::core::Settings::getInstance();
        bool enable_load_detect = (s.getValue<string>("main.enable_load_detect", "True") == "True");
        if(enable_load_detect) {
            // 2.取货目标点位姿检测
            detectDst3DPose();
            step_ = DETECTING;
        } else {
            //发起直线路径
            launchMoveTask(true);
            step_ = GOBACK;
        }

        return ;
    }

    // 检测到位信号是否触发，判断进叉是否到位
    if(!checkPalletInPlaceSignal()) {
        //防止状态未及时更新到retry
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        if(!checkPalletInPlaceSignal()) {
            LOG(ERROR) << "checkPalletInPlaceSignal: false";
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
            doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
            enableBackLidar(true);
            return;
        }
    }
    //成功返回
    doActionFinishSucceed();
}

void Action162LowFaster::onSrcAcFinishFailed(int result_value) {
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

void Action162LowFaster::onSrcAcFinishCanceled(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    step_ = FAILED;
}

void Action162LowFaster::onSrcMcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcMcFinishSucceed";
    LOG(INFO) << "step_:"<<step_;
    if(step_ == BEZIER){
        step_ = AC_RUNING;
        src_sdk->executeAction(src_action_no_, 24, 3, action_param1_);
        LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
                    << "id 24, p0 3, p1 " << action_param1_;
    }
}

void Action162LowFaster::onSrcMcFinishFailed(int result_value) {
    LOG(ERROR) << "货叉取货路径前往对接失败, result_value" << result_value;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);
    step_ = FAILED;
    enableBackLidar(true);
}

void Action162LowFaster::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {
    auto mm = dynamic_pointer_cast<PerceptionStateMsg>(msg);
    dst_3d_pose_.x() = mm->goal_in_global_pose.x();
    dst_3d_pose_.y() = mm->goal_in_global_pose.y();
    dst_3d_pose_.yaw() = mm->goal_in_global_pose.yaw();

    dst_3d_agv_pose_.x() = mm->goal_in_agv_pose.x();
    dst_3d_agv_pose_.y() = mm->goal_in_agv_pose.y();
    dst_3d_agv_pose_.yaw() = mm->goal_in_agv_pose.yaw();

    int detect_result = mm->detect_result;
    goal_id_ = mm->goal_id;

    LOG(INFO) << "vision [dst_3d__global_pose x: " << dst_3d_pose_.x() << " y: " << dst_3d_pose_.y() << " yaw: " << dst_3d_pose_.yaw()
              << "] [goal_in_agv_pose x: " << dst_3d_agv_pose_.x() << " y: " << dst_3d_agv_pose_.y() << " yaw: " << dst_3d_agv_pose_.yaw()
              << "] [detect_result: " << detect_result << " goal_id_: " << goal_id_ << "]";

    bool result = (detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS);
    //拿到结果之后
    if (!result) {
        //未检测到货物
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_NOT_DETECTED);
        return;
    }

    // 推算得出的视觉检测到位姿
    double goods_len = calcGoodsLength(goal_id_);
    Pose calc_dst_3d_pose;
    calcDstPose(dst_pose_, calc_dst_3d_pose, goods_len/2);
    LOG(INFO) << "calc_dst_3d_pose, [x: " << calc_dst_3d_pose.x()
              << "][y: " << calc_dst_3d_pose.y() << "][yaw: " << calc_dst_3d_pose.yaw() << "]";

    //计算全局坐标系下X的误差
    auto distance_two_pose = [](const Pose &p1,const Pose &p2) {
        return sqrt((p1.x()- p2.x()) * (p1.x()- p2.x()) + (p1.y()- p2.y()) * (p1.y()- p2.y()));
    };

    auto curr_pose = src_sdk->getCurPose();
    double len_agv_to_real = distance_two_pose(dst_3d_pose_, curr_pose);
    double len_agv_to_theory = distance_two_pose(calc_dst_3d_pose, curr_pose);

    offset_x_real_sub_theory_ = len_agv_to_real - len_agv_to_theory;

    //计算相机方向下Y和YAW的误差
    double offset_y = std::fabs( dst_3d_pose_.y()-calc_dst_3d_pose.y());
    double offset_yaw = std::fabs( dst_3d_pose_.yaw()-calc_dst_3d_pose.yaw());

    LOG(INFO) << "offset, [x: " << offset_x_real_sub_theory_ << "][y: " << offset_y << "][yaw: " << offset_yaw << "]";

    auto& s = sros::core::Settings::getInstance();
    double auto_adjust_need_distance = s.getValue<double>("forklift.auto_adjust_need_distance", 0.5);
    double bezier_adjust_distance = s.getValue<double>("forklift.bezier_adjust_distance", 0.3);
    bool enough_adjust = (std::fabs(dst_3d_agv_pose_.x()) > (auto_adjust_need_distance + bezier_adjust_distance));

    //对检测后的位姿进行校验
    checkDst3DPose(offset_x_real_sub_theory_, offset_y, offset_yaw, enough_adjust);
}

void Action162LowFaster::checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust) {

    auto& s = sros::core::Settings::getInstance();
    double deviate_distance_x_forward = s.getValue<double>("forklift.deviate_distance_x_forward", -0.1);
    double deviate_distance_x_back = s.getValue<double>("forklift.deviate_distance_x_back", 0.1);
    double deviate_max_y = s.getValue<double>("forklift.deviate_max_y", 0.3);
    double deviate_max_yaw = s.getValue<double>("forklift.deviate_max_yaw", 10.0);
    deviate_max_yaw = deviate_max_yaw / 180 * M_PI;
    LOG(INFO) << "distance_x_forward: " << deviate_distance_x_forward
              << ", distance_x_back: "  <<  deviate_distance_x_back
              << ", max_y: " << deviate_max_y 
              << ", max_yaw: " << deviate_max_yaw;

    //判断是否超过最大误差
    if (offset_yaw > deviate_max_yaw) {
        LOG(ERROR) << "offset_yaw(" << offset_yaw <<  ") > max_yaw(" << deviate_max_yaw << ")";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_YAW_DEVIATE);
        return;
    }
    if (offset_y > deviate_max_y) {
        LOG(ERROR) << "offset_y(" << offset_y << ") > max_y(" << deviate_max_y << ")";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_Y_DEVIATE);
        return;
    }
    if (offset_x < deviate_distance_x_forward) {
        LOG(ERROR) << "offset_x(" << offset_x << ") < forward(" << deviate_distance_x_forward << ")";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_FORWARD);
        return;
    }
    if (offset_x > deviate_distance_x_back) {
        LOG(ERROR) << "offset_x(" << offset_x << ") > back(" << deviate_distance_x_back << ")";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_BACK); 
        return;
    }

    double deviate_min_y = s.getValue<double>("forklift.deviate_min_y", 0.01);
    double deviate_min_yaw = s.getValue<double>("forklift.deviate_min_yaw", 1.0);
    deviate_min_yaw = deviate_min_yaw / 180 * M_PI;

    LOG(INFO) << "min_y: " << deviate_min_y << ", min_yaw: " << deviate_min_yaw;

    bool is_one_line = (offset_y < deviate_min_y && offset_yaw < deviate_min_yaw);

    //角度需要调整，但是调整距离不够直接报错
    if ( !is_one_line && !enough_adjust) {
        LOG(ERROR) << "需要自适应调整，但是调整距离不够";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_X_DEVIATE_BACK);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_ADJUST_LEN_NOT_ENOUGH);
        return;
    }

    launchMoveTask(is_one_line); 

}

void Action162LowFaster::launchMoveTask(bool is_one_line) {

    //关闭后侧避障雷达，打开R200滤波
    enableBackLidar(false);

    //生成移动对接路径
    sros::core::NavigationPath_vector dst_paths;

    if(is_one_line) {
        genLinePath(dst_paths);
    } else {
        g_state.is_fork_pose_adjust = true;
        genMovePath(dst_paths);
    }

    //调试
    debugOuputPaths(dst_paths);

    // 向SRC发送动作指令，上货路径监控动作
    src_sdk->executeAction(src_action_no_, 24, 30, 0);
    LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
              << "id 24, p0 30, p1 0";

    step_ = GOBACK;

    //向SRC发送移动对接路径
    sendMoveTask(dst_paths);

    //记录当前的货物id
    g_state.cur_goal_id = goal_id_;
}


void Action162LowFaster::genLinePath(sros::core::NavigationPath_vector& dst_paths) {
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

void Action162LowFaster::genMovePath(sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "genMovePath";

    double adjust_pose_optimal_distance = calcAdjustPoseOptimalDistance();

    //计算中间点位姿（第一次调整的最佳位姿）
    Pose mid_pose;
    calcDstPose(dst_3d_pose_, mid_pose, adjust_pose_optimal_distance);
    LOG(INFO) << "mid_pose, [x: " << mid_pose.x() << "][y: " << mid_pose.y() << "][yaw: " << mid_pose.yaw() << "]";

    //计算最终移动点位姿
    double fork_end_coordinate = calcForkEndCoordinateLength();
    //计算最终移动点偏移
    double deviation_len = 0 - fork_end_coordinate;

    Pose fin_pose;
    calcDstPose(dst_3d_pose_, fin_pose, deviation_len);
    LOG(INFO) << "fin_pose, [x: " << fin_pose.x() << "][y: " << fin_pose.y() << "][yaw: " << fin_pose.yaw() << "]";
    
    //根据检测返回的目标点位姿生成路径
    auto curr_pose = src_sdk->getCurPose();
    LOG(INFO) << "curr_pose, [x: " << curr_pose.x() << "][y: " << curr_pose.y() << "][yaw: " << curr_pose.yaw() << "]";
    
    BezierPath bez = genBezierPath(curr_pose, mid_pose, PATH_BACKWARD);
    bez.updateFacing();
    dst_paths.push_back(bez);

    LinePath p;
    //p = makeLine(curr_pose.x(), curr_pose.y(), mid_pose.x(), mid_pose.y(), PATH_BACKWARD);
    //dst_paths.push_back(p);
    p = makeLine(mid_pose.x(), mid_pose.y(), fin_pose.x(), fin_pose.y(), PATH_BACKWARD);
    dst_paths.push_back(p);
    genRotateBetweenPaths(dst_paths);
    genStartRotatePath(dst_paths, curr_pose.yaw());
}

void Action162LowFaster::detectDst3DPose() {

    auto& s = sros::core::Settings::getInstance();
    auto fork_goods_type = s.getValue<string>("perception.detect_goods_type", "CIRCLE");

    PerceptionCommandMsg::Command cmd;
    cmd.detect_stage = PerceptionCommandMsg::DetectStage::DETECT_STAGE_LOAD;
    if(fork_goods_type == "CARD") {
        //卡板检测
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CARD;
    } else if(fork_goods_type == "CIRCLE"){
        //圆盘检测
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CIRCLE;
    } else {
        //手推车
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_HANDCART;
    }

    sendPerceptionCmd(action_no_, cmd, -1, dst_pose_);

}

bool Action162LowFaster::onSubSrcActionCheck(uint32_t& sub_src_action_no) {
    sub_src_action_no = src_action_no_;
    return true;
}
//第一次调用返回action_no_
uint32_t Action162LowFaster::CalcSrcActionNo() {
    int no = action_no_ - count_;
    count_++;
    return no ;
}

}
