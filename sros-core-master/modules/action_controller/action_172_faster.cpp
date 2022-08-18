	
#include "action_172_faster.h"
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

#define STR_UNLOAD_LASER_NONE "UNLOAD_LASER_NONE"
#define STR_UNLOAD_LASER_HAVE "UNLOAD_LASER_HAVE"

namespace ac {

void Action172Faster::doStart() {

    src_action_no_ = CalcSrcActionNo();
    //参数读取
    auto& s = sros::core::Settings::getInstance();
    limit_height_ = s.getValue<int>("forklift.unload_beizer_forkarm_height_limit", 1000);
    tmp_paths_.clear();
    dst_paths_.clear();

    detect_distance_ = s.getValue<double>("forklift.faster_load_detect_distance", 0.4);
    bool enable_load_detect = (s.getValue<string>("main.enable_load_detect", "True") == "True");
    if(!enable_load_detect) {
        detect_distance_ = 0;
    }

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

void Action172Faster::sendGeneratePathsCmd(){
    // 发送导航命令到NavigationModule，获取到达物料点的路径
    Pose tmp_pose;  //空值
    auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("NAV_PATH_BUILD");
    // mm->command = COMMAND_NAV_NAVIGATE_TO;
    mm->str0 = g_state.getCurMapName();
    mm->pose = tmp_pose;
    mm->param0 = action_param0_; //站点id
    mm->param1 = ObstacleAvoidPolicy::OBSTACLE_AVOID_WAIT;
    mm->param_boolean = false;
    sros::core::MsgBus::sendMsg(mm);
}

void Action172Faster::onNavResultCallback(const sros::core::base_msg_ptr& msg) {

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
        tmp_paths_.push_back(path);
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
void Action172Faster::calculatePoint(){
    //求货物前沿点
    double goods_len = calcGoodsLength(g_state.cur_goal_id);
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
    if((distance - forktip_agv_distance) < (detect_end_distance + detect_distance_)) {
        // //检测终点离货物过近
        // LOG(ERROR) << "detect distance too big!";
        // // 检测距离设置太大，推算的检测结束点和货物之间的距离过近
        doActionFinishFailed(sros::core::ERROR_CODE_FASTAER_LOAD_DETECT_DISTANCE_TOO_BIG);
        return ;
    }

    // 检测终点;
    calcDstPose(guess_goods_front_pose, detect_end_pose_, detect_end_distance + forktip_agv_distance);
    LOG(INFO) << "detect_end_pose_, [x: " << detect_end_pose_.x() 
            << "][y: " << detect_end_pose_.y() 
            << "][yaw: " << detect_end_pose_.yaw() << "]";

    calcDstPose(guess_goods_front_pose, detect_start_pose_, detect_end_distance + forktip_agv_distance + detect_distance_);
    LOG(INFO) << "detect_start_pose_, [x: " << detect_start_pose_.x() 
            << "][y: " << detect_start_pose_.y() 
            << "][yaw: " << detect_start_pose_.yaw() << "]";    
    
    ForkStep next_step;
    int height;
    if(action_param1_ > limit_height_){
        height = limit_height_; //限定高度
        next_step = UPING_TO_LIMIT;
        LOG(INFO) << "UPING_TO_LIMIT ";
    }else if(action_param1_ < 150){ //货物拖着地面跑
        height = 150;
    }else{
        height = action_param1_; //目标高度
        next_step = UPING_TO_TARGET;
        LOG(INFO) << "UPING_TO_TARGET ";
    }

    //move
    starFirstMoveTask();
    std::thread thread2(&Action172Faster::waitPoseInPlace, this);
    thread2.detach();
    
    src_sdk->executeAction(src_action_no_, 24, 33, 0);
    LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
            << "id 24, p0 33, p1 0， height" << height;
    std::vector<int> values{1, height}; //启动标志位，高度
    bool ret = src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET, values, 500);
    if(!ret){
        LOG(ERROR) << "setParameters failed!";
    }
    fork_step_ = next_step;

    //考虑使用on50ms优化性能
    std::thread thread1(&Action172Faster::waitFrokInPlace, this);
    thread1.detach();
}

//当前位置检测
void Action172Faster::waitPoseInPlace(){
    //L3+L2
    len_ = distanceTwoPose(dst_pose_,detect_start_pose_) * 1000;
    // LOG(INFO) << "len_: " << len_;

    while(mc_stage_ < L2_FINISH){
        
        //到达C1点就可以退出检测了
        double distance = distanceTwoPose(src_sdk->getCurPose(),dst_pose_)*1000;
        // LOG(INFO) << "distance " << distance; 
        // if(distance < len_ - 50){  //过了5cm再检测
        if(distance < len_ ){  //过了5cm再检测
            LOG(INFO) << "enter C1 "; 
            if(fork_step_ < UPING_TO_TARGET){
                LOG(INFO) << "UPING_TO_TARGET "; 
                //降速
                src_sdk->setSpeedLevel(50);
                //升到目标值
                std::vector<int> val{2, 0}; //启动标志位，高度
                src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET, val);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
                std::vector<int> values{1, action_param1_}; //启动标志位，高度
                src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
                fork_step_ = UPING_TO_TARGET;
                LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
                        << "UPING_TO_TARGET, target_height_:" << action_param1_;
            }else if(fork_step_ == UPING_TO_TARGET){ //正在升叉臂，在叉臂结束回调中开启检测
                LOG(INFO) << "nothing "; 
                //nothing
            }else if(fork_step_ == UP_FINISH){ //到达C1且叉臂已到位
                LOG(INFO) << "detectDst3DPose "; 
                auto& s = sros::core::Settings::getInstance();
                bool enable_unload_detect = (s.getValue<string>("main.enable_unload_detect", "True") == "True");
                if(enable_unload_detect) {
                    detectGoodsRacks();
                }else {
                    mc_stage_ = L3ING;
                    //向SRC发送移动对接路径
                    sendReplaceTask(tmp_paths_);
                }
            }else{
                LOG(INFO) << "detectDst3DPose "; 
            }
            mc_stage_ = L2ING;
            //结束
            return ;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
//叉臂到位信号检测
void Action172Faster::waitFrokInPlace() {
            
    while (fork_step_ <= UPING_TO_TARGET ) {
        // LOG(INFO) << "fork_step_:" << fork_step_;
        LOG(INFO) << "current height:" << g_state.fork_height_encoder;
        int inPlace_flag = 0;
        if (!src_sdk->getFunctionParam(sdk::SrcSdk::EM_FORKLIFTSTATE, inPlace_flag)){
            LOG(WARNING) << "src getParameter timeout!";
            continue;
        }
        if(mc_stage_ == MC_FAILED){
            break;
        }

        float latst_height = g_state.fork_height_encoder;

        if(inPlace_flag){  //到位
            LOG(INFO) << "In Place";
            if(fork_step_ == UPING_TO_LIMIT){
                //pause
                std::vector<int> values{2, 0};
                src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
                fork_step_ = UPING_TO_LIMIT_FINISHH;
                LOG(INFO) << "UPING_TO_LIMIT,stop";
                continue ;
            }else if(fork_step_ == UPING_TO_TARGET){

                //结束
                std::vector<int> values{3, 0}; 
                src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
                fork_step_ = UP_FINISH;
                LOG(INFO) << "fork lift finish， exit thread！";
                //假如进入检测区
                if(mc_stage_ != L1ING){
                    auto& s = sros::core::Settings::getInstance();
                    bool enable_load_detect = (s.getValue<string>("main.enable_load_detect", "True") == "True");
                    if(enable_load_detect) {
                        detectGoodsRacks();
                    }else {
                        sendReplaceTask(tmp_paths_);
                    } 
                }   
                return ;
                
            }else if(fork_step_ == UPING_TO_LIMIT_FINISHH){
                //noting    
            }else{
                //改变标志位结束动作，
                std::vector<int> values{3, 0};
                src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
                LOG(INFO) << "unkown fork_step_:"<<fork_step_<<", exit thread！";
                fork_step_ = AC_FAILED;
                return ;
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    LOG(ERROR) << "waitFrokInPlace fail, exit thread";
 };

//路径规划
void Action172Faster::generatePaths(){

    auto& s = sros::core::Settings::getInstance();
    if(detect_distance_  == 0) {
        LinePath detect_path;
        detect_path = makeLine(bezier_end_pose_.x(), bezier_end_pose_.y(), detect_end_pose_.x(), detect_end_pose_.y(), PATH_BACKWARD);
        tmp_paths_.push_back(detect_path);
        detect_path.radius_ = -2 ;  //停止标志
        dst_paths_.push_back(detect_path);
    }else {
        LinePath path;
        path = makeLine(bezier_end_pose_.x(), bezier_end_pose_.y(), detect_start_pose_.x(), detect_start_pose_.y(), PATH_BACKWARD);
        tmp_paths_.push_back(path);
        dst_paths_.push_back(path);

        LinePath detect_path;
        detect_path = makeLine(detect_start_pose_.x(), detect_start_pose_.y(), detect_end_pose_.x(), detect_end_pose_.y(), PATH_BACKWARD);
        tmp_paths_.push_back(detect_path);
        detect_path.radius_ = -2 ;  //停止标志
        dst_paths_.push_back(detect_path);

    }
    sros::core::NavigationPath_vector paths;
    genLinePath(paths);
     for (int i = 0; i < paths.size(); i++) {
        tmp_paths_.push_back(paths[i]);
        dst_paths_.push_back(paths[i]);
    }
}

void Action172Faster::starFirstMoveTask() {

    //关闭后侧避障雷达，打开R200滤波
    enableBackLidar(false);
    generatePaths();
    //调试
    debugOuputPaths(dst_paths_);
    mc_stage_ = L1ING;
    //向SRC发送移动对接路径
    sendMoveTask(dst_paths_);
}

void Action172Faster::doCancel() {
    mc_stage_ = MC_FAILED; //小车移动到第几阶段
    fork_step_ = AC_FAILED;
    src_sdk->cancelAction(src_action_no_);
}

void Action172Faster::onSrcAcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcAcFinishSucceed" ;  
    //防止多个AC动作时，某个结束后一直触发此函数
    src_action_no_ = CalcSrcActionNo();
	detectGoodsRacks();
}

void Action172Faster::onSrcAcFinishFailed(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    fork_step_ = AC_FAILED;
}

void Action172Faster::onSrcAcFinishCanceled(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    mc_stage_ = MC_FAILED;
}

void Action172Faster::onSrcMcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcMcFinishSucceed";
    doActionFinishSucceed();
}

void Action172Faster::onSrcMcFinishFailed(int result_value) {
     LOG(ERROR) << "卸货路径导航失败, result_value:" << result_value;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_UNLOAD_MOVE_NOT_REACH);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_UNLOAD_MOVE_NOT_REACH);
    enableBackLidar(true);
    mc_stage_ = MC_FAILED;
}

void Action172Faster::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {

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
    mc_stage_ = L3ING;
    //向SRC发送移动对接路径
    sendReplaceTask(tmp_paths_);
}

void Action172Faster::genLinePath(sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "genLinePath";

    double goods_len = calcGoodsLength(g_state.cur_goal_id);
    double fork_end_coordinate = calcForkEndCoordinateLength();

    //计算最终移动点偏移
    double deviation_len = (goods_len / 2) - fork_end_coordinate;

    //计算最终移动点位姿
    Pose fin_pose;
    calcDstPose(dst_pose_, fin_pose, deviation_len);
    LOG(INFO) << "fin_pose, [x: " << fin_pose.x() << "][y: " << fin_pose.y() << "][yaw: " << fin_pose.yaw() << "]";

    // auto curr_pose = src_sdk->getCurPose();
    LinePath p;
    p = makeLine(detect_end_pose_.x(), detect_end_pose_.y(), fin_pose.x(), fin_pose.y(), PATH_BACKWARD);
    dst_paths.push_back(p);
    genStartRotatePath(dst_paths, detect_end_pose_.yaw());
}

void Action172Faster::detectGoodsRacks() {
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

bool Action172Faster::onSubSrcActionCheck(uint32_t& sub_src_action_no) {
    sub_src_action_no = src_action_no_;
    return true;
}

//第一次调用返回action_no_
uint32_t Action172Faster::CalcSrcActionNo() {
    int no = action_no_ - count_;
    count_++;
    return no ;
}

}
