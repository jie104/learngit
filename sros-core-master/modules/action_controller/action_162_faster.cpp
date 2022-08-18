
#include "action_162_faster.h"
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

void Action162Faster::doStart() {


    src_action_no_ = CalcSrcActionNo();
    //参数读取
    auto& s = sros::core::Settings::getInstance();
    limit_height_ = s.getValue<int>("forklift.load_beizer_forkarm_height_limit", 2400);
    // limit_height_ = 2400;
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

void Action162Faster::sendGeneratePathsCmd(){
    // 发送导航命令到NavigationModule，获取到达物料点的路径
    Pose tmp_pose;  //空值
    auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("NAV_PATH_BUILD");
    mm->str0 = g_state.getCurMapName();
    mm->pose = tmp_pose;
    mm->param0 = action_param0_; //站点id
    mm->param1 = ObstacleAvoidPolicy::OBSTACLE_AVOID_WAIT;
    mm->param_boolean = false;
    sros::core::MsgBus::sendMsg(mm);
}

void Action162Faster::onNavResultCallback(const sros::core::base_msg_ptr& msg) {

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
        // //停车检测，设置bezier终点停止
        // if(path.type_ == PATH_BEZIER){
        //     if(detect_distance_ == 0){
        //         path.radius_ = -2 ; //停止标志
        //     }
        // }
        
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
void Action162Faster::calculatePoint(){

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
    LOG(INFO) << "action_param1_ " << action_param1_;
    LOG(INFO) << "limit_height_ " << limit_height_;
    if(action_param1_ > limit_height_){
        height = limit_height_; //限定高度
        next_step = UPING_TO_LIMIT;
        LOG(INFO) << "UPING_TO_LIMIT ";
    }else{
        height = action_param1_; //目标高度
        next_step = UPING_TO_TARGET;
        LOG(INFO) << "UPING_TO_TARGET ";
    }

    starFirstMoveTask();
    std::thread thread2(&Action162Faster::waitPoseInPlace, this);
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
    // fork_step_ = DOWNING;
    std::thread thread1(&Action162Faster::waitFrokInPlace, this);
    thread1.detach();
}

//当前位置检测7
void Action162Faster::waitPoseInPlace(){

    //L3+L2
    len_ = distanceTwoPose(dst_pose_,detect_start_pose_) * 1000;
    LOG(INFO) << "len_: " << len_;

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
                src_sdk->setSpeedLevel(30);
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
                bool enable_load_detect = (s.getValue<string>("main.enable_load_detect", "True") == "True");
                if(enable_load_detect) {
                    detectDst3DPose();
                }else {
                    launchMoveTask(true); 
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
    LOG(ERROR) << "waitPoseInPlace, exit thread";

}
//叉臂到位信号检测
void Action162Faster::waitFrokInPlace() {
            
    while (fork_step_ <= UPING_TO_TARGET ) {
        // LOG(INFO) << "fork_step_:" << fork_step_;
        // LOG(INFO) << "current height:" << g_state.fork_height_encoder;
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
                        detectDst3DPose();
                    }else {
                        launchMoveTask(true); 
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
void Action162Faster::generatePaths(){

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
}

void Action162Faster::starFirstMoveTask() {

    //关闭后侧避障雷达，打开R200滤波
    enableBackLidar(false);

    generatePaths();

    //调试
    debugOuputPaths(dst_paths_);

    mc_stage_ = L1ING;

    //向SRC发送移动对接路径
    sendMoveTask(dst_paths_);

}

void Action162Faster::doCancel() {
    mc_stage_ = MC_FAILED; //小车移动到第几阶段
    fork_step_ = AC_FAILED;
    src_sdk->cancelAction(src_action_no_);
}

void Action162Faster::onSrcAcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcAcFinishSucceed" ;  
    //防止多个AC动作时，某个结束后一直触发此函数
    src_action_no_ = CalcSrcActionNo();

    if(fork_step_ == UPING_TO_TARGET){
        fork_step_ = UP_FINISH;
        return ;
    }else if(fork_step_ == MC_RUNING){
        enableBackLidar(true);
        // 检测到位信号是否触发，判断进叉是否到位
        if(!checkPalletInPlaceSignal()) {
            //防止状态未及时更新到retry
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
            if(!checkPalletInPlaceSignal()) {
                LOG(ERROR) << "checkPalletInPlaceSignal: false";
                sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
                doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
                // enableBackLidar(true);
                mc_stage_ = MC_FAILED;
                get_rfid_data_running_ = false;
                return;
            }
        }

        //rfid比较
        if (get_rfid_data_running_) {
            get_rfid_data_running_ = false;

            //Matrix配置增加“rfid请求epc模式“项（SS_MODE 同步读 ES_MODE 异步读）
            auto &s = sros::core::Settings::getInstance();
            std::string rfid_reqepc_mode = s.getValue<std::string>("device.rfid_reqepc_mode", "SS_MODE");

            if (rfid_reqepc_mode == "ES_MODE") {
                get_rfid_data_ = RfidManager::getInstance()->asyncGetRfidData();
            }

            if(get_rfid_data_.empty()) {
                doActionFinishFailed(sros::core::ERROR_CODE_RFID_GET_NONE);
                mc_stage_ = MC_FAILED;
                return;
            } else if (action_param_str_ != get_rfid_data_) {
                LOGGER(ERROR, ACTION_TASK) << "match fail, rfid(" << get_rfid_data_ << ") != param_str(" << action_param_str_ << ")";
                action_task_->setResultValueStr(get_rfid_data_);
                mc_stage_ = MC_FAILED;
                doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                return;
            }
        }

        LOG(INFO) << "166 action finish!" ;
        //成功返回
        doActionFinishSucceed();
    }
}

void Action162Faster::onSrcAcFinishFailed(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    fork_step_ = AC_FAILED;
}

void Action162Faster::onSrcAcFinishCanceled(int result_value) {
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    mc_stage_ = MC_FAILED;
}

void Action162Faster::onSrcMcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcMcFinishSucceed";
}

void Action162Faster::onSrcMcFinishFailed(int result_value) {
    LOG(ERROR) << "货叉取货路径前往对接失败, result_value" << result_value;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);
    mc_stage_ = MC_FAILED;
    enableBackLidar(true);
}

void Action162Faster::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {
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
        mc_stage_ = MC_FAILED; //小车移动到第几阶段
        fork_step_ = AC_FAILED;
        src_sdk->cancelAction(src_action_no_);

        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_NOT_DETECTED);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_NOT_DETECTED);
        mc_stage_ = MC_FAILED;

        get_rfid_data_running_ = false;
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
    // double offset_y = std::fabs(mm->goal_in_agv_pose.y());
    // double offset_yaw = std::fabs(mm->goal_in_agv_pose.yaw());
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

void Action162Faster::checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust) {

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
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_YAW_DEVIATE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_YAW_DEVIATE);
        get_rfid_data_running_ = false;
        mc_stage_ = MC_FAILED;
        return;
    }
    if (offset_y > deviate_max_y) {
        LOG(ERROR) << "offset_y(" << offset_y << ") > max_y(" << deviate_max_y << ")";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_Y_DEVIATE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_Y_DEVIATE);
        get_rfid_data_running_ = false;
        mc_stage_ = MC_FAILED;
        return;
    }
    if (offset_x < deviate_distance_x_forward) {
        LOG(ERROR) << "offset_x(" << offset_x << ") < forward(" << deviate_distance_x_forward << ")";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_X_DEVIATE_FORWARD);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_FORWARD);
        get_rfid_data_running_ = false;
        mc_stage_ = MC_FAILED;
        return;
    }
    if (offset_x > deviate_distance_x_back) {
        LOG(ERROR) << "offset_x(" << offset_x << ") > back(" << deviate_distance_x_back << ")";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_X_DEVIATE_BACK);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_BACK); 
        get_rfid_data_running_ = false;
        mc_stage_ = MC_FAILED;
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
        get_rfid_data_running_ = false;
        return;
    }

    // //若只测试算法，则到此结束(用于校验和常规162的数据差异)
    // mc_stage_ = MC_FAILED; //小车移动到第几阶段
    // fork_step_ = AC_FAILED;
    // src_sdk->cancelAction(action_no_); //此处需要使用启动移动任务时的id
    // doActionFinishSucceed();
    // return ;

    launchMoveTask(is_one_line); 

}

void Action162Faster::launchMoveTask(bool is_one_line) {

    //生成移动对接路径
    // sros::core::NavigationPath_vector dst_paths;

    if(is_one_line) {
        genLinePath(tmp_paths_);
    } else {
        g_state.is_fork_pose_adjust = true;
        genMovePath(tmp_paths_);
    }

    //原有路径
    debugOuputPaths(dst_paths_);

    //替换后路径
    debugOuputPaths(tmp_paths_);

    // 向SRC发送动作指令，上货路径监控动作
    src_sdk->executeAction(src_action_no_, 24, 30, 0);
    LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
              << "id 24, p0 30, p1 0";
    fork_step_ = MC_RUNING;

    //向SRC发送移动对接路径
    sendReplaceTask(tmp_paths_);

    //记录当前的货物id
    g_state.cur_goal_id = goal_id_;
}

void Action162Faster::genLinePath(sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "genLinePath";

    double goods_len = calcGoodsLength(goal_id_);
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
    // genStartRotatePath(dst_paths, curr_pose.yaw());
}

void Action162Faster::genMovePath(sros::core::NavigationPath_vector& old_paths) {
    LOG(INFO) << "genMovePath";
    sros::core::NavigationPath_vector paths;
    double adjust_pose_optimal_distance = calcAdjustPoseOptimalDistance();

    //计算中间点位姿（第一次调整的最佳位姿）
    Pose mid_pose;
    calcDstPose(dst_3d_pose_, mid_pose, adjust_pose_optimal_distance);
    LOG(INFO) << "mid_pose, [x: " << mid_pose.x() << "][y: " << mid_pose.y() << "][yaw: " << mid_pose.yaw() << "]";

    //怀疑bezier位置推算有问题，后续验证
    //计算最终移动点位姿
    double fork_end_coordinate = calcForkEndCoordinateLength();
    //计算最终移动点偏移
    double deviation_len = 0 - fork_end_coordinate;

    Pose fin_pose;
    calcDstPose(dst_3d_pose_, fin_pose, deviation_len);
    LOG(INFO) << "fin_pose, [x: " << fin_pose.x() << "][y: " << fin_pose.y() << "][yaw: " << fin_pose.yaw() << "]";
    
    //根据检测返回的目标点位姿生成路径
    // auto curr_pose = src_sdk->getCurPose();
    // LOG(INFO) << "curr_pose, [x: " << curr_pose.x() << "][y: " << curr_pose.y() << "][yaw: " << curr_pose.yaw() << "]";
    
    BezierPath bez = genBezierPath(detect_end_pose_, mid_pose, PATH_BACKWARD);
    bez.updateFacing();
    paths.push_back(bez);

    LinePath p;
    //p = makeLine(curr_pose.x(), curr_pose.y(), mid_pose.x(), mid_pose.y(), PATH_BACKWARD);
    //dst_paths.push_back(p);
    p = makeLine(mid_pose.x(), mid_pose.y(), fin_pose.x(), fin_pose.y(), PATH_BACKWARD);
    paths.push_back(p);
    genRotateBetweenPaths(paths);
    genStartRotatePath(paths, detect_end_pose_.yaw());

    for (int i = 0; i < paths.size(); i++) {
        old_paths.push_back(paths[i]);
    }

    // old_paths.push_back(dst_paths);
}

void Action162Faster::detectDst3DPose() {

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

bool Action162Faster::onSubSrcActionCheck(uint32_t& sub_src_action_no) {
    sub_src_action_no = src_action_no_;
    return true;
}

//第一次调用返回action_no_
uint32_t Action162Faster::CalcSrcActionNo() {
    int no = action_no_ - count_;
    count_++;
    return no ;
}

}
