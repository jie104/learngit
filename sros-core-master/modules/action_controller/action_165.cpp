
#include "action_165.h"
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

void Action165::doStart() {

    count_ = 0;
    first_ = true;
    goal_id_ = -1; 

    //清空货物的识别坐标
    Pose pose;
    g_state.goods_pose = pose;
    
    // 1.通过目标站点的id去获取目标点的位姿
    auto dst_station_id = action_param0_;
    auto dst_station = MapManager::getInstance()->getStation(dst_station_id);
    if (dst_station.id == 0) {  //这个逻辑很多地方用了，封装一下
        LOG(ERROR) << "not exist station id: " << dst_station_id;
        doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
        return;
    }

    dst_pose_.x() = dst_station.pos.x / 100.0;
    dst_pose_.y() = dst_station.pos.y / 100.0;
    dst_pose_.yaw() = normalizeYaw(dst_station.pos.yaw);

    LOG(INFO) << "dst_pose, [x: " << dst_pose_.x() << "][y: " << dst_pose_.y() << "][yaw: " << dst_pose_.yaw() << "]";

    int bottom_height = 100;
    //支持升降过程中升叉的指令
    src_action_no_ = CalcSrcActionNo();
    src_sdk->executeAction(src_action_no_, 24, 33, 0);
    LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
                << "id 24, p0 33, p1 0， bottom_height：" << bottom_height;
    std::vector<int> values{START, bottom_height}; //启动标志位，高度
    bool ret = src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET, values, 500);
    if(!ret){
        LOG(ERROR) << "setParameters failed!";
    }
    setState(DOWNING);
    //监测叉臂到位情况
    std::thread thread1(&Action165::waitFrokInPlace, this);
    thread1.detach();

}

void Action165::waitFrokInPlace() {
            
    while (cur_state_ <= UPING_TO_TARGET ) {

        // int inPlace_flag = 0;
        // if (!src_sdk->getParameter(0x1247, inPlace_flag)){
        //     LOG(WARNING) << "src getParameter timeout!";
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     continue;
        // }

        int inPlace_flag = 0;
        if (!src_sdk->getFunctionParam(sdk::SrcSdk::EM_FORKLIFTSTATE, inPlace_flag)){
            LOG(WARNING) << "src getParameter timeout!";
            continue;
        }

        if(inPlace_flag){  //到位
            LOG(INFO) << "In Place,current height:" << g_state.fork_height_encoder;
            if(cur_state_ == DOWNING){
                //pause
                std::vector<int> values{PAUSE, 0}; //暂停
                src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));//等待src的inPlace_flag切换Ok
                setState(DOWN_FINISH);
                enableCardDetect(true,PerceptionCommandMsg::ActionState::ACTION_START);
                continue ;
            }else if(cur_state_ == UPING_TO_TARGET){

                //临时方案，解决偶发读取到错误的到位信号问题，升级到18后可考虑删除
                auto& s = sros::core::Settings::getInstance();
                int deviate_fork_height = s.getValue<double>("forklift.deviate_fork_height", 0.03) * 1000;
                auto deviate_value = std::fabs(g_state.fork_height_encoder * 1000 - target_height_);

                if (deviate_value > deviate_fork_height) {  //实际并没有达到目标高度，错误的到位信号
                    LOG(INFO) << "g_state.fork_height_encoder: " << g_state.fork_height_encoder 
                                << ", dst_fork_height: " << target_height_ 
                                << ", deviate_fork_height: " << deviate_fork_height; 
                    LOG(ERROR) << "big difference in height";
                    continue;
                } 

                //叉臂抬升到顶部栈板位置，移动进叉
                std::vector<int> values{STOP, 0}; //结束
                src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
                setState(UP_FINISH);
                
                //校验误差
                if (checkForkHeightFautl(target_height_ / 1000.0)) {
                    return;
                }
                LOG(INFO) << "fork lift finish， exit thread！";
                return ;
                
            }else if(cur_state_ == UPING_TO_MAX){
                //stop
                std::vector<int> values{STOP, 0}; //结束
                src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
                setState(FAILED);
                enableCardDetect(false,PerceptionCommandMsg::ActionState::ACTION_FAIL);
                LOG(INFO) << " exit thread！";
                return ;
            }else if(cur_state_ == DOWN_FINISH || cur_state_ == STOPING){
                //nothing
            }
            else{
                //改变标志位结束动作，
                std::vector<int> values{STOP, 0}; //结束
                src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
                LOG(INFO) << "unkown cur_state_:"<<cur_state_<<", exit thread！";
                setState(FAILED);
                enableCardDetect(false,PerceptionCommandMsg::ActionState::ACTION_FAIL);
                doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);

                return ;
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::vector<int> values{STOP, 0};
    src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values); //确保正常结束掉24,33动作
    LOG(ERROR) << "waitFrokInPlace fail, exit thread";
 };

void Action165::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {

    auto mm = dynamic_pointer_cast<PerceptionStateMsg>(msg);
    int detect_result = mm->detect_result;
    int detect_height = mm->goal_in_global_pose.z() * 1000;
    goal_id_ = mm->goal_id;
    LOG(INFO) << "detect_height_: " << detect_height;
    // auto& s = sros::core::Settings::getInstance();
    // float deviate_fork_height = s.getValue<double>("forklift.deviate_fork_height", 0.03);
    if( detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_INTERMEDIATE_STATE_ONE ){
        if(!first_)
            return ;
        auto& s = sros::core::Settings::getInstance();
        int max_height = s.getValue<double>("forklift.detect_decards_top_fork_height", 1.4) * 1000;
        
        LOG(INFO) << "max_height" << max_height;
        std::vector<int> values{START, max_height}; //启动标志位，高度
        src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
        setState(UPING_TO_MAX);
        LOG(INFO) << "src_ac: no " << action_no_ << ", "
                << "UPING_TO_MAX, max_height:" << max_height;
        first_ = false;

    }else if( detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_INTERMEDIATE_STATE_TWO ){
        //暂停升叉，继续扫码
        std::vector<int> val{PAUSE, 0}; //暂停
        src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,val);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        setState(STOPING);
        LOG(INFO) << "stop  forklft";
    }else if ( detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS ) {
        LOG(INFO) << "DETECT_RESULT_SUCCESS！";
        //target_height_ = detect_height - deviate_fork_height*1000; 
        target_height_ = detect_height; 
        // if(cur_state_ == STOPING){    //第一次检测就扫到了顶部
        //     std::vector<int> values{START, target_height_}; //启动标志位，高度
        //     src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
        //     setState(UPING_TO_TARGET);
        //     LOG(INFO) << "src_ac: no " << action_no_ << ", "
        //             << "UPING_TO_TARGET, target_height_:" << target_height_ <<"goal_id_" <<goal_id_;
        // }else if(cur_state_ == UPING_TO_MAX){   //
        //     std::vector<int> val{PAUSE, 0}; //暂停
        //     src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,val);

        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        //     std::vector<int> values{START, target_height_}; //调整到目标高度
        //     src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
        //     setState(UPING_TO_TARGET);
        //     LOG(INFO) << "src_ac: no " << action_no_ << ", "
        //             << "UPING_TO_TARGET, target_height_:" << target_height_ << ",current height:" << g_state.fork_height_encoder;
        // }else{
        //      LOG(ERROR) << "unkonwn cur_state_:"<<cur_state_;
        // }

        dst_3d_pose_.x() = mm->goal_in_global_pose.x();
        dst_3d_pose_.y() = mm->goal_in_global_pose.y();
        dst_3d_pose_.yaw() = mm->goal_in_global_pose.yaw();

        dst_3d_agv_pose_.x() = mm->goal_in_agv_pose.x();
        dst_3d_agv_pose_.y() = mm->goal_in_agv_pose.y();
        dst_3d_agv_pose_.yaw() = mm->goal_in_agv_pose.yaw();

        LOG(INFO) << "vision [dst_3d__global_pose x: " << dst_3d_pose_.x() << " y: " << dst_3d_pose_.y() << " yaw: " << dst_3d_pose_.yaw()
                << "] [goal_in_agv_pose x: " << dst_3d_agv_pose_.x() << " y: " << dst_3d_agv_pose_.y() << " yaw: " << dst_3d_agv_pose_.yaw()
                << "] [z: " << detect_height << " goal_id_: " << goal_id_ << "]";

        g_state.goods_pose = mm->goal_in_agv_pose;

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

        double offset_x_real_sub_theory_ = len_agv_to_real - len_agv_to_theory;


        //计算相机方向下Y和YAW的误差
        double offset_y = std::fabs(mm->goal_in_agv_pose.y());
        double offset_yaw = std::fabs(mm->goal_in_agv_pose.yaw());

        LOG(INFO) << "offset, [x: " << offset_x_real_sub_theory_ << "][y: " << offset_y << "][yaw: " << offset_yaw << "]";

        auto& s = sros::core::Settings::getInstance();
        double auto_adjust_need_distance = s.getValue<double>("forklift.auto_adjust_need_distance", 0.5);
        double bezier_adjust_distance = s.getValue<double>("forklift.bezier_adjust_distance", 0.3);
        bool enough_adjust = (std::fabs(dst_3d_agv_pose_.x()) > (auto_adjust_need_distance + bezier_adjust_distance));

        //对检测后的位姿进行校验
        checkDst3DPose(offset_x_real_sub_theory_, offset_y, offset_yaw, enough_adjust);


    }else{
        setState(FAILED);
        LOG(ERROR) << "detect error_code:" << mm->error_code;
        //定义错误码，要根据算法的错误码进行映射
        enableCardDetect(false,PerceptionCommandMsg::ActionState::ACTION_FAIL);
        doActionFinishFailed(sros::core::ERROR_CODE_TARGET_DETECT_OVERTIME);
    }

}

void Action165::doCancel() {
    enableCardDetect(false,PerceptionCommandMsg::ActionState::ACTION_ABORT);
    src_sdk->cancelAction(src_action_no_);
    setState(CANCLING);
}

void Action165::enableCardDetect(bool enable, PerceptionCommandMsg::ActionState state) {

    PerceptionCommandMsg::Command cmd;
    cmd.detect_stage = PerceptionCommandMsg::DetectStage::DETECT_STAGE_LOAD;
    cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_DECARDS;
    cmd.action_state = state;
    cmd.is_enable = enable;

    sendPerceptionCmd(action_no_, cmd, goal_id_, dst_pose_);
}

void Action165::onSrcAcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcAcFinishSucceed" ;
    LOG(INFO) << "current height:" << g_state.fork_height_encoder;
    src_action_no_ = CalcSrcActionNo();
    if(cur_state_ == MC_RUNING){
        int height = g_state.fork_height_encoder * 1000 + 100;
        src_sdk->executeAction(src_action_no_, 24, 3, height);
        LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
                  << "id 24, p0 3, p1 " << height;
        setState(LOAD_UPING);
        return ;
    }else if(cur_state_ == UP_FINISH){
        launchMoveTask(is_one_line_);
        return ;
    }else if(cur_state_ == FAILED){ //升到最高没有识别到
        enableCardDetect(false,PerceptionCommandMsg::ActionState::ACTION_FAIL);
        doActionFinishFailed(sros::core::ERROR_CODE_TARGET_DETECT_OVERTIME);
        return ;
    }

    //取完最后一个栈板给中控返回:  id@1 ,未取完 id@0
    int flag = 0;   //0 未取完，1 已取完
    if(target_height_ < 170)    //目前采用高度判别，
        flag = 1;
    string result = to_string(action_param0_) + string("@") + to_string(flag);
    LOG(INFO) << "result:" << result;
    action_task_->setResultValueStr(result);

    enableBackLidar(true);
    doActionFinishSucceed();
    
    return;
}

void Action165::onSrcAcFinishFailed(int result_value) {
    enableCardDetect(false,PerceptionCommandMsg::ActionState::ACTION_FAIL);
    if((cur_state_ == DOWN_FINISH) && result_value == 2482 ){  //感知超过15秒无结果返回导致叉臂动作超时
        setState(FAILED);
        doActionFinishFailed(sros::core::ERROR_CODE_TARGET_DETECT_OVERTIME);
    } else if(result_value == 2498) {
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
    }else {
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    }
    setState(FAILED);
    LOG(INFO) << "current height:" << g_state.fork_height_encoder;
}

void Action165::onSrcAcFinishCanceled(int result_value) {
    enableCardDetect(false,PerceptionCommandMsg::ActionState::ACTION_ABORT);
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    setState(FAILED);
}

void Action165::onSrcMcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcMcFinishSucceed";
}

void Action165::onSrcMcFinishFailed(int result_value) {
    LOG(ERROR) << "货叉取货路径前往对接失败, result_value" << result_value;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
    enableCardDetect(false,PerceptionCommandMsg::ActionState::ACTION_FAIL);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);
    setState(FAILED);
    enableBackLidar(true);
}

void Action165::checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust) {

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
        setState(FAILED);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_YAW_DEVIATE);
        return;
    }
    if (offset_y > deviate_max_y) {
        LOG(ERROR) << "offset_y(" << offset_y << ") > max_y(" << deviate_max_y << ")";
        setState(FAILED);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_Y_DEVIATE);
        return;
    }
    if (offset_x < deviate_distance_x_forward) {
        LOG(ERROR) << "offset_x(" << offset_x << ") < forward(" << deviate_distance_x_forward << ")";
        setState(FAILED);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_FORWARD);
        return;
    }
    if (offset_x > deviate_distance_x_back) {
        LOG(ERROR) << "offset_x(" << offset_x << ") > back(" << deviate_distance_x_back << ")";
        setState(FAILED);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_BACK);
        return;
    }

    double deviate_min_y = s.getValue<double>("forklift.deviate_min_y", 0.01);
    double deviate_min_yaw = s.getValue<double>("forklift.deviate_min_yaw", 1.0);
    deviate_min_yaw = deviate_min_yaw / 180 * M_PI;

    LOG(INFO) << "min_y: " << deviate_min_y << ", min_yaw: " << deviate_min_yaw;

    is_one_line_ = (offset_y < deviate_min_y && offset_yaw < deviate_min_yaw);

    //角度需要调整，但是调整距离不够直接报错
    if ( !is_one_line_ && !enough_adjust) {
        LOG(ERROR) << "需要自适应调整，但是调整距离不够";
        setState(FAILED);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_ADJUST_LEN_NOT_ENOUGH);
        return;
    }

    std::vector<int> values{START, target_height_}; //调整到目标高度
    src_sdk->setFunctionParam(sdk::SrcSdk::EM_REMOVEPALLET,values);
    setState(UPING_TO_TARGET);
    LOG(INFO) << "src_ac: no " << action_no_ << ", "
            << "UPING_TO_TARGET, target_height_:" << target_height_ << ",current height:" << g_state.fork_height_encoder;

}

void Action165::launchMoveTask(bool is_one_line) {
 
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
 
    setState(MC_RUNING);

    //向SRC发送移动对接路径
    sendMoveTask(dst_paths);

    //记录当前的货物id
    g_state.cur_goal_id = goal_id_;
}

void Action165::genLinePath(sros::core::NavigationPath_vector& dst_paths) {
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

void Action165::genMovePath(sros::core::NavigationPath_vector& dst_paths) {
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

uint32_t Action165::CalcSrcActionNo() {
    int no = action_no_ - count_;
    count_++;
    if( no == 0) {
        no = 65535;
    }
    return no ;
}

//src会一直上传上一个ac_no的结果，检查只处理当前src_action_no_的结果
bool Action165::onSubSrcActionCheck(uint32_t& sub_src_action_no) {
    sub_src_action_no = src_action_no_;
    return true;
}

string Action165::state2Str (State state) {

    static std::map<int,std::string> state_map;
    state_map[State::NONE] = "NONE";
    state_map[State::DOWNING] = "DOWNING";
    state_map[State::DOWN_FINISH] = "DOWN_FINISH";
    state_map[State::STOPING] = "STOPING";
    state_map[State::UPING_TO_MAX] = "UPING_TO_MAX";
    state_map[State::UPING_TO_TARGET] = "UPING_TO_TARGET";
    state_map[State::UP_FINISH] = "UP_FINISH";
    state_map[State::MC_RUNING] = "MC_RUNING";
    state_map[State::LOAD_UPING] = "LOAD_UPING";
    state_map[State::LOAD_UP_FINISH] = "LOAD_UP_FINISH";
    state_map[State::CANCLING] = "CANCLINFAILEDG";
    state_map[State::FAILED] = "FAILED";

    if((state < State::NONE) || (state > State::FAILED)){
        LOG(WARNING)<<" unkonw enum type:" << state;
        return  string("unkonw"); //返回值待定
    }
    return state_map[state];
};

void Action165::setState(State state) {
    if(cur_state_ != state){
        LOG(INFO) << "state change:" << state2Str(cur_state_) << " => " << state2Str(state);
        cur_state_ = state;
    }
}

}
