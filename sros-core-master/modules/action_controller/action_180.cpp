
#include "action_180.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "core/fault_center.h"

using namespace std;
using namespace sros::core;

namespace ac {

//cmd：180, 1， 0/1   =》  使能钩子
//cmd：180, 0， x     =》  向前走x距离
void Action180::doStart() {
    if(action_param0_){ //使能钩子
        src_sdk->executeAction(action_no_, 24, 31, action_param1_);
        LOG(INFO) << "src_ac: no " << action_no_ << ", id 24, p0 31, p1 "<<action_param1_<<".";
    }else{  //往前走
        if(action_param1_ < 0 ) {
            src_sdk->executeAction(action_no_, 24, 31, 0);
            LOG(INFO) << "src_ac: no " << action_no_ << ", id 24, p0 31, p1 "<<action_param1_<<".";
        }
        src_sdk->executeAction(action_no_, 28, action_param1_, 0);
        LOG(INFO) << "src_ac: no " << action_no_ << ", id 28, p0 "<< action_param1_ <<", p1 0.";

        result_count_ = 0;
    }

}
void Action180::sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type) {
    AvdObaCommandMsg::Command command;
    command.avdoba_type = avdoba_type;
    command.oba_model.forklif_type =  AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_3000;
    command.oba_model.is_change_oba_model = false;
    command.oba_model.restore_oba_model = true;
    
    sendAvdObaCmd(action_no_, command);
}
void Action180::onSrcAcFinishSucceed(int result_value) {
    if(action_param1_ < 0 ) {
        result_count_++;
        LOG(INFO) << "result_count_:" << result_count_;
        if( result_count_ == 1){
            return ;
        }
    }
    doActionFinishSucceed();
    if(action_param0_ == 0 && action_param1_ > 0){
        // 脱钩就下发避障模型复原
        sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType::CHANGE_MDOEL_AVD); 
    }
}

void Action180::onSrcAcFinishFailed(int result_value) {
    LOG(INFO) << "action failed, result_value:" << result_value;  
    doActionFinishFailed(result_value);
    if(action_param0_){
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_HOOK_SWITCH);
    }else{
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_HOOK_OFF_FAILED);
    }
}

}
