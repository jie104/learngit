//
// Created by caoyan on 4/10/21.
//

#ifndef SROS_ACTION_172_H
#define SROS_ACTION_172_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_UNLOAD_PICKDOWN_GOODS = 172,  //卸货动作
class Action172 : public BaseAction {
 public:
    Action172() : BaseAction(ACTION_ID_UNLOAD_PICKDOWN_GOODS) {}
    virtual ~Action172() {}

    void doStart() override;
    void doCancel() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;
    void onSrcAcFinishCanceled(int result_value) override;

    void onSrcMcFinishSucceed(int result_value) override;
    void onSrcMcFinishFailed(int result_value) override;

 private:
    void genLinePath(sros::core::NavigationPath_vector& dst_paths);
    void sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type,
                              AvdObaCommandMsg::DetectAndAvdObaState state, 
                              Pose pose, NavigationPath_vector paths);
    void sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type);

 private:
    //站点位姿
    Pose dst_pose_;

    bool is_forklift_running_ = false;

    bool enable_action_check_pallet_signal_ = false;
    bool enable_fetch_release_goods_avd = false;
};

}


#endif  // SROS_ACTION_172_H
