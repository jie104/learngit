//
// Created by caoyan on 4/1/2021.
//

#ifndef SROS_ACTION_161_H
#define SROS_ACTION_161_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_LOAD_SEARCH_GOOGS = 161,   //取货前寻货动作
class Action161 : public BaseAction {
 public:
    Action161(): BaseAction(ACTION_ID_LOAD_SEARCH_GOOGS) {}
    virtual ~Action161() {}

    void doStart() override;

    void doCancel() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;
    void onSrcAcFinishCanceled(int result_value) override;

    void onSrcMcFinishSucceed(int result_value) override;
    void onSrcMcFinishFailed(int result_value) override;

    bool onSubSrcActionCheck(uint32_t& sub_src_action_no) override;

 private:
    //void getGoodsRfid();

    //生成前往对接点路径
    bool genGoToDockingPosePath(sros::core::NavigationPath_vector& dst_paths);

 private:
    //std::string rfid_data_;

    bool is_forklift_running_ = false;
    //bool is_get_rfid_running_ = false;

     bool is_paths_running_ = false;

    bool is_keep_recv_src_ret_ = true;

    //站点位姿（货物中心点）
    Pose dst_pose_;
};


}


#endif  // SROS_ACTION_161_H
