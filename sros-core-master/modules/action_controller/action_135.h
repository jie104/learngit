//
// Created by caoyan on 7/1/21.
//

#ifndef SROS_ACTION_135_H
#define SROS_ACTION_135_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_TRACTOR_DOCKING = 135,    //二维码牵引对接
class Action135 : public BaseAction {
 public:
    Action135(): BaseAction(ACTION_ID_TRACTOR_DOCKING) {}
    virtual ~Action135() {}

    void doStart() override;
    void doCancel() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;
    void onSrcAcFinishCanceled(int result_value) override;
    void sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type,
                              const std::string& code_str, Pose& pose);

    void onAlgoResultCallback(const sros::core::base_msg_ptr& msg) override;

 private:
    void saveCameraImg(bool enable);
    void enableAlgTractorDocking(bool enable);
    uint16_t setBit(uint16_t val, int bits);
    uint16_t resetBit(uint16_t val, int bits);

 private:
   bool is_src_tractor_docking_running_ = false;
   double tractor_docking_deviate_distance_;
   //站点位姿（货物中心点）
   Pose dst_pose_;

   // 二維碼對應的ID
   std::string code_str;

   // 载货模型使能
   bool enable_change_avd_model = false;
   int bits_; //补光灯的io位

};


}


#endif  // SROS_ACTION_135_H
