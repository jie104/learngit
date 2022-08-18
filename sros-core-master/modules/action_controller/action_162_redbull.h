//
// Created by caoyan on 4/2/21.
//

#ifndef SROS_ACTION_162_REDBULL_H
#define SROS_ACTION_162_REDBULL_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_LOAD_MOVE_DOCKING = 162,   //取货前移动对接动作
class Action162Redbull : public BaseAction {
public:
   enum State {
      NONE,
      AC_RUNING,
      MC_RUNING,
      CANCLING,         //取消中
      FAILED
   };
   Action162Redbull() : BaseAction(ACTION_ID_LOAD_MOVE_DOCKING) {}
   virtual ~Action162Redbull() {}

   void doStart() override;

   void doCancel() override;

   void onSrcAcFinishSucceed(int result_value) override;
   void onSrcAcFinishFailed(int result_value) override;
   void onSrcAcFinishCanceled(int result_value) override;

   void onAlgoResultCallback(const sros::core::base_msg_ptr& msg) override;

   void onSrcMcFinishSucceed(int result_value) override;
   void onSrcMcFinishFailed(int result_value) override;
   bool onSubSrcActionCheck(uint32_t& sub_src_action_no) override;

private:
   void detectDst3DPose();
   void checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust);
   void genMovePath(sros::core::NavigationPath_vector& dst_paths);
   void genLinePath(sros::core::NavigationPath_vector& dst_paths);

   void launchMoveTask(bool is_one_line);

   void retryJudeg();
   uint32_t CalcSrcActionNo();

private:
   int count_ = 0;
   uint32_t last_src_action_no_;
   uint32_t src_action_no_;
   // 3d视觉检测到位姿
   Pose dst_3d_pose_;  // x: m, y: m, yaw: 弧度

   Pose dst_3d_agv_pose_;
   //站点位姿（货物中心点）
   Pose dst_pose_;

   int detect_height_;

   //当前的货物id
   int goal_id_;
   State state_;

   bool is_paths_monitor_running_ = true;

   double offset_x_real_sub_theory_;

   std::string get_rfid_data_;

   bool get_rfid_data_running_ = false;
   bool is_one_line_ = true;
};

}

#endif  // SROS_ACTION_162_REDBULL_H
