//
// Created by lbx on 2021/12/15
//

#ifndef SROS_ACTION_162_LOW_FASTER_H
#define SROS_ACTION_162_LOW_FASTER_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//不停车取货对接动作
class Action162LowFaster : public BaseAction {
public:

   enum Step {
      NONE,
      BEZIER,
      AC_RUNING,
      DETECTING,
      GOBACK,
      CANCLING,         //取消中
      FAILED
   };
   Action162LowFaster() : BaseAction(ACTION_ID_LOAD_MOVE_DOCKING) {}
   virtual ~Action162LowFaster() {}

   void doStart() override;

   void doCancel() override;

   void onSrcAcFinishSucceed(int result_value) override;
   void onSrcAcFinishFailed(int result_value) override;
   void onSrcAcFinishCanceled(int result_value) override;

   void onSrcMcFinishSucceed(int result_value) override;
   void onSrcMcFinishFailed(int result_value) override;

   void onAlgoResultCallback(const sros::core::base_msg_ptr& msg) override;
   void onNavResultCallback(const sros::core::base_msg_ptr& msg) override;
   bool onSubSrcActionCheck(uint32_t& sub_src_action_no) override;

 private:
   void calculatePoint();
   void sendGeneratePathsCmd();
   void starFirstMoveTask(const sros::core::NavigationPath_vector& dst_paths);
   void detectDst3DPose();
   void checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust);
   void launchMoveTask(bool is_one_line);
   void genMovePath(sros::core::NavigationPath_vector& dst_paths);
   void genLinePath(sros::core::NavigationPath_vector& dst_paths);
   uint32_t CalcSrcActionNo();

private:
   //推算点位
   Pose bezier_end_pose_;
   Pose final_pose_;

   NavigationPath_vector dst_paths_;

   Step step_ = NONE;
   
   // 3d视觉检测到位姿
   Pose dst_3d_pose_;  // x: m, y: m, yaw: 弧度
   Pose dst_3d_agv_pose_;
   //站点位姿（货物中心点）
   Pose dst_pose_;

   //当前的货物id
   int goal_id_;
   double offset_x_real_sub_theory_;

   uint32_t src_action_no_;
   int count_ = 0;
};

}

#endif  // SROS_ACTION_162_LOW_FASTER_H
