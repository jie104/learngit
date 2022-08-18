//
// Created by lbx on 2021/12/15
//

#ifndef SROS_ACTION_162_FASTER_H
#define SROS_ACTION_162_FASTER_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//不停车取货对接动作
class Action162Faster : public BaseAction {
public:
    enum MoveState {
      L1ING = 0, //bezier
      L2ING,   //detect
      L2_FINISH,
      MC_CANCLING,
      MC_FAILED
   };
   enum ForkStep {
      UPING_TO_LIMIT,
      UPING_TO_LIMIT_FINISHH,
      UPING_TO_TARGET,
      UP_FINISH,
      MC_RUNING,
      AC_CANCLING,         //取消中
      AC_FAILED
   };
   Action162Faster() : BaseAction(ACTION_ID_LOAD_MOVE_DOCKING) {}
   virtual ~Action162Faster() {}

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
   void waitPoseInPlace();
   void waitFrokInPlace();
   void calculatePoint();
   void sendGeneratePathsCmd();
   void detectDst3DPose();
   void checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust);
   void genMovePath(sros::core::NavigationPath_vector& dst_paths);
   void genLinePath(sros::core::NavigationPath_vector& dst_paths);

   void starFirstMoveTask();
   void launchMoveTask(bool is_one_line);
   void generatePaths();

   void retryJudeg();
   uint32_t CalcSrcActionNo();

private:
   //推算点位
   Pose bezier_end_pose_;  
   Pose detect_start_pose_;  
   Pose detect_end_pose_;
   Pose final_pose_;

   int target_height_;

   double len_; //c1到P的距离

   double detect_distance_;

   NavigationPath_vector tmp_paths_;
   NavigationPath_vector dst_paths_;

   MoveState mc_stage_ = L1ING; //小车移动到第几阶段
   ForkStep fork_step_ = UPING_TO_LIMIT;
   
   // 3d视觉检测到位姿
   Pose dst_3d_pose_;  // x: m, y: m, yaw: 弧度
   Pose dst_3d_agv_pose_;
   //站点位姿（货物中心点）
   Pose dst_pose_;

   //当前的货物id
   int goal_id_;

   double offset_x_real_sub_theory_;

   std::string get_rfid_data_;

   bool get_rfid_data_running_ = false;
   uint32_t src_action_no_;
   int count_ = 0;
   //配置参数
   int limit_height_;  //mm

};

}

#endif  // SROS_ACTION_162_FASTER_H
