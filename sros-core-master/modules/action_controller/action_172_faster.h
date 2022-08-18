//
// Created by lbx on 2021/3/1
//

#ifndef SROS_ACTION_172_FASTER_H
#define SROS_ACTION_172_FASTER_H

#include "base_action.h"

using namespace std;
using namespace sros::core;



namespace ac {

//不停车取货对接动作
class Action172Faster : public BaseAction {
public:
    enum MoveState {
      L1ING = 0, //bezier
      L2ING,   //detect
      L2_FINISH,
      L3ING,
      L3_FINISH,
      MC_CANCLING,
      MC_FAILED
   };
   enum ForkStep {
      UPING_TO_LIMIT,
      UPING_TO_LIMIT_FINISHH,
      UPING_TO_TARGET,
      UP_FINISH,
      AC_CANCLING,         //取消中
      AC_FAILED
   };
   Action172Faster() : BaseAction(ACTION_ID_UNLOAD_PICKDOWN_GOODS) {}
   virtual ~Action172Faster() {}

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
   void detectGoodsRacks();
   void genLinePath(sros::core::NavigationPath_vector& dst_paths);

   void starFirstMoveTask();
   void generatePaths();

   uint32_t CalcSrcActionNo();

private:
   Pose bezier_end_pose_;  
   //推算点位
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

   //配置参数
   int limit_height_;  //mm

   uint32_t src_action_no_;
   int count_ = 0;

};

}

#endif  // SROS_ACTION_172_FASTER_H
