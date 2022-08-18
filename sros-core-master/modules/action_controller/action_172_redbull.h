//
// Created by caoyan on 4/10/21.
//

#ifndef SROS_ACTION_172_REDBULL_H
#define SROS_ACTION_172_REDBULL_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

#define STR_UNLOAD_LASER_NONE "UNLOAD_LASER_NONE"
#define STR_UNLOAD_LASER_HAVE "UNLOAD_LASER_HAVE"

namespace ac {

// ACTION_ID_UNLOAD_PICKDOWN_GOODS = 172,  //卸货动作
class Action172RedBull : public BaseAction {
public:
   enum State {
      NONE,
      AC_RUNING,
      MC_RUNING,
      CANCLING,         //取消中
      FAILED
   };
   Action172RedBull() : BaseAction(ACTION_ID_UNLOAD_PICKDOWN_GOODS) {}
   virtual ~Action172RedBull() {}

   void doStart() override;
   void doCancel() override;

   void onSrcAcFinishSucceed(int result_value) override;
   void onSrcAcFinishFailed(int result_value) override;
   void onSrcAcFinishCanceled(int result_value) override;

   void onSrcMcFinishSucceed(int result_value) override;
   void onSrcMcFinishFailed(int result_value) override;

   void onAlgoResultCallback(const sros::core::base_msg_ptr& msg) override;


private:
   void detectGoodsRacks();

   void sendUnloadDetect();
   void checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust);
   void genMovePath(sros::core::NavigationPath_vector& dst_paths);
   void genLinePath(sros::core::NavigationPath_vector& dst_paths);
   void launchMoveTask(bool is_one_line);


 private:
   //站点位姿
   Pose dst_pose_;
   Pose dst_3d_pose_;  // x: m, y: m, yaw: 弧度
   Pose dst_3d_agv_pose_;
   State state_ = NONE;
   int height_;
   int detect_height_;
   // int camera_height_;
   int goal_id_;
   bool is_groud_unload_;  //是否是地面放货

   bool is_forklift_running_ = false;

   bool enable_action_check_pallet_signal_ = false;
};

}


#endif  // SROS_ACTION_172_REDBULL_H
