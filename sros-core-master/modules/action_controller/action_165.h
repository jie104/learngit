//
// Created by lbx on 2021/12/06
//

#ifndef SROS_ACTION_165_H
#define SROS_ACTION_165_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

/**
 * @brief 165动作的错误码
 *    xxxx  检测超时，当叉臂高度高于空栈板高度切感知未返回识别结果，则触发此错误
 *    xxx   常规式升降叉臂错误
 *    xxx   可调整式升降叉臂错误
 *    xxx   src移动指令错误
 *    xxx   感知的错误，参考xxxx   #可重试
 */

/**
 * AC 动作
 * （24, 33, 0） 可改变叉臂升降目标高度的指令
 * 
 */

namespace ac {

//ACTION_ID_GET_TOP_DECARDS = 165, 空栈板解堆叠，获取最上面的一个空栈板
class Action165 : public BaseAction {
public:
   enum State {
      NONE = 0,   //降叉中
      DOWNING,   //降叉中
      DOWN_FINISH,   //降叉结束
      STOPING, 
      UPING_TO_MAX,
      UPING_TO_TARGET,
      UP_FINISH,
      MC_RUNING,
      LOAD_UPING,          //升叉载货中
      LOAD_UP_FINISH,         //升叉载货
      CANCLING,         //取消中
      FAILED
   };
   enum FORKARM_CMD {
      START = 1,
      PAUSE,
      STOP
   };
   Action165(): BaseAction(ACTION_ID_GET_TOP_DECARDS) {}
   virtual ~Action165() {}

   void doStart() override;

   void doCancel() override;

   void onSrcAcFinishSucceed(int result_value) override;
   void onSrcAcFinishFailed(int result_value) override;
   void onSrcAcFinishCanceled(int result_value) override;

   void onSrcMcFinishSucceed(int result_value) override;
   void onSrcMcFinishFailed(int result_value) override;


   void onAlgoResultCallback(const sros::core::base_msg_ptr& msg) override;
   bool onSubSrcActionCheck(uint32_t& sub_src_action_no) override;

private:
   void enableCardDetect(bool enable, PerceptionCommandMsg::ActionState state);
   void waitFrokInPlace();
   void checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust);
   void launchMoveTask(bool is_one_line);
   void genMovePath(sros::core::NavigationPath_vector& dst_paths);
   void genLinePath(sros::core::NavigationPath_vector& dst_paths);
   uint32_t CalcSrcActionNo();

   string state2Str (State state);
   void setState(State step);


private:
   int target_height_;
   Pose dst_pose_;
   // 3d视觉检测到位姿
   Pose dst_3d_pose_;
   Pose dst_3d_agv_pose_;

   State cur_state_ = NONE;
   bool first_ = true;
   bool thread_runing_ = false;
   //当前的货物id
   int goal_id_;
   uint32_t src_action_no_;
   int count_ = 0;
   bool is_one_line_; 

};

}
#endif  // SROS_ACTION_165_H
