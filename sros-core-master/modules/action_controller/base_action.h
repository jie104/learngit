//
// Created by caoyan on 1/9/21.
//

#ifndef SROS_BASE_ACTION_H
#define SROS_BASE_ACTION_H

#include "core/task/action_task.h"
#include "core/src.h"
#include "core/msg/common_msg.hpp"
#include "core/msg/command_msg.hpp"
#include "core/settings.h"
#include "core/msg/base_msg.h"
#include "core/error_code.h"
#include "core/fault_center.h"
#include "core/pose.h"
#include "core/task/movement_task.h"
#include "core/navigation_path.h"
#include "core/map/NavigationMap.hpp"
#include "core/map/mark/StationMark.hpp"
#include "core/device/device.h"
#include "core/logger.h"
#include "core/msg/perception_state_msg.hpp"
#include "core/msg/perception_command_msg.hpp"
#include "core/msg/posture_correct_command_msg.hpp"
#include "core/msg/gulf_avdoba_msg.hpp"
#include "core/device/device.h"
#include <vector>
#include <string>

using namespace std;
using namespace sros::core;

/*
ID起始	        ID终止	        用途
0x00(0)	        0x3F(63)	外部硬件动作A
0x40(64)	0x7F(127)	外部硬件动作B
0x80(128)	0xBF(191)	内部动作
0xC0(192)	0xFF(255)	用户开发的控制器，定制的动作
*/

namespace ac {

class BaseAction {
 public:
    //构造函数
    BaseAction(int reg_act_id);
    virtual ~BaseAction();

    //初始化
    int getRegActionId() { return REG_ACT_ID_; }
    std::string getActionParamStr() { return action_param_str_; }

    bool initAction();

    //动作的启动和取消
    void startAction(ActionTask_ptr action_task);
    void cancelAction(int result_value = 0);

    //定时任务
    void onTimer50ms(uint64_t cur_time);

    //src动作结果返回回调
    virtual bool onSrcAcFinishFirst() { return true; }
    virtual void onSrcAcFinishSucceed(int result_value) {
        doActionFinishSucceed(result_value);
    }
    virtual void onSrcAcFinishFailed(int result_value) {
        doActionFinishFailed(result_value);
    }
    virtual void onSrcAcFinishCanceled(int result_value) {
        doActionFinishCanceled(result_value);
    }

    //算法结果返回回调
    virtual void onAlgoResultCallback(const sros::core::base_msg_ptr &msg) {}

    //区域避障消息
    virtual void onRegionObstacleMsg(const sros::core::base_msg_ptr &msg) {}

   //路网导航生成路径结果
    virtual void onNavResultCallback(const sros::core::base_msg_ptr &msg) {}

    //src移动任务结果返回回调
    virtual void onSrcMcFinishSucceed(int result_value) {}
    virtual void onSrcMcFinishFailed(int result_value) {}

    virtual bool onSubSrcActionCheck(uint32_t& sub_src_action_no) { return false;}

 protected:
    //具体动作定制
    virtual bool doInit() { return true;}
    virtual void doStart() = 0;
    virtual void doCancel() {}
    virtual bool doCancelToInCancel() { return false;}
    virtual void doInCancel() {}
    virtual void doTimerEvent(uint64_t cur_time) {}

 protected:
    //动作结束接口
    void doActionFinishSucceed(int result_value = 0);
    void doActionFinishFailed(int result_value);
    void doActionFinishCanceled(int result_value = 0);

 public:
    //功能函数
    bool isEnableEac();
    bool isForkControlTypeSrc();
    void sendEacActionTask();   //发送eac动作
    void sendEacActionTask(int action_no, int action_id, int param0, int param1);  //发送eac动作
    void cancelEacActionTask(int action_no); //发送取消eac动作

    void sendMoveTask(const sros::core::NavigationPath_vector& dst_paths);      //发送移动任务
    void sendReplaceTask(const sros::core::NavigationPath_vector& dst_paths);   //发送移动替换任务
    void cancelMoveTask();

    bool checkCurPoseArriveDestStation(sros::map::StationMark& dst_station); //判断当前位姿是否在站点上
    bool checkPalletInPlaceSignal();       //判断栈板到位信号

    void sendPerceptionCmd(uint32_t seq, PerceptionCommandMsg::Command detect_type, int goal_id, const Pose& pose, const int& obj_center_ground_clearance);
    void sendPerceptionCmd(uint32_t seq, PerceptionCommandMsg::Command detect_type, int goal_id, const Pose& pose);
    void sendAvdObaCmd(uint32_t seq,  AvdObaCommandMsg::Command command);

    void sendPerceptionCmd(uint32_t seq,
                           PerceptionCommandMsg::Command detect_type,
                           int goal_id,
                           const Pose& pose,
                           int obj_self_height,
                           int obj_center_ground_clearance,
                           int current_camera_height);


    void enableSVC100Camera(bool enable, const std::string &which_camera);     //svc100
    void enableBackLidar(bool enable);       //控制叉车后退雷达，true 打开，false 关闭

    void sendCommonPosesInfo(const Location_Vector& loc_vec, std::string sensor_name);

    void sendPostureCorrectCmd(uint32_t seq,
                               PostureCorrectCommandMsg::EnumCorrectCmd correct_cmd,
                               const std::string sensor_name,
                               PostureCorrectCommandMsg::EnumCorrectDir correct_dir,
                               bool first_send_cmd);

    bool checkForkHeightFautl(float dst_fork_height);  // 检测叉臂高度                           

 protected:
    //动作ID
    int REG_ACT_ID_ = -1;

    //动作参数
    ActionTask_ptr action_task_ = nullptr;
    int action_no_;
    int action_id_;
    int action_param0_;
    int action_param1_;
    int action_param2_;

    std::string action_param_str_;

};

typedef std::shared_ptr<BaseAction> BaseAction_ptr;

}

#endif  // SROS_BASE_ACTION_H
