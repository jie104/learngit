//
// Created by caoyan on 7/21/21.
//

#ifndef SROS_POSTURE_CORRECT_COMMAND_MSG_HPP
#define SROS_POSTURE_CORRECT_COMMAND_MSG_HPP

#include "core/msg/base_msg.h"
#include "core/navigation_path.h"
#include "core/pose.h"

namespace sros {
namespace core {

class PostureCorrectCommandMsg : public BaseMsg {
public:
    explicit PostureCorrectCommandMsg(const topic_t& topic): BaseMsg(topic, TYPE_COMMON_COMMAND) {}
    virtual ~PostureCorrectCommandMsg() = default;

    typedef std::shared_ptr<PostureCorrectCommandMsg> Ptr;

    void getTime() override {}

    enum EnumCorrectCmd {
        CORRECT_CMD_INVALID = 0,
        CORRECT_CMD_START = 1,
        CORRECT_CMD_END = 2,
    };

    enum EnumCorrectDir {
        CORRECT_DIR_INVILAD = 0,
        CORRECT_DIR_FORWARD = 1,
        CORRECT_DIR_BACK = 2,
        CORRECT_DOWN_FORWARD = 3,
        CORRECT_DOWN_BACK = 4,
        CORRECT_DOWN_CHARGE_FORWARD = 5,
        CORRECT_DOWN_CHARGE_BACK = 6,
    };

    enum EnumCorrectResult {
        CORRECT_RESULT_INVALID = 0,
        CORRECT_RESULT_SUCCESS = 1,
        CORRECT_RESULT_FAIL = 2,
        CORRECT_RESULT_REPLAN_PATHS = 3,
        CORRECT_RESULT_POSE_CORRECT = 4,
    };

    //
    struct StCommand {
        EnumCorrectCmd correct_cmd;
        std::string sensor_name;
        EnumCorrectDir correct_dir;
        bool first_send_cmd;
    };

    //偏移量
    struct StOffset {
        double offset_x;  // X轴偏移（车头方向为X轴正方向，单位：万分之一米，取值范围:[-500, 500], 分辨率0.1mm）
        double offset_y;  // Y轴偏移（车头左边为Y轴正方向，单位：万分之一米，取值范围:[-500, 500], 分辨率0.1mm）
        double offset_angle;  //角度偏移（逆时针为正，单位：十分之一度，取值范围:（-180, 180], 分辨率0.1°）
    };
    //调整点
    struct StPose {
        Pose mid_pose;   //第一阶段调整的中间点
        Pose dst_pose;   //目标位姿
    };

    struct StCorrectResult {
        EnumCorrectResult correct_result;
        uint32_t error_code;
        StPose pose;
        StOffset offset;
    };

    //
    uint32_t seq;

    StCommand command;

    StCorrectResult result;
};

using PostureCorrectCommandMsgPtr = PostureCorrectCommandMsg::Ptr;

}
}

#endif //SROS_POSTURE_CORRECT_COMMAND_MSG_HPP
