/**
 * @file perception_state_massage.h
 * @brief detection results massage.
 *
 * The detection results of all the sensing modules are sent through
 * PerceptionStateMsg messages.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/1/13
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */


#ifndef SROS_PERCEPTION_STATE_MSG_HPP
#define SROS_PERCEPTION_STATE_MSG_HPP
//INCLUDE
#include "base_msg.h"
#include "perception_command_msg.hpp"
#include "../pose.h"

//CODE
namespace sros {
namespace core {
/**
 * @description : detection results massage.
 * @author      : zhangxu
 * @date        : 2021/1/13 下午2:33
 */
class PerceptionStateMsg : public BaseMsg {
 public:
    /**
     * @brief Constructors
     * @param[in] topic topic name;
     */
    PerceptionStateMsg(const topic_t & topic)
        : BaseMsg(topic, TYPE_COMMON_COMMAND) {
        // nothing to do.
    }

    /**
     * @brief Destructor
     */
    virtual ~PerceptionStateMsg() { };

    /**
     * @brief It has been abandoned.
     */
    virtual void getTime() { };



    enum DetectResult{
        /** @brief The default value is invalid. */
        DETECT_RESULT_INVALID = 0,

        /** @brief Detection failed. */
        DETECT_RESULT_FAIL = 1,

        /** @brief The Detection is successful. */
        DETECT_RESULT_SUCCESS = 2,

        /** @brief detect intermediate state one.*/
        DETECT_RESULT_INTERMEDIATE_STATE_ONE = 3,

        /** @brief detect intermediate state two.*/
        DETECT_RESULT_INTERMEDIATE_STATE_TWO = 4
    };


    /**
     * @brief Failure type
     */
    enum ErrorCode{
        /** @brief The default value is invalid. */
        ERROR_CODE_INVALID = 0,

        /** @brief No target found. */
        ERROR_CODE_NOT_FIND_GOAL = 1,

        /** @brief Parameter parsing error. */
        ERROR_CODE_PARAMETER_ERROR = 2,

        /** @brief Custom region existence obstacle. */
        ERROR_CODE_EXISTENCE_OBSTACLE = 3,

        /** @brief Custom region not existence obstacle. */
        ERROR_CODE_NOT_EXISTENCE_OBSTACLE = 4,

        /** @brief Detect time over. */
        ERROR_CODE_TIMEOUT = 5,

        /** @brief Detect function turn off */
        ERROR_FUNCTION_TURN_OFF = 6,

        /** @brief Detected coordinate of code out range. */
        ERROR_CODE_COORDINATE_OUT_RANGE = 7

    };

    
    static int errCodeConvert (ErrorCode code) {
        if(code < ErrorCode::ERROR_CODE_INVALID || code >  ErrorCode::ERROR_CODE_COORDINATE_OUT_RANGE ){
            LOG(WARNING)<<" unkonw enum type:" << code;
            return  0; //返回值待定
        }
        static std::map<int,int> err_map;
        err_map[ErrorCode::ERROR_CODE_INVALID]=sros::core::ERROR_CODE_DETECT_INVALID ;
        err_map[ErrorCode::ERROR_CODE_NOT_FIND_GOAL]=sros::core::ERROR_CODE_DETECT_NOT_FIND_GOAL ;
        err_map[ErrorCode::ERROR_CODE_PARAMETER_ERROR]=sros::core::ERROR_CODE_DETECT_PARAMETER_ERROR ;
        err_map[ErrorCode::ERROR_CODE_EXISTENCE_OBSTACLE]=sros::core::ERROR_CODE_DETECT_EXISTENCE_OBSTACLE ;
        err_map[ErrorCode::ERROR_CODE_NOT_EXISTENCE_OBSTACLE]=sros::core::ERROR_CODE_DETECT_NOT_EXISTENCE_OBSTACLE ;
        err_map[ErrorCode::ERROR_CODE_TIMEOUT]=sros::core::ERROR_CODE_DETECT_TIMEOUT ;
        err_map[ErrorCode::ERROR_FUNCTION_TURN_OFF]=sros::core::ERROR_CODE_DETECT_FUNCTION_TURN_OFF ;
        err_map[ErrorCode::ERROR_CODE_COORDINATE_OUT_RANGE]=sros::core::ERROR_CODE_DETECT_OUT_RANGE;
        return err_map[code];

    };

    /** @brief Sequence number of the massage. */
    uint32_t seq;

    /** @brief detect object index.
      * goal_id = -1  detect object area decide by biggest object.
      * goal_id =  0  invalid value.
      * goal_id >  0  detect object area which index equal to goal_id
      * */
    int goal_id;

    /** @brief obstacle detect result. */
    PerceptionCommandMsg::Command command;

    /** @brief goal detect result. */
    DetectResult detect_result = DETECT_RESULT_INVALID;

    /** @brief error code when detection fails. */
    ErrorCode error_code = ERROR_CODE_INVALID;

    /** @brief goal in agv world pose. */
    Pose goal_in_agv_pose;

    /** @brief goal in global map pose. */
    Pose goal_in_global_pose;

    /** @brief is save to put */
    bool is_safe_to_put = false;

    /**
     * @brief put detection area.
     * @note This parameter is only valid for perception command DETECT_PUT_SPACE
     */
    Location_Vector put_detect_region;

    /** @brief QRcode id */
    std::string code_str_;

    typedef std::shared_ptr<PerceptionStateMsg> Ptr;
    typedef std::shared_ptr<const PerceptionStateMsg> ConstPtr;
};

using PerceptionStateMsgPtr = PerceptionStateMsg::Ptr;
using PerceptionStateMsgConstPtr = PerceptionStateMsg::ConstPtr;
}
}
#endif  // SROS_PERCEPTION_STATE_MSG_HPP
