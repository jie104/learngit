/**
 * @file perception_command_massage.hpp
 * @brief detection command massage.
 *
 * All the detection commands of the perception module are sent through
 * PerceptionCommandMsg messages.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/1/13
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_PERCEPTION_MSG_HPP
#define SROS_PERCEPTION_MSG_HPP
//INCLUDE
#include "base_msg.h"
#include "../pose.h"

//CODE
namespace sros {
namespace core {
/**
 * @description : detection command massage.
 * @author      : zhangxu
 * @date        : 2021/1/13 下午2:33
 */
class PerceptionCommandMsg : public BaseMsg {
 public:
    /**
     * @brief Constructors
     * @param[in] topic topic name;
     */
    explicit PerceptionCommandMsg(const topic_t & topic)
        : BaseMsg(topic, TYPE_COMMON_COMMAND){
        // nothing to do.
    }

    /**
     * @brief Destructor
     */
    virtual ~PerceptionCommandMsg() = default;

    /**
     * @brief It has been abandoned.
     */
    void getTime() override { };

    /**
     * @brief Phase of detection
     */
    enum DetectStage{
        /** @brief The default value is invalid. */
        DETECT_STAGE_INVALID = 0,

        /** @brief Material loading stage. */
        DETECT_STAGE_LOAD = 1,

        /** @brief Material unloading stage. */
        DETECT_STAGE_UNLOAD = 2
    };

    /**
     * @brief Detection state
     */
    enum DetectionState{
        /** @brief The default value is invalid. */
        DETECT_INVALID = 0,

        /** @brief Material loading stage. */
        DETECTION_ON = 1,

        /** @brief Material unloading stage. */
        DETECTION_OFF = 2
    };

    /**
     * @brief The kind of detect object.
     */
    enum ObjectType{
        /** @brief The default value is invalid. */
        OBJECT_TYPE_INVALID = 0,

        /** @brief Confirm that there are obstacles in the region. */
        OBJECT_TYPE_CUSTOM_REGION_HAVE_OBSTACLE = 1,

        /** @brief Confirm that there are obstacles in the region. */
        OBJECT_TYPE_CUSTOM_REGION_NO_OBSTACLE = 2,
		
        /** @brief Confirm that there are obstacles in the unloading space. */
        OBJECT_TYPE_PUT_SPACE = 3,

        /** @brief Check the position and posture of the card before picking up. */
        OBJECT_TYPE_CARD = 4,

        /** @brief Check the position and posture of the circle-disk before picking up. */
        OBJECT_TYPE_CIRCLE = 5,

        /** @brief Check the position and posture of the handcart before picking up. */
        OBJECT_TYPE_HANDCART = 6,

        /** @brief Check the position and posture of the QRcode. */
        OBJECT_TYPE_QRCODE = 7,

        /** @brief Check the position and posture of the QRcode global. */
        OBJECT_TYPE_QRCODE_GLOBAL = 8,
	
	OBJECT_TYPE_DECARDS = 9,
	
	 /** @brief Confirm that there are obstacles in the unloading space AND Check the position and posture of the cube. */
        OBJECT_TYPE_HEAPUP_CUBE = 10
    };

    /**
     * @brief The kind of action state.
     */
    enum ActionState {
        ACTION_START = 0,
        ACTION_ABORT = 1,
        ACTION_SUSPEND = 2,
        ACTION_SUFFCESS = 3,
        ACTION_FAIL = 4,
        ACTION_INVAILD =5
    };

    /**
     * @brief All of perception command.
     */
    struct Command{
        /** @brief Phase of detection */
        DetectStage detect_stage = DETECT_STAGE_INVALID;

        /** @brief The kind of detect object. */
        ObjectType object_type = OBJECT_TYPE_INVALID;

        /** @brief The kind of action state.*/
        ActionState action_state = ACTION_INVAILD;

        /** @brief enable detection. */
        bool is_enable = true;

        /** @brief detection state. */
        DetectionState detection_state =  DETECT_INVALID;
    };

    /** @brief Sequence number of the massage. */
    uint32_t seq{};

    /** @brief Functions enabled by perception module. */
    Command command;

    /** @brief detect object index.
     * goal_id = -1  detect all object
     * goal_id =  0  invalid value.
     * goal_id >  0   detect the object which index equal to goal_id
     * */
    int goal_id;

    /** @brief Theoretical pose of detected object. */
    Pose theory_pose;

    int obj_center_ground_clearance; // unit mm

    int obj_self_height; // unit mm

    int current_camera_height; // unit mm


    typedef std::shared_ptr<PerceptionCommandMsg> Ptr;
    typedef std::shared_ptr<const PerceptionCommandMsg> ConstPtr;
};

using PerceptionCommandMsgPtr = PerceptionCommandMsg::Ptr;
using PerceptionCommandMsgConstPtr = PerceptionCommandMsg::ConstPtr;
}
}
#endif  // SROS_PERCEPTION_MSG_HPP
