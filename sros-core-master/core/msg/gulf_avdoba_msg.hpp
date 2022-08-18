/**
 * @file gulf_release_fetch_avdoba_massage.hpp
 * @brief avdoba command massage.
 *
 * All the avdoba commands of the avdoba module are sent through
 * AvdObaCommandMsg messages.
 *
 * @author wuchaohuo@standard-robots.com
 * @date create date：2022/04/20 
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef GULF_AVDOBA_MSG_HPP
#define GULF_AVDOBA_MSG_HPP
//INCLUDE
#include "base_msg.h"
#include "../pose.h"
#include <eigen3/Eigen/Core>
#include "core/navigation_path.h"

//CODE
namespace sros {
namespace core {
/**
 * @description : avdoba command massage.
 * @author      : wuchaohuo
 * @date        : 2022/04/20 
 */
class AvdObaCommandMsg : public BaseMsg {
 public:
    /**
     * @brief Constructors
     * @param[in] topic topic name;
     */
    explicit AvdObaCommandMsg(const topic_t & topic)
        : BaseMsg(topic, TYPE_COMMON_COMMAND){
        // nothing to do.
    }

    /**
     * @brief Destructor
     */
    virtual ~AvdObaCommandMsg() = default;

    /**
     * @brief It has been abandoned.
     */
    void getTime() override { };

    /**
     * @brief Phase of avdoba
     */
    enum DetectAndAvdObaState{
        /** @brief invalid detection stage */
        COMMAND_INVALID = 0,
       
        /** @brief Fetch goods on stage. */
        AVDOBA_STAGE_FETCH_ON = 3,

        /** @brief Release goods on stage. */
        AVDOBA_STAGE_RELEASE_ON = 4,

        /** @brief ACTION FINISH */
        ACTION_FINISH = 5,

        /** @brief CHANGE MODEL FINISH */
        CHANGE_MODEL_FINISH = 6
    };

    /**
     * @brief type of avdoba
     */
    enum AvoidObstacleFunctionType{
        /** @brief avoid obstacle type invalid */
        TYPE_INVALID = 0,

        /** @brief fetech release avoid obstacle */
        FETCH_RELEASE_AVD = 1,

        /** @brief change avoid obstacle model */
        CHANGE_MDOEL_AVD = 2
    };

    /**
    * @brief forklift type.
    */
    enum FORKLIFT_TYPE{
        INVALID_TYPE = 0,

        FORKLIFT_3000 = 1,

        FORKLIFT_1400 = 2
    };

    /**
     * @brief Change avoid obstacle model command.
     */
    struct ObaModel{
        /** @brief QR code id */
        std::string QR_Code_id;

        /** @brief pallet id */
        int pallet_id;

        /** @brief forklift type */
        FORKLIFT_TYPE forklif_type = FORKLIFT_TYPE::INVALID_TYPE;

        /** @brief forklift size */
        double goods_width = 0.0;
        double goods_depth = 0.0;

        /** @brief  if change avoid obstacle model */
        bool is_change_oba_model = true;

        /** @brief  if change avoid obstacle model */
        bool restore_oba_model = false;
    };

    /**
     * @brief All of perception command.
     */
    struct Command{

        /** @brief type of avdoba */  
        AvoidObstacleFunctionType avdoba_type = TYPE_INVALID;

        /** @brief Phase of avdoba */
        DetectAndAvdObaState avdoba_state = COMMAND_INVALID;

        /** @brief Simply region points*/
        Pose goal_position;

        NavigationPath_vector paths;

        /** @brief enable avdoba. */
        bool is_enable = false;

        /** @brief obstacle avoid model */
        ObaModel oba_model;
    };

    /** @brief Sequence number of the massage. */
    uint32_t seq{};

    /** @brief Functions enabled by avdoba module. */
    Command command;

    typedef std::shared_ptr<AvdObaCommandMsg> Ptr;
    typedef std::shared_ptr<const AvdObaCommandMsg> ConstPtr;
};

using AvdObaCommandMsgPtr = AvdObaCommandMsg::Ptr;
using AvdObaCommandMsgConstPtr = AvdObaCommandMsg::ConstPtr;
}
}
#endif  // GULF_AVDOBA_MSG_HPP
