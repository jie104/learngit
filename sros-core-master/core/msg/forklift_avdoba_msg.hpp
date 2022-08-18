/**
 * @file gulf_release_fetch_avdoba_massage.hpp
 * @brief avdoba command massage.
 *
 * All the avdoba commands of the avdoba module are sent through
 * AvdObaCommandMsg messages.
 *
 * @author weiyanzhi@standard-robots.com
 * @date create date：2021/12/20
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_GULF_RELEASE_FETCH_AVDOBA_MSG_HPP
#define SROS_GULF_RELEASE_FETCH_AVDOBA_MSG_HPP
//INCLUDE
#include "base_msg.h"
#include <eigen3/Eigen/Core>
#include "core/navigation_path.h"

//CODE
namespace sros {
namespace core {
/**
 * @description : avdoba command massage.
 * @author      : weiyanzhi
 * @date        : 2021/12/20 上午10:28
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
    enum AvdObaState{
        /** @brief The default value is invalid. */
        AVDOBA_STAGE_INVALID = 0,

        /** @brief Fetch goods on stage. */
        AVDOBA_STAGE_FETCH_ON = 1,

        /** @brief Release goods on stage. */
        AVDOBA_STAGE_RELEASE_ON = 2

    };

    /**
     * @brief All of perception command.
     */
    struct Command{
        /** @brief Phase of avdoba */
        AvdObaState avdoba_state = AVDOBA_STAGE_INVALID;

        /** @brief Simply region points*/

        Eigen::Vector3f goal_position;

        NavigationPath_vector paths;

        /** @brief enable avdoba. */
        bool is_enable = true;
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
#endif  // SROS_GULF_RELEASE_FETCH_AVDOBA_MSG_HPP
