/**
 * @file perception_module.h
 * @brief The class responsible for the card awareness module.
 *
 * If the parameter of main.enable_card_detect is set to true, the card sensing module will be
 * turned on and waiting for the collet to be RECEIVED_QUERY_COMMAND and IFM3D_IMG message,
 * once received PALLET_QUERY_COMMAND message, that is enable_img_process_ will be set to
 * true, the next IFM3D_IMG received will be processed by run(), than the card detection
 * program will be executed and the result will be sent to PALLET_INFO topic.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/11/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __SROS_CARD_DETECTOR_MODULE_H__
#define __SROS_CARD_DETECTOR_MODULE_H__

// INCLUDE
#include "../../lib/object_detectors/card_detector/card_detect_param.hpp"
#include "../../lib/object_detectors/card_detector/card_detector.h"
#include "modules/perception/perception-sros/detector_base/detection_base.h"

// CODE
namespace object_detector {
/**
 * @description : The class responsible for the card awareness module.
 * @author      : zhangxu
 * @date        : 2020/11/25 下午4:01
 */
class CardDetectModule : public DetectorModuleBase {
 public:
    typedef std::shared_ptr<CardDetectModule> Ptr;
    typedef std::shared_ptr<const CardDetectModule> ConstPtr;

    /**
     * @brief construction.
     */
    CardDetectModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic);

    /**
     * @brief deconstruction.
     */
    ~CardDetectModule() override = default;

    /**
     * @brief Main processing functions.
     */
    virtual bool init() override;

    /**
     * @brief Check whether the function is on
     * @return if detector is enable then return true. otherwise return false.
     */
    static bool isEnable();

    /**
     * @brief decode card info in info_str and show decode recode result.
     * @param[in] info_str card info string.
     * @param[out] card_info decode info_str result.
     * @return If the decoding string fails or the decoding string is inconsistent with
     *         the card parameters, false is returned; otherwise, true is returned.
     */
    static bool buildCardInfo(const std::string& info_str, perception::Card &card_info);

    /**
     * @brief Handling IFM3D_IMG command.
     * @param msg pointer of sros::core::ImageMsg.
     */
    void onSensorMsg(const sros::core::base_msg_ptr& msg) override;

    /**
     * @brief After receiving the command, turn on the detection.
     * @param msg pointer of sros::core::ImageMsg.
     */
    void onDetectCommandMsg(const sros::core::base_msg_ptr &msg) override;

 private:

    /**
     * @brief load card detector parameter;
     */
    void loadDetectParam() override;

    /**
     * @brief Load the target parameters to be detected.
     */
    void loadTargetParam() override;

 private:
    /** @brief The parameter of the card detector. */
    CardDetectParam param_{};

    /** @brief Objects responsible for detect card. */
    perception::CardDetector card_detector_;

    /** @brief The parameter of the card. */
    std::map<int, perception::Card> cards_map_;

    float camera_to_fork_height_ = 0.0f;
    
    float camera_to_agv_pitch_ = 0.0f; //unit deg

    float camera_to_agv_roll_ = 0.0f; //unit deg

    float deviate_fork_height_ = 0.03f; // uint m

    int stack_fork_updown_delta_ = -10; // uint mm
};

using CardDetectModulePtr = CardDetectModule::Ptr;
using CardDetectModuleConstPtr = CardDetectModule::ConstPtr;

} // end of object_detector namespace
#endif  // SROS_CARD_DETECTOR_MODULE_H
