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
 * @author lijunhong@standard-robots.com
 * @date create date：2021/12/03
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __SROS_DECARDS_DETECTOR_MODULE_H__
#define __SROS_DECARDS_DETECTOR_MODULE_H__

// INCLUDE
#include "../../lib/object_detectors/decards_detector/decards_detect_param.hpp"
#include "../../lib/object_detectors/decards_detector/decards_detector.h"
#include "modules/perception/perception-sros/detector_base/detection_base.h"

// CODE
namespace object_detector {
/**
 * @description : The class responsible for the decards awareness module.
 * @author      : lijunhong
 * @date        : 2021/12/03 下午2:54
 */
class DecardsDetectModule : public DetectorModuleBase {
 public:
    typedef std::shared_ptr<DecardsDetectModule> Ptr;
    typedef std::shared_ptr<const DecardsDetectModule> ConstPtr;

    /**
     * @brief construction.
     */
    DecardsDetectModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic);

    /**
     * @brief deconstruction.
     */
    ~DecardsDetectModule() override = default;

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
    DecardsDetectParam param_{};

    /** @brief Objects responsible for detect card. */
    perception::DecardsDetector decards_detector_;

    /** @brief The parameter of the card. */
    std::map<int, perception::Card> cards_map_;

    bool is_stack_top_;

    float camera_to_fork_height_ = 0.0f;
    
    float camera_to_agv_pitch_ = 0.0f; //unit deg

    float deviate_fork_height_ = 0.03f; // uint m

    int stack_fork_updown_delta_ = -10; // uint mm     

};

using DecardsDetectModulePtr = DecardsDetectModule::Ptr;
using DecardsDetectModuleConstPtr = DecardsDetectModule::ConstPtr;

} // end of object_detector namespace
#endif  // SROS_DECARDS_DETECTOR_MODULE_H