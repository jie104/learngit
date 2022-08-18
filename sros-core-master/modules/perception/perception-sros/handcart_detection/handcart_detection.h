/**
 * @file perception_module.h
 * @brief The class responsible for the handcart awareness module.
 *
 * If the parameter of main.enable_handcart_detect is set to true, the handcart sensing module will be
 * turned on and waiting for the collet to be RECEIVED_QUERY_COMMAND and IFM3D_IMG message,
 * once received PALLET_QUERY_COMMAND message, that is enable_img_process_ will be set to
 * true, the next IFM3D_IMG received will be processed by run(), than the handcart detection
 * program will be executed and the result will be sent to PALLET_INFO topic.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/11/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __SROS_HANDCART_DETECTOR_MODULE_H__
#define __SROS_HANDCART_DETECTOR_MODULE_H__

// INCLUDE
#include "../../lib/object_detectors/handcart_detector/handcart_detect_param.hpp"
#include "../../lib/object_detectors/handcart_detector/handcart_detector.h"
#include "modules/perception/perception-sros/detector_base/detection_base.h"

// CODE
namespace object_detector {
/**
 * @description : The class responsible for the handcart awareness module.
 * @author      : zhangxu
 * @date        : 2020/11/25 下午4:01
 */
class HandcartDetectModule : public DetectorModuleBase {
 public:
    typedef std::shared_ptr<HandcartDetectModule> Ptr;
    typedef std::shared_ptr<const HandcartDetectModule> ConstPtr;

    /**
     * @brief construction.
     */
    HandcartDetectModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic);

    /**
     * @brief deconstruction.
     */
    ~HandcartDetectModule() override = default;

    /**
     * @brief Main processing functions.
     */
    virtual bool init() override;

    /**
     * @brief enable livox-midxx publish data.
     */
    virtual void enableSensor() override;

    /**
     * @brief disable livox-midxx publish data.
     */
    virtual void disableSensor() override;

    virtual void checkDetectTime(const sros::core::base_msg_ptr &msg) override;

    /**
     * @brief Check whether the function is on
     * @return if detector is enable then return true. otherwise return false.
     */
    static bool isEnable();

    /**
     * @brief decode handcart info in info_str and show decode recode result.
     * @param[in] info_str handcart info string.
     * @param[out] handcart_info decode info_str result.
     * @return If the decoding string fails or the decoding string is inconsistent with
     *         the handcart parameters, false is returned; otherwise, true is returned.
     */
    static bool buildHandcartInfo(const std::string& info_str, perception::Handcart &handcart_info);

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
     * @brief load handcart detector parameter;
     */
    void loadDetectParam() override;

    /**
     * @brief Load the target parameters to be detected.
     */
    void loadTargetParam() override;

 private:
    /** @brief The parameter of the handcart detector. */
    HandcartDetectParam param_{};

    std::deque<PointCloudPtr> frame_queue_;

    /** @brief Objects responsible for detect handcart. */
    perception::HandcartDetector handcart_detector_;

    /** @brief The parameter of the handcart. */
    std::map<int, perception::Handcart> handcarts_map_;

    /** @brief Time required for detecting. */
    const int DETECTED_TIME_OUT = 5000;

    /** @brief The number of accumulated laser frames. */
    const int ACCUMULATE_QUEUE_SIZE_COUNT = 3000 ;
};

using HandcartDetectModulePtr = HandcartDetectModule::Ptr;
using HandcartDetectModuleConstPtr = HandcartDetectModule::ConstPtr;

} // end of object_detector namespace
#endif  // SROS_HANDCART_DETECTOR_MODULE_H