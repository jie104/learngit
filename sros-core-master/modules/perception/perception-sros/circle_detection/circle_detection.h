/**
 * @file perception_module.h
 * @brief The class responsible for the circle awareness module.
 *
 * If the parameter of main.enable_circle_detect is set to true, the circle sensing module will be
 * turned on and waiting for the collet to be RECEIVED_QUERY_COMMAND and IFM3D_IMG message,
 * once received CIRCLE_QUERY_COMMAND message, that is enable_img_process_ will be set to
 * true, the next IFM3D_IMG received will be processed by run(), than the circle detection
 * program will be executed and the result will be sent to PALLET_INFO topic.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/11/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __SROS_CIRCLE_DETECTOR_MODULE_H__
#define __SROS_CIRCLE_DETECTOR_MODULE_H__

// INCLUDE
#include "../../lib/object_detectors/circle_detector/circle_detect_param.hpp"
#include "../../lib/object_detectors/circle_detector/circle_detector.h"
#include "modules/perception/perception-sros/detector_base/detection_base.h"

// CODE
namespace object_detector {
/**
 * @description : The class responsible for the circle awareness module.
 * @author      : zhangxu
 * @date        : 2020/11/25 下午4:01
 */
class CircleDetectModule : public DetectorModuleBase {
 public:
    typedef std::shared_ptr<CircleDetectModule> Ptr;
    typedef std::shared_ptr<const CircleDetectModule> ConstPtr;

    /**
     * @brief construction.
     */
    CircleDetectModule(const MsgCallback &sendMsg,const  TopicCallback &subscribeTopic);

    /**
     * @brief deconstruction.
     */
    ~CircleDetectModule() override = default;

    /**
     * @brief Main process responsible for detection。
     */
    virtual bool init() override;


    /**
     * @brief Check whether the function is on
     * @return if detector is enable then return true. otherwise return false.
     */
    static bool isEnable();

    /**
     * @brief decode circle info in info_str and show decode recode result.
     * @param[in] info_str circle info string.
     * @param[out] circle_info decode info_str result.
     * @return If the decoding string fails or the decoding string is inconsistent with
     *         the circle parameters, false is returned; otherwise, true is returned.
     */
    static bool buildCircleInfo(const std::string& info_str, perception::Circle &circle_info);

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
     * @brief Load the target parameters to be detected.
     */
    void loadDetectParam() override;

    /**
     * @brief load circle detector parameter;
     */
    void loadTargetParam() override;

 private:

    /** @brief The parameter of the circle detector. */
    CircleDetectParam param_{};

    /** @brief Objects responsible for detect circle. */
    perception::CircleDetector circle_detector_;

    /** @brief The parameter of the circle. */
    std::map<int, perception::Circle> circles_map_;
};

using CircleDetectModulePtr = CircleDetectModule::Ptr;
using CircleDetectModuleConstPtr = CircleDetectModule::ConstPtr;

} // end of object_detector namespace
#endif  // SROS_CIRCLE_DETECTOR_MODULE_H

