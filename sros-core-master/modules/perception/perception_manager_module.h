/**
 * @file perception_module_manager.h
 * @brief 简述文件内容
 *
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/4/15
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_PERCEPTION_MODULE_MANAGER_H
#define SROS_PERCEPTION_MODULE_MANAGER_H

// INCLUDE
#include "core/core.h"
#include "modules/perception/perception-sros/card_detection/card_detection.h"
#include "modules/perception/perception-sros/circle_detection/circle_detection.h"
#include "modules/perception/perception-sros/custom_region_detection/custom_region_detection.h"
#include "modules/perception/perception-sros/detector_base/detection_base.h"
#include "modules/perception/perception-sros/handcart_detection/handcart_detection.h"
#include "modules/perception/perception-sros/put_space_detection/put_space_detection.h"
#include "modules/perception/perception-sros/QRcode_detection/QRcode_detection.h"
#include "modules/perception/perception-sros/QRcode_detection_global/QRcode_detection_global.h"
#include "modules/perception/perception-sros/decards_detection/decards_detection.h"

// CODE
/**
 * @description : TODO
 * @author      : zhangxu
 * @date        : 2021/4/15 下午7:32
 */
namespace object_detector {

#define SUBSCRIBE_TOPIC_CALLBACK(F) boost::bind(F, this, _1, _2)

class PerceptionManager : public sros::core::Module {
 public:
    typedef boost::function<void(sros::core::base_msg_ptr)> MsgCallback;
    /**
     * @brief constructor.
     */
    PerceptionManager();

    /**
     * @brief Deconstruction
     */
    ~PerceptionManager() override = default;

    /**
     * @brief Delete copy constructor.
     */
    PerceptionManager(const PerceptionManager &) = delete;

    /**
     * @brief Delete assignment constructor.
     */
    PerceptionManager &operator=(PerceptionManager &) = delete;

    /**
     * @brief Main process responsible for detection。
     */
    void run() override;

 private:

    /**
     * @brief if receive undefined command load command, reture detection fail massage。
     */
    void sendDetectFailedMsg(sros::core::PerceptionCommandMsgConstPtr cmd);

    template<typename Detector>
    bool initDetectModule(std::shared_ptr<Detector> detector) {
        if (Detector::isEnable()) {
            detector.reset(new Detector(CALLBACK(&PerceptionManager::massageCallback),
                                        SUBSCRIBE_TOPIC_CALLBACK(&PerceptionManager::subscribeTopicCallback)));
            if (nullptr != detector){
                return detector->init();
            } else {
                LOG(INFO) << "detector init fail!";
                return false;
            }
        }
    }

    /**
     * @brief when receive a massage callback the function.
     * @param[in] m sensor massage.
     */
    void massageCallback(const sros::core::base_msg_ptr &m);


    void subscribeTopicCallback(const std::string &topic_name,const MsgCallback &function);

    /**
     * @brief Responsible for distributing the DETECT_COMMAND topic detection  message
     *        tasks to different detectors to perform target detection.
     * @param[in] msg sros::core::PerceptionCommandMsg.
     */
    void onDetectCommandMsg(const sros::core::base_msg_ptr &msg);

    /** @brief card detect object is init successful. */
    bool is_card_detector_enable_;

    /** @brief cricle detect object is init successful. */
    bool is_circle_detector_enable_;

    /** @brief put space detect object is init successful. */
    bool is_put_space_detector_enable_;

    /** @brief custom region detect object is init successful. */
    bool is_custom_region_detector_enable_;

    /** @brief handcart detect object is init successful. */
    bool is_handcart_detector_enable_;

    /** @brief QRcode detect object is init successful. */
    bool is_QRcode_detector_enable_;

    /** @brief decards detect object is init successful. */
    bool is_decards_detector_enable_;

    /** @brief detect object of card board. */
    CardDetectModulePtr card_detector_;

    /** @brief detect object of circle disk. */
    CircleDetectModulePtr circle_detector_;

    /** @brief detect object of put space. */
    PutspaceDetectModulePtr put_space_detector_;

    /** @brief detect object of custom region. */
    CustomRegionDetectorModulePtr custom_region_detector_;

    /** @brief detect object of handcart. */
    HandcartDetectModulePtr handcart_detector_;

    /** @brief detect object of QRcode. */
    QRcodeDetectionPtr QRcode_detector_;

    /** @brief detect object of QRcode. */
    QRcodeDetectionGlobalPtr QRcode_global_detector_;

    /** @brief detect object of decards stack. */
    DecardsDetectModulePtr decards_detector_;

};
}
#endif  // SROS_PERCEPTION_MODULE_MANAGER_H
