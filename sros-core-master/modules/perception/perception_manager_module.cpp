/**
 * @file perception_module_manager.cpp
 * @brief 简述文件内容
 *
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/4/15
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "perception_manager_module.h"

#include <utility>
#include "core/msg/base_msg.h"
#include "core/msg/perception_command_msg.hpp"
#include "core/msg/perception_state_msg.hpp"
#include "core/settings.h"
#include "core/logger.h"

// CODE
namespace object_detector {

using PerceptionCommandMsg = sros::core::PerceptionCommandMsg;
using PerceptionCommandMsgPtr = sros::core::PerceptionCommandMsgPtr;
using PerceptionStateMsg = sros::core::PerceptionStateMsg;
using PerceptionStateMsgPtr = sros::core::PerceptionStateMsgPtr;
using DetectStage = sros::core::PerceptionCommandMsg::DetectStage;
using ObjectType = sros::core::PerceptionCommandMsg::ObjectType;

PerceptionManager::PerceptionManager()
    : Module("PerceptionManager"),
      card_detector_(nullptr),
      circle_detector_(nullptr),
      put_space_detector_(nullptr),
      custom_region_detector_(nullptr),
      is_card_detector_enable_(false),
      is_circle_detector_enable_(false),
      is_put_space_detector_enable_(false),
      is_custom_region_detector_enable_(false),
      is_decards_detector_enable_(false) {
    // to do nothing.
}

void PerceptionManager::run() {
    //       is_card_detector_enable_ = initDetectModule<CardDetectModule>(card_detector_);
    //       is_circle_detector_enable_ = initDetectModule<CircleDetectModule>(circle_detector_);
    //       is_put_space_detector_enable_ = initDetectModule<PutSpaceDetectorModule>(put_space_detector_);
    //       is_custom_region_detector_enable_ = initDetectModule<CustomRegionDetectorModule>(custom_region_detector_);

    if (CardDetectModule::isEnable()) {
        card_detector_.reset(
            new CardDetectModule(CALLBACK(&PerceptionManager::massageCallback),
                                 SUBSCRIBE_TOPIC_CALLBACK(&PerceptionManager::subscribeTopicCallback)));
        if (nullptr != card_detector_) {
            is_card_detector_enable_ = card_detector_->init();
        } else {
            LOG(INFO) << "card detector init fail!";
        }
    }

    if (CircleDetectModule::isEnable()) {
        circle_detector_.reset(
            new CircleDetectModule(CALLBACK(&PerceptionManager::massageCallback),
                                   SUBSCRIBE_TOPIC_CALLBACK(&PerceptionManager::subscribeTopicCallback)));
        if (nullptr != circle_detector_) {
            is_circle_detector_enable_ = circle_detector_->init();
        } else {
            LOG(INFO) << "circle detector init fail!";
        }
    }

    if (PutspaceDetectModule::isEnable()) {
        put_space_detector_.reset(
            new PutspaceDetectModule(CALLBACK(&PerceptionManager::massageCallback),
                                       SUBSCRIBE_TOPIC_CALLBACK(&PerceptionManager::subscribeTopicCallback)));
        if (nullptr != put_space_detector_) {
            is_put_space_detector_enable_ = put_space_detector_->init();
        } else {
            LOG(INFO) << "put space detector init fail!";
        }
    }

    if (CustomRegionDetectorModule::isEnable()) {
        custom_region_detector_.reset(
            new CustomRegionDetectorModule(CALLBACK(&PerceptionManager::massageCallback),
                                           SUBSCRIBE_TOPIC_CALLBACK(&PerceptionManager::subscribeTopicCallback)));
        if (nullptr != custom_region_detector_) {
            is_custom_region_detector_enable_ = custom_region_detector_->init();
        } else {
            LOG(INFO) << "custom region detector init fail!";
        }
    }

    if (HandcartDetectModule::isEnable()) {
        handcart_detector_.reset(
            new HandcartDetectModule(CALLBACK(&PerceptionManager::massageCallback),
                                     SUBSCRIBE_TOPIC_CALLBACK(&PerceptionManager::subscribeTopicCallback)));
        if (nullptr != handcart_detector_) {
            is_handcart_detector_enable_ = handcart_detector_->init();
        } else {
            LOG(INFO) << "handcart detector init fail!";
        }
    }

    if (QRcodeDetection::isEnable()) {
        QRcode_detector_.reset(
            new QRcodeDetection(CALLBACK(&PerceptionManager::massageCallback),
                                SUBSCRIBE_TOPIC_CALLBACK(&PerceptionManager::subscribeTopicCallback)));
        if (nullptr != QRcode_detector_) {
            is_QRcode_detector_enable_ = QRcode_detector_->init();
        } else {
            LOG(INFO) << "QRcode detector init fail!";
        }

        QRcode_global_detector_.reset(
                new QRcodeDetectionGlobal(CALLBACK(&PerceptionManager::massageCallback),
                                    SUBSCRIBE_TOPIC_CALLBACK(&PerceptionManager::subscribeTopicCallback)));
        if (nullptr != QRcode_global_detector_) {
            is_QRcode_detector_enable_ = QRcode_global_detector_->init();
        } else {
            LOG(INFO) << "QRcode detector global init fail!";
        }
    }

    if (DecardsDetectModule::isEnable()) {
        decards_detector_.reset(
                new DecardsDetectModule(CALLBACK(&PerceptionManager::massageCallback),
                                     SUBSCRIBE_TOPIC_CALLBACK(&PerceptionManager::subscribeTopicCallback)));
        if (nullptr != decards_detector_) {
            is_decards_detector_enable_ = decards_detector_->init();
        } else {
            LOG(INFO) << "decards detector init fail!";
        }
    }


    LOG(INFO) << "perception manager start running";
    subscribeTopic("DETECT_COMMAND", CALLBACK(&PerceptionManager::onDetectCommandMsg));

    dispatch();
}

void PerceptionManager::massageCallback(const sros::core::base_msg_ptr &m) {
    sendMsg((m));
}

void PerceptionManager::subscribeTopicCallback(const std::string &topic_name,const MsgCallback &process) {
    subscribeTopic((topic_name), (process));
}

void PerceptionManager::sendDetectFailedMsg(sros::core::PerceptionCommandMsgConstPtr cmd) {
    auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
    result_msg->goal_id = -1;
    result_msg->command = cmd->command;
    result_msg->detect_result = PerceptionStateMsg::DetectResult::DETECT_RESULT_INVALID;
    sendMsg(result_msg);
}

void PerceptionManager::onDetectCommandMsg(const sros::core::base_msg_ptr &msg) {
    PerceptionCommandMsgPtr cmd = std::dynamic_pointer_cast<PerceptionCommandMsg>(msg);
    LOGGER(INFO, ACTION_TASK) << "onDetectCommandMsg: cmd:" << cmd->command.detect_stage * 10 + cmd->command.object_type
                              << " pose:" << cmd->theory_pose.x() << "," << cmd->theory_pose.y() << ","
                              << cmd->theory_pose.yaw() << " goal_id," << cmd->goal_id;
    bool is_receive_undefined_command = false;
    // distributing the detection tasks to different detectors to perform target detection.
    switch (cmd->command.detect_stage) {
        case DetectStage::DETECT_STAGE_LOAD:
            if (ObjectType::OBJECT_TYPE_CARD == cmd->command.object_type && is_card_detector_enable_) {
                card_detector_->onDetectCommandMsg(msg);
            } else if (ObjectType::OBJECT_TYPE_CIRCLE == cmd->command.object_type && is_circle_detector_enable_) {
                circle_detector_->onDetectCommandMsg(msg);
            } else if (ObjectType::OBJECT_TYPE_HANDCART == cmd->command.object_type && is_handcart_detector_enable_) {
                handcart_detector_->onDetectCommandMsg(msg);
            }else if (ObjectType::OBJECT_TYPE_QRCODE == cmd->command.object_type && is_QRcode_detector_enable_) {
                QRcode_detector_->onDetectCommandMsg(msg);
            }else if (ObjectType::OBJECT_TYPE_QRCODE_GLOBAL == cmd->command.object_type && is_QRcode_detector_enable_) {
                QRcode_global_detector_->onDetectCommandMsg(msg);
            }else if ((ObjectType::OBJECT_TYPE_CUSTOM_REGION_HAVE_OBSTACLE == cmd->command.object_type ||
                        ObjectType::OBJECT_TYPE_CUSTOM_REGION_NO_OBSTACLE == cmd->command.object_type) &&
                       is_custom_region_detector_enable_) {
                custom_region_detector_->onDetectCommandMsg(msg);
            } else if (ObjectType::OBJECT_TYPE_DECARDS == cmd->command.object_type && is_decards_detector_enable_){
                decards_detector_->onDetectCommandMsg(msg);
            } else {
                LOGGER(INFO, ACTION_TASK) << "undefined command load command";
                sendDetectFailedMsg(cmd);
            }
            break;
        case DetectStage::DETECT_STAGE_UNLOAD:
            if ((ObjectType::OBJECT_TYPE_CARD == cmd->command.object_type ||
                 ObjectType::OBJECT_TYPE_CIRCLE == cmd->command.object_type ||
                 ObjectType::OBJECT_TYPE_HANDCART == cmd->command.object_type ||
                 ObjectType::OBJECT_TYPE_PUT_SPACE == cmd->command.object_type ||
                 ObjectType::OBJECT_TYPE_HEAPUP_CUBE == cmd->command.object_type) &&
                is_put_space_detector_enable_) {
                put_space_detector_->onDetectCommandMsg(msg);
            } else if ((ObjectType::OBJECT_TYPE_CUSTOM_REGION_HAVE_OBSTACLE == cmd->command.object_type ||
                        ObjectType::OBJECT_TYPE_CUSTOM_REGION_NO_OBSTACLE == cmd->command.object_type) &&
                       is_custom_region_detector_enable_) {
                custom_region_detector_->onDetectCommandMsg(msg);
            } else {
                LOGGER(INFO, ACTION_TASK) << "undefined command unload command";
                sendDetectFailedMsg(cmd);
            }
            break;
        case DetectStage::DETECT_STAGE_INVALID:
            LOGGER(INFO, ACTION_TASK) << "cmd->command.detect_stage = DETECT_STAGE_INVALID";
            sendDetectFailedMsg(cmd);
            break;
        default:
            LOGGER(INFO, ACTION_TASK) << "Detect command massage undefined";
            sendDetectFailedMsg(cmd);
            break;
    }

    dispatch();
}

}
