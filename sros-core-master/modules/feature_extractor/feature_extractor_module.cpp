//
// Created by lfc on 2020/12/23.
//

#include "feature_extractor_module.h"
#include <core/settings.h>
#include <core/src.h>
#include <core/msg/common_command_msg.hpp>
#include <core/msg/common_msg.hpp>
#include <core/msg/data_matrix_code_msg.hpp>
#include <core/msg/feature_info_msg.hpp>
#include <core/msg/common_state_msg.hpp>
namespace sros {
static void convertDmToStandardHandSystem(const sros::core::DataMatrixCodeMsg_ptr& dm_pose,sros::core::DataMatrixCodeMsg_ptr& right_pose){
    //目前这种坐标系变换只针对下视摄像头，为了兼容之前其他传感器数据。以后，内循环系统内的摄像头全部都是标准右手系，不需要变换。
    if (dm_pose == right_pose) {
        LOG(INFO) << "cannot convert same ptr!";
    }
    *right_pose = *dm_pose;
    if (dm_pose->right_handed_system) {
        right_pose->x_ = dm_pose->y_;
        right_pose->y_ = -dm_pose->x_;
        right_pose->angle_ = (dm_pose->angle_ - M_PI_2);
    }else{
        right_pose->right_handed_system = true;
        right_pose->x_ = dm_pose->x_;
        right_pose->y_ = -dm_pose->y_;
        right_pose->angle_ = -dm_pose->angle_;
    }
}

FeatureExtractorModule::FeatureExtractorModule()
    : Module("FeatureExtractorModule"), scan_extractor_state_(STATE_IDLE), camera_extractor_state_(STATE_IDLE) {}

FeatureExtractorModule::~FeatureExtractorModule() {}

void FeatureExtractorModule::run() {
    subscribeTopic("TOPIC_EXTRACT_COMMAND", CALLBACK(&FeatureExtractorModule::extractCommandCallback));
    subscribeTopic("TOPIC_LASER", CALLBACK(&FeatureExtractorModule::scanCallback));
    subscribeTopic("DM_CODE_INFO", CALLBACK(&FeatureExtractorModule::visionCallback));
    computeAligenSensorTF();
    dispatch();
}

void FeatureExtractorModule::extractCommandCallback(sros::core::base_msg_ptr cmd) {
    //"DEBUG"为 上位机调试模式，打开，然后上传特征信息
    //如果打开相机的上位机调试模式，也应该由该模块转换成世界坐标生成位置
    //“正常提取模式”定位融合模式，打开，然后将特征信息以code的形式传递给定位模块。
    auto extract_cmd = std::dynamic_pointer_cast<sros::core::CommonCommandMsg<std::string>>(cmd);//dynamic pointer专用
    LOG(INFO) << "extractCommandCallback: " << extract_cmd->command << " " << extract_cmd->str1;
    auto copyCmd = [&](std::shared_ptr<sros::core::CommonCommandMsg<std::string>> cmd_msg) {
        cmd_msg->pose = extract_cmd->pose;
        cmd_msg->str0 = extract_cmd->str0;
        cmd_msg->str1 = extract_cmd->str1;
    };

    if (extract_cmd->command == "DEBUG_START_EXTRACTOR") {//debug模式，用向上位机上传特征信息
        if (extract_cmd->str1 == sros::device::DEVICE_LIDAR) {
            debug_info_queues_.clear();
            in_scan_debug_state_ = true;
        } else {
            //TODO 如果不是雷达则就为相机,而后根据此前的命令str1 去发布开启那个相机的命令
            debug_info_queues_.clear();
            in_camera_debug_state_ = true;

            std::shared_ptr<sros::core::CommonMsg> cmd(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
            cmd->str_0_ = extract_cmd->str1;
            cmd->flag = true;  //打开摄像头
            sendMsg(cmd);
        }
    } else if (extract_cmd->command == "DEBUG_STOP_EXTRACTOR") {//debug模式，关闭向上位机上传特征信息
        if (extract_cmd->str1 == sros::device::DEVICE_LIDAR) {
            in_scan_debug_state_ = false;
        } else {
            in_camera_debug_state_ = false;
            std::shared_ptr<sros::core::CommonMsg> cmd(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
            cmd->str_0_ = extract_cmd->str1;
            cmd->flag = false;  //关闭摄像头
            sendMsg(cmd);
        }
    } else if (extract_cmd->command == "START_EXTRACTOR") {//开始提取，向定位模块发送特征信息
        if (extract_cmd->str1 == sros::device::DEVICE_LIDAR) {
            scan_matched_info_.feature_name = extract_cmd->str0; //是何种特征 三角 直角 等
            scan_matched_info_.pose = slam::tf::TransForm(0, extract_cmd->pose);  //TODO 变换到世界坐标系下 ?
            scan_matched_info_.sensor_name = extract_cmd->str1;

            scan_extractor_state_ = STATE_SEND_SCAN_TO_LOCATION;  //将当前位置发送给定位模块
        } else {
            camera_matched_info_.feature_name = extract_cmd->str0;  //TODO tag name?
            camera_matched_info_.pose = slam::tf::TransForm(0, extract_cmd->pose);
            camera_matched_info_.sensor_name = extract_cmd->str1;

            camera_extractor_state_ = STATE_SEND_VISION_TO_LOCATION;
            std::shared_ptr<sros::core::CommonMsg> cmd(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
            cmd->str_0_ = extract_cmd->str1;
            cmd->flag = true;  //打开摄像头
            sendMsg(cmd);
        }

        std::shared_ptr<sros::core::CommonCommandMsg<std::string>> location_cmd(
            new sros::core::CommonCommandMsg<std::string>("TOPIC_ALIGNMENT"));
        location_cmd->command = "START_ALIGNMENT";//slam 中有对应的命令
        copyCmd(location_cmd);
        sendMsg(location_cmd);
    } else if (extract_cmd->command == "STOP_EXTRACTOR") {//停止提取，终止特征融合定位
        if (extract_cmd->str1 == sros::device::DEVICE_LIDAR) {
            info_queues_.clear();
            scan_extractor_state_ = STATE_CHECK_SCAN_RESO;  //
        } else {
            camera_extractor_state_ = STATE_CHECK_VISION_RESO;
            need_close_camera_ = true;
        }
        std::shared_ptr<sros::core::CommonCommandMsg<std::string>> location_cmd(
            new sros::core::CommonCommandMsg<std::string>("TOPIC_ALIGNMENT"));
        location_cmd->command = "STOP_ALIGNMENT";
        copyCmd(location_cmd);
        sendMsg(location_cmd);

        if (extract_cmd->str1 != sros::device::DEVICE_LIDAR) {
            auto device = sros::device::DeviceManager::getInstance()->getDeviceByName(extract_cmd->str1);
            if (device && device->isOk()) {
                LOG(INFO) << "device " << extract_cmd->str1 << "ok!";
            } else {
                LOG(INFO) << "cannot get device:" << extract_cmd->str1 << ",will return recognize failed!";
                camera_extractor_state_ = STATE_IDLE;
                MatchedFeatureInfo match_info;
                match_info.sensor_name = extract_cmd->str1;
                match_info.feature_name = extract_cmd->str0;
                sendUnWatchedState(match_info);
            }
        }

    }else if(extract_cmd->command == "START_RECORD_DM_CODE"||extract_cmd->command == "START_FUSION_DM_CODE"){
        LOG(INFO) << "get command:" << extract_cmd->command;
        if(extract_cmd->str1==sros::device::DEVICE_SVC100_DOWN){
            if (camera_extractor_state_ == STATE_IDLE) {
                std::shared_ptr<sros::core::CommonCommandMsg<std::string>> location_cmd(
                    new sros::core::CommonCommandMsg<std::string>("TOPIC_ALIGNMENT"));
                location_cmd->command = extract_cmd->command;
                copyCmd(location_cmd);
                sendMsg(location_cmd);

                camera_extractor_state_ = STATE_LOCATION_VISION_FUSION;
                std::shared_ptr<sros::core::CommonMsg> cmd(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
                cmd->str_0_ = extract_cmd->str1;
                cmd->flag = true;  //打开摄像头
                sendMsg(cmd);
            }
        }else if(extract_cmd->str1==sros::device::DEVICE_LIDAR){
            if (scan_extractor_state_ == STATE_IDLE) {
                LOG(INFO) << "record lidar!";
                std::shared_ptr<sros::core::CommonCommandMsg<std::string>> location_cmd(
                    new sros::core::CommonCommandMsg<std::string>("TOPIC_ALIGNMENT"));
                location_cmd->command = extract_cmd->command;
                copyCmd(location_cmd);
                sendMsg(location_cmd);
                scan_extractor_state_ = STATE_RECORD_SCAN_FEATURE;
            }
        }else{
            LOG(INFO) << "cannot record the camera data:" << extract_cmd->str1;
        }
    }else if(extract_cmd->command == "STOP_RECORD_DM_CODE"||extract_cmd->command == "STOP_FUSION_DM_CODE"){
        LOG(INFO) << "get command:" << extract_cmd->command;
        if(extract_cmd->str1==sros::device::DEVICE_SVC100_DOWN){
            if (camera_extractor_state_ == STATE_LOCATION_VISION_FUSION) {
                std::shared_ptr<sros::core::CommonCommandMsg<std::string>> location_cmd(
                    new sros::core::CommonCommandMsg<std::string>("TOPIC_ALIGNMENT"));
                location_cmd->command = extract_cmd->command;
                copyCmd(location_cmd);
                sendMsg(location_cmd);

                camera_extractor_state_ = STATE_IDLE;
                std::shared_ptr<sros::core::CommonMsg> cmd(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
                cmd->str_0_ = extract_cmd->str1;
                cmd->flag = false;  //打开摄像头
                sendMsg(cmd);
            }else{
                LOG(INFO) << "cannot stop record! curr state:" << camera_extractor_state_;
            }
        }else if(extract_cmd->str1==sros::device::DEVICE_LIDAR){
            if (scan_extractor_state_ == STATE_RECORD_SCAN_FEATURE) {
                LOG(INFO) << "record lidar!";
                std::shared_ptr<sros::core::CommonCommandMsg<std::string>> location_cmd(
                    new sros::core::CommonCommandMsg<std::string>("TOPIC_ALIGNMENT"));
                location_cmd->command = extract_cmd->command;
                copyCmd(location_cmd);
                sendMsg(location_cmd);
                scan_extractor_state_ = STATE_IDLE;
            }else{
                LOG(INFO) << "cannot stop record! curr state:" << scan_extractor_state_;
            }
        }else{
            LOG(INFO) << "cannot record the camera data:" << extract_cmd->str1;
        }
    }
}

void FeatureExtractorModule::scanCallback(sros::core::base_msg_ptr scan) {
    if (scan_extractor_state_ == STATE_SEND_SCAN_TO_INTERACT || scan_extractor_state_ == STATE_SEND_SCAN_TO_LOCATION ||
        scan_extractor_state_ == STATE_CHECK_SCAN_RESO||scan_extractor_state_ == STATE_RECORD_SCAN_FEATURE||in_scan_debug_state_) {
        std::vector<extractor::FeatureInfo_ptr> infos;
        sros::core::LaserScan_ptr laser_scan = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(scan);
        if (scan_extractor_.extract(laser_scan, infos,2.5)) {
            for (auto& info : infos) {
                info->sensor_name = sros::device::DEVICE_LIDAR;
                info->time = scan->time_;
                Eigen::Vector3f pose_in_sensor_point;
                info->pose_in_sensor.get2DPose(pose_in_sensor_point);
            }
            if (in_scan_debug_state_) {
                //目前可使用多个做滤波，求最优的外发
                debug_info_queues_.push_back(infos);
                if (debug_info_queues_.size() > mean_queue_size_) {
                    debug_info_queues_.pop_front();
                }
                if (infos.size()&&debug_info_queues_.size() >= mean_queue_size_) {
                    sros::core::FeatureInfoMsg_ptr feature_msg(new sros::core::FeatureInfoMsg());
                    feature_msg->topic_ =
                        "SCAN_FEATURE_INFO";  //这里，需要把camerafeature和scanfeature区分开来，防止同时打开
                    feature_msg->time_ = scan->time_;
                    for (auto& info : infos) {
                        slam::tf::TransForm mean_tf;
                        if(isMeasureEnough(info, debug_info_queues_, mean_tf)){
                            convertToFeatureMsg(info, mean_tf, feature_msg);
                        }
                    }
                    sendMsg(feature_msg);
                    debug_info_queues_.clear();
                }
            }

            if (scan_extractor_state_ == STATE_RECORD_SCAN_FEATURE) {
                for(auto& info:infos) {
                    if (info->code_type == TYPE_SCAN_LMK_CODE) {
                        sros::core::DataMatrixCodeMsg_ptr data_msg(new sros::core::DataMatrixCodeMsg("LOCATION_CODE_INFO"));
                        convertToDataMsg(info, data_msg);  //发给定位模块的所有信息，均应该是基于运动中心的
                        sendMsg(data_msg);
                    }
                }
            }else if (scan_extractor_state_ == STATE_SEND_SCAN_TO_LOCATION) {
                extractor::FeatureInfo_ptr specify_info;
                if (getSpecifyFeature(scan_matched_info_, infos, specify_info)) {
                    sros::core::DataMatrixCodeMsg_ptr data_msg(new sros::core::DataMatrixCodeMsg("LOCATION_CODE_INFO"));
                    convertToDataMsg(specify_info, data_msg);  //发给定位模块的所有信息，均应该是基于运动中心的
                    sendMsg(data_msg);
                }
            }
        }
        if (scan_extractor_state_ == STATE_CHECK_SCAN_RESO) {
            LOG(INFO) << "info size:" << infos.size();
            extractor::FeatureInfo_ptr specify_info;
            if (getSpecifyFeature(scan_matched_info_, infos, specify_info)) {
                check_feature_not_watched_count_ = 0;
                info_queues_.push_back(infos);
                LOG(INFO) << "info queue size:" << info_queues_.size();
                if (info_queues_.size() >= 3) {
                    slam::tf::TransForm mean_tf, world_tf;
                    computeMeanPose(specify_info, info_queues_, mean_tf);

                    Eigen::Vector3f pose_2d;
                    mean_tf.get2DPose(pose_2d);

                    LOG(INFO) << "scan ref pose:" << pose_2d[0] << "," << pose_2d[1] << "," << pose_2d[2];

                    std::shared_ptr<sros::core::CommonStateMsg<bool>> check_state;
                    checkAlignmentResult(specify_info, mean_tf, scan_matched_info_, check_state);
                    LOG(INFO)<<"check Scan Alignment Result: "<<check_state->failed_code_;
                    sendMsg(check_state);
                    scan_extractor_state_ = STATE_IDLE;
                    info_queues_.clear();
                }
            }else{
                check_feature_not_watched_count_++;
                if (check_feature_not_watched_count_ > check_feature_not_watched_thresh_) {
                    info_queues_.clear();
                    check_feature_not_watched_count_ = 0;
                    scan_extractor_state_ = STATE_IDLE;
                    sendUnWatchedState(scan_matched_info_);
                }
            }
        }
    }
}

void FeatureExtractorModule::visionCallback(sros::core::base_msg_ptr msg) {
    auto closeCamera = [&](const std::string camera_name){
      std::shared_ptr<sros::core::CommonMsg> cmd(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
      cmd->str_0_ = camera_name;
      cmd->flag = false;  //关闭摄像头
      sendMsg(cmd);
    };
    if (need_close_camera_&&camera_extractor_state_ == STATE_IDLE) {
        need_close_camera_ = false;
        auto device = sros::device::DeviceManager::getInstance()->getDeviceByName(camera_matched_info_.sensor_name);
        if (device) {
            closeCamera(camera_matched_info_.sensor_name);
        }
    }

    if (camera_extractor_state_ == STATE_SEND_VISION_TO_LOCATION ||
        camera_extractor_state_ == STATE_SEND_VISION_TO_INTERACT ||
        camera_extractor_state_ == STATE_CHECK_VISION_RESO || camera_extractor_state_ == STATE_LOCATION_VISION_FUSION ||
        in_camera_debug_state_ ) {
        sros::core::DataMatrixCodeMsg_ptr data_msg = std::dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(msg);
        bool matched_state = false;
        if (data_msg->state_ == DM_CODE_DETECTED) {
            extractor::FeatureInfo_ptr feature_info(new extractor::FeatureInfo);
            convertDataMsgToFeatureInfo(data_msg, feature_info);
            if (in_camera_debug_state_) {  //世界坐标系
                std::vector<extractor::FeatureInfo_ptr> infos;
                infos.push_back(feature_info);
                //目前可使用多个做滤波，求最优的外发
                debug_info_queues_.push_back(infos);
                if (debug_info_queues_.size() > mean_queue_size_) {
                    debug_info_queues_.pop_front();
                }
                if (infos.size() && debug_info_queues_.size() >= mean_queue_size_) {
                    sros::core::FeatureInfoMsg_ptr feature_msg(new sros::core::FeatureInfoMsg());
                    feature_msg->topic_ =
                        "CAMERA_FEATURE_INFO";  //这里，需要把camerafeature和scanfeature区分开来，防止同时打开
                    feature_msg->time_ = data_msg->time_;
                    for (auto& info : infos) {
                        slam::tf::TransForm mean_tf;
                        if (isMeasureEnough(info, debug_info_queues_, mean_tf)) {
                            convertToFeatureMsg(info, mean_tf, feature_msg);
                        }
                    }
                    sendMsg(feature_msg);
                    debug_info_queues_.clear();
                }
            }
            if (camera_extractor_state_ == STATE_SEND_VISION_TO_LOCATION) {  //运动中心
                std::vector<extractor::FeatureInfo_ptr> infos;
                extractor::FeatureInfo_ptr matched_info;
                infos.push_back(feature_info);
                if (getSpecifyFeature(camera_matched_info_, infos,
                                      matched_info)) {  //防止识别更多的二维码，统统的输入到定位模块
                    sros::core::DataMatrixCodeMsg_ptr data_msg(new sros::core::DataMatrixCodeMsg("LOCATION_CODE_INFO"));
                    convertToDataMsg(matched_info, data_msg);  //发给定位模块的所有信息，均应该是基于运动中心的
                    sendMsg(data_msg);
                } else {
                    LOG(INFO) << "cannot get!" << camera_matched_info_.feature_name << "," << feature_info->feature_name
                              << "," << feature_info->sensor_name;
                }
            }else if (camera_extractor_state_ == STATE_CHECK_VISION_RESO) {
                std::vector<extractor::FeatureInfo_ptr> infos;
                extractor::FeatureInfo_ptr matched_info;
                infos.push_back(feature_info);
                if (getSpecifyFeature(camera_matched_info_, infos, matched_info)) {
                    matched_state = true;
                    check_feature_not_watched_count_ = 0;
                    camera_extractor_state_ = STATE_IDLE;
                    std::shared_ptr<sros::core::CommonStateMsg<bool>> check_state;
                    checkAlignmentResult(matched_info, matched_info->pose_in_sensor, camera_matched_info_, check_state);
                    LOG(INFO) << "check camera Alignment Result: " << check_state->failed_code_;
                    sendMsg(check_state);
                } else {
                    for (auto& info : infos) {
                        //如果进入此处证明getSpecifyFeature失败,则如找到对应特征则必须发送两次位置有误差大于阈值
                        if (info->feature_name == camera_matched_info_.feature_name) {
                            std::shared_ptr<sros::core::CommonStateMsg<bool>> check_state(
                                new sros::core::CommonStateMsg<bool>("TOPIC_ALIGNMENT_STATE"));
                            matched_state = true;
                            check_feature_not_watched_count_ = 0;
                            camera_extractor_state_ = STATE_IDLE;
                            check_state->map_name = camera_matched_info_.sensor_name;
                            check_state->state = false;
                            check_state->failed_code_ = ERROR_DEVIATION_ISLARGE;  //最后识别特征距离偏差过大
                            sendMsg(check_state);
                            break;
                        }
                    }
                }
            } else if (camera_extractor_state_ == STATE_LOCATION_VISION_FUSION) {
                if (data_msg->camera_name_ == sros::device::DEVICE_SVC100_DOWN) {
                    sros::core::DataMatrixCodeMsg_ptr right_msg(
                        new sros::core::DataMatrixCodeMsg("LOCATION_CODE_INFO"));
                    convertDmToStandardHandSystem(data_msg, right_msg);
                    convertDataMsgToFeatureInfo(right_msg, feature_info);
                    sros::core::DataMatrixCodeMsg_ptr send_data_msg(
                        new sros::core::DataMatrixCodeMsg("LOCATION_CODE_INFO"));
                    if (convertToDataMsg(feature_info, send_data_msg)) {
//                        LOG(INFO) << "send info:" << send_data_msg->x_ << "," << send_data_msg->y_ << ","
//                                  << send_data_msg->angle_ << "," << right_msg->x_ << "," << right_msg->y_ << ","
//                                  << right_msg->angle_;
                        sendMsg(send_data_msg);
                    }  //发给定位模块的所有信息，均应该是基于运动中心的
                }
            }
        }
        // TODO Fix me
        // 此处如果在下发STOP_EXTRACTOR指令后,连续没有提取到次数小于5次,则会无法进入此处,实际使用出现,在下发关闭相机指令后,可能只运行不够5次
        if (!matched_state && camera_extractor_state_ == STATE_CHECK_VISION_RESO) {
            check_feature_not_watched_count_++;
            if (check_feature_not_watched_count_ >= check_feature_not_watched_thresh_) {
                check_feature_not_watched_count_ = 0;
                camera_extractor_state_ = STATE_IDLE;
                sendUnWatchedState(camera_matched_info_);
            }
        }
    }
}

std::vector<double> splitStrsToDoubles(const std::string& s, const char seperator) {
    std::vector<double> result;
    typedef std::string::size_type string_size;

    string_size i = 0;
    string_size j = 0;
    char c = s[0];
    if (c == '"')  // chip 设置的话一般带“”
        j = 1;
    // LOG(INFO) << "the s is:" << s;
    while (i < s.size()) {
        if (s[i] == seperator || i == s.size() - 1) {
            if (j != i || i == s.size() - 1) {
                auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                std::string item_s = s.substr(j, len);
                if (item_s == "\"") break;
                try {
                    double item = stof(item_s);
                    result.push_back(item);
                } catch (std::exception& e) {
                    LOG(INFO) << "throw error:" << e.what() << item_s;
                }
            }
            j = i + 1;
        }
        i++;
    }
    return result;
}
void FeatureExtractorModule::computeAligenSensorTF() {
    auto resolveTF = [&](const std::string& tf_para) {
        slam::tf::TransForm tf;
        auto paras = splitStrsToDoubles(tf_para, ';');
        int para_size = paras.size();
        int curr_index = 0;
        if (para_size > curr_index) {
            tf.position.x() = paras[curr_index++];
        }
        if (para_size > curr_index) {
            tf.position.y() = paras[curr_index++];
        }
        if (para_size > curr_index) {
            tf.position.z() = paras[curr_index++];
        }
        if (para_size > curr_index) {
            tf.rotation.yaw() = paras[curr_index++];  //先解算yaw，后解算roll
        }
        if (para_size > curr_index) {
            tf.rotation.pitch() = paras[curr_index++];
        }
        if (para_size > curr_index) {
            tf.rotation.roll() = paras[curr_index++];
        }
        return tf;
    };

    auto& s = sros::core::Settings::getInstance();
    auto use_scan_to_alignment = (s.getValue<std::string>("slam.use_scan_to_alignment", "True") == "True");
    if (use_scan_to_alignment) {
        auto device_name = sros::device::DEVICE_LIDAR;
        auto laser_coordx = s.getValue("posefilter.laser_coordx", 0.29);
        auto laser_coordy = s.getValue("posefilter.laser_coordy", 0.0);
        auto laser_coordyaw = s.getValue("posefilter.laser_coordyaw", 0.0);
        slam::tf::TransForm laser_tf;
        laser_tf.position.x() = laser_coordx;
        laser_tf.position.y() = laser_coordy;
        laser_tf.rotation.yaw() = laser_coordyaw;
        sensor_transforms_[device_name] = laser_tf;
    }
    auto use_up_camera_to_alignment = (s.getValue<std::string>("slam.use_up_camera_to_alignment", "False") == "True");
    if (use_up_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_SVC100_UP;
        slam::tf::TransForm svc100_up_tf;
        sensor_transforms_[device_name] = svc100_up_tf;
    }
    auto use_down_camera_to_alignment =
        (s.getValue<std::string>("slam.use_down_camera_to_alignment", "True") == "True");
    if (use_down_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_SVC100_DOWN;
        slam::tf::TransForm svc100_down_tf;
        sensor_transforms_[device_name] = svc100_down_tf;
    }
    auto use_forward_camera_to_alignment =
        (s.getValue<std::string>("slam.use_forward_camera_to_alignment", "True") == "True");
    if (use_forward_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_CAMERA_FORWARD;
        std::string tf_str = s.getValue<std::string>("camera.forward_camera_install_pose", "0;");
        sensor_transforms_[device_name] = resolveTF(tf_str);
    }
    auto use_backward_camera_to_alignment =
        (s.getValue<std::string>("slam.use_backward_camera_to_alignment", "True") == "True");
    if (use_backward_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_CAMERA_BACKWARD;
        std::string tf_str = s.getValue<std::string>("camera.backward_camera_install_pose", "0;");
        sensor_transforms_[device_name] = resolveTF(tf_str);
    }
    auto use_left_camera_to_alignment =
        (s.getValue<std::string>("slam.use_left_camera_to_alignment", "True") == "True");
    if (use_left_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_CAMERA_LEFT;
        std::string tf_str = s.getValue<std::string>("camera.left_camera_install_pose", "0;");
        sensor_transforms_[device_name] = resolveTF(tf_str);
    }
    auto use_right_camera_to_alignment =
        (s.getValue<std::string>("slam.use_right_camera_to_alignment", "True") == "True");
    if (use_right_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_CAMERA_RIGHT;
        std::string tf_str = s.getValue<std::string>("camera.right_camera_install_pose", "0;");
        sensor_transforms_[device_name] = resolveTF(tf_str);
    }
}
bool FeatureExtractorModule::getSpecifyFeature(const MatchedFeatureInfo& matched_info,
                                               std::vector<extractor::FeatureInfo_ptr>& infos,
                                               extractor::FeatureInfo_ptr& specify_info) {
    auto curr_pose = src_sdk->getCurPose();
    slam::tf::TransForm curr_tf(0, curr_pose);
    for (auto& info : infos) {
        if (info->feature_name == matched_info.feature_name) {
            auto sensor_iter = sensor_transforms_.find(info->sensor_name);
            if (sensor_iter != sensor_transforms_.end()) {
                auto& sensor_tf = sensor_iter->second;
                slam::tf::TransForm sensor_in_world;
                sensor_tf.transformTF(info->pose_in_sensor, sensor_in_world);

                curr_tf.transformTF(sensor_in_world, sensor_in_world);
                Eigen::Vector3f pose_2d, matched_2d;
                sensor_in_world.get2DPose(pose_2d);
                matched_info.pose.get2DPose(matched_2d);

                // LOG(INFO) << "ref pose:" << matched_2d[0] << "," << matched_2d[1] << "," << matched_2d[2];
                // LOG(INFO) << "ext pose:" << pose_2d[0] << "," << pose_2d[1] << "," << pose_2d[2] << ","
                //           << (matched_2d - pose_2d).head<2>().norm();
                if ((matched_2d - pose_2d).head<2>().norm() < 0.2f) {  //TODO 设定阈值，若超过，则不进行match
                    specify_info = info;
                    return true;
                } else{
                    LOG(WARNING)<<"ref pose and ext pose dis > 0.2";
                }
            }else{
                LOG(INFO) << "cannot find name:" << info->sensor_name;
            }
        }
    }
    return false;
}
bool FeatureExtractorModule::convertToDataMsg(const extractor::FeatureInfo_ptr& info,
                                              sros::core::DataMatrixCodeMsg_ptr& data_msg) {
    auto sensor_iter = sensor_transforms_.find(info->sensor_name);
    if (sensor_iter != sensor_transforms_.end()) {
        auto& sensor_tf = sensor_iter->second;
        slam::tf::TransForm info_in_center;
        sensor_tf.transformTF(info->pose_in_sensor, info_in_center);
        data_msg->x_ = info_in_center.position.x();
        data_msg->y_ = info_in_center.position.y();
        data_msg->z_ = info_in_center.position.z();
        data_msg->angle_ = info_in_center.rotation.yaw();
        data_msg->pitch_ = info_in_center.rotation.pitch();
        data_msg->roll_ = info_in_center.rotation.roll();
        data_msg->code_type_ = info->code_type;
        data_msg->code_str_ = info->feature_name;
        data_msg->time_ = info->time;
        data_msg->camera_name_ = info->sensor_name;
        data_msg->state_ = DM_CODE_DETECTED;
        return true;
    }else{
        LOG(INFO) << "cannot find sensor:" << info->sensor_name;
        return false;
    }
}
void FeatureExtractorModule::computeMeanPose(const extractor::FeatureInfo_ptr& input_info,
                                             const std::deque<std::vector<extractor::FeatureInfo_ptr>>& info_queues,
                                             slam::tf::TransForm& pose) {
    std::vector<extractor::FeatureInfo_ptr> finded_infos;
    for (auto& infos : info_queues) {
        for (auto& info : infos) {
            if (isSameFeature(input_info, info, 0.05)) {
                finded_infos.push_back(info);
            }
        }
    }
    pose = slam::tf::TransForm();
    if (finded_infos.size()) {
        for (auto& info : finded_infos) {
            pose.position.x() += info->pose_in_sensor.position.x();
            pose.position.y() += info->pose_in_sensor.position.y();
            pose.position.z() += info->pose_in_sensor.position.z();  //目前只有位置影响比较大
        }
        pose.rotation = input_info->pose_in_sensor.rotation;
        pose.position.x() /= finded_infos.size();
        pose.position.y() /= finded_infos.size();
        pose.position.z() /= finded_infos.size();
    } else {
        pose = input_info->pose_in_sensor;
    }
}
bool FeatureExtractorModule::isSameFeature(const extractor::FeatureInfo_ptr& base_info,
                                           const extractor::FeatureInfo_ptr& test_info, const float& dist_thresh) {
    if (base_info->feature_name == test_info->feature_name) {
        Eigen::Vector3f base_pose, test_pose;
        base_info->pose_in_sensor.get2DPose(base_pose);
        test_info->pose_in_sensor.get2DPose(test_pose);
        if ((base_pose - test_pose).head<2>().norm() < dist_thresh) {
            return true;
        }
    }
    return false;
}
bool FeatureExtractorModule::convertToFeatureMsg(const extractor::FeatureInfo_ptr& info,
                                                 const slam::tf::TransForm& mean_tf,
                                                 sros::core::FeatureInfoMsg_ptr& msg_info) {

    slam::tf::TransForm feature_in_world;
    if (getWorldTF(info, mean_tf, feature_in_world)) {
        msg_info->feature_infos.emplace_back();
        auto& curr_info = msg_info->feature_infos.back();
        curr_info.pose_in_world_.location() = feature_in_world.position;
        curr_info.pose_in_world_.rotation() = feature_in_world.rotation;
        curr_info.code_type_ = info->code_type;
        curr_info.feature_name_ = info->feature_name;
        curr_info.sensor_name_ = info->sensor_name;
        // LOG(INFO) << "msg:" << curr_info.pose_in_world_.location().x() << "," << curr_info.pose_in_world_.location().y()
        //           << "," << curr_info.pose_in_world_.location().z() << "," << curr_info.feature_name_;
        for (auto& point : info->points) {
            curr_info.points_.push_back(point);
        }
        return true;
    }
    return false;
}
void FeatureExtractorModule::convertDataMsgToFeatureInfo(const sros::core::DataMatrixCodeMsg_ptr& msg_info,
                                                         extractor::FeatureInfo_ptr& feature_info) {
    feature_info->feature_name = msg_info->code_str_;
    feature_info->sensor_name = msg_info->camera_name_;
    feature_info->pose_in_sensor.position = sros::core::Location(msg_info->x_, msg_info->y_, msg_info->z_);
    feature_info->pose_in_sensor.rotation = sros::core::Rotation(msg_info->angle_, msg_info->pitch_, msg_info->roll_);
    slam::tf::TransForm zero_tf;
    feature_info->pose_in_sensor.transformTFByInverse(zero_tf, feature_info->pose_in_sensor);//默认的坐标为，相机在code坐标系下位姿，应该求个逆，获得code在相机坐标系位姿
    feature_info->code_type = msg_info->code_type_;
    feature_info->time = msg_info->time_;
}
bool FeatureExtractorModule::getWorldTF(const extractor::FeatureInfo_ptr& info, const slam::tf::TransForm& mean_tf,
                                        slam::tf::TransForm& world_tf) {
    auto curr_pose = src_sdk->getCurPose();
    slam::tf::TransForm curr_tf(0, curr_pose);
    slam::tf::TransForm sensor_in_world;
    auto sensor_iter = sensor_transforms_.find(info->sensor_name);
    if (sensor_iter != sensor_transforms_.end()) {
        curr_tf.transformTF(sensor_iter->second, sensor_in_world);
        // LOG(INFO) << "sensor_transforms_ info:" << sensor_iter->second.position.x() << "," << sensor_iter->second.position.y() << ","
        //           << sensor_iter->second.position.z() << "," << sensor_iter->second.rotation.yaw() << ","
        //           << sensor_iter->second.rotation.roll() << "," << sensor_iter->second.rotation.pitch() << ","
        //           << info->feature_name;
    }else{
        LOG(INFO) << "cannot get tf:" << info->sensor_name << ",maybe have not enable the sensor to align!";
        return false;
    }
    sensor_in_world.transformTF(mean_tf, world_tf);
    return true;
}
bool FeatureExtractorModule::checkAlignmentResult(const extractor::FeatureInfo_ptr& base_info,const slam::tf::TransForm& feature,
                                                  const FeatureExtractorModule::MatchedFeatureInfo& match_info,
                                                  std::shared_ptr<sros::core::CommonStateMsg<bool>>& state) {
    slam::tf::TransForm world_tf;
    getWorldTF(base_info, feature, world_tf);
    auto delta_dist = std::hypot(world_tf.position.x() - match_info.pose.position.x(),
                                 world_tf.position.y() - match_info.pose.position.y());

    LOG(INFO)<<"-----------------------------";
    LOG(INFO)<<"test alignment: "<<delta_dist;
    LOG(INFO)<<"test alignment: x "<<world_tf.position.x()<<" y: "<< world_tf.position.y();
    LOG(INFO)<<"test alignment: x "<<match_info.pose.position.x()<<" y: "<< match_info.pose.position.y();

    state.reset(new sros::core::CommonStateMsg<bool>("TOPIC_ALIGNMENT_STATE"));
    state->map_name = match_info.sensor_name;
    if (delta_dist < check_feature_dist_para) {
        state->state = true;
        state->failed_code_ = SUCCESSFUL;//识别成功
    }else{
        state->state = false;
        state->failed_code_ = ERROR_DEVIATION_ISLARGE;//最后识别特征距离偏差过大
    }
    return true;
}
void FeatureExtractorModule::sendUnWatchedState(const FeatureExtractorModule::MatchedFeatureInfo& match_info) {
    std::shared_ptr<sros::core::CommonStateMsg<bool>> check_state(
        new sros::core::CommonStateMsg<bool>("TOPIC_ALIGNMENT_STATE"));
    check_state->state = false;
    check_state->failed_code_ = ERROR_SPECIFIED_FEATURE_NOT_RECOGNIZED;//未识别到指定特征
    check_state->map_name = match_info.sensor_name;
    sendMsg(check_state);
}

bool FeatureExtractorModule::isMeasureEnough(const extractor::FeatureInfo_ptr& input_info,
                                             const std::deque<std::vector<extractor::FeatureInfo_ptr>>& info_queues,
                                             slam::tf::TransForm& pose) {
    std::vector<extractor::FeatureInfo_ptr> finded_infos;
    for (auto& infos : info_queues) {
        for (auto& info : infos) {
            if (isSameFeature(input_info, info, 0.05)) {
                finded_infos.push_back(info);
            }
        }
    }
    pose = slam::tf::TransForm();
    if (finded_infos.size() >= mean_queue_size_ - 1) {
        for (auto& info : finded_infos) {
            pose.position.x() += info->pose_in_sensor.position.x();
            pose.position.y() += info->pose_in_sensor.position.y();
            pose.position.z() += info->pose_in_sensor.position.z();  //目前只有位置影响比较大
        }
        pose.rotation = input_info->pose_in_sensor.rotation;
        pose.position.x() /= finded_infos.size();
        pose.position.y() /= finded_infos.size();
        pose.position.z() /= finded_infos.size();
        return true;
    } else {
        return false;
    }
}

}
