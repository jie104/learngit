/**
 * @file QRcode_detection.cpp
 * @brief 简述文件内容
 *
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/7/1
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "QRcode_detection.h"
#include "core/settings.h"
#include "core/msg/common_msg.hpp"

#include "core/logger.h"
#include "core/msg/data_matrix_code_msg.hpp"
#include "core/msg/odo_pose_msg.hpp"
#include "core/msg/PoseStampedMsg.h"
#include "core/tf/TF.h"
#include "core/tf/TFOperator.h"
#include "core/tf/transform_3d.hpp"
// CODE
namespace object_detector {

const double MM_TO_M = 0.001;
const double DEG_TO_RAD = M_PI / 180.0;

bool convertStringToQuaternion(const std::string &string_data, Eigen::Quaterniond &quat) {
    std::stringstream string_data_stream(string_data);
    std::string item;
    std::vector<double> data_array;
    try {
        while (std::getline(string_data_stream, item, ';')) {
            data_array.push_back(std::stod(item));
        }
    } catch (...) {
        LOG(INFO) << "cannot get quat! will return normal quat!";
        quat.setIdentity();
    }
    if (data_array.size() == 9) {
        LOG(INFO) << "get right size array!";
        Eigen::Matrix3d install_info = Eigen::Map<Eigen::Matrix3d>(data_array.data(), 3, 3);
        quat = install_info;
        return true;
    } else {
        LOG(INFO) << "cannot get right size! will set normal quat!";
        quat.setIdentity();
        return false;
    }
}

bool QRcodeDetection::isEnable() {
    auto &s = sros::core::Settings::getInstance();
    bool enable_load_detect = (s.getValue<std::string>("main.enable_load_detect", "False") == "True");
    bool enable_QRcode_detect = (s.getValue<std::string>("perception.detect_goods_type", "") == "QRCODE");
    return enable_load_detect && enable_QRcode_detect;
}

QRcodeDetection::QRcodeDetection(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic)
    : sendMsg_(sendMsg),
      subscribeTopic_(subscribeTopic),
      is_init_detector_(false),
      enable_process_(false),
      is_receive_cmd_(false),
      is_find_QRcode_(false),
      DETECTED_TIME_OUT(3000) {
    slam::tf::FrameToFrame base_to_odo_frame;
    base_to_odo_frame.parent_frame = "odom";
    base_to_odo_frame.child_frame = "base_link";
    tf_base_to_odo_ = std::make_shared<slam::tf::TFOperator>(base_to_odo_frame);

    auto &s = sros::core::Settings::getInstance();
    // 相机相对于AGV中心的位姿 (AGV前进方向为x正方向,右手坐标系)
    sros::core::Pose camera_in_agv;
    camera_in_agv.x() = s.getValue("camera.d435_2_install_x_offset", 390.0) * MM_TO_M;
    camera_in_agv.y() = s.getValue("camera.d435_2_install_y_offset", 0.0) * MM_TO_M;
    camera_in_agv.z() = s.getValue("camera.d435_2_install_height", 0.0) * MM_TO_M;
    camera_in_agv.yaw() = s.getValue("camera.d435_2_install_yaw", 0.0) * DEG_TO_RAD;
    camera_in_agv.pitch() = s.getValue("camera.d435_2_install_pitch", 0.0) * DEG_TO_RAD;
    camera_in_agv.roll() = s.getValue("camera.d435_2_install_roll", 0.0) * DEG_TO_RAD;
    std::string install_matrix = s.getValue<std::string>("camera.d435_2_calibration_matrix", "");

    Eigen::Quaterniond install_q;
//    convertStringToQuaternion(install_matrix, install_q);
    install_q = Eigen::AngleAxisd(camera_in_agv.pitch(), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(
                    M_PI_2, Eigen::Vector3d::UnitZ());  //相机坐标系是Y朝下，Z轴朝前，后面两个90°旋转是为了考虑这个
    auto curr_camera_q = install_q;
    // 插销中心相对于二维码检测相机在X和Y方向的偏移量(相机正前方为x正方向，右手坐标系)
    sros::core::Pose bolt_in_camera;  // TODO: 注意相机正前方为x正方向,右手坐标系
    bolt_in_camera.x() = s.getValue("perception.bolt_relative_camera_x_offset", 100.0) * MM_TO_M;
    bolt_in_camera.y() = s.getValue("perception.bolt_relative_camera_y_offset", 0.0) * MM_TO_M;
    bolt_in_camera.z() = -camera_in_agv.z();
    bolt_in_camera.yaw() = 0;
    this->camere_in_center_tf_ = slam::tf::Transform3D(
        Eigen::Vector3d(camera_in_agv.location().x(), camera_in_agv.location().y(), camera_in_agv.location().z()),
        install_q);
    camere_in_center_tf_.transformTFByInverse(slam::tf::Transform3D::zero(), center_in_camera_tf_);
    LOG(INFO) << "center_in_camera_tf_"
              << "," << center_in_camera_tf_.roll() << "," << center_in_camera_tf_.pitch()
              << "," << center_in_camera_tf_.yaw();
    camera_in_bolt_tf_ = slam::tf::Transform3D(Eigen::Vector3d(-bolt_in_camera.x(),-bolt_in_camera.y(),-bolt_in_camera.z()),install_q);
    camera_in_bolt_tf_.transformTFByInverse(slam::tf::Transform3D::zero(), bolt_in_camera_tf_);
  //取相反的意思是相对于bolt的方向，因为这里bolt和camera安装位置的角度是相同的
    this->camera_in_bolt_tf_.transformTFByInverse(slam::tf::TransForm(), this->bolt_in_camera_tf_);
    //插孔中心相对于二维码在X和Y方向的偏移量(二维码正前方为x正方向，右手坐标系)
    sros::core::Pose hole_in_QRcode;  // TODO: 注意二维码正前方为x正方向,右手坐标系
    hole_in_QRcode.z() = s.getValue("perception.hole_relative_QRcode_x_offset", 600.0) * MM_TO_M;
    hole_in_QRcode.x() = s.getValue("perception.hole_relative_QRcode_y_offset", 0.0) * MM_TO_M;
    hole_in_QRcode.yaw() = 0;
    Eigen::Quaterniond quat = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY());//二维码的Z轴朝实际的前方，需要将孔的朝向给变换过去。
    hole_in_QRcode_tf_ = slam::tf::Transform3D(Eigen::Vector3d(
        hole_in_QRcode.location().x(), hole_in_QRcode.location().y(), hole_in_QRcode.location().z()), quat);
    LOG(INFO) << "bolt info:" << bolt_in_camera_tf_.point().x() << "," << bolt_in_camera_tf_.point().y() << ","
              << bolt_in_camera_tf_.point().z() << ",," << camera_in_bolt_tf_.point().x() << ","
              << camera_in_bolt_tf_.point().y() << "," << camera_in_bolt_tf_.point().z();
    //    this->hole_in_QRcode_tf_.buildTFfrom2DPose(
//        Eigen::Vector3f(hole_in_QRcode.x(), hole_in_QRcode.y(), hole_in_QRcode.yaw()));
}

bool QRcodeDetection::init() {
    if (!isEnable()) {
        LOG(WARNING) << "QRcodeDetect module init fail running(disable)";
        return false;
    }

    LOG(INFO) << "QR code Detect module start running";
    subscribeTopic_("OdoPoseStamped", CALLBACK(&QRcodeDetection::onOdomMsg));
    subscribeTopic_("DM_CODE_INFO", CALLBACK(&QRcodeDetection::onQRcodeMsg));
    subscribeTopic_("TIMER_50MS", CALLBACK(&QRcodeDetection::checkDetectTime));

    is_init_detector_ = true;
    return true;
}

void QRcodeDetection::enableSensor() {
    LOG(INFO) << "send TOPIC_SVC100_ENABLE_PUBLISH msg: enableSensor";
    std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
    cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_D435_2;
    cmd_msg->flag = true;
    sendMsg_(cmd_msg);

    // TODO: camera_module::onEnableSVC100Msg函数没有处理cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_D435的打开与关闭
}

void QRcodeDetection::disableSensor() {
    LOG(INFO) << "send TOPIC_SVC100_ENABLE_PUBLISH msg: disableSensor";
    std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
    cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_D435_2;
    cmd_msg->flag = false;
    sendMsg_(cmd_msg);
}

void QRcodeDetection::startDetector() {
    enable_process_ = true;
    enableSensor();
}

void QRcodeDetection::stopDetector() {
    enable_process_ = false;
    is_find_QRcode_ = false;
    disableSensor();
}

void QRcodeDetection::onDetectCommandMsg(const sros::core::base_msg_ptr &msg) {
    if (!is_init_detector_) {
        LOG(WARNING) << "QRcode detector is disable!";
        return;
    }

    // Analyse detect command and parameter.
    PerceptionCommandMsgPtr cmd = std::dynamic_pointer_cast<PerceptionCommandMsg>(msg);
    LOGGER(INFO, ACTION_TASK) << "receive QRcode detect command : cmd:"
                              << cmd->command.detect_stage * 10 + cmd->command.object_type
                              << " pose:" << cmd->theory_pose.x() << "," << cmd->theory_pose.y() << ","
                              << cmd->theory_pose.yaw() << " goal_id," << cmd->goal_id;
    this->is_receive_cmd_ = true;
    this->command_ = cmd->command;
    this->last_found_QRcode_time_ = sros::core::util::get_time_in_ms();
    this->receive_cmd_time_ = sros::core::util::get_time_in_ms();
    if (cmd->command.is_enable) {
        startDetector();
    } else {
        if(enable_process_){
            stopDetector();
        }
    }
}

void QRcodeDetection::checkDetectTime(const sros::core::base_msg_ptr &msg) {
    if (!enable_process_){
        return;
    }

    auto send_over_time_msg = [&](const int64_t cur_time){
      auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
      result_msg->command = this->command_;
      result_msg->goal_in_global_pose.x() = .0f;
      result_msg->goal_in_global_pose.y() = .0f;
      result_msg->goal_in_global_pose.yaw() = .0f;
      result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
      result_msg->error_code = PerceptionStateMsg::ERROR_CODE_TIMEOUT;
      LOGGER(INFO, SROS) << "detect time out! " << cur_time - receive_cmd_time_ << ">" << DETECTED_TIME_OUT;
      is_receive_cmd_ = false;
      stopDetector();
      sendMsg_(result_msg);
    };

    if (is_find_QRcode_) {
        auto cur_time = sros::core::util::get_time_in_ms();
        if (cur_time > DETECTED_TIME_OUT + last_found_QRcode_time_) {
            send_over_time_msg(cur_time);
        }
    } else if (is_receive_cmd_) {
        auto cur_time = sros::core::util::get_time_in_ms();
        if (cur_time > DETECTED_TIME_OUT + receive_cmd_time_) {
            send_over_time_msg(cur_time);
        }
    }
}


void QRcodeDetection::onOdomMsg(const sros::core::base_msg_ptr &msg) {
   if (!enable_process_) {
       return;
   }

    auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
    if (!is_find_QRcode_) {
        return;
    }

    if (code_with_odo_pairs_.empty()) {
        return;
    }

    auto odo_pose_stamped = std::dynamic_pointer_cast<sros::core::PoseStampedMsg>(msg);
    slam::tf::TransForm odo_tf(odo_pose_stamped->time_, odo_pose_stamped->pose);
    slam::tf::Transform3D odo_tf_3d(odo_tf);
    std::vector<slam::tf::Transform3D> bolt_in_holes;
    auto pairs = code_with_odo_pairs_;
    if (code_with_odo_pairs_.size() >= 5) {
        code_with_odo_pairs_.pop_front();
    }
    std::vector<double> xes,ys,yaws;
    for (auto &pair : pairs) {
        bolt_in_holes.push_back(getBoltInHoleTF(pair.code_pose, pair.odo_pose, odo_tf_3d));
        xes.push_back(bolt_in_holes.back().point()[0]);
        ys.push_back(bolt_in_holes.back().point()[1]);
        yaws.push_back(bolt_in_holes.back().yaw());
    }
    Eigen::Vector3d bolt_in_hole;
    std::nth_element(xes.begin(), xes.begin() + xes.size() / 2, xes.end());
    bolt_in_hole[0] = xes[xes.size() / 2];
    std::nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
    bolt_in_hole[1] = ys[ys.size() / 2];
    std::nth_element(yaws.begin(), yaws.begin() + yaws.size() / 2, yaws.end());
    bolt_in_hole[2] = yaws[yaws.size() / 2];

    result_msg->command = this->command_;
    result_msg->goal_in_global_pose.x() =  bolt_in_hole[0];
    result_msg->goal_in_global_pose.y() = bolt_in_hole[1];
    result_msg->goal_in_global_pose.yaw() = bolt_in_hole[2];
    result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_SUCCESS;
    result_msg->error_code = PerceptionStateMsg::ERROR_CODE_EXISTENCE_OBSTACLE;
    result_msg->goal_id = this->QRcode_id_;

    sendMsg_(result_msg);
    // 控制1秒钟刷新一次位姿信息
    static int count = 0;
    if (0 == count % 100){
        LOG(INFO) << "            bolt in hole world pose:  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << result_msg->goal_in_global_pose.x() << ", "
                  << result_msg->goal_in_global_pose.y() << ", " << result_msg->goal_in_global_pose.yaw();
//        slam::tf::Transform3D center_in_bolt;
//        hole_in_QRcode_tf_.transformTFByInverse(center_in_realtime_code, center_in_bolt);
//        LOG(INFO) << "center_in_realtime_code" << center_in_bolt.point()[0] << ","
//                  << center_in_bolt.point()[1] << "," << center_in_bolt.point()[2] << ","
//                  << center_in_bolt.yaw() << "," << center_in_bolt.roll() << ","
//                  << center_in_bolt.pitch();
        //        LOG(INFO) << "camera_in_QRcode_pose_"
//                  << "," << camera_in_QRcode_pose_.roll() << "," << camera_in_QRcode_pose_.pitch()
//                  << "," << camera_in_QRcode_pose_.yaw();
//        LOG(INFO) << "center_in_camera_tf_"
//                  << "," << center_in_camera_tf_.roll() << "," << center_in_camera_tf_.pitch()
//                  << "," << center_in_camera_tf_.yaw();
//        LOG(INFO) << "camere_in_center_tf_"
//                  << "," << camere_in_center_tf_.roll() << "," << camere_in_center_tf_.pitch()
//                  << "," << camere_in_center_tf_.yaw();
//        LOG(INFO) << "center_in_code"
//                  << "," << center_in_code.roll() << "," << center_in_code.pitch()
//                  << "," << center_in_code.yaw();
//        LOG(INFO) << "delta_in_last_odo"
//                  << "," << delta_in_last_odo.roll() << "," << delta_in_last_odo.pitch()
//                  << "," << delta_in_last_odo.yaw();
//        LOG(INFO) << "camera_in_realtime_code"
//                  << "," << camera_in_realtime_code.roll() << "," << camera_in_realtime_code.pitch()
//                  << "," << camera_in_realtime_code.yaw();
//        LOG(INFO) << "bolt_in_realtime_code"
//                  << "," << bolt_in_realtime_code.roll() << "," << bolt_in_realtime_code.pitch()
//                  << "," << bolt_in_realtime_code.yaw();
//        LOG(INFO) << "bolt_in_hole"
//                  << "," << bolt_in_hole.roll() << "," << bolt_in_hole.pitch()
//                  << "," << bolt_in_hole.yaw();
    }
    count++;
}

void QRcodeDetection::onQRcodeMsg(const sros::core::base_msg_ptr &msg) {
   if (!enable_process_) {
       return;
   }

    sros::core::DataMatrixCodeMsg_ptr QRcode_info = std::dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(msg);
    if (QRcode_info->state_ == DM_CODE_NOT_DETECTED) {
        return;
    }

    if (!is_find_QRcode_) {
        is_find_QRcode_ = true;
    }

    slam::tf::TransForm curr_pose;
    if (!tf_base_to_odo_->lookUpTransForm(msg->time_, curr_pose, 50000)) {
        LOG(INFO) << "delta time is wrong! cannot get real time pose!";
        return;
    }
    this->odom_base_tf_ = slam::tf::Transform3D(curr_pose);
    this->QRcode_id_ = QRcode_info->code_int_;
    this->last_found_QRcode_time_ = QRcode_info->getTimestamp();
    this->camera_in_QRcode_pose_.point().x() = QRcode_info->x_;
    this->camera_in_QRcode_pose_.point().y() = QRcode_info->y_;
    this->camera_in_QRcode_pose_.point().z() = QRcode_info->z_;
//    this->camera_in_QRcode_pose_.rotation.yaw() = QRcode_info->angle_;
//    this->camera_in_QRcode_pose_.rotation.pitch() = QRcode_info->pitch_;
//    this->camera_in_QRcode_pose_.rotation.roll() = QRcode_info->roll_;
    Eigen::Quaterniond quat = Eigen::AngleAxisd(QRcode_info->angle_, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(QRcode_info->pitch_, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(QRcode_info->roll_, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond rotate = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());//这里相机的实际朝向与二维码给出的朝向相差了一个X轴180°的旋转。

    camera_in_QRcode_pose_.rot() = (quat * rotate.inverse());
    if(checkDetectAngleThresh(camera_in_QRcode_pose_,0.30)) {
        QRCodeWidthOdoPair odo_pair;
        odo_pair.odo_pose = odom_base_tf_;
        odo_pair.code_pose = camera_in_QRcode_pose_;
        code_with_odo_pairs_.push_back(odo_pair);
    }
//    LOG(INFO) << "pose info:" << camera_in_QRcode_pose_.point()[0] << "," << camera_in_QRcode_pose_.point()[1];
    //    camera_in_QRcode_pose_.position.x() = QRcode_info->x_;
//    camera_in_QRcode_pose_.position.y() = QRcode_info->y_;
//    camera_in_QRcode_pose_.position.z() = QRcode_info->z_;
//    sros::core::Rotation rot;
//    camera_in_QRcode_pose_.getRotation(rot);
}
bool QRcodeDetection::checkDetectAngleThresh(const slam::tf::Transform3D &camera_in_code, double yaw_thresh) {
    slam::tf::Transform3D bolt_in_code,bolt_in_hole;
    camera_in_code.transformTF(bolt_in_camera_tf_, bolt_in_code);
    hole_in_QRcode_tf_.transformTFByInverse(bolt_in_code, bolt_in_hole);

    auto real_yaw = atan2(sin(bolt_in_hole.yaw()), cos(bolt_in_hole.yaw()));
    if (fabs(real_yaw) >= yaw_thresh) {
        LOG(INFO) << "bolt_in_hole:" << bolt_in_hole.point()[0] << "," << bolt_in_hole.point()[1] << "<< "
                  << bolt_in_hole.yaw() << ", " << bolt_in_hole.pitch() << ", " << bolt_in_hole.roll();
        return false;
    }
    return true;
}
slam::tf::Transform3D QRcodeDetection::getBoltInHoleTF(const slam::tf::Transform3D &code_pose,
                                                       const slam::tf::Transform3D &odo_pose,
                                                       const slam::tf::Transform3D &real_time_odo) {

    slam::tf::Transform3D center_in_code, delta_in_last_odo, center_in_realtime_code, camera_in_realtime_code,
        bolt_in_realtime_code, bolt_in_hole;
    code_pose.transformTF(center_in_camera_tf_, center_in_code);
    odo_pose.transformTFByInverse(real_time_odo, delta_in_last_odo);
    center_in_code.transformTF(delta_in_last_odo,
                               center_in_realtime_code);  //获取到当前运动中心在code坐标系中心的位置。
    center_in_realtime_code.transformTF(camere_in_center_tf_,
                                        camera_in_realtime_code);  //获取相机在code坐标系中心位置
    camera_in_realtime_code.transformTF(bolt_in_camera_tf_,
                                        bolt_in_realtime_code);  //获取对接在添加里程计补偿条件下位姿
    hole_in_QRcode_tf_.transformTFByInverse(bolt_in_realtime_code, bolt_in_hole);  //计算对接机构在孔坐标系下位姿
    return bolt_in_hole;
}
}
