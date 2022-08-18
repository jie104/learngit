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
#include "QRcode_detection_global.h"
#include "core/settings.h"
#include "core/msg/common_msg.hpp"

#include "core/logger.h"
#include "core/msg/odo_pose_msg.hpp"
#include "core/msg/PoseStampedMsg.h"
#include "core/tf/TF.h"
#include "core/tf/TFOperator.h"
#include "core/tf/transform_3d.hpp"
// CODE
namespace object_detector {

const double MM_TO_M = 0.001;
const double DEG_TO_RAD = M_PI / 180.0;


bool QRcodeDetectionGlobal::isEnable() {
    auto &s = sros::core::Settings::getInstance();
    bool enable_load_detect = (s.getValue<std::string>("main.enable_load_detect", "False") == "True");
    bool enable_QRcode_detect = (s.getValue<std::string>("perception.detect_goods_type", "") == "QRCODE");
    return enable_load_detect && enable_QRcode_detect;
}

QRcodeDetectionGlobal::QRcodeDetectionGlobal(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic)
    : sendMsg_(sendMsg),
      subscribeTopic_(subscribeTopic),
      is_init_detector_(false),
      enable_process_(false),
      is_receive_cmd_(false),
      is_find_QRcode_(false),
      DETECTED_TIME_OUT(5000),
      flag_confirm_QRcode_target_(false){
    slam::tf::FrameToFrame base_to_odo_frame;
    base_to_odo_frame.parent_frame = "odom";
    base_to_odo_frame.child_frame = "base_link";
    tf_base_to_odo_ = std::make_shared<slam::tf::TFOperator>(base_to_odo_frame);

    // add by ljh 20211020
    slam::tf::FrameToFrame base_to_world_frame;
    base_to_world_frame.parent_frame = "world";
    base_to_world_frame.child_frame = "base_link";
    tf_base_to_world_ = std::make_shared<slam::tf::TFOperator>(base_to_world_frame);

    auto &s = sros::core::Settings::getInstance();
    // 相机相对于AGV中心的位姿 (AGV前进方向为x正方向,右手坐标系)
    sros::core::Pose camera_in_agv;
    camera_in_agv.x() = -s.getValue("camera.mvs_ce013_install_x_offset", 390.0) * MM_TO_M;
    camera_in_agv.y() = s.getValue("camera.mvs_ce013_install_y_offset", 0.0) * MM_TO_M;
    camera_in_agv.z() = s.getValue("camera.mvs_ce013_install_height", 0.0) * MM_TO_M;
    camera_in_agv.yaw() = s.getValue("camera.mvs_ce013_install_yaw", 0.0) * DEG_TO_RAD;
    camera_in_agv.pitch() = s.getValue("camera.mvs_ce013_install_pitch", 0.0) * DEG_TO_RAD;
    camera_in_agv.roll() = s.getValue("camera.mvs_ce013_install_roll", 0.0) * DEG_TO_RAD;
    //std::string install_matrix = s.getValue<std::string>("camera.d435_2_calibration_matrix", "");

    // add by ljh 20211020
    camera_in_agv_ = camera_in_agv;

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

bool QRcodeDetectionGlobal::init() {
    if (!isEnable()) {
        LOG(WARNING) << "QRcodeDetect module init fail running(disable)";
        return false;
    }

    LOG(INFO) << "QR code Detect module start running";
    subscribeTopic_("OdoPoseStamped", CALLBACK(&QRcodeDetectionGlobal::onOdomMsg));
    subscribeTopic_("DM_CODE_INFO", CALLBACK(&QRcodeDetectionGlobal::onQRcodeMsg));
    subscribeTopic_("TIMER_50MS", CALLBACK(&QRcodeDetectionGlobal::checkDetectTime));

    is_init_detector_ = true;
    return true;
}

void QRcodeDetectionGlobal::enableSensor() {
    LOG(INFO) << "send TOPIC_SVC100_ENABLE_PUBLISH msg: enableSensor";
    std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
    cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_MV_CE013;
    cmd_msg->flag = true;
    sendMsg_(cmd_msg);

    // TODO: camera_module::onEnableSVC100Msg函数没有处理cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_D435的打开与关闭
}

void QRcodeDetectionGlobal::disableSensor() {
    LOG(INFO) << "send TOPIC_SVC100_ENABLE_PUBLISH msg: disableSensor";
    std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
    cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_MV_CE013;
    cmd_msg->flag = false;
    sendMsg_(cmd_msg);
}

void QRcodeDetectionGlobal::startDetector() {
    enable_process_ = true;
    flag_confirm_QRcode_target_=false;
    target_QRcode_id = -1;
    enableSensor();
}

void QRcodeDetectionGlobal::stopDetector() {
    enable_process_ = false;
    is_find_QRcode_ = false;
    flag_confirm_QRcode_target_=false;
    target_QRcode_id = -1;
    disableSensor();
}

void QRcodeDetectionGlobal::onDetectCommandMsg(const sros::core::base_msg_ptr &msg) {
    if (!is_init_detector_) {
        LOG(WARNING) << "QRcode detector is disable!";
        return;
    }

    // Analyse detect command and parameter.
    PerceptionCommandMsgPtr cmd = std::dynamic_pointer_cast<PerceptionCommandMsg>(msg);
    LOGGER(INFO, ACTION_TASK) << "receive QRcode detect command : cmd:"
                              << cmd->command.detect_stage * 10 + cmd->command.object_type
                              << " pose:" << cmd->theory_pose.x() << "," << cmd->theory_pose.y() << ","
                              << cmd->theory_pose.yaw() << " goal_id," << cmd->goal_id
                              << " is_enable=" << cmd->command.is_enable;

    // 存储目标货车所在站点坐标 用于目标二维码的在全局坐标系的比较
    this->theory_pose_[0] = cmd->theory_pose.x();
    this->theory_pose_[1] = cmd->theory_pose.y();
    this->theory_pose_[2] = cmd->theory_pose.yaw();

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

void QRcodeDetectionGlobal::checkDetectTime(const sros::core::base_msg_ptr &msg) {
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


void QRcodeDetectionGlobal::onOdomMsg(const sros::core::base_msg_ptr &msg) {
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

//    result_msg->command = this->command_;
//    result_msg->goal_in_global_pose.x() =  bolt_in_hole[0];
//    result_msg->goal_in_global_pose.y() = bolt_in_hole[1];
//    result_msg->goal_in_global_pose.yaw() = bolt_in_hole[2];
//    result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_SUCCESS;
//    result_msg->error_code = PerceptionStateMsg::ERROR_CODE_EXISTENCE_OBSTACLE;
//    result_msg->goal_id = this->QRcode_id_;
//
//    sendMsg_(result_msg);
    // 控制1秒钟刷新一次位姿信息
    static int count = 0;
    if (0 == count % 100){
        LOG(INFO) << "bolt in hole world pose:  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << result_msg->goal_in_global_pose.x() << ", "
                  << result_msg->goal_in_global_pose.y() << ", " << result_msg->goal_in_global_pose.yaw();
//        slam::tf::Transform3D center_in_bolt;
//        hole_in_QRcode_tf_.transformTFByInverse(center_in_realtime_code, center_in_bolt);
//        LOG(INFO) << "center_in_realtime_code" << center_in_bolt.point()[0] << ","
//                  << center_in_bolt.point()[1] << "," << center_in_bolt.point()[2] << ","
//                  << center_in_bolt.yaw() << "," << center_in_bolt.roll() << ","
//                  << center_in_bolt.pitch();
//                LOG(INFO) << "#camera_in_QRcode_pose_"
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

void QRcodeDetectionGlobal::onQRcodeMsg(const sros::core::base_msg_ptr &msg) {
    if (!enable_process_) {
        return;
    }

    sros::core::DataMatrixCodeMsg_ptr QRcode_info = std::dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(msg);
    if (QRcode_info->state_ == DM_CODE_NOT_DETECTED) {
        return;
    }

    // 通过目的二维码ID以及空间坐标距离判断识别到的二维码是否为需要牵引的料车，避免误牵引其他料车  add by ljh 2021019
    if (flag_confirm_QRcode_target_ == false){
        // 判断二维码是否为目标料车
        if(judgeQRcodeIsTarget(QRcode_info, 1.0, 1.0, 1.0, true)){
            this->target_QRcode_id = QRcode_info->code_int_;
            //this->flag_confirm_QRcode_target_ = true;
        }
        else {
            LOG(INFO) << "Fisrt:Judget QRcode is not Target.";
            return;
        }
    }
    else{
        // 判断二维码ID 是否一致
        if(this->target_QRcode_id != QRcode_info->code_int_){
            LOG(INFO) << "QRcode ID is not Target ID.";
            return ;
        }

        // 判断二维码是否为目标料车
        if (!judgeQRcodeIsTarget(QRcode_info, 1.0, 1.0, 1.0)){
            LOG(INFO) << "Judget QRcode is not Target.";
            return ;
        }
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
    if(checkDetectAngleThresh(camera_in_QRcode_pose_,0.50)) {
        QRCodeGlobalWidthOdoPair odo_pair;
        odo_pair.odo_pose = odom_base_tf_;
        odo_pair.code_pose = camera_in_QRcode_pose_;
        code_with_odo_pairs_.push_back(odo_pair);
    }
    else{
        if(flag_confirm_QRcode_target_==false){
            auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
            result_msg->command = this->command_;
            result_msg->goal_in_global_pose.x() = .0f;
            result_msg->goal_in_global_pose.y() = .0f;
            result_msg->goal_in_global_pose.yaw() = .0f;
            result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
            result_msg->error_code = PerceptionStateMsg::ERROR_CODE_COORDINATE_OUT_RANGE;
            LOGGER(INFO, SROS) << "first detect QRcode angle too max (yaw thresh = 0.50). ";
            is_receive_cmd_ = false;
            stopDetector();
            sendMsg_(result_msg);
        }
    }

    if(flag_confirm_QRcode_target_ == false){
        this->flag_confirm_QRcode_target_ = true;
    }


//    LOG(INFO) << "pose info:" << camera_in_QRcode_pose_.point()[0] << "," << camera_in_QRcode_pose_.point()[1];
    //    camera_in_QRcode_pose_.position.x() = QRcode_info->x_;
//    camera_in_QRcode_pose_.position.y() = QRcode_info->y_;
//    camera_in_QRcode_pose_.position.z() = QRcode_info->z_;
//    sros::core::Rotation rot;
//    camera_in_QRcode_pose_.getRotation(rot);
}
bool QRcodeDetectionGlobal::checkDetectAngleThresh(const slam::tf::Transform3D &camera_in_code, double yaw_thresh) {
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
slam::tf::Transform3D QRcodeDetectionGlobal::getBoltInHoleTF(const slam::tf::Transform3D &code_pose,
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





bool QRcodeDetectionGlobal::computeDistanceQRcodeToTarget(sros::core::DataMatrixCodeMsg_ptr QRcode_info, float distance){

    // 1. get  base in world pose
    slam::tf::TransForm base_in_world_pose_2d;
    if (!tf_base_to_world_->lookUpTransForm(QRcode_info->time_, base_in_world_pose_2d, 50000)) {
        LOG(INFO) << "delta time is wrong! cannot get real time pose ( base to world 2d)!";
        distance = 0 ;
        return false;
    }

    // 2. compute QRcode in camera pose
    this->camera_in_QRcode_pose_.point().x() = QRcode_info->x_;
    this->camera_in_QRcode_pose_.point().y() = QRcode_info->y_;
    this->camera_in_QRcode_pose_.point().z() = QRcode_info->z_;
    Eigen::Quaterniond quat_pose = Eigen::AngleAxisd(QRcode_info->angle_, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(QRcode_info->pitch_, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(QRcode_info->roll_, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond rotate_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());//这里相机的实际朝向与二维码给出的朝向相差了一个X轴180°的旋转。
    camera_in_QRcode_pose_.rot() = (quat_pose * rotate_pose.inverse());
    this->camera_in_QRcode_pose_.transformTFByInverse(slam::tf::Transform3D::zero(), this->QRcode_in_camera_pose_);

    slam::tf::TransForm QRcode_in_camera_pose_2d(QRcode_info->time_, sros::core::Pose(this->QRcode_in_camera_pose_.point().x(),
                                                                              this->QRcode_in_camera_pose_.point().y(),
                                                                              this->QRcode_in_camera_pose_.pitch()));

    // 3. get camere in base pose
    slam::tf::TransForm camera_in_base_pose_2d(QRcode_info->time_, this->camera_in_agv_);

    // 4. compute  qrcode in world pose
    slam::tf::TransForm QRcode_in_base_pose_2d;
    camera_in_base_pose_2d.transformTF(QRcode_in_camera_pose_2d, QRcode_in_base_pose_2d);
    slam::tf::TransForm QRcode_in_world_pose_2d;
    base_in_world_pose_2d.transformTF(QRcode_in_base_pose_2d, QRcode_in_world_pose_2d);

    // 5. compute qrcode (0,0,0) point tranform to world coordinate;
    Eigen::Vector2d QRcode_center(0,0);
    Eigen::Vector2d QRcode_center_world;
    QRcode_in_world_pose_2d.transform2DPoint(QRcode_center, QRcode_center_world);

    float distance_code_station = distanceBothPoint2d(QRcode_center_world.x(), QRcode_center_world.y(),
                                                      this->theory_pose_.x(), this->theory_pose_.y());

    LOG(INFO) << " target theory locations (x ,y , yaw):  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << this->theory_pose_[0] << ", "
              << this->theory_pose_[1] << ", " << this->theory_pose_[2];

    LOG(INFO) << " qrcode locations (x ,y , yaw):  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << QRcode_center_world.x() << ", "
              << QRcode_center_world.y() ;

    LOG(INFO) << "Distance qrcode to target(m):  "<<distance_code_station;

    return true;
}

bool QRcodeDetectionGlobal::computeDistanceQRcodeToTarget3D(sros::core::DataMatrixCodeMsg_ptr QRcode_info, float distance){
    // 1. get  base in world pose
    slam::tf::TransForm base_in_world_pose_2d;
    if (!tf_base_to_world_->lookUpTransForm(QRcode_info->time_, base_in_world_pose_2d, 50000)) {
        LOG(INFO) << "delta time is wrong! cannot get real time pose ( base to world 2d)!";
        distance = 0 ;
        return false;
    }
    slam::tf::Transform3D base_in_world_pose;
    base_in_world_pose = slam::tf::Transform3D(base_in_world_pose_2d);

    Eigen::Vector3d ptf;
    base_in_world_pose.transformPoint(Eigen::Vector3d(0,0,0),ptf);

    LOG(INFO) << "#1 base in world pose:  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << base_in_world_pose.point().x() << ", "
              << base_in_world_pose.point().y() << ", " << base_in_world_pose.point().z()<<","
              << base_in_world_pose.roll() << ", " << base_in_world_pose.pitch()
              << ", " << base_in_world_pose.yaw();
    LOG(INFO) << "@1 base in world point:  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << ptf.x() << ", "
              << ptf.y() << ", " << ptf.z();


    // 2. get camere in base pose

    Eigen::Quaterniond  camera_to_base_quater;
    camera_to_base_quater = Eigen::AngleAxisd (M_PI_2, Eigen::Vector3d::UnitZ())*
                            Eigen::AngleAxisd ( 0, Eigen::Vector3d::UnitY())*
                            Eigen::AngleAxisd ( -M_PI_2,  Eigen::Vector3d::UnitX());
    slam::tf::Transform3D camera_in_base_pose(Eigen::Vector3d(camera_in_agv_.location().x(),camera_in_agv_.location().y(),
                                                              camera_in_agv_.location().z()),camera_to_base_quater);



    LOG(INFO) << "#2 camera in base pose:  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << camera_in_base_pose.point().x() << ", "
              << camera_in_base_pose.point().y() << ", " << camera_in_base_pose.point().z()<<","
              << camera_in_base_pose.roll() << ", " << camera_in_base_pose.pitch()
              << ", " << camera_in_base_pose.yaw();

    camera_in_base_pose.transformPoint(Eigen::Vector3d(0,0,0),ptf);

    LOG(INFO) << "@2 camera in base pose:   " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << ptf.x() << ", "
              << ptf.y() << ", " << ptf.z();


    // 3. compute QRcode in camera pose
    this->camera_in_QRcode_pose_.point().x() = QRcode_info->x_;
    this->camera_in_QRcode_pose_.point().y() = QRcode_info->y_;
    this->camera_in_QRcode_pose_.point().z() = QRcode_info->z_;
    Eigen::Quaterniond quat_pose = Eigen::AngleAxisd(QRcode_info->angle_, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(QRcode_info->pitch_, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(QRcode_info->roll_, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond rotate_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());//这里相机的实际朝向与二维码给出的朝向相差了一个X轴180°的旋转。
    camera_in_QRcode_pose_.rot() = (quat_pose* rotate_pose.inverse() );
    this->camera_in_QRcode_pose_.transformTFByInverse(slam::tf::Transform3D::zero(), this->QRcode_in_camera_pose_);

    LOG(INFO) << "#3 QRcode in camera pose:  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << QRcode_in_camera_pose_.point().x() << ", "
              << QRcode_in_camera_pose_.point().y() << ", " << QRcode_in_camera_pose_.point().z()<<","
              << QRcode_in_camera_pose_.roll() << ", " << QRcode_in_camera_pose_.pitch()
              << ", " << QRcode_in_camera_pose_.yaw();

    QRcode_in_camera_pose_.transformPoint(Eigen::Vector3d(0,0,0),ptf);

    LOG(INFO) << "@3 QRcode in camera point:   " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << ptf.x() << ", "
              << ptf.y() << ", " << ptf.z();


    // 4. compute  qrcode in world pose
    slam::tf::Transform3D QRcode_in_base_pose;
    camera_in_base_pose.transformTF(QRcode_in_camera_pose_, QRcode_in_base_pose);

    LOG(INFO) << "#4-1 QRcode in base pose:  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << QRcode_in_base_pose.point().x() << ", "
              << QRcode_in_base_pose.point().y() << ", " << QRcode_in_base_pose.point().z()<<","
              << QRcode_in_base_pose.roll() << ", " << QRcode_in_base_pose.pitch()
              << ", " << QRcode_in_base_pose.yaw();

    QRcode_in_base_pose.transformPoint(Eigen::Vector3d(0,0,0),ptf);

    LOG(INFO) << "@4-1 QRcode in base point:   " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << ptf.x() << ", "
              << ptf.y() << ", " << ptf.z();

    slam::tf::Transform3D QRcode_in_world_pose;
    base_in_world_pose.transformTF(QRcode_in_base_pose, QRcode_in_world_pose);

    LOG(INFO) << "#4-2 QRcode in world pose:  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << QRcode_in_world_pose.point().x() << ", "
              << QRcode_in_world_pose.point().y() << ", " << QRcode_in_world_pose.point().z()<<","
              << QRcode_in_world_pose.roll() << ", " << QRcode_in_world_pose.pitch()
              << ", " << QRcode_in_world_pose.yaw();

    QRcode_in_world_pose.transformPoint(Eigen::Vector3d(0,0,0),ptf);

    LOG(INFO) << "@4-2 QRcode in world point:   " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << ptf.x() << ", "
              << ptf.y() << ", " << ptf.z();

    // 5. compute qrcode (0,0,0) point tranform to world coordinate;
    Eigen::Vector3d QRcode_center(0,0, 0);
    Eigen::Vector3d QRcode_center_world;
    QRcode_in_world_pose.transformPoint(QRcode_center, QRcode_center_world);

    float distance_code_station = distanceBothPoint2d(QRcode_center_world.x(), QRcode_center_world.y(),
                                                      this->theory_pose_.x(), this->theory_pose_.y());

    LOG(INFO) << " target theory locations (x ,y , yaw):  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << this->theory_pose_[0] << ", "
              << this->theory_pose_[1] << ", " << this->theory_pose_[2];

    LOG(INFO) << " qrcode locations (x ,y , z):  " << std::setw(8) << std::setfill(' ')
              << std::setiosflags(std::ios::left) << QRcode_center_world.x() << ", "
              << QRcode_center_world.y() << ", " << QRcode_center_world.z();

    LOG(INFO) << "Distance qrcode to target(m):  "<<distance_code_station;

    return true;
}

bool QRcodeDetectionGlobal::judgeQRcodeIsTarget(sros::core::DataMatrixCodeMsg_ptr QRcode_info , float dist_thresh , float xdist_thresh  , float ydist_thresh, bool first_code ) {

        // 1. get  base in world pose
        slam::tf::TransForm base_in_world_pose_2d;
        if (!tf_base_to_world_->lookUpTransForm(QRcode_info->time_, base_in_world_pose_2d, 50000)) {
            LOG(INFO) << "delta time is wrong! cannot get real time pose ( base to world 2d)!";
            return false;
        }

        slam::tf::Transform3D base_in_world_pose;
        base_in_world_pose = slam::tf::Transform3D(base_in_world_pose_2d);

        Eigen::Vector3d ptf;
        base_in_world_pose.transformPoint(Eigen::Vector3d(0,0,0),ptf);

        LOG(INFO) << "#1 base in world pose:  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << base_in_world_pose.point().x() << ", "
                  << base_in_world_pose.point().y() << ", " << base_in_world_pose.point().z()<<","
                  << base_in_world_pose.roll() << ", " << base_in_world_pose.pitch()
                  << ", " << base_in_world_pose.yaw();
        LOG(INFO) << "@1 base in world point:  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << ptf.x() << ", "
                  << ptf.y() << ", " << ptf.z();


        // 2. get camere in base pose
        Eigen::Quaterniond  camera_to_base_quater;
        camera_to_base_quater = Eigen::AngleAxisd (M_PI_2, Eigen::Vector3d::UnitZ())*
                                Eigen::AngleAxisd ( 0, Eigen::Vector3d::UnitY())*
                                Eigen::AngleAxisd ( -M_PI_2,  Eigen::Vector3d::UnitX());
        slam::tf::Transform3D camera_in_base_pose(Eigen::Vector3d(camera_in_agv_.location().x(),camera_in_agv_.location().y(),
                                                                  camera_in_agv_.location().z()),camera_to_base_quater);


        LOG(INFO) << "#2 camera in base pose:  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << camera_in_base_pose.point().x() << ", "
                  << camera_in_base_pose.point().y() << ", " << camera_in_base_pose.point().z()<<","
                  << camera_in_base_pose.roll() << ", " << camera_in_base_pose.pitch()
                  << ", " << camera_in_base_pose.yaw();

        camera_in_base_pose.transformPoint(Eigen::Vector3d(0,0,0),ptf);

        LOG(INFO) << "@2 camera in base pose:   " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << ptf.x() << ", "
                  << ptf.y() << ", " << ptf.z();


        // 3. compute QRcode in camera pose
        this->camera_in_QRcode_pose_.point().x() = QRcode_info->x_;
        this->camera_in_QRcode_pose_.point().y() = QRcode_info->y_;
        this->camera_in_QRcode_pose_.point().z() = QRcode_info->z_;
        Eigen::Quaterniond quat_pose = Eigen::AngleAxisd(QRcode_info->angle_, Eigen::Vector3d::UnitZ()) *
                                       Eigen::AngleAxisd(QRcode_info->pitch_, Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(QRcode_info->roll_, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond rotate_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());//这里相机的实际朝向与二维码给出的朝向相差了一个X轴180°的旋转。
        camera_in_QRcode_pose_.rot() = (quat_pose* rotate_pose.inverse() );
        this->camera_in_QRcode_pose_.transformTFByInverse(slam::tf::Transform3D::zero(), this->QRcode_in_camera_pose_);

        LOG(INFO) << "#3 QRcode in camera pose:  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << QRcode_in_camera_pose_.point().x() << ", "
                  << QRcode_in_camera_pose_.point().y() << ", " << QRcode_in_camera_pose_.point().z()<<","
                  << QRcode_in_camera_pose_.roll() << ", " << QRcode_in_camera_pose_.pitch()
                  << ", " << QRcode_in_camera_pose_.yaw();

        QRcode_in_camera_pose_.transformPoint(Eigen::Vector3d(0,0,0),ptf);

        LOG(INFO) << "@3 QRcode in camera point:   " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << ptf.x() << ", "
                  << ptf.y() << ", " << ptf.z();


        // 4. compute  qrcode in world pose
        slam::tf::Transform3D QRcode_in_base_pose;
        camera_in_base_pose.transformTF(QRcode_in_camera_pose_, QRcode_in_base_pose);

        LOG(INFO) << "#4-1 QRcode in base pose:  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << QRcode_in_base_pose.point().x() << ", "
                  << QRcode_in_base_pose.point().y() << ", " << QRcode_in_base_pose.point().z()<<","
                  << QRcode_in_base_pose.roll() << ", " << QRcode_in_base_pose.pitch()
                  << ", " << QRcode_in_base_pose.yaw();

        QRcode_in_base_pose.transformPoint(Eigen::Vector3d(0,0,0),ptf);

        LOG(INFO) << "@4-1 QRcode in base point:   " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << ptf.x() << ", "
                  << ptf.y() << ", " << ptf.z();

        slam::tf::Transform3D QRcode_in_world_pose;
        base_in_world_pose.transformTF(QRcode_in_base_pose, QRcode_in_world_pose);

        LOG(INFO) << "#4-2 QRcode in world pose:  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << QRcode_in_world_pose.point().x() << ", "
                  << QRcode_in_world_pose.point().y() << ", " << QRcode_in_world_pose.point().z()<<","
                  << QRcode_in_world_pose.roll() << ", " << QRcode_in_world_pose.pitch()
                  << ", " << QRcode_in_world_pose.yaw();

        QRcode_in_world_pose.transformPoint(Eigen::Vector3d(0,0,0),ptf);

        LOG(INFO) << "@4-2 QRcode in world point:   " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << ptf.x() << ", "
                  << ptf.y() << ", " << ptf.z();

        // 5. compute qrcode (0,0,0) point tranform to world coordinate;
        Eigen::Vector3d QRcode_center(0,0, 0);
        Eigen::Vector3d QRcode_center_world;
        QRcode_in_world_pose.transformPoint(QRcode_center, QRcode_center_world);


        // 5. compare pose of qrcode and theory pose of target
        float distance_code_station = distanceBothPoint2d(QRcode_center_world.x(), QRcode_center_world.y(),
                                                          this->theory_pose_.x(), this->theory_pose_.y());

        LOG(INFO) << "Target theory locations (x ,y , yaw):  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << this->theory_pose_[0] << ", "
                  << this->theory_pose_[1] << ", " << this->theory_pose_[2];

        LOG(INFO) << "QRcode locations (x ,y , z):  " << std::setw(8) << std::setfill(' ')
                  << std::setiosflags(std::ios::left) << QRcode_center_world.x() << ", "
                  << QRcode_center_world.y() << ", " << QRcode_center_world.z();

        LOG(INFO) << "Distance qrcode to target(m):  "<< distance_code_station;



        if ( distance_code_station < dist_thresh &&
             fabs(this->theory_pose_.x() - QRcode_center_world.x()) < xdist_thresh &&
             fabs(this->theory_pose_.y() - QRcode_center_world.y()) < ydist_thresh) {

            // when detected first code send message that qrcode cooridate of world.
            if(first_code){
                auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
                result_msg->command = this->command_;
                result_msg->goal_in_global_pose.x() =  QRcode_center_world.x();
                result_msg->goal_in_global_pose.y() = QRcode_center_world.y();
                result_msg->goal_in_global_pose.yaw() = QRcode_in_world_pose.yaw();
                result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_SUCCESS;
                result_msg->error_code = PerceptionStateMsg::ERROR_CODE_COORDINATE_OUT_RANGE;
                result_msg->goal_id = QRcode_info->code_int_;
                result_msg->code_str_ = QRcode_info->code_str_;
                sendMsg_(result_msg);
                stopDetector();
                LOGGER(INFO, ACTION_TASK)  << "#Send First QRcode world location (x ,y , yaw):  " << std::setw(8) << std::setfill(' ')
                          << std::setiosflags(std::ios::left) << QRcode_center_world.x() << ", "
                          << QRcode_center_world.y() << ", " << QRcode_in_world_pose.yaw();
            }
            return true;
        }else{
            if(first_code){
                auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
                result_msg->command = this->command_;
                result_msg->goal_in_global_pose.x() = .0f;
                result_msg->goal_in_global_pose.y() = .0f;
                result_msg->goal_in_global_pose.yaw() = .0f;
                result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
                result_msg->error_code = PerceptionStateMsg::ERROR_CODE_COORDINATE_OUT_RANGE;
                LOGGER(INFO, SROS) << "Fisrt:Judget QRcode is not Target(more than threshold).";
                is_receive_cmd_ = false;
                stopDetector();
                sendMsg_(result_msg);
            }
        }

        return false;
    }

}