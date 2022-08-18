//
// Created by lfc on 16-1-29.
//

#include "PoseManagerModule.h"
#include "core/src.h"

#include "modules/imu/imu_module.h"
#include "transform.hpp"
#include <core/msg/parameter_msg.hpp>
#include <memory>
//#include "core/temp/global_bag_config.h"
#include "core/settings.h"
#include "core/util/record_file_manager.hpp"
#include "fusion_manager.hpp"

using namespace slam::tf;
using namespace sros;

namespace slam
{
    namespace tf
    {
        template <class T>
        static void normalizeAngle(T &angle)
        {
            angle = fmod(angle, 2.0 * M_PI);
            if (angle >= M_PI)
            {
                angle -= 2.0f * M_PI;
            }
            else if (angle < -M_PI)
            {
                angle += 2.0f * M_PI;
            }
        }
    }
}

namespace sros
{
    namespace pose_filter
    {

        std::unique_ptr<ConciseOdometry> PoseManagerModule::pose_fusion_(nullptr);

        PoseManagerModule::PoseManagerModule()
            : Module("PoseManager"),
              tf_base_to_odo(NULL),
              tf_base_to_world(NULL),
              tf_scan_to_base(NULL),
              get_match_pose(false),
              laser_coordyaw(0.0),
              rotate_offset(0),
              laser_coordx(0.0),
              laser_coordy(0),
              stamp_match_thresh(20000),
              is_first_bagpose(true),
              match_pose("TOPIC_MATCHPOSE")
        {

            FrameToFrame base_to_odo_frame;
            base_to_odo_frame.parent_frame = "odom";
            base_to_odo_frame.child_frame = "base_link";
            tf_base_to_odo = new TFOperator(base_to_odo_frame);

            FrameToFrame base_to_world_frame;
            base_to_world_frame.parent_frame = "world";
            base_to_world_frame.child_frame = "base_link";
            tf_base_to_world = new TFOperator(base_to_world_frame);

            FrameToFrame scan_to_base_frame;
            scan_to_base_frame.parent_frame = "base_link";
            scan_to_base_frame.child_frame = "scan";
            tf_scan_to_base = new TFOperator(scan_to_base_frame);

            slam::tf::FrameToFrame fusion_to_world_frame;
            fusion_to_world_frame.parent_frame = "world";
            fusion_to_world_frame.child_frame = "fusion";
            tf_fusion_to_world = new slam::tf::TFOperator(fusion_to_world_frame);

            laser_coordx = sros::core::Settings::getInstance().getValue<float>( //getValue = mainSetting.ini中参数
                "posefilter.laser_coordx", 0.29);
            laser_coordy = sros::core::Settings::getInstance().getValue<float>( //getValue = mainSetting.ini中参数
                "posefilter.laser_coordy", 0.0);
            laser_coordyaw = sros::core::Settings::getInstance().getValue<float>( //getValue = mainSetting.ini中参数
                "posefilter.laser_coordyaw", 0.0);

            pose_fusion_.reset(new ConciseOdometry());
            pose_fusion_->setAdvertiseStandstillFunc([this](const bool standstill)
                                                     { imu::ImuModule::setStandstill(standstill); });
            state_detector_ = sros::MotionStateDetector::getInstance();

            deal_pose_flag_ = false;
        }

        PoseManagerModule::~PoseManagerModule()
        {
            if (tf_base_to_world)
            {
                delete tf_base_to_world;
                tf_base_to_world = nullptr;
            }
            if (tf_base_to_odo)
            {
                delete tf_base_to_odo;
                tf_base_to_odo = nullptr;
            }

            if (tf_scan_to_base)
            {
                delete tf_scan_to_base;
                tf_scan_to_base = nullptr;
            }
            if (tf_fusion_to_world)
            {
                delete tf_fusion_to_world;
                tf_fusion_to_world = nullptr;
            }
        }

        void PoseManagerModule::run()
        {
            DLOG(INFO) << "begin to run the posemanager";
            subscribeTopic("TOPIC_MATCHPOSE", CALLBACK(&PoseManagerModule::onMatchPoseMsg));
            subscribeTopic("POSEFILTER_PARAMETER", CALLBACK(&PoseManagerModule::onParameterMsg));
             src_sdk->setPoseCallback([this](core::Pose p)
                                      { onOdoPoseCallback(p); });
//            src_sdk->setPoseCallback([this](core::Pose p)
//                                     { handleOdom(p); });

            dispatch();
        }

        void PoseManagerModule::onMatchPoseMsg(sros::core::base_msg_ptr msg_ptr)
        {
            core::PoseStamped_ptr matchpose_ptr = std::dynamic_pointer_cast<core::PoseStampedMsg>(msg_ptr);
            if (matchpose_ptr->session_id == 1)
            {
                static std::fstream record_loc_stream;
                if (!record_loc_stream.is_open())
                {
                    auto file_name = record::RecordFileManager::creatImgName(record::RecordFileManager::getCurrSaveTime());
                    std::string map_path = "/sros/record/";
                    std::string suffix = ".txt";
                    record::RecordFileManager::manageRecordFile(map_path, suffix, 5);
                    LOG(INFO) << "will reopen record file:" << map_path << "," << file_name;
                    record_loc_stream.open(map_path + file_name + "_odo" + suffix, std::ios_base::out);
                    record_loc_stream << "time,pose_x,pose_y,pose_yaw,score,delta_delay_time" << std::endl;
                }
                else
                {
                    auto curr_sstamp_time = sros::core::util::get_time_in_us();
                    auto curr_time = record::RecordFileManager::getCurrSaveTime();
                    record_loc_stream << curr_time->tm_mday << "-" << curr_time->tm_hour << ":" << curr_time->tm_min << ":" << curr_time->tm_sec << "." << record::RecordFileManager::timeinus() << ",";
                    record_loc_stream << last_base_pose.x() << "," << last_base_pose.y() << ","
                                      << atan2(sin(last_base_pose.yaw()), cos(last_base_pose.yaw())) << ","
                                      << matchpose_ptr->pose.confidence() << ","
                                      << (curr_sstamp_time - last_base_pose.synctimestamp()) / 1.0e6 << std::endl;
                }
            }
//            if(matchpose_ptr->session_id == 1){
//                static std::fstream record_loc_stream;
//                if (!record_loc_stream.is_open()) {
//                    auto file_name = record::RecordFileManager::creatImgName(record::RecordFileManager::getCurrSaveTime());
//                    std::string map_path = "/sros/record/";
//                    std::string suffix = ".txt";
//                    record::RecordFileManager::manageRecordFile(map_path, suffix, 5);
//                    LOG(INFO) << "will reopen record file:" << map_path << "," << file_name;
//                    record_loc_stream.open(map_path + file_name + "_odo" + suffix, std::ios_base::out);
//                    record_loc_stream << "time,pose_x,pose_y,pose_yaw,score,delta_delay_time" << std::endl;
//                }else{
//                    auto curr_sstamp_time = sros::core::util::get_time_in_us();
//                    auto curr_time = record::RecordFileManager::getCurrSaveTime();
//                    record_loc_stream << curr_time->tm_mday<<"-"<<curr_time->tm_hour << ":" << curr_time->tm_min << ":" << curr_time->tm_sec<<"."<<record::RecordFileManager::timeinus()<<",";
//                    record_loc_stream << last_base_pose.x() << "," << last_base_pose.y() << ","
//                                      << atan2(sin(last_base_pose.yaw()), cos(last_base_pose.yaw())) << ","
//                                      << matchpose_ptr->pose.confidence() << ","
//                                      << (curr_sstamp_time - last_base_pose.synctimestamp())/1.0e6 << std::endl;
//                }
//            }
            if (!get_match_pose)
            {
                match_pose = *matchpose_ptr;
                //LOG(INFO) << "match pose: " << match_pose.pose.x() << ", " << match_pose.pose.y() << ", " << match_pose.pose.yaw();
                if (match_pose.time_ == 0)
                {
                    match_pose.time_ = sros::core::util::get_time_in_us();
                }
                TransForm odo_tf;
                if (!tf_base_to_odo->lookUpTransForm(match_pose.time_, odo_tf, stamp_match_thresh))
                {
                    LOG(INFO) << "error to get the time!\n"
                              << match_pose.time_;
                }
                else
                {
                    core::Pose tmppose(odo_tf.position, odo_tf.rotation);
                    match_odo_pose = tmppose;
                    get_match_pose = true;
                }
            }
        }

        /**
 * Pose回调，注意调用此函数的线程是src收数据线程
 * @param p
 */
        void PoseManagerModule::onOdoPoseCallback(core::Pose &p)
        {
            if (std::isnan(p.x()) || !finite(p.x()))
            {
                LOG(WARNING) << "p.x is nan!" << p.x();
                return;
            }
            if (std::isnan(p.y()) || !finite(p.y()))
            {
                LOG(WARNING) << "p.x is nan!" << p.y();
                return;
            }
            if (std::isnan(p.yaw()) || !finite(p.yaw()))
            {
                LOG(WARNING) << "p.yaw is nan!" << p.yaw();
                return;
            }
            if (std::isnan(p.roll()) || !finite(p.roll()))
            {
                LOG(WARNING) << "p.roll is nan!" << p.roll();
                p.roll() = 0;
            }
            if (std::isnan(p.pitch()) || !finite(p.pitch()))
            {
                LOG(WARNING) << "p.pich is nan!" << p.pitch();
                p.pitch() = 0;
            }
            //TODO:当前pitch和roll被其他功能复用，暂时置位0,以后一定要记得修改！
            p.roll() = 0;
            p.pitch() = 0;

            int64_t odo_timestamp = p.timestamp() * 1e4;
            //    static int64_t last_timestamp = 0;//防止出现雷达掉电,导致时间戳不连续的情况.
            if (last_odo_stamp_ > odo_timestamp)
            {
                LOG(INFO) << "odo stamp is wrong!will clear stamp array!";
                delta_time_stamp_.resize(max_stamp_cache_size_);
            }
            last_odo_stamp_ = odo_timestamp;

            if (delta_time_stamp_.empty())
            {
                delta_time_stamp_.resize(max_stamp_cache_size_);
            }
            int64_t delta_time_stamp = p.synctimestamp() - odo_timestamp;
            delta_time_stamp_.push_back(delta_time_stamp);
            auto min_delta_stamp = delta_time_stamp_.getMinValue();

            if (min_delta_stamp != 0)
            {
                p.synctimestamp() = min_delta_stamp + odo_timestamp;
            }
            int64_t curr_real_time = sros::core::util::get_time_in_us();

            TransForm origin_odo_tf(p.synctimestamp(), p);
            state_detector_->updateTF(origin_odo_tf);
            //    if((curr_real_time-last_real_time)>2e4){
            //        LOG(INFO) << "odo delta time is large!" << (curr_real_time - last_real_time) / 1.e6;
            //    }
            if ((curr_real_time - p.synctimestamp()) > 5e4)
            {
                // 开启了本地调试，里程计数据不对，但是不让其刷屏
                auto &s = sros::core::Settings::getInstance();
                auto enable_sros_native_debug = (s.getValue<std::string>("debug.enable_sros_native_debug", "False") == "True");
                if (!enable_sros_native_debug)
                {
                    LOG(INFO) << "curr delta time is large!" << ((curr_real_time - p.synctimestamp())) / 1.e6;
                }

                delta_time_stamp_.resize(max_stamp_cache_size_);
            }

            is_first_bagpose = true;
            if (first_odo_pose)
            {
                first_odo_pose = false;
                last_base_pose = p;
                last_odo_pose = p;
            }
            else
            {
                if (std::hypot(p.x() - last_odo_pose.x(), p.y() - last_odo_pose.y()) > 0.8)
                {
                    LOG(INFO) << "delta pose is wrong:" << p.x() << "," << p.y() << "," << p.yaw();
                    LOG(INFO) << "last pose is :" << last_odo_pose.x() << "," << last_odo_pose.y() << "," << last_odo_pose.yaw();
                    last_odo_pose = p;
                    //            LOG(INFO) << "delta pose is wrong:" << p.x() << "," << p.y() << "," << p.yaw();
                    //            LOG(INFO) << "last pose is :" << last_odo_pose.x() << "," << last_odo_pose.y() << "," <<
                    //                      last_odo_pose.yaw();
                    return;
                }
                double delta_yaw = p.yaw() - last_odo_pose.yaw();
                normalizeAngle(delta_yaw);
                if (fabs(delta_yaw) > 0.104)
                {
                    LOG(ERROR) << "delta odo is wrong!" << delta_yaw;
                    last_odo_pose = p;
                    LOG(INFO) << "delta pose is wrong:" << p.x() << "," << p.y() << "," << p.yaw();
                    LOG(INFO) << "last pose is :" << last_odo_pose.x() << "," << last_odo_pose.y() << "," << last_odo_pose.yaw();
                    return;
                }
            }
            core::Pose incre_p = addPose(last_base_pose, last_odo_pose, p);
            last_odo_pose = p;
            last_base_pose = incre_p;

            int64_t odo_time = p.synctimestamp();
            TransForm incre_origin_odo(odo_time, incre_p);
            if (!fusion_manager_) {
                fusion_manager_.reset(new fusion::FusionManager);
            }
            auto imu_data = imu::ImuModule::getImuWithStamp(odo_time);
            auto odo_tf = fusion_manager_->fusionOdoAndImu(incre_origin_odo, imu_data);
            core::Pose correct_p = incre_p;
            correct_p.rotation() = odo_tf.rotation;
            correct_p.location() = odo_tf.position;
            tf_base_to_odo->pushbackTransForm(odo_tf);

            //LOG(WARNING) << "onOdoPoseMsg " << " odo_tf.position.x() " << odo_tf.position.x()
            //             <<"odo_tf.position.y() " << odo_tf.position.y() << " odo_tf.rotation.yaw() " <<odo_tf.rotation.yaw();

            core::PoseStamped_ptr odo_posestamped = std::make_shared<sros::core::PoseStampedMsg>("OdoPoseStamped");
            odo_posestamped->time_ = odo_time;
            odo_posestamped->pose = correct_p;
            sendMsg(odo_posestamped);

            if (get_match_pose)
            {
                odo_syn_pose = match_odo_pose;
                match_syn_pose = match_pose.pose;
                get_match_pose = false;
            }
            core::Pose fusion_pose = addPose(match_syn_pose, odo_syn_pose, correct_p);

            //LOG(INFO) << "fusion pose: " << fusion_pose.x() << ", " << fusion_pose.y() << ", " << fusion_pose.yaw();
            TransForm world_tf(odo_time, fusion_pose);

            tf_base_to_world->pushbackTransForm(world_tf);

            tf_fusion_to_world->pushbackTransForm(world_tf);
            core::Pose loc_p;

            if (rotate_offset == 90)
            {
                loc_p.roll() = -p.pitch();
                loc_p.pitch() = p.roll();
            }
            else if (rotate_offset == -90)
            {
                loc_p.roll() = p.pitch();
                loc_p.pitch() = -p.roll();
            }
            else if (rotate_offset == 180)
            {
                loc_p.roll() = -p.roll();
                loc_p.pitch() = -p.pitch();
            }
            else if (rotate_offset == 0)
            {
                loc_p.roll() = p.roll();
                loc_p.pitch() = p.pitch();
            }
            else
            {
                loc_p.roll() = 0.0;
                loc_p.pitch() = 0.0;
            }

            loc_p.x() = laser_coordx; // A2 �激光雷达位置
            loc_p.yaw() = laser_coordyaw;
            loc_p.y() = laser_coordy;
            TransForm scan_tf(odo_time, loc_p);
            tf_scan_to_base->pushbackTransForm(scan_tf);

            src_sdk->sendPoseBack(fusion_pose); // 向src返回融合后位姿
        }

        core::Pose PoseManagerModule::addPose(sros::core::Pose &matchpose, sros::core::Pose &odosynpose,
                                              sros::core::Pose &newpose)
        {
            core::Pose delta_pose1, delta_pose2;

            delta_pose1.x() = newpose.x() - odosynpose.x();
            delta_pose1.y() = newpose.y() - odosynpose.y();
            delta_pose2.x() = delta_pose1.x() * cos(odosynpose.yaw()) + delta_pose1.y() * sin(odosynpose.yaw());
            delta_pose2.y() = -delta_pose1.x() * sin(odosynpose.yaw()) + delta_pose1.y() * cos(odosynpose.yaw());
            delta_pose1.x() = delta_pose2.x() * cos(matchpose.yaw()) - delta_pose2.y() * sin(matchpose.yaw());
            delta_pose1.y() = delta_pose2.x() * sin(matchpose.yaw()) + delta_pose2.y() * cos(matchpose.yaw());

            core::Location return_loc(matchpose.x() + delta_pose1.x(), matchpose.y() + delta_pose1.y());
            core::Rotation return_rot(matchpose.yaw() + newpose.yaw() - odosynpose.yaw());
            slam::tf::normalizeAngle(return_rot.yaw());
            return core::Pose(return_loc, return_rot, matchpose.confidence());
        }

        void PoseManagerModule::onParameterMsg(sros::core::base_msg_ptr m)
        {
            auto msg = std::dynamic_pointer_cast<sros::core::ParameterMsg>(m);
            std::string name = msg->name;
            std::string value = msg->value;

            //    LOG(INFO) << "PoseManagerModule::onParameterMsg -> " << name << ": " << value;

            //    // TODO 修改参数
            if (name == "posefilter.laser_coordx")
            {
                //         laser_coordx = atof(value.c_str());
            }
            else if (name == "posefilter.laser_coordy")
            {
                //        laser_coordy = atof(value.c_str());
            }
            else if (name == "posefilter.laser_coordyaw")
            {
                //        laser_coordyaw = atof(value.c_str());
            }
            else if (name == "posefilter.stamp_match_thresh")
            {
                stamp_match_thresh = atoi(value.c_str());
            }
        }

        void PoseManagerModule::handleOdom(core::Pose p)
        {
            // LOG(INFO) << "recive src pose:" << p.x() << "," << p.y() << "," << p.yaw();
            if (dealWithOdomMsg(p))
            {
                auto imu_data = imu::ImuModule::getImuWithStamp(p.synctimestamp());

                auto odom = corePoseToOdometry(p);
                auto last_odom = pose_fusion_->getOptionPose();
                auto fused_pose = pose_fusion_->fuseData(std::move(odom), std::move(imu_data));

                auto pose = odometryToCorePose(fused_pose);

                auto last_fusion_odom = last_odom.second ? last_odom.first : pose;
                auto matched_pose = updateMatchingPose(p.synctimestamp(), last_fusion_odom, pose);

                advertiseScanTF(p);

                src_sdk->sendPoseBack(matched_pose); // 向src返回融合后位姿
            }
            else
            {
                auto odom = corePoseToOdometry(p);
                pose_fusion_->duplicateAndUpdateTime(std::move(odom));
                LOG(WARNING) << "src odom has problem!!!";
            }
        }

        bool PoseManagerModule::dealWithOdomMsg(core::Pose &p)
        {
            if (std::isnan(p.x()) || !finite(p.x()))
            {
                LOG(WARNING) << "p.x is nan!" << p.x();
                return false;
            }
            if (std::isnan(p.y()) || !finite(p.y()))
            {
                LOG(WARNING) << "p.x is nan!" << p.y();
                return false;
            }
            if (std::isnan(p.yaw()) || !finite(p.yaw()))
            {
                LOG(WARNING) << "p.yaw is nan!" << p.yaw();
                return false;
            }
            if (std::isnan(p.roll()) || !finite(p.roll()))
            {
                LOG(WARNING) << "p.roll is nan!" << p.roll();
                p.roll() = 0;
            }
            if (std::isnan(p.pitch()) || !finite(p.pitch()))
            {
                LOG(WARNING) << "p.pich is nan!" << p.pitch();
                p.pitch() = 0;
            }
            // zmy ： 轮速计是以脉冲形式计时 脉冲间隔100us
            int64_t odo_timestamp = p.timestamp() * 1e4;
            static int64_t last_timestamp = 0; //防止出现雷达掉电,导致时间戳不连续的情况.
            if (last_timestamp > odo_timestamp)
            {
                LOG(INFO) << "odo stamp is wrong!will clear stamp array!";
                delta_time_stamp_.resize(max_stamp_cache_size_);
            }
            last_timestamp = odo_timestamp;
            if (delta_time_stamp_.empty())
            {
                delta_time_stamp_.resize(max_stamp_cache_size_);
            }

            int64_t delta_time_stamp = p.synctimestamp() - odo_timestamp;
            delta_time_stamp_.push_back(delta_time_stamp);
            auto min_delta_stamp = delta_time_stamp_.getMinValue();
            if (min_delta_stamp != 0)
            {
                p.synctimestamp() = min_delta_stamp + odo_timestamp;
            }
            static int64_t last_p_time = p.synctimestamp() - 1;
            int64_t curr_real_time = sros::core::util::get_time_in_us();
            //    if((curr_real_time-last_real_time)>2e4){
            //        LOG(INFO) << "odo delta time is large!" << (curr_real_time - last_real_time) / 1.e6;
            //    }
            if ((curr_real_time - p.synctimestamp()) > 5e4)
            {
                // 开启了本地调试，里程计数据不对，但是不让其刷屏
                auto &s = sros::core::Settings::getInstance();
                auto enable_sros_native_debug = (s.getValue<std::string>("debug.enable_sros_native_debug", "False") == "True");
                if (!enable_sros_native_debug)
                {
                    LOG(INFO) << "curr delta time is large!" << ((curr_real_time - p.synctimestamp())) / 1.e6;
                }

                delta_time_stamp_.resize(max_stamp_cache_size_);
            }

            auto delta_encoder_time = static_cast<double>(p.synctimestamp() - last_p_time) / 1e6;
            if (delta_encoder_time > 2e-2) // 小于50hz
            {
                LOG(INFO) << "The encoder frequency is too low:" << (1 / delta_encoder_time);
                last_p_time = p.synctimestamp();
                return false;
            }
            last_p_time = p.synctimestamp();

            if (first_odo_pose)
            {
                first_odo_pose = false;
                last_base_pose = p;
                last_odo_pose = p;
            }
            else
            {
                double delta_x = p.x() - last_odo_pose.x();
                double delta_y = p.y() - last_odo_pose.y();
                if (std::hypot(delta_x, delta_y) > max_linear_ * delta_encoder_time)
                {
                    LOG(INFO) << "delta pose is wrong:" << p.x() << "," << p.y() << "," << p.yaw();
                    LOG(INFO) << "last pose is :" << last_odo_pose.x() << "," << last_odo_pose.y() << "," << last_odo_pose.yaw();
                    last_odo_pose = p;
                    //            LOG(INFO) << "delta pose is wrong:" << p.x() << "," << p.y() << "," << p.yaw();
                    //            LOG(INFO) << "last pose is :" << last_odo_pose.x() << "," << last_odo_pose.y() << "," <<
                    //                      last_odo_pose.yaw();
                    return false;
                }

                double delta_yaw = p.yaw() - last_odo_pose.yaw();
                normalizeAngle(delta_yaw);
                if (fabs(delta_yaw) > max_angular_ * delta_encoder_time)
                {
                    LOG(WARNING) << "delta odo is wrong!" << delta_yaw;
                    last_odo_pose = p;
                    LOG(INFO) << "delta pose is wrong:" << p.x() << "," << p.y() << "," << p.yaw();
                    LOG(INFO) << "last pose is :" << last_odo_pose.x() << "," << last_odo_pose.y() << "," << last_odo_pose.yaw();
                    return false;
                }
                p.confidence() = delta_x == 0 && delta_y == 0 && delta_yaw == 0 ? 2 : 1; // 判断是否停止
            }
            last_odo_pose = p;
            return true;
        }

        OdomData PoseManagerModule::corePoseToOdometry(core::Pose &p)
        {
            OdomData odo;
            odo.header.stamp = p.synctimestamp();
            odo.header.sync_stamp = p.timestamp();
            odo.pose.position.x() = p.x();
            odo.pose.position.y() = p.y();
            // odo.pose.position.z() = p.z();
            // tf::EulerToEigenQuaternion<core::geometry_msgs::Quaternion>(p.yaw(), p.pitch(), p.roll(), odo.pose.orientation);

            odo.pose.position.z() = 0.f;
            tf::EulerToEigenQuaternion<core::geometry_msgs::Quaternion>(p.yaw(), 0, 0, odo.pose.orientation);

            odo.status = p.confidence();

            static core::Pose last_p = p;
            static sros::core::geometry_msgs::Quaternion last_quat = odo.pose.orientation;

            odo.twist.linear.x() = (p.x() - last_p.x()) / (p.synctimestamp() - last_p.synctimestamp()) * 1e6;
            odo.twist.linear.y() = (p.y() - last_p.y()) / (p.synctimestamp() - last_p.synctimestamp()) * 1e6;
            odo.twist.linear.z() = (p.z() - last_p.z()) / (p.synctimestamp() - last_p.synctimestamp()) * 1e6;

            odo.twist.linear = last_quat.inverse() * odo.twist.linear;

            float delta_yaw = p.yaw() - last_p.yaw();

            normalizeAngle(delta_yaw);
            odo.twist.angular = sros::core::geometry_msgs::Vector3(0, 0, delta_yaw / (p.synctimestamp() - last_p.synctimestamp()) * 1e6);

            last_p = p;
            last_quat = odo.pose.orientation;

            return odo;
        }

        core::Pose PoseManagerModule::odometryToCorePose(const OdomData &odom)
        {
            core::Pose p;
            p.synctimestamp() = odom.header.stamp;
            p.timestamp() = odom.header.sync_stamp;
            p.x() = odom.pose.position.x();
            p.y() = odom.pose.position.y();
            p.z() = odom.pose.position.z();
            tf::QuaternionToEuler<core::geometry_msgs::Quaternion>(odom.pose.orientation, p.yaw(), p.roll(), p.pitch());

            // Eigen::Vector3f vec = Eigen::Vector3f(1, 0, 0);
            // Eigen::Vector3f vec1 = odom.pose.orientation.toRotationMatrix() * vec;
            // LOG(INFO) << "vecccccccccccccccccccccccccccccccccccccccccccccccc:\n"
            //           << vec1;
            // p.roll() = 0;
            // p.pitch() = 0;
            // p.yaw() = std::atan2(vec1[1], vec1[0]);
            // LOG(INFO) << "yawwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww:" << p.yaw();

            return p;
        }

        void PoseManagerModule::advertiseScanTF(const core::Pose &p) const
        {
            core::Pose loc_p;

            if (rotate_offset == 90)
            {
                loc_p.roll() = -p.pitch();
                loc_p.pitch() = p.roll();
            }
            else if (rotate_offset == -90)
            {
                loc_p.roll() = p.pitch();
                loc_p.pitch() = -p.roll();
            }
            else if (rotate_offset == 180)
            {
                loc_p.roll() = -p.roll();
                loc_p.pitch() = -p.pitch();
            }
            else if (rotate_offset == 0)
            {
                loc_p.roll() = p.roll();
                loc_p.pitch() = p.pitch();
            }
            else
            {
                loc_p.roll() = 0.0;
                loc_p.pitch() = 0.0;
            }

            loc_p.x() = laser_coordx; // A2 �激光雷达位置
            loc_p.yaw() = laser_coordyaw;
            loc_p.y() = laser_coordy;
            TransForm scan_tf(p.synctimestamp(), loc_p);
            tf_scan_to_base->pushbackTransForm(scan_tf);
        }

        core::Pose PoseManagerModule::updateMatchingPose(const int64_t stamp, core::Pose &last_pose, core::Pose &current_pose)
        {
            core::Pose correct_p = addPose(last_base_pose, last_pose, current_pose);
            last_base_pose = correct_p;

            TransForm odo_tf(stamp, correct_p);
            tf_base_to_odo->pushbackTransForm(odo_tf);

            core::PoseStamped_ptr odo_posestamped = std::make_shared<sros::core::PoseStampedMsg>("OdoPoseStamped");
            odo_posestamped->time_ = stamp;
            odo_posestamped->pose = correct_p;
            sendMsg(odo_posestamped);

            if (get_match_pose)
            {
                odo_syn_pose = match_odo_pose;
                match_syn_pose = match_pose.pose;
                get_match_pose = false;
                if (abs(match_pose.time_ - stamp) > 3e5)
                {
                    LOG(INFO) << "match pose and syncpose time is too large!" << (match_pose.time_ - stamp);
                }
            }
            core::Pose fusion_pose = addPose(match_syn_pose, odo_syn_pose, correct_p);

            //LOG(INFO) << "fusion pose: " << fusion_pose.x() << ", " << fusion_pose.y() << ", " << fusion_pose.yaw();
            TransForm world_tf(stamp, fusion_pose);

            tf_base_to_world->pushbackTransForm(world_tf);

            tf_fusion_to_world->pushbackTransForm(world_tf);

            return fusion_pose;
        }

        OdomData PoseManagerModule::getOdomWithStamp(const uint64_t stamp)
        {
            return pose_fusion_->getOdometryWithStamp(stamp);
        }

        std::vector<OdomData> PoseManagerModule::getOdomWithStampDuration(const uint64_t stamp, const int64_t duration)
        {
            return pose_fusion_->getOdomWithStampDuration(stamp, duration);
        }

    } // namespace pose_filter
}
