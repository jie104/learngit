//
// Created by getup on 18-12-13.
//

#ifndef SROS_RACK_QUERY_MODULE_H
#define SROS_RACK_QUERY_MODULE_H

#include "core/core.h"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/laser_scan_msg.hpp"
#include "rack_detection/rack_detecter.hpp"
#include "core/rack_para.h"
#include "core/tf/fix_array.hpp"

namespace slam{
namespace tf{
    class TFOperator;
}
}

namespace rack_query {
class RackQueryModule : public sros::core::Module {
public:
    struct RackInfoWithPose {
        RackInfo rack_info;
        Eigen::Vector3d pose;
    };

    enum ProcessScanState {
        STATE_IDLE_PROCESS = 0,
        STATE_DETECT_PROCESS = 1, //判断货架尺寸
        STATE_QUERY_PROCESS = 2,//判别是否含有货架
    };

    RackQueryModule();

    virtual ~RackQueryModule() = default;

    virtual void run() override;

private:
    void onNavigationCommandMsg(sros::core::base_msg_ptr msg);

    void onLaserScanMsg(sros::core::base_msg_ptr msg);

    void onPostureCorrectCmd(sros::core::base_msg_ptr msg);

    bool updateDownCameraOffsetInfo(Eigen::Vector3d &center_offset);

    void onPostureReceive(sros::core::base_msg_ptr msg);

    void onDmCodeMsg(sros::core::base_msg_ptr msg);

    void onTimer_100ms(sros::core::base_msg_ptr msg);

    void processDetect(sros::core::base_msg_ptr msg);

    void processQuery(sros::core::base_msg_ptr msg);

    std::vector<double> splitStrsToDoubles(const std::string &s, const char seperator);

    std::vector<std::string> splitStrsToStrs(const std::string &s, const char seperator);

    RackInfoWithPose computeMeanRack(std::vector<RackInfoWithPose> infos);

    bool findRightRackInfo(const Eigen::Vector3d &world_pose, const std::vector<RackInfo> &rack,
                           const std::vector<Eigen::Vector3d> &rack_poses, Eigen::Vector3d &right_pose,
                           RackInfoWithPose &rack_with_pose);

    Eigen::Vector3d cvtToWorldPose(const Eigen::Vector3d &world_pose, const Eigen::Vector3d &delta_pose) {
        Eigen::Vector3d curr_pose;
        Eigen::Affine2d world_tf(Eigen::Translation2d(world_pose.head<2>()) * Eigen::Rotation2Dd(world_pose[2]));
        curr_pose.head<2>() = world_tf * delta_pose.head<2>();
        curr_pose[2] = world_pose[2] + delta_pose[2];
        normalizeAngle(curr_pose[2]);
        return curr_pose;
    }

    bool near(const Eigen::Vector3d &curr_pose, const Eigen::Vector3d &cand_pose, double dist_thresh) {
        auto delta_dist = (curr_pose - cand_pose).head<2>().norm();
        LOG(INFO) << "deta_dist:" << delta_dist << "," << dist_thresh;
        return delta_dist < dist_thresh;
    }

    double distance(const Eigen::Vector3d &curr_pose, const Eigen::Vector3d &cand_pose) {
        return (curr_pose - cand_pose).head<2>().norm();
    }

    template<class T>
    inline T normalizeAngle(T &angle) {
        angle = fmod(angle, 2.0 * M_PI);
        if (angle >= M_PI) {
            angle -= 2.0f * M_PI;
        } else if (angle < -M_PI) {
            angle += 2.0f * M_PI;
        }
        return angle;
    }

    void resetDetectPara(std::shared_ptr<sros::core::CommonCommandMsg<std::string>> cmd);

    void resetQueryPara(std::shared_ptr<sros::core::CommonCommandMsg<std::string>> cmd);

    void updatePara();

private:
    std::vector<sros::core::base_msg_ptr> dm_code_msgs_;
    std::vector<sros::core::base_msg_ptr> all_dm_code_msgs_;
    sros::core::base_msg_ptr last_posture_corr_cmd_;
    std::string sensor_name_;
    Eigen::Vector3d curr_zero_center_pose_;
    float max_height_err = 0.008f;
    bool in_posture_correct_state = false;
    bool have_small_correct = false;
    int watch_dog_time = 0;
    const int watch_dog_timeout_1s = 10;
    bool enable_rack_query_;
    rack_detection::RackDetecter<sros::core::LaserScanMsg> detector_;
    ProcessScanState process_state_;
    Eigen::Vector3d true_rack_pose_;
    std::vector<RackInfoWithPose> world_rack_infos_;
    std::vector<RackInfo> rack_infos_;
    Eigen::Vector3d scan_pose_;
    std::shared_ptr<slam::tf::TFOperator> tf_base_to_world_;

    int detect_count_ = 0;
    int un_detect_id_ = 0;
    const int detect_count_thresh_ = 10;
    const double rack_dist_thresh_ = 0.3;
    const double rack_angle_thresh = 0.34;//20°左右
    uint16_t cur_station_no;  // 当前所在站点
};
}

#endif //SROS_RACK_QUERY_MODULE_H
