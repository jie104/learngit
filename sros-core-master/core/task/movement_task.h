//
// Created by lhx on 17-1-12.
//

#ifndef SROS_MOVEMENT_TASK_H
#define SROS_MOVEMENT_TASK_H

#include <deque>
#include "task.h"

#include "../pose.h"
#include "../state.h"
#include "core/navigation_path.h"

namespace sros {
namespace core {

enum MovementType {
    MOVE_TYPE_NONE = 0,
    MOVE_FOLLOW_PATH = 1,
    MOVE_TO_STATION = 2,
    MOVE_TO_POSE = 3,
    MOVE_MIX2 = 4,
};

enum DstCheckResult {
    DC_NONE = 0,    //未扫到二维码
    DC_RIGHT = 1,   //位置精度符合要求
    DC_WRONG = 2,   //位置精度不符合要求
};

class MovementTask : public Task {
public:
    MovementTask(TaskNo_t no, const std::string &source_module, const std::deque<StationNo_t> &stations, ObstacleAvoidPolicy policy);

    MovementTask(TaskNo_t no, const std::string &source_module, const std::deque<sros::core::Pose> &poses, ObstacleAvoidPolicy policy);

    MovementTask(TaskNo_t no, const std::string &source_module, NavigationPathi_vector paths);

    MovementTask(TaskNo_t no, const std::string &source_module, NavigationPathi_vector paths, const std::deque<StationNo_t> &stations,
                 ObstacleAvoidPolicy policy);

    MovementType getMovementType() const { return type_; }
    bool movementTypeIsFollowPath() const { return type_ == MOVE_FOLLOW_PATH; }

    StationNo_t getDstStationNo();

    bool remainStationEmpty() const;

    Pose getDstPose();

    bool remainPoseEmpty() const;

    NavigationPathi_vector getPaths() const;

    void setDstStationType(DstStationType dst_station_type);

    DstStationType getDstStationType() const;

    void setPaths(const NavigationPathi_vector &paths);

    void pathsReplace(const NavigationPathi_vector &paths);

    uint32_t getPathsReplacementTimes() const { return paths_replacement_times_; }

    void setDstStationNo(StationNo_t stations_no);

    ObstacleAvoidPolicy getAvoidPolicy() const;

    virtual int getProcessPercentage() const;

    virtual int getRemainTime() const;

    int getRemainDistance() const;

    int getTotalDistance() const;

    void setCurDstStation(StationNo_t stations_no);

    void setCurStartStation(StationNo_t stations_no);

    StationNo_t getCurDstStation();

    StationNo_t getCurStartStation();

    void setCurDstPose(sros::core::Pose pos);

    void setCurStartPose(sros::core::Pose pos);

    sros::core::Pose getCurDstPose();

    sros::core::Pose getCurStartPose();

    void updateProcessInfo(int remain_time, int remain_distance, int total_distance);

    void setDstCheckResult(DstCheckResult result);

    DstCheckResult getDstCheckResult() const;

    void setDownCameraOffset(const DMCodeOffset &info) { cur_down_camera_offset_.set(info); }
    DMCodeOffset getDownCameraOffset() const { return cur_down_camera_offset_.get(); }
    bool isSetDownCameraOffset() const { return !cur_down_camera_offset_.get().no.empty(); } // 是否设置了下视摄像头偏差

    void setFailedCode(int failed_code) { failed_code_  = failed_code; }
    int getFailedCode() const { return failed_code_; }

    void setForceNavOnMap(bool force_nav_on_map) { force_nav_on_map_ = force_nav_on_map; }
    bool getForceNavOnMap() { return force_nav_on_map_; }

    void finishMoveTask(TaskResult result, int result_value) { failed_code_ = result_value; Task::finishTask(result);}

private:

    MovementType type_ = MOVE_TYPE_NONE;

    ObstacleAvoidPolicy avoid_policy_;

    ///< 目标站点序列
    //std::vector<StationNo_t> stations_;
    std::deque<StationNo_t> stations_;
    std::deque<StationNo_t> remain_stations_;

    ReadWriteLockContainer<DMCodeOffset> cur_down_camera_offset_;  // 当前下视摄像头偏差

    ///< 目标位置序列
    //Pose_Vector poses_;
    std::deque<sros::core::Pose> poses_;
    std::deque<sros::core::Pose> remain_poses_;

    ///< 路径序列
    NavigationPathi_vector paths_;
    mutable boost::shared_mutex paths_mutex_;

    int remain_time_ = 0;
    int remain_distance_ = 0;
    int total_distance_ = 0;

    uint32_t paths_replacement_times_ = 0; // 路径被替换过了多少次，替换一次要加1，通过这种方式来通知前端更新路径，前端检测路径的变化比较困难

    StationNo_t cur_dst_station_ = 0;
    StationNo_t cur_start_station_ = 0;
    sros::core::Pose cur_dst_pose_;
    sros::core::Pose cur_start_pose_;

    sros::core::DstStationType dst_station_type_ = DS_DEFAULT;

    DstCheckResult dst_check_result_;

    int failed_code_ = 0; // 失败代码

    bool force_nav_on_map_ = false;
};

typedef std::shared_ptr<MovementTask> MovementTask_ptr;

}
}

#endif //SROS_MOVEMENT_TASK_H
