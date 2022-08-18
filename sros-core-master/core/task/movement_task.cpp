//
// Created by lhx on 17-1-12.
//

#include "movement_task.h"

namespace sros {
namespace core {

using namespace std;

MovementTask::MovementTask(TaskNo_t no, const std::string &source_module, const deque<StationNo_t> &stations,
                           ObstacleAvoidPolicy policy)
    : Task(no, source_module),
      type_(MOVE_TO_STATION),
      stations_(stations),
      remain_stations_(stations),
      avoid_policy_(policy) {}

MovementTask::MovementTask(TaskNo_t no, const std::string &source_module, const deque<Pose> &poses,
                           ObstacleAvoidPolicy policy)
    : Task(no, source_module), type_(MOVE_TO_POSE), poses_(poses), remain_poses_(poses), avoid_policy_(policy) {}

MovementTask::MovementTask(TaskNo_t no, const std::string &source_module, NavigationPathi_vector paths)
    : Task(no, source_module), type_(MOVE_FOLLOW_PATH), paths_(paths) {}

MovementTask::MovementTask(TaskNo_t no, const std::string &source_module, NavigationPathi_vector paths,
                           const deque<StationNo_t> &stations, ObstacleAvoidPolicy policy)
    : Task(no, source_module),
      type_(MOVE_MIX2),
      stations_(stations),
      remain_stations_(stations),
      paths_(paths),
      avoid_policy_(policy) {}

int MovementTask::getProcessPercentage() const {
    if (total_distance_ <= 0) {
        return -1;
    } else {
        return (int)(remain_distance_ * 100.0 / total_distance_);
    }
}

int MovementTask::getRemainTime() const { return remain_time_; }

StationNo_t MovementTask::getDstStationNo() {
    StationNo_t station_no = 0;
    if (!remain_stations_.empty()) {
        station_no = remain_stations_.front();
        remain_stations_.pop_front();
    }
    return station_no;
}

bool MovementTask::remainStationEmpty() const { return remain_stations_.empty(); }

bool MovementTask::remainPoseEmpty() const { return remain_poses_.empty(); }

Pose MovementTask::getDstPose() {
    if (!remain_poses_.empty()) {
        Pose pose;
        pose = remain_poses_.front();
        remain_poses_.pop_front();

        return pose;
    } else {
        return Pose();
    }
}

NavigationPathi_vector MovementTask::getPaths() const {
    boost::shared_lock<boost::shared_mutex> lock(paths_mutex_);
    return paths_;
}

DstStationType MovementTask::getDstStationType() const { return dst_station_type_; }

ObstacleAvoidPolicy MovementTask::getAvoidPolicy() const { return avoid_policy_; }

int MovementTask::getRemainDistance() const { return remain_distance_; }

int MovementTask::getTotalDistance() const { return total_distance_; }

void MovementTask::setCurDstStation(StationNo_t stations_no) { cur_dst_station_ = stations_no; }

void MovementTask::setCurStartStation(StationNo_t stations_no) { cur_start_station_ = stations_no; }

StationNo_t MovementTask::getCurDstStation() { return cur_dst_station_; }

StationNo_t MovementTask::getCurStartStation() { return cur_start_station_; }

void MovementTask::setCurDstPose(sros::core::Pose pos) { cur_dst_pose_ = pos; }

void MovementTask::setCurStartPose(sros::core::Pose pos) { cur_start_pose_ = pos; }

sros::core::Pose MovementTask::getCurDstPose() { return cur_dst_pose_; }

sros::core::Pose MovementTask::getCurStartPose() { return cur_start_pose_; }

void MovementTask::setPaths(const NavigationPathi_vector &paths) {
    boost::lock_guard<boost::shared_mutex> look(paths_mutex_);
    paths_replacement_times_ = 0;
    paths_ = paths;
}

void MovementTask::pathsReplace(const NavigationPathi_vector &paths) {
    boost::lock_guard<boost::shared_mutex> look(paths_mutex_);
    ++paths_replacement_times_;
    paths_ = paths;
}

void MovementTask::setDstStationType(sros::core::DstStationType dst_station_type) {
    dst_station_type_ = dst_station_type;
}

void MovementTask::updateProcessInfo(int remain_time, int remain_distance, int total_distance) {
    remain_time_ = remain_time;
    remain_distance_ = remain_distance;
    total_distance_ = total_distance;
}

void MovementTask::setDstStationNo(StationNo_t stations_no) {
    stations_.clear();

    stations_.push_back(stations_no);
}

void MovementTask::setDstCheckResult(DstCheckResult result) { dst_check_result_ = result; }

DstCheckResult MovementTask::getDstCheckResult() const { return dst_check_result_; }

}  // namespace core
}  // namespace sros
