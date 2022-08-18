//
// Created by lfc on 18-12-11.
//

#ifndef PROJECT_OBSTACLE_POINTS_MANAGER_HPP
#define PROJECT_OBSTACLE_POINTS_MANAGER_HPP

#include <Eigen/Dense>
#include <memory>
#include "core/util/time.h"

namespace avoidoba {
struct ObstaclePoints{
    enum ObaState{
        STATE_OBA_STOP_0 = 0,
        STATE_OBA_STOP_1,
        STATE_OBA_STOP_2,
        STATE_OBA_STOP_3,
        STATE_OBA_STOP_4,
        STATE_OBA_SLOW,
        STATE_OBA_FREE,
    };
    std::string obstacle_name;
    int64_t time;
    std::vector<Eigen::Vector2d> points;
    bool is_region_oba = false;
    ObaState oba_state;
};

typedef std::shared_ptr<ObstaclePoints> ObstaclePoints_Ptr;
class ObstaclePointsManager {//目前这部分还不成熟,后期可能需要引入时间间隔.如果障碍信息太落后,则认为不具有实时性,可以不使用.
public:
    ObstaclePointsManager(){
        delta_time_thresh_ = 5e5;
    }

    void updateObstaclePoints(ObstaclePoints_Ptr& oba_points) {
        if (oba_points->obstacle_name == "") {
            LOG(INFO) << "obstacle name is null! will return";
            return;
        }
        obstacles_[oba_points->obstacle_name] = oba_points;
    }

    void getObstaclePoints(std::vector<Eigen::Vector2d>& points,bool use_ifm_points = true) {
        std::vector<std::string> timeout_sensors;
        for (auto &obas:obstacles_) {
            auto &curr_points = obas.second;
            if (!use_ifm_points) {
                if (findD435Camera(curr_points->obstacle_name, "D435")) {
                    continue;
                }
            }

            if (curr_points->obstacle_name != "LASER_OBA") {
                auto curr_time = sros::core::util::get_time_in_us();
                if (curr_time - curr_points->time > delta_time_thresh_) {
                    timeout_sensors.push_back(curr_points->obstacle_name);
                    //                    LOG(INFO) << "delta time too large!" << double(curr_time - curr_points->time) / 1e6 <<
//                    curr_points->obstacle_name;
                    continue;
                }
            }
            points.reserve(points.size() + curr_points->points.size());
            for (auto &point:curr_points->points) {
                points.push_back(point);
            }
        }
        eraseTimeOutSensor(timeout_sensors);
    }

    void getObstacleRegions(std::vector<ObstaclePoints::ObaState>& oba_states){
        auto curr_time = sros::core::util::get_time_in_us();
        for(auto& oba:obstacles_){
            auto &curr_points = oba.second;
            if (curr_points->is_region_oba) {
                if (curr_time - curr_points->time > delta_time_thresh_) {
                    LOG(INFO) << "delta time too large!"<<double(curr_time - curr_points->time)/1e6;
                    continue;
                }
                oba_states.push_back(curr_points->oba_state);
            }
        }
    }

    bool findD435Camera(const std::string& oba_name, const std::string d435){
        int find_result = oba_name.find(d435);
        bool find_d435 = false;
        if (find_result >= 0 && find_result <= oba_name.size()) {
            find_d435 = true;
        }
        if (oba_name == "IFM_STEREO_OBA" || find_d435) {
            return true;
        }
        return false;
    }

    void getObstaclePoints(std::map<std::string,std::vector<Eigen::Vector2d>>& all_points,bool use_ifm_points = true,bool log_output = false) {
        std::vector<std::string> timeout_sensors;
        for (auto &obas:obstacles_) {
            auto &curr_points = obas.second;
            if (!use_ifm_points) {
                if (findD435Camera(curr_points->obstacle_name, "D435")) {
                    continue;
                }
                if (findD435Camera(curr_points->obstacle_name, "DEPTH")) {
                    continue;
                }
            }

            if (curr_points->obstacle_name != "LASER_OBA") {
                auto curr_time = sros::core::util::get_time_in_us();
                if (curr_time - curr_points->time > delta_time_thresh_) {
                    if (log_output) {
                        LOG(INFO) << "delta time too large!" << double(curr_time - curr_points->time) / 1e6 <<
                                  curr_points->obstacle_name;
                    }
                    timeout_sensors.push_back(curr_points->obstacle_name);
                    continue;
                }
            }
            std::vector<Eigen::Vector2d> points;
            points.reserve(curr_points->points.size());
            for (auto &point:curr_points->points) {
                points.push_back(point);
            }
            all_points[curr_points->obstacle_name].swap(points);
        }
        eraseTimeOutSensor(timeout_sensors);
    }

    void getObstacleRegions(std::map<std::string,ObstaclePoints::ObaState>& oba_states){
        auto curr_time = sros::core::util::get_time_in_us();
        for(auto& oba:obstacles_){
            auto &curr_points = oba.second;
            if (curr_points->is_region_oba) {
                if (curr_time - curr_points->time > delta_time_thresh_) {
                    LOG(INFO) << "delta time too large!"<<double(curr_time - curr_points->time)/1e6;
                    continue;
                }
                oba_states[curr_points->obstacle_name] = (curr_points->oba_state);
            }
        }
    }
private:
    void eraseTimeOutSensor(const std::vector<std::string>& sensors){
        for (auto& sensor : sensors) {
            auto iter = obstacles_.find(sensor);
            if (iter != obstacles_.end()) {
                obstacles_.erase(iter);
            }
        }
    }

    std::map<std::string,ObstaclePoints_Ptr> obstacles_;//障碍点,考虑到不同传感器生成不同类型障碍点,这里分别存储,后期分别进行处理
    int64_t delta_time_thresh_ = 5e5;//如果障碍信息的时间太滞后,则舍弃;目前尚未处理
};

}


#endif //PROJECT_OBSTACLE_POINTS_MANAGER_HPP
