//
// Created by lfc on 18-10-19.
//

#ifndef PROJECT_MOTION_PREDICTION_HPP
#define PROJECT_MOTION_PREDICTION_HPP

#include "fix_array.hpp"
#include <vector>
#include <math.h>
#include <glog/logging.h>
#include <chrono>

namespace rack{
class MotionPrediction {
public:

    MotionPrediction():delta_yaw_array(10),last_yaw(0) {
        initial_state = false;
        reset();
        last_watch_dog_time = 0;
    }

    bool isInitialized() {
        return initial_state;
    }

    void reset() {
        initial_state = false;
        int size = delta_yaw_array.arraySize();
        for (int i = 0; i < size; ++i) {
            delta_yaw_array[i] = 0;
        }
    }

    void initialize(double curr_yaw) {
        reset();
        initial_state = true;
        last_yaw = curr_yaw;
        last_watch_dog_time = get_timestamp_in_ms();
//        delta_yaw_array.push_back(curr_yaw);
    }

    bool isNear(double curr_yaw,double curr_yaw_thresh){
        if(!isInitialized()) {
            return true;//如果没有初始化,那么直接认为是near,意思是无法判断是否很近.
        }
        auto curr_time = get_timestamp_in_ms();
        if((curr_time-last_watch_dog_time)>400) {
            LOG(INFO) << "too long time! will return true,delta is:"<<(curr_time - last_watch_dog_time);
            return true;
        }
        double delta_yaw = curr_yaw - last_yaw;
        normalizeAngle(delta_yaw);
        if (fabs(delta_yaw) < curr_yaw_thresh) {
            return true;
        }
        return false;
    }

    int findNearestIndex(std::vector<double> yaws) {
        if(!isInitialized()) {
            LOG(INFO) << "have not initialized! will return false!";
        }
        if (yaws.size() == 0) {
            LOG(INFO) << "cannot find! will return -1";
            return -1;
        }
        double delta_yaw = M_PI;
        int index = 0;
        int count = 0;
        for (auto &yaw:yaws) {
            double tmp_delta = normalizeAngle(last_yaw - yaw);
            if (delta_yaw > fabs(tmp_delta)) {
                delta_yaw = tmp_delta;
                index = count;
            }
            count++;
        }
        return index;
    }

    void pushBackAngle(double curr_yaw) {
        if (isInitialized()) {
            delta_yaw_array.push_back(normalizeAngle(last_yaw - curr_yaw));
            last_yaw = curr_yaw;
            last_watch_dog_time = get_timestamp_in_ms();
        }else {
            LOG(INFO) << "have not initialized!";
        }
    }

    bool isRotate(double rotate_thresh){
        double sum_angle = 0;
        int size = delta_yaw_array.arraySize();
        for (int i = 0; i < size; ++i) {
            sum_angle += delta_yaw_array[i];
        }
        if (fabs(sum_angle) > rotate_thresh) {
            return true;
        }
        return false;
    }


    inline double normalizeAngle(const double &angle) {
        auto new_angle = fmod(angle, M_PI);
        if (new_angle >= M_PI_2) {
            new_angle -= M_PI;
        } else if (new_angle < -M_PI_2) {
            new_angle += M_PI;
        }
        return new_angle;
    }

    inline uint64_t get_time_in_ns() {
        return static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    }


    uint64_t get_timestamp_in_ms() {
        return static_cast<uint64_t>(get_time_in_ns() / 1000000L);
    }
private:

    FixArray<double> delta_yaw_array;
    double last_yaw ;
    bool initial_state;
    int64_t last_watch_dog_time;
};

}


#endif //PROJECT_MOTION_PREDICTION_HPP
