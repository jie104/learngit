//
// Created by lfc on 2021/10/14.
//

#ifndef SROS_MOTION_STATE_DETECTOR_H
#define SROS_MOTION_STATE_DETECTOR_H
#include <boost/thread/mutex.hpp>
#include <memory>
#include <core/tf/TransForm.h>
#include <core/util/time.h>

namespace sros{
class MotionStateDetector {
 public:
    static std::shared_ptr<MotionStateDetector> getInstance();

    virtual ~MotionStateDetector(){

    }

    void updateTF(const slam::tf::TransForm& curr_tf){
        if (last_tf_.pose_time == 0) {
            last_tf_ = curr_tf;
        }
        Eigen::Vector3d delta_move;
        delta_move[0] = (curr_tf.position.x() - last_tf_.position.x());
        delta_move[1] = (curr_tf.position.y() - last_tf_.position.y());
        delta_move[2] = (curr_tf.rotation.yaw() - last_tf_.rotation.yaw());
        if (delta_move.norm() >= 1.0e-5) {//运动
            static_state_ = false;
            last_tf_ = curr_tf;
        }else{
            int64_t curr_time = sros::core::util::get_time_in_us();
            if ((curr_time - last_tf_.pose_time) > static_time_thresh_) {
                if (!static_state_) {
                    static_state_ = true;
                }
            }else{
                if (static_state_) {
                    static_state_ = false;
                }
            }
        }
    }

    bool staticForLongDuration(const int64_t &duration_time){
        if (static_state_) {
            int64_t curr_time = sros::core::util::get_time_in_us();
            if ((curr_time - last_tf_.pose_time) > duration_time) {
                return true;
            }
        }
        return false;
    }

    bool staticState(){ return static_state_; }//这个值设置了一个默认static的时长，如果满足默认时长，那就说明是静止的。如果外部需要停止更长时间，可以通过另一个函数调用。

 private:
    MotionStateDetector(){
        last_tf_.pose_time = 0;
    }

    static std::shared_ptr<MotionStateDetector> state_detector_;
    static boost::mutex thread_mutex;

    slam::tf::TransForm last_tf_;
    bool static_state_ = false;
    const int64_t static_time_thresh_ = 1e6;//1秒以上
};

}

#endif  // SROS_MOTION_STATE_DETECTOR_H
