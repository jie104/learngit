//
// Created by lfc on 2021/10/14.
//

#include "motion_state_detector.h"

namespace sros{
std::shared_ptr<MotionStateDetector> MotionStateDetector::state_detector_;
boost::mutex MotionStateDetector::thread_mutex;

std::shared_ptr<MotionStateDetector> MotionStateDetector::getInstance() {
    if (!state_detector_) {
        boost::mutex::scoped_lock lock(thread_mutex);
        if (!state_detector_) {
            state_detector_.reset(new MotionStateDetector());
        }
    }
    return state_detector_;
}
}
