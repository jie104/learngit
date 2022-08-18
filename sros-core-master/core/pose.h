/*
 * location.h
 *
 *  Created on: 2015年12月10日
 *      Author: lhx
 */

#ifndef CORE_POSE_H_
#define CORE_POSE_H_

#include <cmath>
#include <string>
#include <vector>
#include <glog/logging.h>
namespace sros {
namespace core {

typedef struct GridPoint {
    int x;
    int y;
} grid_point;

class Location {
 public:
    Location() : x_(0), y_(0), z_(0) {}
    Location(double x, double y) : x_(x), y_(y), z_(0) {}
    Location(double x, double y, double z) : x_(x), y_(y), z_(z) {}

    virtual ~Location() = default;

    const double& x() const { return x_; }
    double& x() { return x_; }

    const double& y() const { return y_; }
    double& y() { return y_; }

    const double& z() const { return z_; }
    double& z() { return z_; }

    template <typename TOut>
    friend TOut& operator<<(TOut& out, const Location& location) {
        out << "Location(" << location.x() << ", " << location.y() << ", " << location.z() << ")";
        return out;
    }

 private:
    double x_;
    double y_;
    double z_;
};

class Rotation {
 public:
    Rotation() : yaw_(0), pitch_(0), roll_(0) {}
    explicit Rotation(double yaw) : yaw_(yaw), pitch_(0), roll_(0) {}

    Rotation(double yaw, double pitch, double roll) : yaw_(yaw), pitch_(pitch), roll_(roll) {}

    virtual ~Rotation() = default;

    const double& yaw() const { return yaw_; }
    double& yaw() { return yaw_; }

    const double& pitch() const { return pitch_; }
    double& pitch() { return pitch_; }

    const double& roll() const { return roll_; }
    double& roll() { return roll_; }

 private:
    double yaw_;
    double pitch_;
    double roll_;
};

class Pose {
 public:
    Pose() : location_(), rotation_() {}
    Pose(const Location& location, const Rotation& rotation) : location_(location), rotation_(rotation) {}

    explicit Pose(const Location& location) : location_(location) {}
    explicit Pose(const Rotation& rotation) : rotation_(rotation) {}

    Pose(const Location& location, const Rotation& rotation, double confidence)
        : location_(location), rotation_(rotation), confidence_(confidence) {}

    Pose(double x, double y) : location_(x, y) {}
    Pose(double x, double y, double yaw) : location_(x, y), rotation_(yaw) {}
    Pose(double x, double y, double yaw, double confidence)
        : location_(x, y), rotation_(yaw), confidence_(confidence) {}

    virtual ~Pose() = default;

    const double& x() const { return location_.x(); }
    double& x() { return location_.x(); }

    const double& y() const { return location_.y(); }
    double& y() { return location_.y(); }

    const double& z() const { return location_.z(); }
    double& z() { return location_.z(); }

    const double& yaw() const { return rotation_.yaw(); }
    double& yaw() { return rotation_.yaw(); }

    const double& pitch() const { return rotation_.pitch(); }
    double& pitch() { return rotation_.pitch(); }

    const double& roll() const { return rotation_.roll(); }
    double& roll() { return rotation_.roll(); }

    Location location() const { return location_; }
    Location& location() { return location_; }

    Rotation rotation() const { return rotation_; }
    Rotation& rotation() { return rotation_; }

    double confidence() const { return confidence_; }
    double& confidence() { return confidence_; }

    int64_t timestamp() const { return odo_timestamp_; }
    int64_t& timestamp() { return odo_timestamp_; }

    int64_t synctimestamp() const { return sync_timestamp_; }
    int64_t& synctimestamp() { return sync_timestamp_; }

    double distance_to(const Pose& p2) const {
        double dx = x() - p2.x();
        double dy = y() - p2.y();
        return std::sqrt(dx * dx + dy * dy);
    }

    bool operator==(const Pose& other) const {
        if (this == &other) {
            return true;
        }

        if (this->x() == other.x() && this->y() == other.y() && this->yaw() == other.yaw()) {
            return true;
        }
        return false;
    }

    template <typename TOut>
    friend TOut& operator<<(TOut& out, const Pose& pose) {
        out << "Pose(" << pose.x() << ", " << pose.y() << ", " << pose.yaw() << ")";
        return out;
    }

 private:
    Location location_;
    Rotation rotation_;
    int64_t odo_timestamp_;
    int64_t sync_timestamp_;

    double confidence_ = 0.0;  // 0 <= confidence <= 1
};

class Velocity {
 public:
    Velocity() {}
    Velocity(double vx, double vy, double vtheta) : v_x_(vx), v_y_(vy), v_theta_(vtheta) {}

    virtual ~Velocity() {}

    double vx() const { return v_x_; }
    double& vx() { return v_x_; }

    double vy() const { return v_y_; }
    double& vy() { return v_y_; }

    double vtheta() const { return v_theta_; }
    double& vtheta() { return v_theta_; }

    double a() const { return a_; }
    double& a() { return a_; }


    double w_a() const { return w_a_; }
    double& w_a() { return w_a_; }

 private:
    double v_x_;
    double v_y_;
    double v_theta_;
    double a_;
    double w_a_;
};

typedef std::vector<sros::core::Location> Location_Vector;
typedef std::vector<sros::core::Rotation> Rotation_Vector;
typedef std::vector<sros::core::Pose> Pose_Vector;

} /* namespace core */
} /* namespace sros */

#endif /* CORE_POSE_H_ */
