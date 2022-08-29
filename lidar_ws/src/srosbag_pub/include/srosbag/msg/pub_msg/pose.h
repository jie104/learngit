/*
 * location.h
 *
 *  Created on: 2015年12月10日
 *      Author: lhx
 */

#ifndef CORE_BAG_POSE_H_
#define CORE_BAG_POSE_H_

#include <vector>

namespace sros {
namespace bag {

class Location {
public:
    Location() : x_(0), y_(0), z_(0) { }
    Location(double x, double y) : x_(x), y_(y), z_(0) { }
    Location(double x, double y, double z): x_(x), y_(y), z_(z) { }

    virtual ~Location() {}

    double x() const { return x_; }
    double& x() { return x_; }

    double y() const { return y_; }
    double& y() { return y_; }

    double z() const { return z_; }
    double& z() { return z_; }

private:
    double x_;
    double y_;
    double z_;
};

class Rotation {
public:
    Rotation() : yaw_(0), pitch_(0), roll_(0) { }
    Rotation(double yaw): yaw_(yaw), pitch_(0), roll_(0) { }

    Rotation(double yaw, double pitch, double roll)
            : yaw_(yaw), pitch_(pitch), roll_(roll) { }

    virtual ~Rotation() { }

    double yaw() const { return yaw_; }
    double& yaw() { return yaw_; }

    double pitch() const { return pitch_; }
    double& pitch() { return pitch_; }

    double roll() const { return roll_; }
    double& roll() { return roll_; }

private:
    double yaw_;
    double pitch_;
    double roll_;
};

class Pose {
public:
    Pose() : location_(), rotation_() { }
    Pose(const Location& location, const Rotation& rotation)
            : location_(location), rotation_(rotation) { }
    Pose(const Location& location) : location_(location) { }
    Pose(const Rotation& rotation) : rotation_(rotation) { }

    virtual ~Pose() { }

    double x() const { return location_.x(); }
    double& x() { return location_.x(); }

    double y() const { return location_.y(); }
    double& y() { return location_.y(); }

    double z() const { return location_.z(); }
    double& z() { return location_.z(); }

    double yaw() const { return rotation_.yaw(); }
    double& yaw() { return rotation_.yaw(); }

    double pitch() const { return rotation_.pitch(); }
    double& pitch() { return rotation_.pitch(); }

    double roll() const { return rotation_.roll(); }
    double& roll() { return rotation_.roll(); }

    Location location() const { return location_;}
    Location& location() { return location_;}

    Rotation rotation() const { return rotation_;}
    Rotation& rotation() { return rotation_;}

private:
    Location location_;
    Rotation rotation_;
};
typedef std::vector<Location> Location_Vector;
typedef std::vector<Rotation> Rotation_Vector;
typedef std::vector<Pose> Pose_Vector;
} /* namespace core */
} /* namespace sros */

#endif /* CORE_POSE_H_ */
