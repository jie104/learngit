/**
 * @file common_msgs.h
 * @author zmy (626670628@qq.com)
 * @brief geometry_msgs
 * @version 0.1
 * @date 2021-03-30
 * 
 * 
 */

#ifndef GEOMETRY_MSGS
#define GEOMETRY_MSGS

#include <Eigen/Geometry>

namespace sros
{
    namespace core
    {
        namespace geometry_msgs
        {
            using Quaternion = Eigen::Quaternionf;
            using Vector3 = Eigen::Vector3f;
            using Point = Eigen::Vector3f;
            struct Pose
            {
                Point position;
                Quaternion orientation;
            };

            struct Twist
            {
                Vector3 linear;
                Vector3 angular;
            };

        } // namespace geometry_msgs
    }
}

#endif
