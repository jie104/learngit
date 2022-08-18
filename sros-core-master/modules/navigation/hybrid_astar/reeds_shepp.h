//
// Created by yj on 19-9-29.
//

#ifndef DUBINS_REEDS_SHEPP_REEDS_SHEPP_H
#define DUBINS_REEDS_SHEPP_REEDS_SHEPP_H

#include <limits>
#include "math.h"
#include "assert.h"
#include "node3d.h"
#include "iostream"
using namespace HybridAStar;
class  ReedsSheppStateSpace{
public:
    /** \brief The Reeds-Shepp path segment types */
    enum ReedsSheppPathSegmentType
    {
        RS_NOP = 0,
        RS_LEFT = 1,
        RS_STRAIGHT = 2,
        RS_RIGHT = 3
    };

    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];


    class ReedsSheppPath
    {
    public:
        ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                       double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                       double w = 0., double x = 0.);
        double length() const
        {
            return totalLength_;
        }

        /** Path segment types */
        const ReedsSheppPathSegmentType *type_;
        /** Path segment lengths */
        double length_[5];
        /** Total length */
        double totalLength_;
    };

    ReedsSheppStateSpace(double turningRadius = 1.0) : rho_(turningRadius)
    {
    }

    ReedsSheppPath reedsShepp(const Node3D state1, const Node3D state2) const;

    void interpolate(const Node3D from, const ReedsSheppPath &path,double t ,Node3D* state) const;

protected:
    double rho_;
};


#endif //DUBINS_REEDS_SHEPP_REEDS_SHEPP_H
