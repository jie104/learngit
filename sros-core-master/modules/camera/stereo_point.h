//
// Created by lfc on 19-5-5.
//

#ifndef SROS_STEREO_POINT_H
#define SROS_STEREO_POINT_H

#include <vector>

struct StereoPoints {
    struct Point {
        float x;
        float y;
        float z;
        float inten;
    };

    void clear() {
        points.clear();
    }

    void reserve(int size) {
        points.reserve(size);
    }

    void push_back(const Point &curr_point) {
        points.push_back(curr_point);
    }

    Point &back(){ return points.back(); }

    void emplace_back(){ points.emplace_back(); }

    std::vector<Point> points;
};


#endif //SROS_STEREO_POINT_H
