//
// Created by lfc on 19-2-15.
//

#ifndef PROJECT_DETECTOR_POINT_HPP
#define PROJECT_DETECTOR_POINT_HPP

#include <glog/logging.h>

namespace detector{
class Point{
public:
    Point():x(0),y(0){}
    Point(double coor_x,double coor_y):x(coor_x),y(coor_y){}
    double x;
    double y;
    //向量加法
    Point operator +(const Point& other) const{
        return Point(x + other.x, y + other.y);
    }

    //向量减法
    Point operator -(const Point& other) const{
        return Point(x - other.x, y - other.y);
    }

    //向量数量积
    double operator *(const Point& other) const{
        return (x * other.x + y * other.y);
    }

    Point operator =(const Point& other) {
        x = other.x;
        y = other.y;
    }

    //数乘
    Point scalarMultiply(const double scalar) const{
        return Point(scalar * x, scalar * y);
    }

    //向量积绝对值
    double height(const Point& other) const{//注意,other为单位向量才有效，可看成向量积的绝对值
        return fabs(x * other.y - other.x * y);
    }

    double norm() const{    //范数(模)
        return std::hypot(x,y);
    }

    void unitlize(){    //单位化
        auto dist = norm();
        if(dist == 0) {
            LOG(INFO) << "dist is zero!";
            x = 1;
            y = 0;
        }
        x /= dist;  //此处不应该是else???
        y /= dist;
    }
};
}


#endif //PROJECT_DETECTOR_POINT_HPP
