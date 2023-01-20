//
// Created by lfc on 19-2-15.
//

#ifndef PROJECT_DETECTOR_POINT_HPP
#define PROJECT_DETECTOR_POINT_HPP

namespace detector{
class Point{
public:
    Point():x(0),y(0){}
    Point(double coor_x,double coor_y):x(coor_x),y(coor_y){}
    double x;
    double y;
    Point operator +(const Point& other) const{
        return Point(x + other.x, y + other.y);
    }

    Point operator -(const Point& other) const{
        return Point(x - other.x, y - other.y);
    }

    double operator *(const Point& other) const{
        return (x * other.x + y * other.y);
    }

    Point operator =(const Point& other) {
        x = other.x;
        y = other.y;
    }

    Point scalarMultiply(const double scalar) const{
        return Point(scalar * x, scalar * y);
    }

    double height(const Point& other) const{//注意,other为单位向量才有效
        return fabs(x * other.y - other.x * y);
    }

    double norm() const{
        return std::hypot(x,y);
    }

    void unitlize(){
        auto dist = norm();
        if(dist == 0) {
            LOG(INFO) << "dist is zero!";
            x = 1;
            y = 0;
        }
        x /= dist;
        y /= dist;
    }
};
}


#endif //PROJECT_DETECTOR_POINT_HPP
