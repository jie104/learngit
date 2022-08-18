//
// Created by lfc on 17-7-11.
//

#ifndef SROS_POINT_CONTAINER_H
#define SROS_POINT_CONTAINER_H
#include <memory>
#include <vector>
#include <Eigen/Dense>
namespace mapping{
struct MapPoint{
    MapPoint(float x_,float y_):x(x_),y(y_){

    }

    MapPoint():x(0),y(0){

    }

    MapPoint(Eigen::Vector2f& point):x(point[0]),y(point[1]){

    }
    float x;
    float y;

    MapPoint operator*(const float scale) {
        return MapPoint(x * scale, y * scale);
    }

};
class PointContainer {
public:
    PointContainer(){

    }
    std::vector<MapPoint> points;
    Eigen::Vector3f origin;
private:

};

typedef std::shared_ptr<PointContainer> PointContainer_Ptr;
}



#endif //SROS_POINT_CONTAINER_H
