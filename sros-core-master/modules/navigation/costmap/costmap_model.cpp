//
// Created by yj on 19-6-14.
//
#include "costmap_model.h"
#include "cost_values.h"
#include "../lib/include/jps_grid.h"
using namespace costmap_2d;

CostmapModel::CostmapModel( struct grid& ma) : costmap_(ma) {}

double CostmapModel::footprintCost(const sros::core::Pose& position,sros::core::grid_point offset,const std::vector<sros::core::Pose>& footprint,
                                   double inscribed_radius, double circumscribed_radius){

    //首先需要根据当前点和ａｇｖ的轮廓计算轮廓点在世界坐标系的位置
    sros::core::Pose p5;
    std::vector<sros::core::Pose> oriented_footprint_spec;
    oriented_footprint_spec.clear();
    //将agv坐标系的点转化为世界坐标系的点。
    for(unsigned int i = 0; i < footprint.size(); ++i) {
        p5.x()=position.x()+footprint[i].y()*cos(position.yaw())+footprint[i].x()*sin(position.yaw());
        p5.y()=position.y()+footprint[i].y()*sin(position.yaw())-footprint[i].x()*cos(position.yaw());
        oriented_footprint_spec.push_back(p5);
    }

    //used to put things into grid coordinates
     int cell_x, cell_y;
    convertMapCoords(offset.x,offset.y, position.x(), position.y(), &cell_x, &cell_y);
    //if number of points in the oriented_footprint_spec is less than 3, we'll just assume a circular robot
    if(oriented_footprint_spec.size() < 3){
        unsigned char cost = getNodeAt(&costmap_,cell_x, cell_y)->grid_layered_walkable_;
        //if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
        if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION)
            return -1.0;
        return cost;
    }
    //now we really have to lay down the footprint in the costmap grid
     int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;
    //we need to rasterize each line in the footprint
    for(unsigned int i = 0; i < oriented_footprint_spec.size() - 1; ++i){
        //get the cell coord of the first point
//        if(!costmap_.worldToMap(oriented_footprint_spec[i].x, oriented_footprint_spec[i].y, x0, y0))
//            return -1.0;
        convertMapCoords(offset.x,offset.y, oriented_footprint_spec[i].x(), oriented_footprint_spec[i].y(), &x0, &y0);
        //get the cell coord of the second point
        convertMapCoords(offset.x,offset.y, oriented_footprint_spec[i + 1].x(), oriented_footprint_spec[i + 1].y(), &x1, &y1);
       // if(!costmap_.worldToMap(oriented_footprint_spec[i + 1].x, oriented_footprint_spec[i + 1].y, x1, y1))
        //    return -1.0;

        if(abs(x0)>295||abs(y0)>295||abs(x1)>295||abs(y1)>295||x0<0||y0<0||x1<0||y1<0)
        {
          //  LOG(INFO)<< "1局部坐标"<<x0<<" "<<y0<<" "<<x1<<" "<<y1<<" ";
            continue;
        }

            line_cost = lineCost(x0, x1, y0, y1);



        footprint_cost = std::max(line_cost, footprint_cost);

        //if there is an obstacle that hits the line... we know that we can return false right away
        if(line_cost < 0)
            return -1.0;
    }
    //we also need to connect the first point in the oriented_footprint_spec to the last point
    //get the cell coord of the last point
//    if(!costmap_.worldToMap(oriented_footprint_spec.back().x, oriented_footprint_spec.back().y, x0, y0))
//        return -1.0;
    convertMapCoords(offset.x,offset.y, oriented_footprint_spec.back().x(), oriented_footprint_spec.back().y(), &x0, &y0);

    convertMapCoords(offset.x,offset.y, oriented_footprint_spec.front().x(), oriented_footprint_spec.front().y(), &x1, &y1);

    if(abs(x0)>295||abs(y0)>295||abs(x1)>295||abs(y1)>295||x0<0||y0<0||x1<0||y1<0)
    {
       // LOG(INFO)<< "2局部坐标"<<x0<<" "<<y0<<" "<<x1<<" "<<y1<<" ";

    }else {
        line_cost = lineCost(x0, x1, y0, y1);
        footprint_cost = std::max(line_cost, footprint_cost);
        if(line_cost < 0)
            return -1.0;
    }
    //get the cell coord of the first point
//    if(!costmap_.worldToMap(oriented_footprint_spec.front().x, oriented_footprint_spec.front().y, x1, y1))
//        return -1.0;





    convertMapCoords(offset.x,offset.y, oriented_footprint_spec[0].x(), oriented_footprint_spec[0].y(), &x0, &y0);

    convertMapCoords(offset.x,offset.y, oriented_footprint_spec[2].x(), oriented_footprint_spec[2].y(), &x1, &y1);

    //get the cell coord of the first point
//    if(!costmap_.worldToMap(oriented_footprint_spec.front().x, oriented_footprint_spec.front().y, x1, y1))
//        return -1.0;

    if(abs(x0)>295||abs(y0)>295||abs(x1)>295||abs(y1)>295||x0<0||y0<0||x1<0||y1<0)
    {
      //  LOG(INFO)<< "3局部坐标"<<x0<<" "<<y0<<" "<<x1<<" "<<y1<<" ";

    }else {
        line_cost = lineCost(x0, x1, y0, y1);
        footprint_cost = std::max(line_cost, footprint_cost);
        if(line_cost < 0)
            return -1.0;
    }

    convertMapCoords(offset.x,offset.y, oriented_footprint_spec[1].x(), oriented_footprint_spec[1].y(), &x0, &y0);

    convertMapCoords(offset.x,offset.y, oriented_footprint_spec[3].x(), oriented_footprint_spec[3].y(), &x1, &y1);

    //get the cell coord of the first point
//    if(!costmap_.worldToMap(oriented_footprint_spec.front().x, oriented_footprint_spec.front().y, x1, y1))
//        return -1.0;

    if(abs(x0)>295||abs(y0)>295||abs(x1)>295||abs(y1)>295||x0<0||y0<0||x1<0||y1<0)
    {
       // LOG(INFO)<< "4局部坐标"<<x0<<" "<<y0<<" "<<x1<<" "<<y1<<" ";

    }else {
        line_cost = lineCost(x0, x1, y0, y1);
        footprint_cost = std::max(line_cost, footprint_cost);
        if(line_cost < 0)
            return -1.0;
    }

    //if all line costs are legal... then we can return that the oriented_footprint_spec is legal
    return footprint_cost;

}

//calculate the cost of a ray-traced line
double CostmapModel::lineCost(int x0, int x1, int y0, int y1)  {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for( LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance() )
    {
        point_cost = pointCost( line.getX(), line.getY() ); //Score the current point

        if(point_cost < 0)
            return -1;

        if(line_cost < point_cost)
            line_cost = point_cost;
    }

    return line_cost;
}

double CostmapModel::pointCost(int x, int y)  {
    unsigned char cost = getNodeAt(&costmap_,x, y)->grid_layered_walkable_;
    //if the cell is in an obstacle the path is invalid
    //if(cost == LETHAL_OBSTACLE){
    if(cost == LETHAL_OBSTACLE || cost == NO_INFORMATION || cost == INSCRIBED_INFLATED_OBSTACLE){
        return -1;
    }

    return cost;
}
