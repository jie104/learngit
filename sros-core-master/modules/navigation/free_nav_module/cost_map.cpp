//
// Created by yj on 20-8-26.
//
// 创建一个栅格地图表示方法。
#include <clocale>
#include <cstring>
#include "cost_map.h"


CostMap::CostMap(unsigned int cells_size_x, unsigned int cells_size_y,double resolution,
        double origin_x,double origin_y, unsigned char default_value):
        size_x_(cells_size_x),size_y_(cells_size_y),
        resolution_(resolution),origin_x_(origin_x),origin_y_(origin_y),costmap_(nullptr),default_value_(default_value)
{
    initMaps(size_x_,size_y_);
}


void CostMap::initMaps(unsigned int size_x,unsigned int size_y){

    delete[] costmap_;
    costmap_ = new unsigned char[size_x * size_y];
}

void CostMap::resetMaps()
{
    memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void CostMap::deleteMaps()
{
    // clean up data
    delete[] costmap_;
    costmap_ = nullptr;
}

void CostMap::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

bool CostMap::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
    if (wx < origin_x_ || wy < origin_y_)
        return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_)
        return true;

    return false;
}

void CostMap::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
    costmap_[getIndex(mx, my)] = cost;
}


unsigned char CostMap::getCost(unsigned int mx, unsigned int my) const
{
    return costmap_[getIndex(mx, my)];
}



double CostMap::getOriginX() const
{
    return origin_x_;
}

double CostMap::getOriginY() const
{
    return origin_y_;
}

double CostMap::getResolution() const
{
    return resolution_;
}