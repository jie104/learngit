//
// Created by yj on 19-6-14.
//

#ifndef SROS_COST_VALUES_H
#define SROS_COST_VALUES_H

namespace costmap_2d
{
static const unsigned char NO_INFORMATION = 255; // 表示没有信息
static const unsigned char LETHAL_OBSTACLE = 254; // 表示障碍物
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253; // 表示地图上障碍物的有非障碍物的交接点
static const unsigned char FREE_SPACE = 0; // 表示地图上没有障碍物
}

#endif //SROS_COST_VALUES_H
