#ifndef __included_map_erode_h
#define __included_map_erode_h

#include "core/map/NavigationMap.hpp"
#include "ipl/pict.hh"

typedef std::vector<sros::map::Point> Obstacle_vector;

enum StructuringElementType {

    StructuringElementSquare = 0x01, //square-shaped structuring element

    StructuringElementCircle = 0x02, //circle-shaped structuring element

    StructuringElementDiamond = 0x03, //diamond-shaped structuring element

    StructuringElementLine = 0x04, //line-shaped structuring element

};

/*
 * @brief 加载map_data,转换为腐蚀操作的数据结构ipl::Region
 * @param map_data 地图数组
 * @param x, y 地图width和height
 */
ipl::Region loadImage(sros::map::PyramidNavMap_ptr map_data, int x, int y);

/*
 * @brief 腐蚀操作结束后数据回写回map_data
 */
void writeBackToMap(ipl::PictImage bin, sros::map::PyramidNavMap_ptr map_data);

/*
 * @brief 腐蚀静态地图并回写
 * @param kenel_type==1表矩形内核， ==2表椭圆内核，==3表菱形内核， ==4表十字形内核
 * @param write_to==1表写入静态不可走区域， write_to==2表写入缓冲区域
 * @param inflation_factor膨胀半径
 */
void inflationStaticMap(sros::map::PyramidNavMap_ptr navigation_map,
                        sros::map::PyramidNavMap_ptr inflation_map_data,
                        int width, int height, int inflation_factor,
                        StructuringElementType kenel_type, int origin_map);

/*
 * @brief 障碍点腐蚀.
 * 地图左下角坐标为(offset_x, offset_y),radius为腐蚀区域半径
 * 产生width * height的地图，输入障碍信息进行腐蚀操作，结果保存在obstacle_data_中.
 */
Obstacle_vector inflationObstacles(Obstacle_vector obstacles, int width, int height,
                                   StructuringElementType kenel_type, int inflation_factor);

#endif