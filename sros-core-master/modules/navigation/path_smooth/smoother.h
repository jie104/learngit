/*
 * @Author: your name
 * @Date: 2020-06-16 09:21:56
 * @LastEditTime: 2021-06-28 20:58:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /sros/modules/navigation/path_smooth/smoother.h
 */ 
//
// Created by yj on 19-11-6.
//

#ifndef SROS_SMOOTHER_H
#define SROS_SMOOTHER_H

#include "vector2d.h"
#include "../hybrid_astar/node3d.h"
#include "../check_collision.h"
#include "../hybrid_astar/constants.h"
#include "helper.h"
#include "../lib/include/jps_grid.h"
#include "../lib/include/navigation.h"
#include "modules/navigation/reference_paths/reference_paths.h"

////////////////////test by 吴运才
extern ReferencePaths g_gradient_of_ref_path;    //加载参考路径势场地图
////////////////////


using namespace HybridAStar;
class Smoother {
public:
    Smoother(){}
    ~Smoother();



    void initGridMap(int width1,int height1 );

    void initNavConfig(NavConfig& cfg);

   // void initGridMap(struct grid& localmap );

    void addObstacleFromGridMap(Navigation& navigate_,std::vector<Eigen::Vector2d>& oba_points);

    void deleteObstacleFromGridMap(Navigation& navigate_,std::vector<Eigen::Vector2d> oba_points);

    void pathSample(sros::core::NavigationPath_vector final_path,Navigation& navigate_);

    void pathResample(sros::core::NavigationPath_vector final_path,Navigation& navigate_);

    void convertPath(Navigation& navigate_,sros::core::NavigationPath_vector& cur_paths_,std::vector<sros::core::Pose>& opt_paths);

    void setPath(std::vector<Node3D> path);
    std::vector<Node3D> getPath() {return path;}
    /*!
       \brief 采用共轭梯度法 优化轨迹点，主要包括远离障碍物，曲率，平滑性三个方面。
    */
    void smoothPath(CheckCollision& global_map);

    void smoothPath1(CheckCollision& global_map);

    /// obstacleCost - pushes the path away from obstacles
    Vector2D obstacleTerm(Vector2D xi,CheckCollision& global_map);

    /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
    Vector2D curvatureTerm(Vector2D xim1, Vector2D xi, Vector2D xip1);

    /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
    Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

    Vector2D smoothnessTerm1(Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

    Vector2D smoothnessTerm2(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1);

    bool   isOnGrid(Vector2D vec){
        if (vec.getX() >= 0 && vec.getX() < width &&
            vec.getY() >= 0 && vec.getY() < height) {
            return true;
        }
        return false;
    }

    //CheckCollision_Ptr smoother_map_;
    // MAP_POINT **collision_map1;

private:
    /// maximum possible curvature of the non-holonomic vehicle
    float kappaMax = 1.f / (Constants::r * 1.1);
    /// maximum distance to obstacles that is penalized
    //float obsDMax = cfg_->smoother_param.minRoadWidth;
    /// maximum distance for obstacles to influence the voronoi field
    float vorObsDMax = Constants::minRoadWidth;
    /// falloff rate for the voronoi field
    float alpha =0.25;
    /// weight for the obstacle terma
    float wObstacle = 0.5;
    /// weight for the voronoi term
    float wVoronoi = 0;
    /// weight for the curvature term
    float wCurvature = 0.0;
    /// weight for the smoothness term
    float wSmoothness = 6.0;
    /// voronoi diagram describing the topology of the map
    unsigned char** gridMap;
    /// width of the map
    int width;
    /// height of the map
    int height;
    /// path to be smoothed
    std::vector<Node3D> path;
    NavConfig* cfg_;


};



#endif //SROS_SMOOTHER_H
