//
// Created by yj on 19-10-8.
//

#ifndef GLOBAL_PLAN_COLLISIONDETECTION_H
#define GLOBAL_PLAN_COLLISIONDETECTION_H

#include <modules/navigation/nav_config.h>
#include "constants.h"
#include "node2d.h"
#include "node3d.h"
#include "../lib/include/jps_grid.h"

namespace HybridAStar {
namespace {
void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
    x = node->getX();
    y = node->getY();
    // avoid 2D collision checking
    t = 99;
}

void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
    x = node->getX();
    y = node->getY();
    t = node->getT();
}
}
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
class CollisionDetection {
public:
    /// Constructor
    CollisionDetection();


    /*!
       \brief evaluates whether the configuration is safe
       \return true if it is traversable, else false
    */
    template<typename T> bool isTraversable(const T* node) {
        /* Depending on the used collision checking mechanism this needs to be adjusted
           standard: collision checking using the spatial occupancy enumeration
           other: collision checking using the 2d costmap and the navigation stack
        */
        float cost = 0;
        float x;
        float y;
        float t;
        // assign values to the configuration
        getConfiguration(node, x, y, t);
        int i_x = (int)x;
        int i_y = (int)y;
        float i_t = t;

        
        // 2D collision test
        if (t == 99) {
            return !(grid_map_->nodes[i_y][i_x].grid_layered_walkable_!=0x3C);
        }

        if (true) {
            cost = configurationTest(x, y, t) ? 0 : 1;
        } else {
            cost = configurationCost(x, y, t);
        }

        return cost <= 0;
    }

    void InitConfig(NavConfig& cfg){
            cfg_ = cfg;
    }

     bool isRotateTraversable(Node3D* node) {
        /* Depending on the used collision checking mechanism this needs to be adjusted
           standard: collision checking using the spatial occupancy enumeration
           other: collision checking using the 2d costmap and the navigation stack
        */

        bool cost = true;
        float x;
        float y;
        float t;
        // assign values to the configuration
        getConfiguration(node, x, y, t);
        int i_x = (int)x;
        int i_y = (int)y;
        // 2D collision test

        // 需要根据车的长和宽来判断
        float agv_length =cfg_.robot.length;
        float agv_width = cfg_.robot.width;
        float margin = 0.05;

        ///////////////////////test 判断是否加入货架模型
        if(g_state.load_state == sros::core::LOAD_FULL){
            agv_length = cfg_.rack_length;
            agv_width = cfg_.rack_width;
            LOG(INFO) << "当前处于背货状态，避障尺寸为：长：" << cfg_.robot.length << ";宽：" << cfg_.robot.width;
        }
        else{
            agv_length = sros::core::Settings::getInstance().getValue<double>("nav.vehicle_length", 0.8);                          
            agv_width =  sros::core::Settings::getInstance().getValue<double>("nav.vehicle_width", 0.6);
            LOG(INFO) << "当前处于空车状态，避障尺寸为：长：" << cfg_.robot.length << ";宽：" << cfg_.robot.width;
        }
        ///////////////////////test

        int h = (int)((sqrt(pow(agv_length,2)+pow(agv_width,2))+margin)/cfg_.nav_resolution*100.0/2.0); //0.04
        for(int i =i_x - h;i<=i_x+h;i++){
            int w = (int)sqrt( pow(h,2) - pow(i-i_x,2));
            for(int j = i_y-w;j<= i_y+w;j++){
                cost = configurationTest(i, j, t) ;
                if(!cost) {
                    return false;
                }
            }
        }
         return  true;
    }

    /*!
       \brief Calculates the cost of the robot taking a specific configuration q int the World W
       \param x the x position
       \param y the y position
       \param t the theta angle
       \return the cost of the configuration q of W(q)
       \todo needs to be implemented correctly
    */
    float configurationCost(float x, float y, float t) {return 0;}

    /*!
       \brief Tests whether the configuration q of the robot is in C_free
       \param x the x position
       \param y the y position
       \param t the theta angle
       \return true if it is in C_free, else false
    */
    bool configurationTest(float x, float y, float t);

    bool check_line_collision(Node3D* node);

    bool check_point_collision1(const Node3D* node);

    unsigned  char getNodeValue(int x,int y){
        return  grid_map_->nodes[y][x].grid_layered_walkable_;
    }

    /*!
       \brief updates the grid with the world map
    */
    void updateGrid( struct grid* map) {grid_map_ = map;}

     NavConfig cfg_;

private:
    /// The occupancy grid
    struct grid*  grid_map_;
   
    /// The collision lookup table
   // Constants::config collisionLookup[Constants::headings * Constants::positions];
};
}


#endif //GLOBAL_PLAN_COLLISIONDETECTION_H
