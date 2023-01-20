#ifndef _BASE_DATA_TYPE_
#define _BASE_DATA_TYPE_


#include <ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include <map>
#include <cmath>
#include <vector>
#include <glog/logging.h>
#include <string>
#include <memory>
#include <fstream>
#include <laser_geometry/laser_geometry.h>




namespace base_data
{
    typedef sensor_msgs::LaserScan::Ptr LaserScan_ptr;

    struct Point{
        Point():x(0),y(0) {}
        Point(double a,double b):x(a),y(b) {}

        Point operator -(Point& P){
            return Point(x-P.x,y-P.y);
        }
        Point operator +(Point& P){
            return Point(x+P.x,y+P.y);
        }

        double operator*(Point& P){
            return x*P.x+y*P.y;
        }


        double Mod(){
            if (x==0 && y==0){
                LOG(INFO) << "the Mod is zero!!!  it is wrong";
                return 0;
            }
            return sqrt(x*x+y*y);
        }

        void unitizate(){
            double mod=sqrt(x*x+y*y);
            if (mod==0){
            }
            x=x/mod;
            y=y/mod;
        }
    

        double x;
        double y;
    };

}



#endif