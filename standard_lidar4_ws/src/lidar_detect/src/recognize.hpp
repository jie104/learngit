#ifndef _RECOGNIZE_
#define _RECOGNIZE_

#include "glog/logging.h"
#include <cmath>
#include<sensor_msgs/LaserScan.h>
#include "base_data_type.hpp"
#include <mutex>


namespace recoginze
{

    double CalScalarProduct(const base_data::LaserScan_ptr &scan,std::vector<float> &ranges,int id,const int &step){
        double angle_increment=scan->angle_increment;
        double angle_min=scan->angle_min;

        double angle_id=angle_min+(float)id*angle_increment;
        base_data::Point middle_point=base_data::Point(ranges[id]*cos(angle_id),ranges[id]*sin(angle_id));
        base_data::Point vector_left,vector_right;
        base_data::Point sumVector_left,sumVector_right;


        for (int i=id-step;i<id;i++){
            double angle=angle_min+(float)i*angle_increment;
            double range=ranges[i];
            if (range <0.1 || range >15){
                continue;
            }

            vector_left=base_data::Point(range*cos(angle),range*sin(angle));
            base_data::Point temp_vector=vector_left-middle_point;
            sumVector_left=sumVector_left+temp_vector;
            
        }
        sumVector_left.unitizate();

        

        for (int j=id;j<id+step;j++){
            double angle=angle_min+(float)j*angle_increment;
            double range=ranges[j];
            if (range <0.1 || range >15){
                continue;
            }
            vector_right=base_data::Point(range*cos(angle),range*sin(angle));
            base_data::Point temp_vector=vector_right-middle_point;
            sumVector_right=sumVector_right+temp_vector;
            
        }
        sumVector_right.unitizate();

        return sumVector_left*sumVector_right;
  


    }

    void findFeaturePoint(const base_data::LaserScan_ptr &scan,std::vector<float> &ranges,int& middle_num,int& beginId,int& endId,const int step=30){
        bool flag1=true,flag2=true;
        int ranges_size=ranges.size();
        double thresh=-0.70;
        std::mutex mute;
        for (int i=0;i<ranges_size/2;i++){
            if (CalScalarProduct(scan,ranges,middle_num-i,step) > thresh && flag1){
                beginId=middle_num-i;
                // LOG(INFO) << "...beginId: " << beginId;
                flag1=false;

            }

            if (CalScalarProduct(scan,ranges,middle_num+i,step) > thresh && flag2){
                endId=middle_num+i;
                flag2=false;
                // LOG(INFO) << "...endId: " << endId;
                // LOG(INFO) << "CalScalarProduct(scan,middle_num+i,step) :" << CalScalarProduct(scan,middle_num+i,step) ;
            }

            if (!flag1 && !flag2){
                break;
            }

        }
    }







} // namespace name

#endif