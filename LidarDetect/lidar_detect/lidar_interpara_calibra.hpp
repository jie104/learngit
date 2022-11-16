//
// Created by zxj on 2022/9/16.
//

#ifndef _LIDAR_INTERPARA__LIB_
#define _LIDAR_INTERPARA__LIB_

#include <ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include <map>
#include <cmath>
#include <vector>
#include <glog/logging.h>
#include <string>
#include <memory>
#include <fstream>
#include "solveCloudPoint.hpp"
#include "recognize.hpp"
#include "base_data_type.hpp"
#include <eigen3/Eigen/Core>

namespace calibra
{

struct InterLibraManager{

    double rotate_angle;    //旋转单位角度
    std::vector<double> error_ranges;  //误差值
    std::vector<double> ave_error_ranges;   //平均距离误差
    std::map<double,double> interpla_container;    //<角度，距离>,用于存放倍加福雷达插值后的数据,角度与低成本雷达对齐
    std::map<int,double> compensate_ranges;  //<点云id，补偿距离>
};

struct Lidar{ 
    double angle_incre;
    double angle_min;
    double angle_max;
    double forward_angle;    //低成本雷达正前方
    std::string topic;
    std::vector<float> ranges;
    std::vector<double> angles;    
    std::pair<int ,double> angle_manager; //<点数，角度>

};


class LidarInterParaLibra
{
public:

    LidarInterParaLibra(){
        fout_.open("/home/zxj/data/lidar_detect/forward_angle_error.txt",std::ios::out );//重新写入会覆盖原来内容
        fout_ << "lidar_topic  " << "forward_line_angle" << "  单位: 度 " << std::endl;
        pub_compensate_angle_=nh_.advertise<sensor_msgs::LaserScan>("/compensate_angle_pld",1000);
        pub_compensate_ranges_=nh_.advertise<sensor_msgs::LaserScan>("/compensate_ranges_pld",1000);
        inter_libra_manager_.ave_error_ranges.resize(3600,0.f);
    //从lidar_error_inspect.launch中读取，当该文件中不存在该项时，取后面的默认值，否则以该文件中的值为准
        nh_.param<std::string>("/lowcost_lidar/LowCostTopic",lowcost_lidar_.topic,"/wanji_scan");
        nh_.param<int>("/lowcost_lidar/Max_num_",max_num_,500);
        // lowcost_sub_=nh_.subscribe("LowCostTopic",30,&LidarInterParaLib::LowCostcallback,this);
        lowcost_sub_=nh_.subscribe(lowcost_lidar_.topic,1000,&LidarInterParaLibra::LowCostcallback,this);

        pepperl_sub_=nh_.subscribe("/r2000_node/scan",1000,&LidarInterParaLibra::r2000callback,this);


    }

    virtual ~LidarInterParaLibra(){
        fout_.close();
    }

//TODO：函数命名规范：第一个是动词，小写onScanCallback();
    void r2000callback(const base_data::LaserScan_ptr& r2000_scan){
        pepperl_lidar_.forward_angle=(r2000_scan->angle_max+r2000_scan->angle_min)/2; //direction_in_forward;
        auto& ranges=r2000_scan->ranges;
        int ranges_size=ranges.size();
        calAngle(r2000_scan,pepperl_lidar_.angle_manager);
        if (pepperl_lidar_.angle_manager.first >min_line_simulation_size_ ){
            fout_ << "/r2000_node/scan " << Rad2Angle(pepperl_lidar_.angle_manager.second) << std::endl;

        }
        
        inter_libra_manager_.interpla_container[lowcost_lidar_.forward_angle]=r2000_scan->ranges[pepperl_lidar_.forward_angle];
        //计算插值
        calInter(r2000_scan,1); 
        calInter(r2000_scan,-1);
        //计算角度误差
        calAngleErrorIndex(r2000_scan);
        //计算距离误差
        calRangeErrorIndex(inter_libra_manager_.interpla_container);

    }

    void LowCostcallback(const base_data::LaserScan_ptr& lowcost_scan){
        if (flag_){
            inter_libra_manager_.rotate_angle=lowcost_scan->angle_increment;
            size_=lowcost_scan->ranges.size()/2;  

            flag_=false;
        }
        updateLowCostLidarParam(lowcost_scan,lowcost_lidar_);
        if (lowcost_lidar_.angle_manager.first >min_line_simulation_size_){
            fout_ << lowcost_lidar_.topic << " " << Rad2Angle(lowcost_lidar_.angle_manager.second) << std::endl;
        }
        // LOG(INFO) << "lowcost_angle_: " << lowcost_angle_*180/M_PI;

    
    }


protected:  
    void pubCompensateAnglePld(const base_data::LaserScan_ptr& scan,double& CompensateAngle){
        sensor_msgs::LaserScan compensate_scan;
        compensate_scan=*scan;
        compensate_scan.angle_min=scan->angle_min+CompensateAngle;

        pub_compensate_angle_.publish(compensate_scan);


    }
    
    //TODO:1. 算法库需要具有可移植性，只有算法才在这里实现，pub属于业务代码，放在外层。要用模板实现算法，提升适应性
    void pubCompensateRangePld(const base_data::LaserScan_ptr& scan,std::map<int,double>& CompensateRanges){
        sensor_msgs::LaserScan compensate_scan;
        auto ranges=scan->ranges;
        for (auto& compensate_range:CompensateRanges){
            if (fabs(compensate_range.second) < 0.10){
                int point_id=compensate_range.first;
                ranges[point_id]=ranges[point_id]+compensate_range.second;
            }
        }
        compensate_scan=*scan;
        compensate_scan.ranges=ranges;
        pub_compensate_ranges_.publish(compensate_scan);
    }

    void updateLowCostLidarParam(const base_data::LaserScan_ptr& scan,Lidar& lowcost_lidar){
        lowcost_scan_=scan;
        lowcost_lidar.forward_angle=(scan->angle_max+scan->angle_min)/2;    //默认雷达正前方在雷达物理前方计算

        lowcost_lidar.angle_incre=scan->angle_increment;
        lowcost_lidar.angle_min=scan->angle_min;
        lowcost_lidar.ranges=scan->ranges;

        auto& ranges=scan->ranges;
        int ranges_size=ranges.size();
        calAngle(scan,lowcost_lidar.angle_manager);
    }

    void Mean(std::vector<double>& laser_angles,double& mean_angle){
        for (int i=0;i<laser_angles.size();i++){
            mean_angle+=laser_angles[i];        
        }
        mean_angle=mean_angle/laser_angles.size();
    }

    void calAngleErrorIndex(const base_data::LaserScan_ptr &scan){

        if (0==angle_num_){
            lowcost_lidar_.angles.reserve(max_num_);
            pepperl_lidar_.angles.reserve(max_num_);
        }
        
        if (angle_num_<max_num_){

            if (lowcost_lidar_.angle_manager.first > min_line_simulation_size_){
                // LOG(INFO) << "lowcost_angle_manager_: " << lowcost_angle_manager_.first;
                lowcost_lidar_.angles.emplace_back(lowcost_lidar_.angle_manager.second);
            }
            if (pepperl_lidar_.angle_manager.first > min_line_simulation_size_){
                // LOG(INFO) << "pepperl_angle_manager_: " << pepperl_angle_manager_.first;
                pepperl_lidar_.angles.emplace_back(pepperl_lidar_.angle_manager.second);//TODO:了解vector特性
            }

        }
        else{
            double mean_lowcost_angle=0;
            double mean_pepperl_angle=0;
            double forward_angle_error=0;

            Mean(lowcost_lidar_.angles,mean_lowcost_angle);
            Mean(pepperl_lidar_.angles,mean_pepperl_angle);
            forward_angle_error=mean_lowcost_angle-mean_pepperl_angle;
            LOG(INFO) << "the number of cycles is " << max_num_;
            LOG(INFO) << "the forward angle error is " << Rad2Angle(forward_angle_error) << " 度";
            LOG(INFO) << "the lowcost angle error is " << Rad2Angle(mean_lowcost_angle) << " 度";
            LOG(INFO) << "the pepperl angle error is " << Rad2Angle(mean_pepperl_angle) << " 度";
            pubCompensateAnglePld(scan,forward_angle_error);
            // LOG(INFO) << "mean_theta_: " << mean_theta_;
            lowcost_lidar_.angles.clear();
            pepperl_lidar_.angles.clear();



            angle_num_=-1;
        }
        angle_num_+=1;

    }


    void calAngle(const base_data::LaserScan_ptr& scan,std::pair<int,double>& angle_manager){

        double angle_min=scan->angle_min;
        double angle_increment=scan->angle_increment;
        auto ranges=scan->ranges;

        scan::LidarPointProcessor::filter(scan,ranges,15,0.02);
        double ranges_size=scan->ranges.size();
        int middle_num=ranges_size/2;
        int begin_Id,end_Id;
        recoginze::findFeaturePoint(scan,ranges,middle_num,begin_Id,end_Id);
        offset_=(end_Id-begin_Id)/10;
        if (offset_<0){
            LOG(INFO) << "offset is negative!!!";
        }
        begin_Id=begin_Id+offset_;
        end_Id=end_Id-offset_;


        std::vector<Eigen::Vector2d> points; 
        points.reserve(end_Id-begin_Id);
        for (int i=begin_Id;i<end_Id;++i){
            double range=ranges[i]; 
            if (range < 0.1 || range >15){  //去噪点
                continue;
            }

            double angle_2=angle_min+angle_increment*(float)(i);
            points.emplace_back(); //此处需对共享指针初始化，否则其将是空指针
            points.back()[0]=range*cos(angle_2);
            points.back()[1]=range*sin(angle_2);
            // LOG(INFO) << ".........";

        }

        angle_manager.first=points.size();
        angle_manager.second=scan::LidarPointProcessor::calAngle(points);
        points.clear();
    }


    void findPointCloudId(const base_data::LaserScan_ptr& scan,double inter_angle){
        double angle_min=scan->angle_min;
        double angle_incre=scan->angle_increment;
        int size=scan->ranges.size();
        // LOG(INFO) << "size: " << size;
        for (int i=0;i<size-1;i++)
        {
            if (inter_angle>= angle_min+i*angle_incre && inter_angle <= angle_min+(i+1)*angle_incre){
                id_=i;
                break;
            }
        }

    }

    void calInter(const base_data::LaserScan_ptr& scan,int weight){ //weight取1或-1，1表示往雷达正前方左侧插值，-1代表右侧
        auto& ranges=scan->ranges;
        double angle_min=scan->angle_min;
        double angle_incre=scan->angle_increment;
        double range=ranges[ranges.size()/2];
        for (int i=1;i<size_;i++){
            double inter_angle=pepperl_lidar_.forward_angle+i*lowcost_lidar_.angle_incre*weight; //在倍加福雷达极坐标系下考虑
            double x_0=range*cos(inter_angle);
            double y_0=range*sin(inter_angle);
            range=sqrt(x_0*x_0+y_0*y_0);

            findPointCloudId(scan,inter_angle);  //找到id
            //计算相邻右侧点云直角坐标
            double right_angle=angle_min+id_*angle_incre;

            double x_1=ranges[id_]*cos(right_angle);
            double y_1=ranges[id_]*sin(right_angle);
            //计算相邻左侧点云直角坐标
            double left_angle=angle_min+(id_+1)*angle_incre;
            double x_2=ranges[id_+1]*cos(left_angle);
            double y_2=ranges[id_+1]*sin(left_angle);

            //计算顺时针方向weight为-1，逆时针方向weight为1
            double X_0=x_0*cos(weight*inter_libra_manager_.rotate_angle)-y_0*sin(weight*inter_libra_manager_.rotate_angle);
            double Y_0=x_0*sin(weight*inter_libra_manager_.rotate_angle)+y_0*cos(weight*inter_libra_manager_.rotate_angle);

            //计算k值
            double K=(y_2-y_1)/(x_2-x_1);
            if (x_1==x_2){
                LOG(INFO) << "x_2==x_1";
            }
            else{
                // LOG(INFO) << "x_2-x1: " << x_2-x_1;
            }
            double k=(y_1-K*x_1)/(Y_0-K*X_0);
            if (Y_0-K*X_0==0){
                LOG(INFO) << "Y_0-K*X_0 is zero";
            }
            else{
                // LOG(INFO) << "Y_0-K*X_0 : " << Y_0-K*X_0;
            }
            
            X_0=X_0*k;
            Y_0=Y_0*k;
            double result=(x_1-x_2)*(y_1-Y_0)-(x_1-X_0)*(y_1-Y_0);
            double inter_range=sqrt(X_0*X_0+Y_0*Y_0);
            inter_libra_manager_.interpla_container[lowcost_lidar_.forward_angle+i*lowcost_lidar_.angle_incre*weight]=inter_range;    //在低成本雷达极坐标系下考虑

        }

    }

    void calRangeErrorIndex(std::map<double,double>& inter_ranges){
        double inter_angle;
        for (int i=0;i<2*size_-1;i++){
            inter_angle=lowcost_lidar_.forward_angle-(size_-1-i)*lowcost_lidar_.angle_incre;    //默认低成本雷达正前方角度为0度

            int lowcost_point_id=(inter_angle-lowcost_lidar_.angle_min)/lowcost_lidar_.angle_incre;   //todo

            double error_range=inter_ranges[inter_angle]-lowcost_lidar_.ranges[lowcost_point_id];
            inter_libra_manager_.compensate_ranges[lowcost_point_id]=error_range;
            error_range=fabs(error_range);
            if (error_range < 0.05){    //若误差距离小于5cm，则保存，否则丢弃
             //**********************由于点云存在跳动，可能会让error_ranges_每次的个数不一样，从而产生问题!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                inter_libra_manager_.error_ranges.emplace_back(error_range);      

            }
        }
        pubCompensateRangePld(lowcost_scan_,inter_libra_manager_.compensate_ranges);
        
        double error_ranges_sum=0;
        if (range_num_<max_num_){
            for (int i=0;i< inter_libra_manager_.error_ranges.size();i++){
                inter_libra_manager_.ave_error_ranges[i]+=inter_libra_manager_.error_ranges[i];
            }
        }
        else{

            for (int j=0;j< inter_libra_manager_.error_ranges.size();j++){
                inter_libra_manager_.ave_error_ranges[j]/=max_num_;
            }
            for (int k=0;k<inter_libra_manager_.error_ranges.size() ;k++){
                error_ranges_sum+=inter_libra_manager_.ave_error_ranges[k];
            }
            int error_ranges_num=inter_libra_manager_.error_ranges.size();

            LOG(INFO) << "lowcost_total_points: " << lowcost_lidar_.ranges.size();
            LOG(INFO) << "lowcost_topic: " << lowcost_lidar_.topic;
            LOG(INFO) << "max_num: " << max_num_ <<  " ,sum: " << error_ranges_sum << " m";
            LOG(INFO) << "error_ranges_num: " << error_ranges_num << ",the average range error of lidar: "
                 << (error_ranges_sum/error_ranges_num)*1000 << " mm";
            std::cout << "===============================================================================" << std::endl;
            ros::Duration du(0.5);//持续2秒钟,参数是double类型的，以秒为单位
            du.sleep();//按照指定的持续时间休眠
            inter_libra_manager_.ave_error_ranges.clear();
            inter_libra_manager_.ave_error_ranges.resize(3600,0.f);

            range_num_=-1;
        }
        range_num_+=1;



        inter_libra_manager_.error_ranges.clear();


    }


    //将弧度转化为角度
    double Rad2Angle(double& rad){
        double Angle=rad*180/M_PI;
        return Angle;
    }
    

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_compensate_angle_;    
    ros::Publisher pub_compensate_ranges_;
    ros::Subscriber lowcost_sub_;    //低成本雷达订阅
    ros::Subscriber pepperl_sub_;    //倍加福雷达订阅
    base_data::LaserScan_ptr lowcost_scan_;

    
    int size_;     //num_=循环次数/2
    int id_;    //插值点云的左侧实际相邻点云序号
    int max_num_;   
    int offset_;  //点数余量
    std::fstream fout_;


    Lidar lowcost_lidar_;
    Lidar pepperl_lidar_;
    InterLibraManager inter_libra_manager_;
    
    bool flag_=true;    
    int range_num_=0;  
    int angle_num_=0; 
    int min_line_simulation_size_=30;   

};

}
#endif
