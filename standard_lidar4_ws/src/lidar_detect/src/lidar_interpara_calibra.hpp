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
#include <eigen3/Eigen/Dense>
#include <stdexcept>

namespace calibra
{

struct InterCalibraManager{

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
        fout_ << "lidar_topic  " << "forward_angle_error" << "  单位: 度 " << std::endl;
        pub_compensate_angle_=nh_.advertise<sensor_msgs::LaserScan>("/compensate_angle_pcl",1000);
        pub_compensate_ranges_=nh_.advertise<sensor_msgs::LaserScan>("/compensate_ranges_pcl",1000);
        pub_pepperl_compensate_ranges_=nh_.advertise<sensor_msgs::LaserScan>("/compensate_average_ten_pcl",1000);
        pub_debug_icp_pcl_=nh_.advertise<sensor_msgs::PointCloud>("/pub_debug_icp_pcl_",1000);

        inter_calibra_manager_.ave_error_ranges.resize(3600,0.f);
    //从lidar_error_inspect.launch中读取，当该文件中不存在该项时，取后面的默认值，否则以该文件中的值为准
        nh_.param<std::string>("/lowcost_lidar/LowCostTopic",lowcost_lidar_.topic,"/simi_node/scan");
        nh_.param<int>("/lowcost_lidar/Max_num_",max_num_,500);
        // lowcost_sub_=nh_.subscribe("LowCostTopic",30,&LidarInterParaLib::LowCostcallback,this);
        lowcost_sub_=nh_.subscribe(lowcost_lidar_.topic,1000,&LidarInterParaLibra::LowCostcallback,this);

        pepperl_sub_=nh_.subscribe("/r2000_node/scan",1000,&LidarInterParaLibra::r2000callback,this);
        R2000_scan_.ranges.resize(3600,0);
//        lowcost_ranges_.reserve(500);
    }

    virtual ~LidarInterParaLibra(){
        fout_.close();
    }

    void r2000callback(const base_data::LaserScan_ptr& r2000_scan){
        pubPepperPcl(r2000_scan);
        pepperl_lidar_.forward_angle=(r2000_scan->angle_max+r2000_scan->angle_min)/2; //direction_in_forward;
        auto& ranges=r2000_scan->ranges;
        int ranges_size=ranges.size();
        calAngle(r2000_scan,pepperl_lidar_.angle_manager);
//        if (pepperl_lidar_.angle_manager.first >min_line_simulation_size_ ) {
//            fout_ << "/r2000_node/scan " << Rad2Angle(pepperl_lidar_.angle_manager.second) << std::endl;
//        }
        inter_calibra_manager_.interpla_container[lowcost_lidar_.forward_angle]=r2000_scan->ranges[pepperl_lidar_.forward_angle];
////计算直线计算点云0角度误差
//        calAngleErrorIndex(r2000_scan);

////利用icp循环计算0度角偏差，标定点云不实时显示
//        if (average_count_<average_count_Max_+1){
//            average_scan(average_r2000_scan_,average_lowcost_scan_,r2000_scan);
//        }else {
//            forward_angle_error_=IcpR2000Lowcost(average_r2000_scan_,average_lowcost_scan_);
//            pubCompensateAnglePcl(lowcost_scan_,forward_angle_error_);
//            LOG(INFO) << "===================forward_angle_error_: " << forward_angle_error_*180/M_PI << " degree";
//            fout_ << "forward_angle_error: " << forward_angle_error_*180/M_PI << std::endl;
//            average_count_=0;
//            average_r2000_scan_->ranges.clear();
//            average_lowcost_scan_->ranges.clear();
//        }


////利用icp只计算一次0度角偏差，实现标定点云实时显示
        if (average_count_<average_count_Max_+1){
            average_scan(average_r2000_scan_,average_lowcost_scan_,r2000_scan);
        }else if (flag2_){
//            LOG(INFO) << "222222222222";
            forward_angle_error_=IcpR2000Lowcost(average_r2000_scan_,average_lowcost_scan_);
            flag2_=false;
            LOG(INFO) << "forward_angle_error_: " << forward_angle_error_*180/M_PI << " degree";
        }else if (forward_angle_error_!=0){
//            LOG(INFO) << "33333333333";
            pubCompensateAnglePcl(lowcost_scan_,forward_angle_error_);
        }

        //计算插值
        if (forward_angle_error_!=0) {
            std::shared_ptr<sensor_msgs::LaserScan> scan_ptr(new sensor_msgs::LaserScan);
            *scan_ptr=*r2000_scan;
            scan_ptr->angle_min=r2000_scan->angle_min-forward_angle_error_;
            calInter(scan_ptr, 1);
            calInter(scan_ptr, -1);
            //计算距离误差
            calRangeErrorIndex(inter_calibra_manager_.interpla_container);
        }

    }

    void LowCostcallback(const base_data::LaserScan_ptr& lowcost_scan){
        if (flag_){
            inter_calibra_manager_.rotate_angle=lowcost_scan->angle_increment;
            size_=lowcost_scan->ranges.size()/2;  

            flag_=false;
        }

//        double sigema=calStd(lowcost_scan);
//        if (sigema!=0){
//            double three_sigema=sigema*3.0*MtoMM;
////            LOG(INFO) << "number is " << max_num_ <<  " ,3*sigema is "
//                << three_sigema;
//        }
        updateLowCostLidarParam(lowcost_scan,lowcost_lidar_);
//        cal3Std(lowcost_scan)
//        if (lowcost_lidar_.angle_manager.first >min_line_simulation_size_){
//            fout_ << lowcost_lidar_.topic << " " << Rad2Angle(lowcost_lidar_.angle_manager.second) << std::endl;
//        }
        // LOG(INFO) << "lowcost_angle_: " << lowcost_angle_*180/M_PI;

    
    }


protected:
    /**
     * 平均20帧点云，试图优化ICP结果
     * @param r2000_scan
     * @param lowcost_scan
     * @param r2000_scan1
     */
    void average_scan(base_data::LaserScan_ptr& r2000_scan,base_data::LaserScan_ptr& lowcost_scan,
                      const base_data::LaserScan_ptr& r2000_scan1){
        int lowcost_scan_size=lowcost_scan_->ranges.size();
        int r2000_scan1_size=r2000_scan1->ranges.size();
        if (!r2000_scan){
            r2000_scan.reset(new sensor_msgs::LaserScan);
            *r2000_scan=*r2000_scan1;
            r2000_scan->ranges.clear();
        }
        if(!lowcost_scan){
            lowcost_scan.reset(new sensor_msgs::LaserScan);
            *lowcost_scan=*lowcost_scan_;
            lowcost_scan->ranges.clear();
        }
        if (average_count_==0){
            for (int i=0;i<lowcost_scan_size;i++){
                lowcost_scan->ranges.emplace_back(lowcost_scan_->ranges[i]);
            }
            for (int j=0;j<r2000_scan1_size;j++){
                r2000_scan->ranges.emplace_back(r2000_scan1->ranges[j]);
            }
//            LOG(INFO) << "r2000_scan->ranges.size: " << r2000_scan->ranges.size();
            ++average_count_;

        }
        else if (average_count_<average_count_Max_+1){
            if (average_count_<average_count_Max_){
                for (int i=0;i<lowcost_scan_size;i++){
                    lowcost_scan->ranges[i]+=lowcost_scan_->ranges[i];
                }
                for (int j=0;j<r2000_scan1_size;j++){
                    r2000_scan->ranges[j]+=r2000_scan1->ranges[j];
                }
                ++average_count_;
            }else{
                for (int i=0;i<lowcost_scan->ranges.size();i++){
                    lowcost_scan->ranges[i]/=average_count_Max_;
//                    LOG(INFO) << "lowcost_scan->ranges[i]: " << lowcost_scan->ranges[i];
                }
                for (int j=0;j<r2000_scan->ranges.size();j++){
                    r2000_scan->ranges[j]/=average_count_Max_;
//                    LOG(INFO) << "r2000_scan->ranges[i]: " << r2000_scan->ranges[j];
                }
                average_count_=average_count_Max_+2;
            }
        }
//        LOG(INFO) << "average_count_: " << average_count_;
    }
    /**
     * 计算点云质心
     * @return
     */
    Eigen::Vector2f calPclCentroid(const std::vector<Eigen::Vector2f>& points){
        Eigen::Vector2f Centroid=Eigen::Vector2f::Zero();
        int size=points.size();
        for (int i=0;i<size;i++){
            Centroid+=points[i];
        }
        return Centroid/size;
    }

    /**
     * 计算第i个点对应的角度
     * @param scan
     * @param i
     * @return
     */
    double calTheta(const base_data::LaserScan_ptr& scan,int i){
        double angle_min=scan->angle_min;
        double angle_increment=scan->angle_increment;
        double angle=angle_min+i*angle_increment;
        return angle;
    }
    /**
     * 寻找最近邻近点对应的距离
     * @param range
     * @param r2000_ranges
     * @return
     */
    Eigen::Vector2f findNearestPoint(const Eigen::Vector2f& point,const std::vector<Eigen::Vector2f>& r2000_points){
        Eigen::Vector2f target_point;
        int size=r2000_points.size();
        double min_distance=MAXFLOAT;
        for (int i=0;i<size;i++){
            double delta_range=(r2000_points[i]-point).norm();
//            LOG(INFO) << "delta_range: " << delta_range;
            if (delta_range < min_distance){
                min_distance=delta_range;
                target_point=r2000_points[i];
            }
        }
        //此现象是雷达噪点造成，需先做滤波处理
        if (min_distance >1){
//            LOG(INFO) << "point.norm: " << point.norm();
//            LOG(INFO) << "min_distance: " << min_distance << ",min_distance is wrong!!!";
        }
        return target_point;
    }

    /**
     * 计算旋转矩阵R
     * @param nearest_points
     * @param lowcost_points
     * @return
     */
    Eigen::Matrix<float,2,2> calRotateMatrix(std::vector<Eigen::Vector2f>& nearest_points,std::vector<Eigen::Vector2f>& lowcost_points){
        Eigen::Matrix<float,2,2> H=Eigen::Matrix<float,2,2>::Zero();
        Eigen::Vector2f nearest_pcl_centroid=calPclCentroid(nearest_points);
        Eigen::Vector2f lowcost_pcl_centroid=calPclCentroid(lowcost_points);
        int size =nearest_points.size();
        for (int i=0;i<size;i++){
            nearest_points[i]=nearest_points[i]-nearest_pcl_centroid;
            lowcost_points[i]=lowcost_points[i]-lowcost_pcl_centroid;
            H+=nearest_points[i]*lowcost_points[i].transpose();
        }
        //SVD分解
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<float,2,2> R=svd.matrixV()*svd.matrixU().transpose();
        double determinant=R.determinant();
//        LOG(INFO) << "determinant is " << determinant;
        if (determinant!=1.0){
            LOG(INFO) << "determinant: " << determinant << " The determinant of R is not 1";
        }
        return R.inverse();

    }

    /**
     * 将极坐标系转化为直角坐标系
     * @param ranges
     * @param scan
     * @return
     */
    std::vector<Eigen::Vector2f> convertToRetangualrCoordinate(std::vector<float>& ranges,const base_data::LaserScan_ptr& scan){
        Eigen::Vector2f ret_coord;
        std::vector<Eigen::Vector2f> ret_coords;
        int size=ranges.size();
        for (int i=0;i<size;i++){
            double angle= calTheta(scan,i);
//            LOG(INFO) << "angle: " << angle*180/M_PI;
            double range=ranges[i];
//            LOG(INFO) << "range: " << range;
            ret_coord=Eigen::Vector2f(range*cos(angle),range*sin(angle));
//            LOG(INFO) << "ret_coord: " << ret_coord.transpose();
            ret_coords.emplace_back(ret_coord);
        }
        return ret_coords;
    }

    /**
     * 旋转点云
     * @param points
     * @param R
     */
    std::vector<Eigen::Vector2f> rotatePcl(std::vector<Eigen::Vector2f> points,Eigen::Matrix<float,2,2>& R){
        for (auto & point:points){
            point=R*point;
//            LOG(INFO) << "point: " << point.transpose();
        }
        return points;

    }
    double calLoss(std::vector<Eigen::Vector2f>& nearest_points,std::vector<Eigen::Vector2f>& lowcost_points,double loss=0){
        int size=nearest_points.size();
        for (int i=0;i<size;i++){
            loss+=(nearest_points[i]-lowcost_points[i]).norm();
        }
        return loss/size;
    }
    /**
     * 利用ICP算法匹配倍加福点云，低成本雷达点云
     * @param r2000_scan
     * @param lowcost_scan
     */
    double IcpR2000Lowcost(const base_data::LaserScan_ptr& r2000_scan,const base_data::LaserScan_ptr& lowcost_scan){
        std::vector<Eigen::Vector2f> nearest_points;
        std::vector<Eigen::Vector2f> lowcost_points;
        std::vector<Eigen::Vector2f> r2000_points;
        std::vector<Eigen::Vector2f> lowcost_filter_points;

        Eigen::Matrix<float,2,2> R=calInitialRotateMatrix();
//        Eigen::Matrix<float,2,2> R=Eigen::Matrix<float,2,2>::Identity();

//        LOG(INFO) << "R: " << R;
        std::vector<float> r2000_scan_ranges=r2000_scan->ranges;
        std::vector<float> lowcost_scan_ranges=lowcost_scan->ranges;
        //需先做滤波处理
//        scan::LidarPointProcessor::filter(r2000_scan,r2000_scan_ranges,20);
//        scan::LidarPointProcessor::filter(lowcost_scan,lowcost_scan_ranges,20);

        lowcost_points=convertToRetangualrCoordinate(lowcost_scan_ranges,lowcost_scan);
        r2000_points=convertToRetangualrCoordinate(r2000_scan_ranges,r2000_scan);

        int cycle_num=0;
//        double loss=0;
        while (cycle_num<5){
//            LOG(INFO) << "cycle_num: " << cycle_num;
            std::vector<Eigen::Vector2f> rotate_lowcost_points=rotatePcl(lowcost_points,R);
            pubComensateDebugPcl(rotate_lowcost_points);
            int size=rotate_lowcost_points.size();
            nearest_points.reserve(size);
            lowcost_filter_points.reserve(size);
            for (int i=0;i<size;i++){
                Eigen::Vector2f point=rotate_lowcost_points[i];
                Eigen::Vector2f nearest_point=findNearestPoint(point,r2000_points);
//                LOG(INFO) << "(point-nearest_point).norm(): " << (point-nearest_point).norm();
                if ((point-nearest_point).norm()<50*MMtoM){
                    Eigen::Vector2f lowcost_filter_point=lowcost_points[i];
                    lowcost_filter_points.emplace_back(lowcost_filter_point);
                    nearest_points.emplace_back(nearest_point);
                }
            }
//            LOG(INFO) << "nearest_points size: " << nearest_points.size();
//            LOG(INFO) << "lowcost_points size: " << lowcost_filter_points.size();
            if (nearest_points.size()!=lowcost_filter_points.size()){
                LOG(INFO) << "=========================the size of nearest_points is wrong!!!";
            }
            R=calRotateMatrix(nearest_points,lowcost_filter_points);
            ++cycle_num;
//            loss= calLoss(nearest_points,lowcost_filter_points);
//            LOG(INFO) << "LOSS: " << loss;
            nearest_points.clear();
            lowcost_filter_points.clear();

        }
        double angle=acos(R(1,1));
        return angle;

    }

    /**
     * 计算雷达0度角精度
     * @param scan
     * @return
     */
    double calStd(const base_data::LaserScan_ptr& scan){
        auto ranges=scan->ranges;
        int forward_id=ranges.size()/2;
        double forward_range=ranges[forward_id];
        if (forward_range>0){
            lowcost_ranges_.emplace_back(forward_range);
        }
        decltype(lowcost_ranges_.size()) size=lowcost_ranges_.size();
        if (size==max_num_){
            double sum=0;
            double mean=Mean(lowcost_ranges_);
            for (const auto& x:lowcost_ranges_){
                sum+=(x-mean)*(x-mean);
            }
            lowcost_ranges_.clear();
            double sigema=sqrt(sum/static_cast<double>(size));
            return sigema;
        }

        return 0;

    }

    /**
     * 发布倍加福雷达每10帧平均点云
     * @param scan
     */
    void pubPepperPcl(const base_data::LaserScan_ptr& scan){
        R2000_scan_=*scan;
        for (int j=0;j<scan->ranges.size();j++){
            R2000_scan_.ranges[j]+=scan->ranges[j];
        }
        if (N==10){
//            LOG(INFO) << "scan->ranges[100]: " << scan->ranges[100] ;
            for (int k=0;k<scan->ranges.size();k++){
                R2000_scan_.ranges[k]/=10;
                R2000_scan_.ranges[k]*=5;
            }
//            LOG(INFO) << "R2000_scan_->ranges[100]:" << R2000_scan_.ranges[100] ;
            pub_pepperl_compensate_ranges_.publish(R2000_scan_);
            R2000_scan_.ranges.resize(R2000_scan_.ranges.size(),0);
            N=0;
        }
        ++N;
    }

    /**
     * 发布角度补偿点云，相对于倍加福雷达
     * @param scan
     * @param CompensateAngle
     */
    void pubCompensateAnglePcl(const base_data::LaserScan_ptr& scan,double& CompensateAngle){
        sensor_msgs::LaserScan compensate_scan;
        compensate_scan=*scan;
        compensate_scan.angle_min=scan->angle_min+CompensateAngle;
        pub_compensate_angle_.publish(compensate_scan);
    }

    void pubComensateDebugPcl(const std::vector<Eigen::Vector2f>& points){
        sensor_msgs::PointCloud compensate_scan;
        compensate_scan.header.frame_id="scan";
        LOG(INFO) << "BEGIN TO DEBUG PCL";
        for (int i=0;i<points.size();i++){
            compensate_scan.points.emplace_back();
            compensate_scan.points.back().x=points[i][0];
            compensate_scan.points.back().y=points[i][1];
        }
        pub_debug_icp_pcl_.publish(compensate_scan);


    }

    /**
     * 发布距离补偿点云
     * @param scan
     * @param CompensateRanges
     */
    void pubCompensateRangePcl(const base_data::LaserScan_ptr& scan,std::map<int,double>& CompensateRanges){
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

    double Mean(const std::vector<double> &v){
        auto end=v.end();
        double sum=0;
        for (auto beg=v.begin();beg!=end;++beg){
            sum+=*beg;
        }
        return sum/v.size();

    }
    /**
     * 计算单帧下的旋转矩阵,作为给定初始旋转
     */
    Eigen::Matrix<float,2,2> calInitialRotateMatrix(){
        Eigen::Matrix<float,2,2> R;
        double InitialRotateAngle;
        double lowcost_lidar_angle,pepperl_lidar_angle;

        if (lowcost_lidar_.angle_manager.first > min_line_simulation_size_){
            lowcost_lidar_angle=lowcost_lidar_.angle_manager.second;
        }else{
            throw std::runtime_error("line_simulation_size less than 30");
        }
        if (pepperl_lidar_.angle_manager.first > min_line_simulation_size_){
            pepperl_lidar_angle=pepperl_lidar_.angle_manager.second;
        }else{
            throw std::runtime_error("line_simulation_size less than 30");
        }
        InitialRotateAngle=pepperl_lidar_angle-lowcost_lidar_angle;
        LOG(INFO) << "InitialRotateAngle: " << InitialRotateAngle*180/M_PI;
        R << cos(InitialRotateAngle),-sin(InitialRotateAngle),
             sin(InitialRotateAngle),cos(InitialRotateAngle);
        return R;

    }

    /**
     * 计算雷达0度角偏差
     * @param scan
     */
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
//            LOG(INFO) << "angle_num_: " << angle_num_;
            if(angle_num_<=max_num_){
//                LOG(INFO) << "lowcost_lidar_.angles size is " << lowcost_lidar_.angles.size();
                if (lowcost_lidar_.angles.size()<5){
                    LOG(INFO) << "wrong!!! wrong!!! wrong!!! lowcost_lidar_.angles size is " << lowcost_lidar_.angles.size()
                              << " less than 5," << "please move lidar tool to a suitable direction !!!";
                }
                Mean(lowcost_lidar_.angles,mean_lowcost_angle);
                Mean(pepperl_lidar_.angles,mean_pepperl_angle);
                forward_angle_error_=mean_pepperl_angle-mean_lowcost_angle;
                LOG(INFO) << "the number of cycles is " << max_num_;
                LOG(INFO) << "the forward angle error is " << fabs(Rad2Angle(forward_angle_error_)) << " 度";
                LOG(INFO) << "the lowcost angle error is " << Rad2Angle(mean_lowcost_angle) << " 度";
                LOG(INFO) << "the pepperl angle error is " << Rad2Angle(mean_pepperl_angle) << " 度";
            }

            pubCompensateAnglePcl(lowcost_scan_,forward_angle_error_);
            // LOG(INFO) << "mean_theta_: " << mean_theta_;
            lowcost_lidar_.angles.clear();
            pepperl_lidar_.angles.clear();

        }
        angle_num_+=1;

    }

/**
 * 计算雷达0度角视野看到的直线角度，相对于雷达坐标系
 * @param scan
 * @param angle_manager
 */
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
            throw std::runtime_error("offset is negative!!!");
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


    void findPclId(std::shared_ptr<sensor_msgs::LaserScan>& scan,double inter_angle,int& id){
        double angle_min=scan->angle_min;
        double angle_incre=scan->angle_increment;
        int size=scan->ranges.size();
        // LOG(INFO) << "size: " << size;
        for (int i=0;i<size-1;i++)
        {
            double angle_left=angle_min+i*angle_incre;
            double angle_right=angle_min+(i+1)*angle_incre;
            if (inter_angle>= angle_left  && inter_angle <= angle_right){
                id=i;
                return;
            }
        }

    }

    /**
     * 对倍加福雷达进行插值
     * @param scan
     * @param weight
     */
    void calInter(std::shared_ptr<sensor_msgs::LaserScan>& scan,int weight){ //weight取1或-1，1表示往雷达正前方左侧插值，-1代表右侧
        int id;
        auto& ranges=scan->ranges;
        double angle_min=scan->angle_min;
        double angle_incre=scan->angle_increment;
        double inter_angle=pepperl_lidar_.forward_angle; //在倍加福雷达极坐标系下考虑
        for (int i=1;i<size_;i++){
            double x_0=cos(inter_angle);
            double y_0=sin(inter_angle);
            inter_angle=pepperl_lidar_.forward_angle+i*lowcost_lidar_.angle_incre*weight; //在倍加福雷达极坐标系下考虑
//            range=sqrt(x_0*x_0+y_0*y_0);

            findPclId(scan,inter_angle,id);  //找到id
            //计算相邻右侧点云直角坐标
            double right_angle=angle_min+id*angle_incre;
            double x_1=ranges[id]*cos(right_angle),y_1=ranges[id]*sin(right_angle);
            //计算相邻左侧点云直角坐标
            double left_angle=angle_min+(id+1)*angle_incre;
            double x_2=ranges[id+1]*cos(left_angle),y_2=ranges[id+1]*sin(left_angle);
            //计算顺时针方向weight为-1，逆时针方向weight为1
            double rotate_angle=weight*inter_calibra_manager_.rotate_angle;
            double X_0=x_0*cos(rotate_angle)-y_0*sin(rotate_angle);
            double Y_0=x_0*sin(rotate_angle)+y_0*cos(rotate_angle);

            //计算k值
            double K=(y_2-y_1)/(x_2-x_1);
            if (x_1==x_2){
                LOG(INFO) << "x_2==x_1";
            }
            double k=(y_1-K*x_1)/(Y_0-K*X_0);
            if (Y_0-K*X_0==0){
                LOG(INFO) << "Y_0-K*X_0 is zero";
            }

            X_0=X_0*k;
            Y_0=Y_0*k;
            double inter_range=sqrt(X_0*X_0+Y_0*Y_0);
            //在低成本雷达坐标下处理
            double lowcost_inter_angle=lowcost_lidar_.forward_angle+i*lowcost_lidar_.angle_incre*weight;
            inter_calibra_manager_.interpla_container[lowcost_inter_angle]=inter_range;    //在低成本雷达极坐标系下考虑

        }

    }

    /**
     * 尝试补偿每次测距跳动带来的误差
     * @param v
     */
    void addRemainderAverage(std::vector<double>& v){
        double mean=Mean(v);
//        LOG(INFO) << mean;
        for (int i=v.size();i<2*size_-1;i++){
            v.emplace_back(mean);
        }
    }

    /**
     * 计算平均测距误差，次数500次
     * @param inter_ranges
     */
    void calRangeErrorIndex(std::map<double,double>& inter_ranges){
        double inter_angle;
        inter_calibra_manager_.error_ranges.reserve(max_num_);
        for (int i=0;i<2*size_-1;i++){
            inter_angle=lowcost_lidar_.forward_angle-(size_-1-i)*lowcost_lidar_.angle_incre;    //默认低成本雷达正前方角度为0度

            int lowcost_point_id=(inter_angle-lowcost_lidar_.angle_min)/lowcost_lidar_.angle_incre;   //todo

            double error_range=inter_ranges[inter_angle]-lowcost_lidar_.ranges[lowcost_point_id];
            inter_calibra_manager_.compensate_ranges[lowcost_point_id]=error_range;
            error_range=fabs(error_range);
            if (error_range < 0.05){    //若误差距离小于5cm，则保存，否则丢弃
             //**********************由于点云存在跳动，可能会让error_ranges_每次的个数不一样，从而产生问题!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                inter_calibra_manager_.error_ranges.emplace_back(error_range);

            }
        }
        int error_ranges_num=inter_calibra_manager_.error_ranges.size();
        //补偿距离偏差开关
//        addRemainderAverage(inter_calibra_manager_.error_ranges);

        pubCompensateRangePcl(lowcost_scan_,inter_calibra_manager_.compensate_ranges);
        
        double error_ranges_sum=0;
        if (range_num_<max_num_){
            for (int i=0;i< inter_calibra_manager_.error_ranges.size();i++){
                inter_calibra_manager_.ave_error_ranges[i]+=inter_calibra_manager_.error_ranges[i];
            }

        }
        else{

            for (int j=0;j< inter_calibra_manager_.error_ranges.size();j++){
                inter_calibra_manager_.ave_error_ranges[j]/=max_num_;
            }
//            std::cout << "inter_calibra_manager_.error_ranges.size(): " << inter_calibra_manager_.error_ranges.size() << std::endl;
            for (int k=0;k<inter_calibra_manager_.error_ranges.size() ;k++){
                error_ranges_sum+=inter_calibra_manager_.ave_error_ranges[k];
            }
//            int error_ranges_num=inter_calibra_manager_.error_ranges.size();

            LOG(INFO) << "lowcost_total_points: " << lowcost_lidar_.ranges.size();
            LOG(INFO) << "lowcost_topic: " << lowcost_lidar_.topic;
            LOG(INFO) << "max_num: " << max_num_ <<  " ,sum: " << error_ranges_sum << " m";
            LOG(INFO) << "error_ranges_num: " << error_ranges_num << ",the average range error of lidar: "
                 << (error_ranges_sum/(inter_calibra_manager_.error_ranges.size()))*1000 << " mm";
            std::cout << "===============================================================================" << std::endl;
            ros::Duration du(0.5);//持续2秒钟,参数是double类型的，以秒为单位
            du.sleep();//按照指定的持续时间休眠
            inter_calibra_manager_.ave_error_ranges.clear();
            inter_calibra_manager_.ave_error_ranges.resize(3600,0.f);

            range_num_=-1;
        }
        range_num_+=1;



        inter_calibra_manager_.error_ranges.clear();


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
    ros::Publisher pub_pepperl_compensate_ranges_;
    ros::Publisher pub_debug_icp_pcl_;
    ros::Subscriber lowcost_sub_;    //低成本雷达订阅
    ros::Subscriber pepperl_sub_;    //倍加福雷达订阅
    base_data::LaserScan_ptr lowcost_scan_;
    sensor_msgs::LaserScan R2000_scan_;
    base_data::LaserScan_ptr average_r2000_scan_;
    base_data::LaserScan_ptr average_lowcost_scan_;

    int size_;     //num_=循环次数/2
    int offset_;  //点数余量
    std::fstream fout_;

    Lidar lowcost_lidar_;
    Lidar pepperl_lidar_;
    std::vector<double> lowcost_ranges_;
    InterCalibraManager inter_calibra_manager_;

    int N=0;
    int average_count_=0;
    int average_count_Max_=50;  //50帧
    double MtoMM=1000.0;
    double MMtoM=0.001;
    bool flag_=true;
    bool flag1_=true;
    bool flag2_=true;
    int max_num_=500;
    int range_num_=0;  
    int angle_num_=0; 
    int min_line_simulation_size_=30;
    double forward_angle_error_=0;
    double initialRotateAngle_=0;


};

}
#endif
