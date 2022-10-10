//
// Created by zxj on 2022/9/16.
//
// #define BOOST_THREAD_VERSION 5
// #include <boost/thread.hpp>
// #include <exception>
#include <ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include <map>
#include <cmath>
#include <vector>
#include <glog/logging.h>
#include <string>
#include <memory>

#ifndef RANGE_AVE_ERROR_
#define RANGE_AVE_ERROR_

// struct LowCost{
//     double lowcost_angle_min;
//     double lowcost_forward_angle;    //低成本雷达正前方
//     std::string lowcost_Topic;

// };

class RangeAveError
{
public:
    typedef sensor_msgs::LaserScan::ConstPtr LaserScan;

    RangeAveError(){
        ave_error_ranges_.resize(3600,0.f);
    //从lidar_error_inspect.launch中读取，当该文件中不存在该项时，取后面的默认值，否则以该文件中的值为准
        nh_.param<std::string>("/lowcost_lidar/LowCostTopic",lowcost_topic_,"/ORADAR/orad_scan");
        nh_.param<int>("/lowcost_lidar/Max_num_",max_num_,500);
        // LowCost_sub_=nh_.subscribe("LowCostTopic",30,&RangeAveError::LowCostcallback,this);
        LowCost_sub_=nh_.subscribe(lowcost_topic_,1000,&RangeAveError::LowCostcallback,this);

        R2000_sub_=nh_.subscribe("/r2000_node/scan",1000,&RangeAveError::R2000callback,this);


    }

    void R2000callback(const LaserScan& r2000_scan){
        r2000_forward_angle_=(r2000_scan->angle_max+r2000_scan->angle_min)/2;

        pepperl_angle_=calAngle(r2000_scan,60);
        // LOG(INFO) << "pepperl_angle_: " << pepperl_angle_;


        
        interpolation_container_[lowcost_forward_angle_]=r2000_scan->ranges[r2000_forward_angle_];
        //计算插值
        calInter(r2000_scan,1); 
        calInter(r2000_scan,-1);
        //计算误差距离
        calErrorRanges(interpolation_container_);
        double error_ranges_sum=0;
        double theta=fabs(pepperl_angle_-lowcost_angle_);
        if (Num_<max_num_){
            mean_theta_+=theta;
            mean_lowcost_angle_+=lowcost_angle_;
            mean_pepperl_angle_+=pepperl_angle_;

            // LOG(INFO) << "Num_: " << Num_;
            for (int i=0;i< error_ranges_.size();i++){
                ave_error_ranges_[i]+=error_ranges_[i];
            }
        }else{
            for (int j=0;j< error_ranges_.size();j++){
                ave_error_ranges_[j]/=max_num_;
            }
            for (int k=0;k<error_ranges_.size() ;k++){
                error_ranges_sum+=ave_error_ranges_[k];
            }
            int error_ranges_num=error_ranges_.size();
            mean_theta_=mean_theta_/max_num_;
            mean_lowcost_angle_=mean_lowcost_angle_/max_num_;
            mean_lowcost_angle_=mean_lowcost_angle_/max_num_;
            mean_pepperl_angle_=mean_pepperl_angle_/max_num_;
            LOG(INFO) << "the number of cycles is " << max_num_;
            LOG(INFO) << "the forward angle error is " << mean_theta_*180/M_PI << " 度";
            LOG(INFO) << "the lowcost angle error is " << mean_lowcost_angle_*180/M_PI << " 度";
            LOG(INFO) << "the pepperl angle error is " << mean_pepperl_angle_*180/M_PI << " 度";

            LOG(INFO) << "lowcost_total_points: " << lowcost_ranges_.size();
            LOG(INFO) << "lowcost_topic: " << lowcost_topic_;
            LOG(INFO) << "max_num_: " << max_num_ <<  " ,sum: " << error_ranges_sum << " m";
            LOG(INFO) << "error_ranges_num: " << error_ranges_num << ",the average range error of lidar: "
                 << (error_ranges_sum/error_ranges_num)*1000 << " mm";
            std::cout << "===============================================================================" << std::endl;
            ros::Duration du(2);//持续2秒钟,参数是double类型的，以秒为单位
            du.sleep();//按照指定的持续时间休眠
            ave_error_ranges_.clear();
            ave_error_ranges_.resize(3600,0.f);
            Num_=-1;
            mean_theta_=0;
        }
        // LOG(INFO) << "Num_:" << Num_;
        Num_+=1;
        error_ranges_.clear();

        // error_ranges_.reserve(2*size_-1);
    

    }

    void LowCostcallback(const LaserScan& lowcost_scan){
        if (flag_){
            rotate_angle_=lowcost_scan->angle_increment;
            flag_=false;
        }
        lowcost_forward_angle_=(lowcost_scan->angle_max+lowcost_scan->angle_min)/2;    //默认雷达正前方在雷达物理前方计算
        size_=lowcost_scan->ranges.size()/2;  

        lowcost_angle_incre_=lowcost_scan->angle_increment;
        lowcost_angle_min_=lowcost_scan->angle_min;
        lowcost_ranges_=lowcost_scan->ranges;
        // for (int i=0;i < lowcost_ranges_.size();i++){
        //     LOG(INFO) << "lowcost_ranges_: " << i << "," << lowcost_ranges_[i];
        // }


        lowcost_angle_=calAngle(lowcost_scan,thresh_);
        LOG(INFO) << "lowcost_angle_: " << lowcost_angle_;


    
    }
private:  

    struct Point{
        Point():x(0),y(0) {}
        Point(double a,double b):x(a),y(b) {}

        Point operator -(Point P){
            return Point(x-P.x,y-P.y);
        }
        Point operator +(Point P){
            return Point(x+P.x,y+P.y);
        }

        double operator*(Point P){
            return x*P.x+y*P.y;
        }

        double Mod(){
            if (x==0 && y==0){
                LOG(INFO) << "the Mod is zero!!!  it is wrong";
                return 0;
            }
            return sqrt(x*x+y*y);
        }

        double x;
        double y;
    };

    //将极坐标转化为直角坐标系
    Point Polar2Rect(double& range,double& angle){
        double x=range*cos(angle);
        double y=range*sin(angle);
        return Point(x,y);
    }

    //计算向量和
    double calAngle(const LaserScan& scan,const int& thresh){
        double angle_min=scan->angle_min;
        double angle_increment=scan->angle_increment;
        auto& ranges=scan->ranges;
        double ranges_size=scan->ranges.size();
        int middle_num=ranges_size/2;
        // Point VectorSum;

        // double range_1=ranges[middle_num-thresh];
        // double angle_1=angle_min+angle_increment*(float)(middle_num-thresh);

        std::vector<std::shared_ptr<Point>> points; 
        points.reserve(2*thresh);
        for (int i=middle_num-thresh;i<middle_num+thresh;++i){
            double range_2=ranges[i];
            double angle_2=angle_min+angle_increment*(float)(i);
            // LOG(INFO) << "RANGE_2: " << range_2;
            // LOG(INFO) << "ANGLE_2: " << angle_2;
            // LOG(INFO) << ".............";
            points.emplace_back(new Point); //此处需对共享指针初始化，否则其将是空指针
            points.back()->x=range_2*cos(angle_2);
            points.back()->y=range_2*sin(angle_2);
            // LOG(INFO) << ".........";

        }
    

        double theta= calAngle(points);
        return theta;
    }

    //计算倾斜角
    double calAngle(std::vector<std::shared_ptr<Point>>& points) {
        double sum_1=0,sum_2=0;
        double mean_x=0,mean_y=0;
        int points_size=points.size();

        for (int i=0;i<points_size;i++){
            sum_1+=points[i]->x;
            sum_2+=points[i]->y;
        }
        mean_x=sum_1/points_size;
        mean_y=sum_2/points_size;

        sum_1=0;
        sum_2=0;
        for (int j=0;j<points_size;j++){
            sum_1+=(points[j]->x-mean_x)*(points[j]->y-mean_y);
            sum_2+=(points[j]->x-mean_x)*(points[j]->x-mean_x);
        }

        // LOG(INFO) << "SUM_1: " << sum_1;
        // LOG(INFO) << "SUM_2: " << sum_2;

        double theta=atan2(sum_1,sum_2);
        // LOG(INFO) << "theta: " << theta;
        return theta;
        // double tan_theta=sum_1/sum_2;


    }


    //寻找该插值点云在倍加福坐标系下的相邻两个点云
    void findTwoPointCloudId(const LaserScan& scan,double inter_angle){
        double angle_min=scan->angle_min;
        double angle_incre=scan->angle_increment;
        int size=scan->ranges.size();
        // LOG(INFO) << "size: " << size;
        for (int i=0;i<size-1;i++)
        {
            if (inter_angle>= angle_min+i*angle_incre && inter_angle <= angle_min+(i+1)*angle_incre){
                id_=i;
                // LOG(INFO) << "id: " << id;   //todo
                // return;
                break;
            }
        }
        // LOG(INFO) << "can't find left_id and right_id";

    }

    void calInter(const LaserScan& scan,int weight){ //weight取1或-1，1表示往雷达正前方左侧插值，-1代表右侧
        auto& ranges=scan->ranges;
        double angle_min=scan->angle_min;
        double angle_incre=scan->angle_increment;
        double range=ranges[ranges.size()/2];
        for (int i=1;i<size_;i++){
            double inter_angle=r2000_forward_angle_+i*lowcost_angle_incre_*weight; //在倍加福雷达极坐标系下考虑
            // LOG(INFO) << "inter_angle: " << inter_angle;
            // LOG(INFO) << "i: " << i;
            double x_0=range*cos(inter_angle);
            double y_0=range*sin(inter_angle);
            range=sqrt(x_0*x_0+y_0*y_0);

            findTwoPointCloudId(scan,inter_angle);  //找到id
            //计算相邻右侧点云直角坐标
            double right_angle=angle_min+id_*angle_incre;
            double x_1=ranges[id_]*cos(right_angle);
            double y_1=ranges[id_]*sin(right_angle);
            //计算相邻左侧点云直角坐标
            double left_angle=angle_min+(id_+1)*angle_incre;
            double x_2=ranges[id_+1]*cos(left_angle);
            double y_2=ranges[id_+1]*sin(left_angle);
            // LOG(INFO) <<"(x_1,y_1): " <<  "(" << x_1 << "," << y_1 <<  ")" << std::endl;
            // LOG(INFO) << "(x_2,y_2): " << "(" << x_2 << "," << y_2 <<  ")" << std::endl;

            //计算顺时针方向weight为-1，逆时针方向weight为1
            double X_0=x_0*cos(weight*rotate_angle_)-y_0*sin(weight*rotate_angle_);
            double Y_0=x_0*sin(weight*rotate_angle_)+y_0*cos(weight*rotate_angle_);

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
            // LOG(INFO) << "(X_0,Y_0): " << "(" << X_0 << "," << Y_0 <<  ")" << std::endl;
            double result=(x_1-x_2)*(y_1-Y_0)-(x_1-X_0)*(y_1-Y_0);
            // if (fabs(result) >0.01){
            //     LOG(INFO) << "i: " << "the result is wrong! " << "result: " << result ;
            //     // ++abandon_num_;
            //     // LOG(INFO) << "abandon_num_: " << abandon_num_;
            // }
            // else{
            double inter_range=sqrt(X_0*X_0+Y_0*Y_0);
            interpolation_container_[lowcost_forward_angle_+i*lowcost_angle_incre_*weight]=inter_range;    //在低成本雷达极坐标系下考虑
            // }
            // LOG(INFO) << "abandon_num_: " << abandon_num_;

        }
        // LOG(INFO) << "the size of interpolation_container_: " << interpolation_container_.size();


    }

    void calErrorRanges(std::map<double,double> inter_ranges){
        int j=0;
        double inter_angle;
        // boost::mutex mu;    //声明互斥量对象
        // try{
        //     mu.lock();
        //     for (int i=0;i<2*size_-1;i++){
        //         // LOG(INFO) << "error_ranges_.size(): " << error_ranges_.size();
        //         inter_angle=lowcost_forward_angle_-(size_-1-i)*lowcost_angle_incre_;

        //         // LOG(INFO) << "inter_angle: " << inter_angle;
        //         j=(int)(Rad2Angle(inter_angle-lowcost_angle_min_)/Rad2Angle(lowcost_angle_incre_));
        //         // LOG(INFO) << "the id of test_ranges: " << j;
        //         // LOG(INFO) << "inter_ranges[inter_angle]: " << inter_ranges[inter_angle];
        //         // LOG(INFO) << "lowcost_ranges_[j]: " << lowcost_ranges_[j];
        //         error_ranges_.push_back(fabs(lowcost_ranges_[j]-inter_ranges[inter_angle]));
        //         // error_ranges_[i]=(lowcost_ranges_[j]-inter_ranges[inter_angle]);
        //         // LOG(INFO) << "error_ranges[" << i << "]: " << error_ranges_[i] ;
        //         mu.lock();
        // }

        // }catch(std::exception& e){
        //     std::cout << e.what() << std::endl; 
        //     mu.unlock();
        // }

        for (int i=0;i<2*size_-1;i++){
            // LOG(INFO) << "error_ranges_.size(): " << error_ranges_.size();
            inter_angle=lowcost_forward_angle_-(size_-1-i)*lowcost_angle_incre_;    //默认低成本雷达正前方角度为0度

            // LOG(INFO) << "inter_angle: " << inter_angle;
            // LOG(INFO) << "inter_angle: " << inter_angle;
            j=(inter_angle-lowcost_angle_min_)/lowcost_angle_incre_;   //todo
            // LOG(INFO) << "the id of test_ranges: " << j;
            // LOG(INFO) << "inter_ranges[inter_angle]: " << inter_ranges[inter_angle];
            // LOG(INFO) << "lowcost_ranges_: " << lowcost_ranges_[j];

            double error_range=fabs(lowcost_ranges_[j]-inter_ranges[inter_angle]);
            if (error_range < 0.05){    //若误差距离小于5cm，则保存，否则丢弃
             //**********************由于点云存在跳动，可能会让error_ranges_每次的个数不一样，从而产生问题!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                error_ranges_.push_back(error_range);      
                // std::cout  << std::endl;
                // LOG(INFO) << "j: " << j << "lowcost_ranges_: " << lowcost_ranges_[j];
                // LOG(INFO) << "inter_ranges: " << inter_ranges[inter_angle];
                // std::cout << std::endl;
                // LOG(INFO) << "error_range(m): " << error_range  << " the id of lowcost lidar: " << j ;

            }else{
                // std::cout  << std::endl;
                // LOG(INFO) << "j: " << j << "lowcost_ranges_: " << lowcost_ranges_[j];
                // LOG(INFO) << "inter_ranges: " << inter_ranges[inter_angle];
                // std::cout << std::endl;
                // LOG(INFO) << "error_range(m): " << error_range  << "the id of lowcost lidar: " << j ;
            }
            // error_ranges_[i]=(lowcost_ranges_[j]-inter_ranges[inter_angle]);
            // LOG(INFO) << "error_ranges[" << i << "]: " << error_ranges_[i] ;
        }

        
               
    }

    //将角度转化为弧度
    double Rad2Angle(double rad){
        rad=rad*180/M_PI;
        return rad;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber R2000_sub_;  //R2000订阅
    ros::Subscriber LowCost_sub_;    //低成本雷达订阅

    double rotate_angle_;    //旋转单位角度
    int size_;     //num_=循环次数/2
    bool flag_=true;    
    std::map<double,double> interpolation_container_;    //<角度，距离>,用于存放倍加福雷达插值后的数据,角度与低成本雷达对齐
    double r2000_forward_angle_;      //倍加福雷达正前方
    
    //插值点云的左侧实际相邻点云序号
    int id_;   

    //低成本雷达
    double lowcost_angle_incre_;
    double lowcost_angle_min_;
    double lowcost_angle_max_;
    double lowcost_forward_angle_;    //低成本雷达正前方
    std::vector<float> lowcost_ranges_;
    std::string lowcost_topic_;


    std::vector<double> error_ranges_;  //误差值
    std::vector<double> ave_error_ranges_;   //平均距离误差

    int Num_=0;   //循环次数，设定每500次求一次平均值
    int max_num_;   //默认设置500次
    // int32_t error_ranges_min_num_=INT32_MAX;
    const int thresh_=40;
    double lowcost_angle_;
    double pepperl_angle_;

    double mean_theta_=0;
    double mean_lowcost_angle_=0;
    double mean_pepperl_angle_=0;
    

};


#endif
