//
// Created by zxj on 2022/12/8.
//

#include <iostream>
#include <glog/logging.h>
#include <vector>
#include <string>
#include "base_data_type.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>



class tf_pcl
{
public:
//    tf_pcl()=default;
    tf_pcl(Eigen::Vector3f src_point,Eigen::Vector3f tar_point):
            src_point_(src_point),tar_point_(tar_point){
//        src_point_=Eigen::Vector3f(1.013,0.425,0.872);
//        tar_point_=Eigen::Vector3f(0.926,0.002,-0.0196);
        sub_=nh_.subscribe<sensor_msgs::LaserScan>("/oradar_node_1/scan",1000,&tf_pcl::tfPcl, this);
        pub_debug_icp_pcl_=nh_.advertise<sensor_msgs::PointCloud>("pub_debug_icp_pcl",1000);

    };
    ~tf_pcl(){}

//    Eigen::Affine3d aff;

    Eigen::Affine2f CoordinateTf1(const Eigen::Vector3f &src_point){
        Eigen::Affine2f T =
                Eigen::Rotation2Df(src_point[2])*Eigen::Translation2f(src_point[0], src_point[1]) ;
        return T;
    }

    Eigen::Isometry2f CoordinateTf(const Eigen::Vector3f& src_point){
        Eigen::Matrix2f R;
        float theta=src_point[2];
        R << cos(theta),-sin(theta),
                sin(theta),cos(theta);
        Eigen::Vector2f t=Eigen::Vector2f(src_point[0],src_point[1]);
//        LOG(INFO) << "R: " << R << "determina " << R.determinant();
//        LOG(INFO) << "t: " << t;
        Eigen::Isometry2f T=Eigen::Isometry2f::Identity();
        T.rotate(R);
        T.pretranslate(t);

        return T;
    }


    double calTheta(const sensor_msgs::LaserScan::ConstPtr& scan,int i){
        double angle_min=scan->angle_min;
        double angle_increment=scan->angle_increment;
        double angle=angle_min+i*angle_increment;
        return angle;
    }

    std::vector<Eigen::Vector2f> convertToRetangualrCoordinate(std::vector<float>& ranges,const sensor_msgs::LaserScan::ConstPtr& scan){
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

    Eigen::Isometry2f EuclTf(const Eigen::Vector3f& src_point,const Eigen::Vector3f& tar_point){
        Eigen::Matrix2f R;
        float theta=tar_point[2]-src_point[2];
//        LOG(INFO) << "theta: " << theta*180/M_PI;
        R << cos(theta),-sin(theta),
                sin(theta),cos(theta);
        Eigen::Vector2f t=tar_point.head(2)-R*src_point.head(2);
        Eigen::Isometry2f T=Eigen::Isometry2f::Identity();
        T.rotate(R);
        T.pretranslate(t);

        return T;

    }

    void tfPcl(const sensor_msgs::LaserScan::ConstPtr& msg_ptr){
        std::vector<float> res_scan_ranges=msg_ptr->ranges;
        std::vector<Eigen::Vector2f> res_points;

        res_points=convertToRetangualrCoordinate(res_scan_ranges,msg_ptr);
//        LOG(INFO) << "size: " << tar_points.size();
        sensor_msgs::PointCloud temp_msg;
        temp_msg.header.frame_id="scan";

        Eigen::Isometry2f coordinate_tf_src=CoordinateTf(src_point_); // Tw2
        Eigen::Isometry2f coordinate_tf_tar=CoordinateTf(tar_point_); // Tw1
        Eigen::Isometry2f eucl_tf= EuclTf(src_point_,tar_point_);
//        LOG(INFO) << "T: " << T.matrix();
        for (int i=0;i< res_points.size();i++){
            Eigen::Vector2f temp_point=res_points[i];
            temp_point=coordinate_tf_tar.inverse()*(coordinate_tf_src*temp_point);
//            temp_point=eucl_tf*temp_point;
//            LOG(INFO) << "temp_point: " << temp_point;
            temp_msg.points.emplace_back();
            temp_msg.points.back().x=temp_point[0];
            temp_msg.points.back().y=temp_point[1];
        }
        pub_debug_icp_pcl_.publish(temp_msg);
    }

    void pubCompensateDebugPcl(const std::vector<Eigen::Vector2f>& points){
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

private:
    Eigen::Vector3f src_point_;
    Eigen::Vector3f tar_point_;
    ros::NodeHandle nh_;

    ros::Subscriber sub_;
    ros::Publisher pub_debug_icp_pcl_;



};

int main(int argc,char* argv[]){
    ros::init(argc,argv,"TF");
    std::cout << "begin to tf" << std::endl;
//    tf_pcl tf_sub_pub({1.013,-0.425,-0.872},{0.926,0.002,-0.0196});
    tf_pcl tf_sub_pub({1.013,-0.425,0.272},{0.926,0.002,-0.0196});
//    tf_pcl tf_sub_pub({3.0,3.0,45.0*M_PI/180.0 },{0.0,0.0,0.0});

    ros::spin();

}
