#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cmath>




class sub_pub
{
public:

    sub_pub(double max_distance,double min_thresh)
        :max_distance_(max_distance),min_thresh_(min_thresh){

        ROS_INFO("订阅话题");
        sub_=nh_.subscribe<sensor_msgs::LaserScan>("/scan",1000,&sub_pub::filter,this);

        // pub_=nh_.advertise<sensor_msgs::LaserScan>("/scan_pub",1000);


    }
    ~sub_pub(){}

    void filter(const sensor_msgs::LaserScan::ConstPtr& scan ){

        ranges = scan->ranges;
        std::vector<int> indexes;
        const int step = 2;
        double cos_increment = cos(scan->angle_increment * (double)step);
        double theta_thresh=sin((double)scan->angle_increment * (double)step)/sin(0.17);//临界值,用于识别断点
        int scan_size = ranges.size() - step;

//        std::vector<int> indexes;
        for (int i = step; i < scan_size; i++) {
            if (ranges[i] == 100 || ranges[i] == 0 || ranges[i] > max_distance_) {
                continue;
            }
            double dist_direction = (ranges[i + step] - ranges[i - step]);            

            for (int k = -step; k < step; ++k) {
                double tmp_direction = (ranges[i + k + 1] - ranges[i + k]);
                if (dist_direction * tmp_direction <= 0) {
                    continue;
                }
            }
            double dist_1 = std::sqrt(ranges[i] * ranges[i] + ranges[i - step] * ranges[i - step] -
                                    2 * ranges[i] * ranges[i - step] * cos_increment);
            double dist_2 = std::sqrt(ranges[i] * ranges[i] + ranges[i + step] * ranges[i + step] -
                                    2 * ranges[i] * ranges[i + step] * cos_increment);
            // double dist_1=ranges[i]*cos((double)scan->angle_increment * (double)step);
            // double dist_2=ranges[i+step]*cos((double)scan->angle_increment * (double)step);

            double range_thresh_1 = ranges[i] * theta_thresh + min_thresh_;
            double range_thresh_2 = ranges[i + step] * theta_thresh + min_thresh_;
            // double range_thresh_1 = ranges[i-step] + min_thresh_;
            // double range_thresh_2 = ranges[i] + min_thresh_;
            // std::cout << "dist_1: " << dist_1 << ", range_thresh_1: "<< range_thresh_1 << std::endl;
            // std::cout << "dist_2: " << dist_2 << ", range_thresh_2: "<< range_thresh_2 << std::endl;
            if(dist_1 > range_thresh_1 && dist_2 > range_thresh_2) {
                for (int j = -step; j <= step; ++j) {
                    indexes.push_back(i + j);
                }
            }


        

        }
<<<<<<< HEAD

        //滤掉雷达噪点
        NoiseFilter(scan,ranges,indexes);

=======
<<<<<<< Updated upstream
=======

        //滤掉雷达噪点
        // Noisefilter(scan,ranges,indexes);

>>>>>>> Stashed changes
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
        for (auto &index:indexes) {
            ranges[index] = 100;
    }

    //再滤一次点
//    filterByDrap(ranges,indexes,step,scan->angle_increment);

    scan_.ranges=ranges;
    scan_.angle_min=scan->angle_min;
    scan_.angle_max=scan->angle_max; 
    scan_.angle_increment=scan->angle_increment;
    scan_.range_max=scan->range_max;
    scan_.range_min=scan->range_min;
    scan_.header.frame_id="scan";
    pub_.publish(scan_);

}

<<<<<<< HEAD
    void NoiseFilter(const sensor_msgs::LaserScan::ConstPtr& scan,std::vector<float>& ranges,std::vector<int>& indexes ){
        int interval=1;
=======
<<<<<<< Updated upstream
=======
    //滤除托尾
    void filterByDrap(std::vector<float>& ranges,std::vector<int>& indexes, const int step,float angle_increment){
        double cos_increment = cos(angle_increment * (double)step);
        double theta_thresh=sin((double)angle_increment * (double)step)/sin(0.17);//临界值,用于识别断点
        int scan_size = ranges.size() - step;

//        std::vector<int> indexes;
        for (int i = step; i < scan_size; i++) {
            if (ranges[i] == 100 || ranges[i] == 0 || ranges[i] > max_distance_) {
                continue;
            }
            double dist_direction = (ranges[i + step] - ranges[i - step]);            

            for (int k = -step; k < step; ++k) {
                double tmp_direction = (ranges[i + k + 1] - ranges[i + k]);
                if (dist_direction * tmp_direction <= 0) {
                    continue;
                }
            }
            double dist_1 = std::sqrt(ranges[i] * ranges[i] + ranges[i - step] * ranges[i - step] -
                                    2 * ranges[i] * ranges[i - step] * cos_increment);
            double dist_2 = std::sqrt(ranges[i] * ranges[i] + ranges[i + step] * ranges[i + step] -
                                    2 * ranges[i] * ranges[i + step] * cos_increment);

            double range_thresh_1 = ranges[i] * theta_thresh + min_thresh_;
            double range_thresh_2 = ranges[i + step] * theta_thresh + min_thresh_;
            if(dist_1 > range_thresh_1 && dist_2 > range_thresh_2) {
                for (int j = -step; j <= step; ++j) {
                    indexes.push_back(i + j);
                }
            }


        

        }


        for (auto &index:indexes) {
            ranges[index] = 100;
        }

    }

    void Noisefilter(const sensor_msgs::LaserScan::ConstPtr& scan,std::vector<float>& ranges,std::vector<int>& indexes ){
        int interval=3;
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
        ranges[0]=100;
        for (int k=interval;k<=ranges.size()-interval-2;k++){
            int num1=0;
            for (int h=-interval;h<=interval-2;h++){
                double theta_1=scan->angle_min+scan->angle_increment*(k+h);
                double theta_2=scan->angle_min+scan->angle_increment*(k+h+1);
                double theta_3=scan->angle_min+scan->angle_increment*(k+h+2);

                double angle_1=Cal_angle(Point(ranges[k+h],theta_1),Point(ranges[k+h+1],theta_2));
                // std::cout << "angle_1: " << angle_1 << std::endl;
                double angle_2=Cal_angle(Point(ranges[k+h+1],theta_2),Point(ranges[k+h+2],theta_3));
                double delta_angle=Delta_angle(angle_1,angle_2);
                // std::cout << "delta_angle: " << delta_angle << std::endl;
                if (h!=-1 && h!=-2 && h!=0){
                    if (delta_angle <= 1.57){
                        num1+=1;
                    }else{
                        continue;
                    }
                }
                else if(h==-1 )
                {
                    if (delta_angle <1){
                        num1+=1;
                        // std::cout << "h==-1: " << num1 << std::endl;
                    }else{
                        continue;
                    }
                }else{
                    // std::cout << "delta_angle: " << delta_angle << std::endl;
                    if (delta_angle > 1.3){
                        num1+=1;
                        // std::cout << "h==0 or h==-2" << num1 << std::endl;
                    }else{
                        continue;
                    }
                
                }     
            } 
            
            // std::cout << "num1: " << num1 << std::endl;
            if (num1==2*interval-1){
                std::cout << "k:" << k << std::endl;
                for (int j = -interval; j <= interval; ++j) {
                    indexes.push_back(k + j);
                }
                // indexes.push_back(k);
            }

            }

    }



    struct point{
        double x;
        double y;
    };


    point Point(double l,double theta){
        point point1; 
        point1.x=l*cos(theta);
        point1.y=l*sin(theta);
        return point1;
    }

    double Cal_angle(point point1,point point2){
        double angle=atan2(point1.y-point2.y,point1.x-point2.x);
        // double mod=M_PI;
        // angle=modf(angle,&mod);
        return angle;
    }

    double Delta_angle(double angle_1,double angle_2){
        Normal_angle(angle_1);
        Normal_angle(angle_2);
        double angle=fabs(angle_1-angle_2);
        if (angle >M_PI){
            angle=2*M_PI-angle;
        }
        // while (angle >=M_PI){
        //     // std::cout << "angle: " << angle << std::endl;
        //     angle=angle-M_PI;
        //     // std::cout << "mode_angle: " << angle << std::endl;
        // }
        return angle;
        
    }
    //正则化
    void Normal_angle(double angle){
        while (angle <0 ){
            angle+=2*M_PI;
        }
        while (angle >2*M_PI){
            angle-=2*M_PI;
        }

    }
<<<<<<< HEAD
=======
>>>>>>> Stashed changes
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4

private:
    double max_distance_;    //TODO
    double min_thresh_;  //TODO
    std::vector<float> ranges;  //测距距离

    //初始节点句柄
    ros::NodeHandle nh_;

    //订阅
    ros::Subscriber sub_;

    //发布
    ros::Publisher pub_=nh_.advertise<sensor_msgs::LaserScan>("/scan_pub",1000);

    sensor_msgs::LaserScan scan_;
};







int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"lidar");
    sub_pub Sub_pub(3,0.020);

    
    ros::spin();
    return 0;
}
