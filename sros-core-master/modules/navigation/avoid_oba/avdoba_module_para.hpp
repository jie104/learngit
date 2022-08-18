//
// Created by lfc on 18-12-5.
//

#ifndef PROJECT_COLLIDE_MODULE_PARA_HPP
#define PROJECT_COLLIDE_MODULE_PARA_HPP

#include <memory>

namespace avoidoba {
struct AvdobaModulePara {
    bool enable_auto_enhance_oba_width = true;
    int max_stop_collide_value = 1000;//触发停车条件碰撞最大值
    int min_stop_collide_value = 901;//触发停车条件碰撞最小值
    int max_slow_collide_value = 900;//触发减速条件碰撞最大值
    int min_slow_collide_value = 1;//触发减速条件碰撞最小值
    int min_collide_value = 0;//默认值

    int max_collide_region_count = 40;//停止区粒子最大个数
    int max_slow_region_count = 40;//减速区粒子最大个数
    double slow_width_offset = 0.5;//减速宽度增量
    double stop_width_offset = 0.01;//停止区宽度增量
    double stop_length_forward_offset = 0.3;//前进时,前侧余量;后退时,后侧余量
    double stop_length_backward_offset = 0;//前进时,后侧余量;后退时,前侧余量
    double station_stop_offset = 0.05;//当到达终点时,如果前方有5cm以内的障碍,则停车
    double deviation_dist_thresh = 0.5;
    double deviation_large_head_stop_offset = 1.0;
    double deviation_large_back_stop_offset = 0.5;
    double deviation_large_width_offset = 0.5;

    double car_body_width = 0.6;//车身宽度
    double car_body_length = 0.8;//车身长度
    double original_car_body_width = 0.6;//原始车身宽度
    double original_car_body_length = 0.8;//原始车身长度
    double rack_body_width = 0.8;//货架宽度
    double rack_body_length = 1.2;//货架长度

    double laser_coord_x = 0.28;//雷达安装位置
    double laser_coord_y = 0;
    double laser_coord_yaw = 0;

    double origin_forward_stop_distance = 1.0;//前进时,原始停车距离
    double origin_forward_slow_distance = 1.5;//前进时,原始减速距离
    double origin_backward_stop_distance = 1.0;//后退时,原始停车距离,减速距离依据停车距离按比例生成.

    double forward_stop_distance = 1.0;//前进时,停车距离，会基于速度自适应改变大小
    double forward_slow_distance = 1.5;//前进时,减速距离，会基于速度自适应改变大小
    double backward_stop_distance = 1.0;//后退时,停车距离，会基于速度自适应改变大小

    const double sample_step = 0.02;//前进时,设置步长为2cm,其他等价该步长.
    double backward_step = 0.02;
    double oba_laser_range_max = 5.0;
    std::string vehicle_type = "oasis";
    int64_t delta_stop_time_stamp_in_us = 5e5;

    double manual_stop_distance = 0.2;
    double manual_stop_rotate = 0.5;
    double manual_slow_distance = 0.2;
    double manual_slow_rotate = 0.5;
    bool enable_auto_stop_dist_by_vel = true;
};

typedef std::shared_ptr<AvdobaModulePara> AvdobaModulePara_Ptr;
}


#endif //PROJECT_COLLIDE_MODULE_PARA_HPP
