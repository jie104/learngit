//
// Created by lfc on 16-11-2.
//

#ifndef SROS_STANDARD_LASER_MODULE_H
#define SROS_STANDARD_LASER_MODULE_H

#include <atomic>

#include "base_laser_module.h"
#include "core/core.h"

#include "core/device/device_manager.h"
#include "dual_laser_processor.hpp"

//#include "laser_filter/rack_legs_filter.hpp"
#include "laser_filter/rack_filter.hpp"
#include "laser_filter/rack_detector.hpp"
#include "laser_filter/rack_para.hpp"
#include "scan_preprocess/scan_preprocess.hpp"

#include <core/circle_optimizer_set.hpp>

namespace laser {
struct RackPara {
    RackPara() = default;

    // 初始搜索角度范围阈值
    double rack_leg_search_thresh = 10.0f * M_PI / 180.0f;

    // 拖尾点角度范围阈值
    double tailing_noise_angle_thresh = 1.0f * M_PI / 180.0f;

    double rack_backlash_rotate_angle = 2.0*M_PI/180.0;

    // 货架所有leg中心点组成的矩形width，单位m
    double rack_leg_center_width = 0.57f;

    // 货架所有leg中心点组成的矩形length，单位m
    double rack_leg_center_length = 1.02f;

    // 货架leg的直径，单位m
    double rack_leg_diameter = 0.1f;

    bool use_region_filter = true;

    double rack_radius_offset = 0.05f;//货架上最远位置调节量,防止货架顶偏
};


enum DeviceState {
    LIDAROPENED,
    LIDARRUNNING,
    LIDARCLOSED,
};

struct LaserModuleInfo {
    std::shared_ptr<BaseLaserModule> laser_module_;
    sros::device::Device_ptr device_;
    std::string device_name;
    std::shared_ptr<boost::thread> publish_laser_thread;
    DeviceState device_state;
    int device_id = 0;
};
typedef std::shared_ptr<LaserModuleInfo> LaserModuleInfo_Ptr;

class StandardLaserModule : public sros::core::Module {
 public:
    //    enum LidarType {
    //        UTM30LX = 0,
    //        LMS151 = 1,
    //        UST10LX = 2,
    //        OMD30M = 3,
    //        LMS511 = 4,
    //        TIM571 = 5,
    //        GAZEBOSIM = 6,
    //        OMDUHD = 7,
    //        BAGSIM = 8,
    //        NOTUSE = 100,
    //    };git


    StandardLaserModule();

    virtual ~StandardLaserModule();

    virtual void run();

 private:
     void inverseAndBeyondRangePoint(sros::core::LaserScan_ptr scan, const bool is_inverse, 
                                     const float min_range, const float max_range);

     void onUpdateRotateValue(sros::core::base_msg_ptr msg);
     void onDebugCmdMsg(sros::core::base_msg_ptr msg);

     void createRackInfos(std::vector<rack::RackInfo_Ptr> &rack_search_infos);

     bool doOpen(LaserModuleInfo_Ptr laser_module_info);

     void doStart(LaserModuleInfo_Ptr laser_module_info, int laser_id);

     bool doOpen();

     void doStart();

     void doStop();

     void doClose();

     void creatLaserDevice(LaserModuleInfo_Ptr laser_module_info, int id);

     void initializeDualPara(DualLaserPara_Ptr &dual_para);

     void recomputeRackInfo(sros::core::LaserScan_ptr &laser_scan);

     void initializeRackPara(bool &enable_remove_rack_leg, bool &enable_filter_only_load_full, std::vector<std::shared_ptr<rack::BaseRackFilter<sros::core::LaserScan_ptr>>> &rack_points_filters);

     LaserModuleInfo_Ptr first_laser, second_laser;

     BaseLaserModule *laser_module;
     LidarType lidar_type;

     boost::thread *publish_laser_thread;

     sros::device::Device_ptr device_;

     std::shared_ptr<DualLaserProcessor> dual_laser_processor;
     //    std::shared_ptr<rack::RackFilter<sros::core::LaserScan_ptr>> rack_filter;
     std::shared_ptr<rack::RackDetector<sros::core::LaserScan_ptr>> rack_detector;
     bool enable_remove_rack_leg_;
     bool enable_filter_only_load_full_;

     std::atomic<bool> first_detect{true};
     std::vector<rack::RackInfo_Ptr> rack_infos;
     laser::RackPara rack_para;
     //    RackLegsFilter<sros::core::LaserScan_ptr> rack_legs_filter;
     std::vector<std::shared_ptr<rack::BaseRackFilter<sros::core::LaserScan_ptr>>> rack_filters;
     sros::core::LaserScan_ptr second_scan;
     std::mutex read_scan_lock;
     //    std::shared_ptr<rack::RackLegsFilter<sros::core::LaserScan_ptr>> rack_leg_filter;
     const double MM_TO_M = 0.001;
     const double DEG_TO_RAD = M_PI / 180.0;

     float max_range_ = 30;
     float min_range_ = 0.05;

     bool first_lidar_install_up = true;
     bool second_lidar_install_up = true;
     bool respective_publish_scan_;
     bool use_second_lidar_to_location_ = true;
     bool use_first_lidar_to_location_ = true;
    bool laser_correct_ = true;
     bool dual_laser_correct_ = true;
     std::vector<ScanPreprocess> scan_compenstators_;
     slam::tf::TFOperator *tf_base_to_odo_;
     int update_para_incre_ = 0;
};

}  // namespace laser

#endif  // SROS_STANDARD_LASER_MODULE_H
