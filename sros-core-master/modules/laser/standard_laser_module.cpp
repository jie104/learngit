//
// Created by lfc on 16-11-2.
//

#include <core/src.h>
#include "core/msg/SlamStateMsg.h"
#include "core/settings.h"
#include "standard_laser_module.h"

#include <core/rack/rack_operator_instance.hpp>
#include "GAZEBOLASER/gazebo_laser.h"
#include "LMS1xx/lms1xx_module.h"
#include "LMS5xx/lms5xx_module.h"
#include "MICRO_SCAN3/micro_scan3_module.h"
#include "OMD30M/omd30m_module.h"
#include "SIM_LASER/simlaser_module.h"
#include "TIM5xx/tim5xx_module.h"
#include "UST10LX/ust10lx_module.h"
#include "UTM30LX/utm30lx_module.h"
#include "laser_module_factory.h"
#include "modules/laser/standard_lidar_protocol/tcp_scan_data_receiver.hpp"
#include "notuse_laser_module.hpp"
#include "core/msg/command_msg.hpp"


using namespace sros::device;

namespace laser {

    StandardLaserModule::StandardLaserModule()
        : Module("laser"),
          respective_publish_scan_(false),
          laser_module(nullptr),
          publish_laser_thread(nullptr),
          tf_base_to_odo_(nullptr),
          lidar_type(UTM30LX)
    {
        slam::tf::FrameToFrame base_to_odo_frame;
        base_to_odo_frame.parent_frame = "odom";
        base_to_odo_frame.child_frame = "base_link";
        tf_base_to_odo_ = new slam::tf::TFOperator(base_to_odo_frame);
        scan_compenstators_.emplace_back(ScanPreprocess(tf_base_to_odo_));
        scan_compenstators_.emplace_back(ScanPreprocess(tf_base_to_odo_));
        scan_compenstators_.emplace_back(ScanPreprocess(tf_base_to_odo_));
    }

void StandardLaserModule::run() {
    subscribeTopic("TIMER_50MS", CALLBACK(&StandardLaserModule::onUpdateRotateValue));
    subscribeTopic("DEBUG_CMD", CALLBACK(&StandardLaserModule::onDebugCmdMsg));

    // 本地调试就不开启雷达模块了
    auto &s = sros::core::Settings::getInstance();
    auto enable_sros_native_debug = (s.getValue<std::string>("debug.enable_sros_native_debug", "False") == "True");
    if (enable_sros_native_debug) {
        stop();
        return;
    }

    respective_publish_scan_ = s.getValue<bool>("laser.respective_publish_scan", false);
    double first_center_x = s.getValue<double>("laser.first_laser_center_x", 270) * MM_TO_M;
    double first_center_y = s.getValue<double>("laser.first_laser_center_y", 170) * MM_TO_M;
    double first_center_yaw = s.getValue<double>("laser.first_laser_center_yaw", 45) * DEG_TO_RAD;

    double second_center_x = s.getValue<double>("laser.second_laser_center_x", -270) * MM_TO_M;
    double second_center_y = s.getValue<double>("laser.second_laser_center_y", -170) * MM_TO_M;
    double second_center_yaw = s.getValue<double>("laser.second_laser_center_yaw", -135) * DEG_TO_RAD;

    double laser_coordx = s.getValue<double>("posefilter.laser_coordx", 0.29);
    double laser_coordy = s.getValue<double>("posefilter.laser_coordy", 0.0);
    double laser_coordyaw = s.getValue<double>("posefilter.laser_coordyaw", 0.0);
    laser_correct_ = s.getValue<bool>("laser.compentate",true);
    dual_laser_correct_ = s.getValue<bool>("laser.dual_compentate", true);
    int correct_type = s.getValue<int>("laser.distortion_correct_type", 0);
    float angle_max = s.getValue<float>("slam.laser_angle_max",M_PI);
    float angle_min = s.getValue<float>("slam.laser_angle_min", -M_PI);
    float first_angle_max = s.getValue<float>("laser.first_laser_angle_max", M_PI);
    float first_angle_min = s.getValue<float>("laser.first_laser_angle_min", -M_PI);
    float second_angle_max = s.getValue<float>("laser.second_laser_angle_max", M_PI);
    float second_angle_min = s.getValue<float>("laser.second_laser_angle_min", -M_PI);

    max_range_ = s.getValue<float>("slam.map_laser_max_dist",30);
    min_range_ = s.getValue<float>("slam.map_laser_min_dist", 0.05);

    for (auto &compenstator : scan_compenstators_)
        compenstator.setDistortionCorrectType(correct_type);
    scan_compenstators_[0].setLaserTF(laser_coordx, laser_coordy, laser_coordyaw);
    scan_compenstators_[1].setLaserTF(first_center_x, first_center_y, first_center_yaw);
    scan_compenstators_[2].setLaserTF(second_center_x, second_center_y, second_center_yaw);
    scan_compenstators_[0].setLaserAngle(angle_max,angle_min);
    scan_compenstators_[1].setLaserAngle(first_angle_max, first_angle_min);
    scan_compenstators_[2].setLaserAngle(second_angle_max, second_angle_min);

    if (laser_correct_)
    {
        LOG(INFO) << "Open laser correct Model,correct type is " << correct_type;
    }
    
    if (dual_laser_correct_)
    {
        LOG(INFO) << "Open dual laser correct Model";
    }

    initializeRackPara(enable_remove_rack_leg_, enable_filter_only_load_full_, rack_filters);
    if (rack_filters.empty()) {
        LOG(ERROR) << "err to initialize rack filter! will return!";
    }

    std::string first_device_address = s.getValue<std::string>("laser.first_lidar_ip_address", "192.168.23.100");
    std::string second_device_address = s.getValue<std::string>("laser.second_lidar_ip_address", "192.168.23.101");

    int first_lidar_port = s.getValue<int>("laser.first_lidar_ip_port", 6060);
    int second_lidar_port = s.getValue<int>("laser.second_lidar_ip_port", 6061);
    first_lidar_install_up = !(s.getValue<std::string>("laser.first_lidar_install_directioin", "UP") == "DOWN");
    second_lidar_install_up = !(s.getValue<std::string>("laser.second_lidar_install_directioin", "UP") == "DOWN");


    auto lidar_type = (LidarType)sros::core::Settings::getInstance().getValue<int>("slam.lidar_type", 0);
    if (lidar_type == DUAL_OMD || lidar_type == DUAL_OMD_PAVO || lidar_type == DUAL_PAVO || lidar_type == DUAL_S300 ||
        lidar_type == DUAL_SCAN3 || lidar_type == DUAL_TIM571 || lidar_type == DUAL_ALL) {
        use_second_lidar_to_location_ = !(s.getValue<std::string>("slam.use_second_lidar_to_location", "True") == "False");
        use_first_lidar_to_location_ = !(s.getValue<std::string>("slam.use_first_lidar_to_location", "True") == "False");
        DualLaserPara_Ptr dual_para(new DualLaserPara);
        initializeDualPara(dual_para);
        dual_laser_processor.reset(new DualLaserProcessor(dual_para));
        LidarType first_type, second_type;

        if (lidar_type == DUAL_OMD) {
            LOG(INFO) << convertToStr(DUAL_OMD);
            first_type = OMD30M;
            second_type = OMD30M;
        } else if (lidar_type == DUAL_OMD_PAVO) {
            LOG(INFO) << convertToStr(DUAL_OMD_PAVO);
            first_type = OMD30M;
            second_type = SIMINICSPAVO;
        } else if (lidar_type == DUAL_PAVO) {
            LOG(INFO) << convertToStr(DUAL_PAVO);
            first_type = SIMINICSPAVO;
            second_type = SIMINICSPAVO;
        } else if (lidar_type == DUAL_S300) {
            LOG(INFO) << convertToStr(DUAL_S300);
            first_type = S300;
            second_type = S300;
        } else if (lidar_type == DUAL_SCAN3 || lidar_type == DUAL_NANO) {
            LOG(INFO) << convertToStr(DUAL_SCAN3);
            first_type = MICRO_SCAN3;
            second_type = MICRO_SCAN3;
        } else if (lidar_type == DUAL_TIM571) {
            LOG(INFO) << convertToStr(DUAL_TIM571);
            first_type = TIM571;
            second_type = TIM571;
        }else if (lidar_type == DUAL_ALL) {
            first_type = (LidarType)sros::core::Settings::getInstance().getValue<int>("laser.first_lidar_type", 0);
            second_type = (LidarType)sros::core::Settings::getInstance().getValue<int>("laser.second_lidar_type", 0);
        }
        LOG(INFO) << "will creat Dual R2000 lidar!";

        first_laser.reset(new LaserModuleInfo);
        first_laser->laser_module_ = LaserModuleFactory::getLaserModule(first_type, first_device_address,
                                                                        first_laser->device_name, first_lidar_port);
        first_laser->device_id = 1;
        // 雷达名称暂时固定，后续待重构
        first_laser->device_name = sros::device::DEVICE_LIDAR_1;

        second_laser.reset(new LaserModuleInfo);
        second_laser->laser_module_ = LaserModuleFactory::getLaserModule(second_type, second_device_address,
                                                                         second_laser->device_name, second_lidar_port);
        second_laser->device_id = 2;
        // 雷达名称暂时固定，后续待重构
        second_laser->device_name = sros::device::DEVICE_LIDAR_2;

    } else {
        LOG(INFO) << "will creat single lidar!";
        first_laser.reset(new LaserModuleInfo);
        first_laser->device_name = sros::device::DEVICE_LIDAR;
        first_laser->laser_module_ = LaserModuleFactory::getLaserModule(lidar_type, first_device_address,
                                                                        first_laser->device_name, first_lidar_port);
        first_laser->device_id = 0;
        // 雷达名称暂时固定，后续待重构
        first_laser->device_name = sros::device::DEVICE_LIDAR_1;
    }

    if (first_laser) {
        LOG(INFO) << "will creat frist laser device!";
        creatLaserDevice(first_laser, first_laser->device_id);
    }

    if (second_laser) {
        LOG(INFO) << "will creat second laser device!";
        creatLaserDevice(second_laser, second_laser->device_id);
    }
    dispatch();
}

bool StandardLaserModule::doOpen() {
    if (laser_module) {
        // NOTE：由于这个laser_module->doOpen()是阻塞的，若链接不上就会一直链接，外出先设置为链接失败的状态，若链接成功就会马上退出  
        device_->setStateOpenFailed();
        if (laser_module->doOpen()) {
            if (device_) {
                device_->setStateOK();

                device_->setSerialNo(laser_module->getSerialNO());
                device_->setModelNo(laser_module->getModelNO());
                device_->setVersionNo(laser_module->getVersionNO());
            }
            return true;
        } else {
            if (device_) {
                device_->setStateOpenFailed();
            }
            return false;
        }
    }
    return false;
}

void StandardLaserModule::doStart() {
    // laser_module->doStart();
    LOG(INFO) << "will publish the scan!";
    DeviceState device_state = LIDARRUNNING;

    while (device_state == LIDARRUNNING) {
        try {
            auto laser_scan = std::make_shared<sros::core::LaserScanMsg>();
            if (laser_module->getScanData(laser_scan)) {
                laser_scan->sensor_name = device_->getName();
                if (enable_remove_rack_leg_ &&
                    (!enable_filter_only_load_full_ || g_state.load_state == sros::core::LOAD_FULL)) {
                    // 只有当顶起货架时才滤除货架腿
                    //                    rack_legs_filter.filterRack(laser_scan);
                    //                    src_sdk->getParameterInTime(0x1410, rotate_value);
                    double rotate = (double)g_state.rotate_value / 1000.0;
                    if (rack_detector) {
                        if (first_detect) {
                            rack::RackInfo_Ptr rack_para;
                            //                            LOG(INFO) << "rotate is:" << rotate;
                            // if (rack_detector->detectRack(laser_scan, rack_para,
                            //                               rotate)) 
                            //XXX:无用的
                            if(true)
                            {  // TODO:这里后期需要改成rotate
                                if (rack_para) {
                                    first_detect = false;
                                    LOG(INFO) << "begin to set!";
                                    auto rack_op = rack::RackOperatorInstance::getInstance();
                                    if (rack_op) {
                                        rack_op->updateRackInfo(rack_para);
                                    }
                                    LOG(INFO) << "update set!";
                                    // rack_filter->computeRackInfo(rack_para->rack_para);
                                    LOG(INFO) << "successfully to set!";
                                } else {
                                    LOG(INFO) << "err to detect rack!";
                                }

                            } else {
                                LOG(INFO) << "err to detect!";
                            }
                        }
                    }
                    //                    LOG(INFO)<<"rotate:"<<rotate;
                    rack_filters[0]->filterRack(laser_scan, rotate);
                    //                    LOG(INFO) << "rotate: down" << rotate;
                } else {
                    first_detect = true;
                }
                sendMsg(laser_scan);

                if (device_) {
                    device_->keepAlive();
                }
            } else {
                continue;
            }
        } catch (std::runtime_error &run_err) {
            LOG(INFO) << "get the error!" << run_err.what();
            return;
        }
    }
}

void StandardLaserModule::doStop() {
    if (laser_module) {
        laser_module->doStop();
        if (first_laser) {
            first_laser->device_state = LIDARCLOSED;
        }
        if (second_laser) {
            second_laser->device_state = LIDARCLOSED;
        }
        if (device_) {
            device_->setStateOff();
        }
    }
}

void StandardLaserModule::doClose() {
    if (laser_module) {
        laser_module->doClose();
    }
}

std::vector<std::string> splitStrsToStrs(const std::string &s, const char seperator) {
    std::vector<std::string> result;
    std::string::size_type i = 0;
    std::string::size_type j = 0;
    char c = s[0];
    if (c == '"')  // chip 设置的话一般带“”
        j = 1;
    // LOG(INFO) << "the s is:" << s;
    while (i < s.size()) {
        if (s[i] == seperator || i == s.size() - 1) {
            if (j != i || i == s.size() - 1) {
                auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                std::string item_s = s.substr(j, len);
                if (item_s == "\"") break;
                result.push_back(item_s);
            }
            j = i + 1;
        }
        i++;
    }

    return result;
}

std::vector<double> splitStrsToDoubles(const std::string &s, const char seperator) {
    std::vector<double> result;
    std::string::size_type i = 0;
    std::string::size_type j = 0;
    char c = s[0];
    if (c == '"')  // chip 设置的话一般带“”
        j = 1;
    // LOG(INFO) << "the s is:" << s;
    while (i < s.size()) {
        if (s[i] == seperator || i == s.size() - 1) {
            if (j != i || i == s.size() - 1) {
                auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                std::string item_s = s.substr(j, len);
                if (item_s == "\"") break;
                try {
                    if (item_s.size()) {
                        double item = stod(item_s);
                        result.push_back(item);
                    }
                }catch (std::exception &e){
                    LOG(ERROR) << "throw error:" << e.what()<<item_s;
                }
            }
            j = i + 1;
        }
        i++;
    }

    return result;
}

void StandardLaserModule::createRackInfos(std::vector<rack::RackInfo_Ptr> &rack_search_infos) {
    auto rack_info_strs = (sros::core::Settings::getInstance().getValue<std::string>("rack.rack_infos", "0"));

    auto racks = splitStrsToStrs(rack_info_strs, ';');
    for (auto &rack : racks) {
        auto rack_values = splitStrsToDoubles(rack, ',');
        int value_size = rack_values.size();
        if (value_size % 2 == 0) {  // value size不能为偶数,否则无法完成赋值
            LOG(INFO) << "the value format is wrong:" << rack_info_strs;
            return;
        }
        if (value_size <= 2) {
            LOG(INFO) << "rack para is wrong!" << rack;
            continue;
        }
        int first_leg_index = 3;
        if (value_size == 3) {
            LOG(INFO) << "value size is smaller than three:" << value_size;
            first_leg_index = 1;  //这里记录该货架的避障尺寸
        }

        rack_search_infos.emplace_back(new rack::RackInfo);
        auto rack_info = rack_search_infos.back();
        rack_info->leg_d = rack_values[0] * MM_TO_M;
        rack_info->avd_oba_length = rack_values[1] * MM_TO_M;
        rack_info->avd_oba_width = rack_values[2] * MM_TO_M;
        for (int i = first_leg_index; i < value_size; i += 2) {
            rack_info->leg_groups.emplace_back();
            rack_info->leg_groups.back().length = rack_values[i] * MM_TO_M;
            LOG(INFO) << "rack value:" << i << "," << rack_values[i];
            if (i + 1 < value_size) {
                LOG(INFO) << "rack value:" << i + 1 << "," << rack_values[i + 1];
                rack_info->leg_groups.back().width = rack_values[i + 1] * MM_TO_M;
            } else {
                LOG(INFO) << "rack info is wrong!";
            }
        }
    }
}

bool StandardLaserModule::doOpen(LaserModuleInfo_Ptr laser_module_info) {
    auto laser_module = laser_module_info->laser_module_;
    auto device = laser_module_info->device_;
    auto &device_state = laser_module_info->device_state;
    if (device) {
        device->setStateOpening();
    }

    if (laser_module) {
        if (laser_module->doOpen()) {
            device_state = LIDAROPENED;
            if (device) {
                device_ = device;
                device->setStateOK();

                device->setSerialNo(laser_module->getSerialNO());
                device->setModelNo(laser_module->getModelNO());
                device->setVersionNo(laser_module->getVersionNO());
            }
            return true;
        } else {
            if (device) {
                device->setStateOpenFailed();
            }
            return false;
        }
    }
    return false;
}

void StandardLaserModule::doStart(LaserModuleInfo_Ptr laser_module_info, int laser_id) {
    auto laser_module = laser_module_info->laser_module_;
    auto device = laser_module_info->device_;
    auto &device_state = laser_module_info->device_state;
    laser_module->doStart();
    LOG(INFO) << "will publish the scan!" << laser_id;
    device_state = LIDARRUNNING;

    while (device_state == LIDARRUNNING) {
        try {
            auto laser_scan = std::make_shared<sros::core::LaserScanMsg>();
            if (laser_module->getScanData(laser_scan)) {
                laser_scan->sensor_name = device->getName();
                bool is_up = laser_id < 2 ? first_lidar_install_up : second_lidar_install_up;
                inverseAndBeyondRangePoint(laser_scan, !is_up, min_range_, max_range_);

                // LOG(INFO)<<"laser delta size id:"<<laser_id<<":"<<(laser_module->getDeltaTimestampSize());

                /******************************** rack filter ****************************************/
                if (rack_filters.size() != 3) {
                    LOG(INFO) << "size is wrong!" << rack_filters.size();
                }
                if (laser_id < 2) {
                    recomputeRackInfo(laser_scan);
                }

                if (enable_remove_rack_leg_ &&
                    (!enable_filter_only_load_full_ || g_state.load_state == sros::core::LOAD_FULL)) {//TODO:这里需要修改，当前处理方式容易出现无法检测出货架尺寸而避障

                    double rotate = (double)g_state.rotate_value / 1000.0;
                 
                    rack_filters[laser_id]->filterRack(laser_scan, rotate);
                    //                    LOG(INFO) << "rotate: down" << rotate;
                }

                /**************************** motion compensation ****************************************/
                laser_scan->undistorted_ranges = laser_scan->ranges;
                laser_scan->undistorted_intensities = laser_scan->intensities;
                if (laser_correct_ && laser_id < 3 ) {
                    scan_compenstators_[laser_id].correctScan(laser_scan, is_up);
                }

                bool erase_undistorted_ranges = laser_id == 2 && !use_second_lidar_to_location_;
                if (erase_undistorted_ranges){
                    laser_scan->undistorted_ranges.assign(laser_scan->ranges.size(), 0);
                }
                if (laser_id == 1) {
                    if(respective_publish_scan_)
                    {
                        auto first_scan = std::make_shared<sros::core::LaserScanMsg>(*laser_scan);
                        first_scan->topic_ = "FIRST_SCAN";
                        sendMsg(first_scan);
                    }
                    read_scan_lock.lock();
                    auto bk_second_scan = second_scan;
                    read_scan_lock.unlock();
                    if (bk_second_scan) {
                        if (dual_laser_processor) {
                            int64_t delta_time = laser_scan->time_ - bk_second_scan->time_;
                            const int64_t delta_time_thresh_200ms = 2e5;
                            if(abs(delta_time)>delta_time_thresh_200ms){
                                LOG(ERROR) << "two scan delta time is large!,will drop second scan data! time:" << delta_time;
                                for (auto &range : bk_second_scan->ranges) {
                                    range = 0;
                                }
                                for (auto &unodered_range : bk_second_scan->undistorted_ranges) {
                                    unodered_range = 0;
                                }
                            }
                            if (!use_first_lidar_to_location_) {
                                laser_scan->undistorted_ranges.assign(laser_scan->ranges.size(), 0);
                            }
                            if (!use_second_lidar_to_location_) {
                                bk_second_scan->undistorted_ranges.assign(laser_scan->ranges.size(), 0);
                            }

                            if (dual_laser_correct_){
                                auto delta_pos = scan_compenstators_[laser_id].getDeltaLaserTF(bk_second_scan->time_, delta_time);
                                dual_laser_processor->setDeltaFirstLaser(delta_pos);
                            }
                            sros::core::LaserScan_ptr scan;
                            if (dual_laser_processor->combineDualLidarPoints(laser_scan, bk_second_scan, scan)) {
                                laser_scan = scan;
                            }
                            dual_laser_processor->setDeltaFirstLaser(Eigen::Affine2f::Identity());
                        }
                    }
                }

                if (laser_id < 2) {
                    sendMsg(laser_scan);
                } else {
                    read_scan_lock.lock();
                    second_scan = laser_scan;
                    read_scan_lock.unlock();
                    
                    if (respective_publish_scan_)
                    {
                        auto second_scan = std::make_shared<sros::core::LaserScanMsg>(*laser_scan);
                        second_scan->topic_ = "SECOND_SCAN";
                        sendMsg(second_scan);
                    }
                }

                if (device) {
                    device->keepAlive();
                }
            } else {
                continue;
            }
        } catch (std::runtime_error &run_err) {
            LOG(INFO) << "get the error!" << run_err.what();
            laser_module->doClose();
            laser_module->doOpen();
            laser_module->doStart();
            continue;
        }
    }
}

void StandardLaserModule::creatLaserDevice(LaserModuleInfo_Ptr laser_module_info, int device_id) {
    auto &device = laser_module_info->device_;
    auto &device_name = laser_module_info->device_name;
    auto &publish_laser_thread = laser_module_info->publish_laser_thread;


    LOG(INFO) << "device name:" << device_name;
    device = sros::device::DeviceManager::getInstance()->registerDevice(
        device_name, DEVICE_ID_LIDAR, DEVICE_COMM_INTERFACE_TYPE_ETH_1, DEVICE_MOUNT_SROS);
    device->setTimeoutTime(1000);
    device->setStateInitialization();
    if (doOpen(laser_module_info)) {
        LOG(INFO) << "will create new thread to publish scan!";
        publish_laser_thread.reset(
            new boost::thread(boost::bind(&StandardLaserModule::doStart, this, laser_module_info, device_id)));
    } else {
        sros::core::slam_state_msg_ptr m(new sros::core::SlamStateMsg());
        m->slam_state = sros::core::STATE_SCAN_NOT_PUBLISHED;
        sendMsg(m);
    }
}

void StandardLaserModule::initializeDualPara(DualLaserPara_Ptr &dual_para) {
    auto &s = sros::core::Settings::getInstance();
    if (!dual_para) {
        dual_para.reset(new DualLaserPara);
    }
    const double DEG_TO_RAD = M_PI / 180.0;
    const double MM_TO_M = 0.001;

    dual_para->first_origin_angle_min = dual_para->first_angle_min = s.getValue<double>("laser.first_laser_angle_min", -135) * DEG_TO_RAD;
    dual_para->first_origin_angle_max = dual_para->first_angle_max = s.getValue<double>("laser.first_laser_angle_max", 135) * DEG_TO_RAD;
    dual_para->first_center_x = s.getValue<double>("laser.first_laser_center_x", 270) * MM_TO_M;
    dual_para->first_center_y = s.getValue<double>("laser.first_laser_center_y", 170) * MM_TO_M;
    dual_para->first_center_yaw = s.getValue<double>("laser.first_laser_center_yaw", 45) * DEG_TO_RAD;

    dual_para->second_origin_angle_min = dual_para->second_angle_min = s.getValue<double>("laser.second_laser_angle_min", -135) * DEG_TO_RAD;
    dual_para->second_origin_angle_max = dual_para->second_angle_max = s.getValue<double>("laser.second_laser_angle_max", 135) * DEG_TO_RAD;
    dual_para->second_center_x = s.getValue<double>("laser.second_laser_center_x", -270) * MM_TO_M;
    dual_para->second_center_y = s.getValue<double>("laser.second_laser_center_y", -170) * MM_TO_M;
    dual_para->second_center_yaw = s.getValue<double>("laser.second_laser_center_yaw", -135) * DEG_TO_RAD;

    dual_para->angle_min = s.getValue<double>("laser.combine_laser_angle_min", -180) * DEG_TO_RAD;
    dual_para->angle_max = s.getValue<double>("laser.combine_laser_angle_max", 180) * DEG_TO_RAD;
    dual_para->range_min = s.getValue<double>("slam.map_laser_min_dist", 0.05);
    dual_para->range_max = s.getValue<double>("slam.map_laser_max_dist", 30.0);
    LOG(INFO) << "dual_para->first_angle_min," << dual_para->first_angle_min << ",dual_para->first_angle_max,"
              << dual_para->first_angle_max << ",dual_para->first_center_x," << dual_para->first_center_x
              << ",dual_para->first_center_y," << dual_para->first_center_y << ",dual_para->first_center_yaw,"
              << dual_para->first_center_yaw << ",dual_para->second_angle_min," << dual_para->second_angle_min
              << ",dual_para->second_angle_max," << dual_para->second_angle_max << ",dual_para->second_center_x,"
              << dual_para->second_center_x << ",dual_para->second_center_y," << dual_para->second_center_y
              << ",dual_para->second_center_yaw," << dual_para->second_center_yaw << ",dual_para->angle_min,"
              << dual_para->angle_min << ",dual_para->angle_max," << dual_para->angle_max << ",dual_para->range_min,"
              << dual_para->range_min << ",dual_para->range_max," << dual_para->range_max;
}

void StandardLaserModule::onUpdateRotateValue(sros::core::base_msg_ptr msg) {
    if (g_state.load_state == sros::core::LOAD_FULL) {
        if (src_sdk->getVersion() == sdk::SRC_PROTO_VERSION_V1) {
            src_sdk->getParameterInTime(0x1410, g_state.rotate_value);
        }
    }
    if (update_para_incre_++ % 20 == 0) {//1秒更新一次
        auto &s = sros::core::Settings::getInstance();
        if (dual_laser_processor) {
            dual_laser_processor->para()->obstacle_angle_cut_offset =
                s.getValue<double>("obstacle.oba_laser_angle_narrow_offset", 0.0) * DEG_TO_RAD;
        }
        laser_correct_ = s.getValue<bool>("laser.compentate",true);
        dual_laser_correct_ = s.getValue<bool>("laser.dual_compentate", true);

        if (dual_laser_processor) {
          use_second_lidar_to_location_ = !(s.getValue<std::string>("slam.use_second_lidar_to_location", "True") == "False");
          use_first_lidar_to_location_ = !(s.getValue<std::string>("slam.use_first_lidar_to_location", "True") == "False");
          auto dual_para = dual_laser_processor->para();
          dual_para->first_origin_angle_min = dual_para->first_angle_min = s.getValue<double>("laser.first_laser_angle_min", -135) * DEG_TO_RAD;
          dual_para->first_origin_angle_max = dual_para->first_angle_max = s.getValue<double>("laser.first_laser_angle_max", 135) * DEG_TO_RAD;
          if (!use_first_lidar_to_location_) {
            auto second_in_first_angle_max = dual_para->second_origin_angle_min +
                                         dual_para->second_center_yaw -
                                         dual_para->first_center_yaw;
            auto second_in_first_angle_min = dual_para->second_origin_angle_max +
                                         dual_para->second_center_yaw -
                                         dual_para->first_center_yaw;
            second_in_first_angle_min = atan2(sin(second_in_first_angle_min),
                                              cos(second_in_first_angle_min));
            second_in_first_angle_max = atan2(sin(second_in_first_angle_max),
                                              cos(second_in_first_angle_max));
            dual_para->first_origin_angle_min = second_in_first_angle_min;
            dual_para->first_origin_angle_max = second_in_first_angle_max;
          }
        }

        if (!rack_detector) {
            auto rack_leg_center_length = s.getValue<double>("rack.rack_leg_center_length", 1060) * MM_TO_M;
            auto rack_leg_center_width = s.getValue<double>("rack.rack_leg_center_width", 600) * MM_TO_M;
            auto rack_leg_diameter = s.getValue<double>("rack.rack_leg_diameter", 100) * MM_TO_M;
            if (rack_leg_diameter != rack_para.rack_leg_diameter ||
                    rack_leg_center_width != rack_para.rack_leg_center_width ||
                rack_leg_center_length != rack_para.rack_leg_center_length) {
                LOG(INFO) << "will update rack para!"
                          << "," << rack_leg_center_length << "," << rack_leg_center_width << "," << rack_leg_diameter;

                rack_para.rack_leg_center_length = rack_leg_center_length;
                rack_para.rack_leg_center_width = rack_leg_center_width;
                rack_para.rack_leg_diameter = rack_leg_diameter;
                rack::RackPara rack_para_1;
                rack_para_1.rack_length = rack_para.rack_leg_center_length;
                rack_para_1.rack_width = rack_para.rack_leg_center_width;
                rack_para_1.rack_leg_diameter = rack_para.rack_leg_diameter;
                for (auto &rack_filter : rack_filters) {
                    rack_filter->computeRackInfo(rack_para_1);
                }
            }
        }
        auto use_region_filter = s.getValue<std::string>("obstacle.rack_enable_region_filter", "False") == "True";
        if (use_region_filter != rack_para.use_region_filter) {
            LOG(INFO) << "will initialize rack filter! use region state:" << use_region_filter;
            rack_para.use_region_filter = use_region_filter;
            initializeRackPara(enable_remove_rack_leg_, enable_filter_only_load_full_, rack_filters);
        }
    }
}

void StandardLaserModule::onDebugCmdMsg(sros::core::base_msg_ptr msg) {
    auto pMsg = std::dynamic_pointer_cast<sros::core::CommandMsg>(msg);
    switch (pMsg->command)
    {
    case sros::core::CMD_SRC_RESET:
    {
        LOG(INFO) << "Handling CMD_SRC_RESET command";
        if (first_laser)
        {
            auto laser_module = first_laser->laser_module_;
            if (laser_module)
            {
                laser_module->doClose();
            }
        }

        if (second_laser)
        {
            auto laser_module = second_laser->laser_module_;
            if (laser_module)
            {
                laser_module->doClose();
            }
        }
        break;
    }
    default:
        break;
    }
}

void StandardLaserModule::initializeRackPara(bool &enable_remove_rack_leg,bool &enable_filter_only_load_full,std::vector<std::shared_ptr<rack::BaseRackFilter<sros::core::LaserScan_ptr>>>& rack_points_filters) {
    auto &s = sros::core::Settings::getInstance();

    enable_remove_rack_leg = s.getValue<std::string>("obstacle.enable_remove_rack_leg", "False") == "True";
    int action_mode = g_state.getActionControllerType();
    if (action_mode != sros::core::ACTION_CONTROLLER_TYPE_SRC_JACKING_ROTATE &&
        action_mode != sros::core::ACTION_CONTROLLER_TYPE_SRC_PUTTER_JACKING) {
        // 如果动作机构不是顶升货架类机构，则不启用货架腿滤除功能
        LOG(INFO) << "src.continuous_mode isn't ACTION_CONTROLLER_TYPE_SRC_JACKING_ROTATE or "
                     "ACTION_CONTROLLER_TYPE_SRC_PUTTER_JACKING, disable rack leg filter function";
        bool enable_load_rack = s.getValue<std::string>("rack.enable_load_rack", "False") == "True";
        if (enable_load_rack) {
            LOG(INFO) << "enable load rack!";
        }else{
            enable_remove_rack_leg = false;
        }
    }
    enable_filter_only_load_full =
        s.getValue<std::string>("obstacle.rack_enable_filter_only_load_full", "True") == "True";


    rack_para.rack_leg_center_length = s.getValue<double>("rack.rack_leg_center_length", 1060) * MM_TO_M;
    rack_para.rack_leg_center_width = s.getValue<double>("rack.rack_leg_center_width", 600) * MM_TO_M;
    rack_para.rack_leg_diameter = s.getValue<double>("rack.rack_leg_diameter", 100) * MM_TO_M;
    rack_para.rack_leg_search_thresh = s.getValue<double>("obstacle.rack_leg_search_thresh", 10) * DEG_TO_RAD;
    rack_para.tailing_noise_angle_thresh =
        s.getValue<double>("obstacle.rack_tailing_noise_angle_thresh", 2) * DEG_TO_RAD;
    rack_para.rack_backlash_rotate_angle = s.getValue<double>("rack.rack_leg_backlash_rotate_angle", 2) * DEG_TO_RAD;
    rack_para.use_region_filter = s.getValue<std::string>("obstacle.rack_enable_region_filter", "False") == "True";
    rack_para.rack_radius_offset = s.getValue<double>("obstacle.rack_region_radius_offset", 50) * MM_TO_M;

    auto coord_x = s.getValue<double>("posefilter.laser_coordx", 0.29f);
    auto coord_y = s.getValue<double>("posefilter.laser_coordy", 0.0f);
    auto coord_yaw = s.getValue<double>("posefilter.laser_coordyaw", 0.0f);

    auto laser_angle_min = s.getValue<double>("slam.laser_angle_min", 2.1f);
    auto laser_angle_max = s.getValue<double>("slam.laser_angle_max", 2.1f);

    double first_center_x = s.getValue<double>("laser.first_laser_center_x", 270) * MM_TO_M;
    double first_center_y = s.getValue<double>("laser.first_laser_center_y", 170) * MM_TO_M;
    double first_center_yaw = s.getValue<double>("laser.first_laser_center_yaw", 45) * DEG_TO_RAD;

    double second_center_x = s.getValue<double>("laser.second_laser_center_x", -270) * MM_TO_M;
    double second_center_y = s.getValue<double>("laser.second_laser_center_y", -170) * MM_TO_M;
    double second_center_yaw = s.getValue<double>("laser.second_laser_center_yaw", -135) * DEG_TO_RAD;
   
    float first_angle_max = s.getValue<float>("laser.first_laser_angle_max", M_PI);
    float first_angle_min = s.getValue<float>("laser.first_laser_angle_min", -M_PI);
    float second_angle_max = s.getValue<float>("laser.second_laser_angle_max", M_PI);
    float second_angle_min = s.getValue<float>("laser.second_laser_angle_min", -M_PI);

    rack::InstallPara para,para_1,para_2;
    para.backlash_angle = rack_para.rack_backlash_rotate_angle;
    para.trailing_angle = rack_para.tailing_noise_angle_thresh;
    para.laser_angle_min = laser_angle_min;
    para.laser_angle_max = laser_angle_max;
    para.laser_coord_x = coord_x;
    para.laser_coord_y = coord_y;
    para.laser_coord_yaw = coord_yaw;

    para_1.backlash_angle = rack_para.rack_backlash_rotate_angle;
    para_1.trailing_angle = rack_para.tailing_noise_angle_thresh;
    para_1.laser_angle_min = first_angle_min;
    para_1.laser_angle_max = first_angle_max;
    para_1.laser_coord_x = first_center_x;
    para_1.laser_coord_y = first_center_y;
    para_1.laser_coord_yaw = first_center_yaw;

    para_2.backlash_angle = rack_para.rack_backlash_rotate_angle;
    para_2.trailing_angle = rack_para.tailing_noise_angle_thresh;
    para_2.laser_angle_min = second_angle_min;
    para_2.laser_angle_max = second_angle_max;
    para_2.laser_coord_x = second_center_x;
    para_2.laser_coord_y = second_center_y;
    para_2.laser_coord_yaw = second_center_yaw;

    rack_infos.clear();
    createRackInfos(rack_infos);
    if (!rack_infos.empty()) {
        LOG(INFO) << "rack info size:" << rack_infos.size();
        for (auto &rack : rack_infos) {
            LOG(INFO) << "rack:" << rack->leg_d << "," << rack->leg_groups.size() << "," << rack->avd_oba_length << ","
                      << rack->avd_oba_width;
        }
        rack_detector.reset(new rack::RackDetector<sros::core::LaserScan_ptr>(para, rack_infos));
    } else {
        LOG(INFO) << "will not use rack detector!";
    }
    rack::RackPara rack_para_1;
    rack_para_1.rack_length = rack_para.rack_leg_center_length;
    rack_para_1.rack_width = rack_para.rack_leg_center_width;
    rack_para_1.rack_leg_diameter = rack_para.rack_leg_diameter;
    std::vector<std::shared_ptr<rack::BaseRackFilter<sros::core::LaserScan_ptr>>> tmp_rack_filters;
    if (rack_para.use_region_filter) {
        tmp_rack_filters.emplace_back(new rack::RackFilter<sros::core::LaserScan_ptr>(para));
        tmp_rack_filters.emplace_back(new rack::RackFilter<sros::core::LaserScan_ptr>(para_1));
        tmp_rack_filters.emplace_back(new rack::RackFilter<sros::core::LaserScan_ptr>(para_2));
    } else {
        tmp_rack_filters.emplace_back(new rack::RackLegsFilter<sros::core::LaserScan_ptr>(para));
        tmp_rack_filters.emplace_back(new rack::RackLegsFilter<sros::core::LaserScan_ptr>(para_1));
        tmp_rack_filters.emplace_back(new rack::RackLegsFilter<sros::core::LaserScan_ptr>(para_2));
    }
    for (auto &filter : tmp_rack_filters) {
        filter->computeRackInfo(rack_para_1);
    }
    rack_points_filters.swap(tmp_rack_filters);
}

StandardLaserModule::~StandardLaserModule() {}

void StandardLaserModule::inverseAndBeyondRangePoint(sros::core::LaserScan_ptr scan, 
                                    const bool is_inverse, const float min_range, const float max_range)
{
     auto point_size = scan->ranges.size();
    
    if(is_inverse)
    {
        std::vector<float> ranges(point_size);
        std::vector<float> intens(point_size);
        for (int i = 0; i < point_size; ++i)
        {
            if (scan->ranges[point_size - 1 - i] >= min_range && scan->ranges[point_size - 1 - i] <= max_range)
                ranges[i] = scan->ranges[point_size - 1 - i] ;
            else
                ranges[i] = 0;
            intens[i] = scan->intensities[point_size - 1 - i];
        }
        scan->ranges.swap(ranges);
        scan->intensities.swap(intens);
    }
    else
    {
        for (int i = 0; i < point_size; ++i)
        {
            if (scan->ranges[i] < min_range || scan->ranges[i] > max_range)
                scan->ranges[i] = 0;
        }
    }
    
   

}
void StandardLaserModule::recomputeRackInfo(sros::core::LaserScan_ptr& laser_scan) {
    if (enable_remove_rack_leg_ &&
        (!enable_filter_only_load_full_ || g_state.load_state == sros::core::LOAD_FULL)) {
        // 只有当顶起货架时才滤除货架腿
        //                    rack_legs_filter.filterRack(laser_scan);
        //                    src_car.getParameterInTime(0x1410, rotate_value);
        double rotate = (double)g_state.rotate_value / 1000.0;
        if (rack_detector) {
            if (first_detect) {
                rack::RackInfo_Ptr rack_para;
                //                            LOG(INFO) << "rotate is:" << rotate;
                if (rack_detector->detectRack(laser_scan, rack_para,
                                              rotate)) {  // TODO:这里后期需要改成rotate
                    if (rack_para) {
                        first_detect = false;
                        LOG(INFO) << "begin to set!";
                        auto rack_op = rack::RackOperatorInstance::getInstance();
                        if (rack_op) {
                            rack_op->updateRackInfo(rack_para);
                        }
                        for(auto& rack_filter:rack_filters){
                            LOG(INFO) << "update set!";
                            rack_filter->computeRackInfo(rack_para->rack_para);
                        }
                        LOG(INFO) << "successfully to set!";
                    } else {
                        LOG(INFO) << "err to detect rack!";
                    }

                } else {
                    LOG(INFO) << "err to detect!";
                }
            }
        }

    } else {
        first_detect = true;
    }
}
}  // namespace laser
