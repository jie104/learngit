/**
 * @file main.cpp
 *
 * @author lhx
 * @date 2016/12/05
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include <iostream>

#include <gflags/gflags.h>

#include "core/version.h"

#include "core/module_manager.h"
#include "core/monitor/soc_id.h"
#include "core/msg_bus.h"
#include "core/settings.h"
#include "core/usart/canbus.h"
#include "core/util/utils.h"

#include "modules/main/main_module.h"
#include "modules/network/network_module.h"
#include "modules/registers/registers_module.h"
#include "modules/timer/timer_module.h"

#include "modules/location/location_module.h"
#include "modules/posefilter/PoseManagerModule.h"
#include "modules/slam/slam_module.h"
// #include "modules/imu/imu_module.h"
#include "modules/imu2/imu_module.h"

#include "modules/station_recognition/station_recognition_module.h"

#include "modules/mission/mission_module.h"
#include "modules/navigation/navigation_module.h"

#include "modules/usart/usart_module.h"

#include "modules/action_controller/action_controller_module.h"
#include "modules/device/device_module.h"
#include "modules/extension_action_controller/extension_action_controller.h"
#include "modules/extension_action_controller/simplify_extension_action_controller.h"
#include "modules/hmi/hmi_module.h"
#include "modules/huawei_communication/huawei_communication_module.h"
#include "modules/laser/standard_laser_module.h"
#include "modules/modbus/modbus_module.h"
#include "modules/monitor/monitor_module.h"
#include "modules/obstacle/obstacle_module.h"
#include "modules/pipe/pipe_module.h"
#include "modules/rack_query/rack_query_module.h"
#include "modules/schedule/schedule_module.h"
#include "modules/security/security_module.h"
#include "modules/touch_screen/screen_module.h"
#include "modules/upgrade/upgrade_module.h"
#include "modules/vsc/vsc_module.h"
#include "modules/spu/spu_module.h"
#include "modules/feature_extractor/feature_extractor_module.h"
#include "modules/perception/perception_manager_module.h"
#include "modules/perception/sensor/livox/livox_midxx_module.h"

#ifdef ENABLE_SROSBAG
#include "modules/srosbag/sros_bag_module.h"
#endif

#ifdef ENABLE_VISION
#include "modules/camera/camera_module.h"
#include "modules/vision/vision_module.h"
#endif
#include "core/tf/imu_circle_array.hpp"
DEFINE_string(log_path, "/tmp/", "Log output path");

void runAllModule();

/**
 * 初始化一些模块启动前需要准备好的东西
 */
void init() {
    auto cpu_serial_no = get_soc_id();  // 使用cpu id作为硬件序列号
    sros::core::Settings& settings = sros::core::Settings::getInstance();
    auto sub_no = cpu_serial_no.substr(0,8); //暂取前8个，最后是一个null会影响写数据库
    settings.setValue("main.serial_no", sub_no);  // 将CPU序列号设置到配置中去
}

void print_runtime_info() {
    LOG(INFO) << "sqlite3_libversion() -> " << sqlite3_libversion();
    LOG(INFO) << "sqlite3_threadsafe() -> " << sqlite3_threadsafe();
}

int main(int argc, char* argv[]) {
    if (argc == 2 && 0 == strncmp(argv[1], "test", 4)) {
        auto version_number = execShell("/sros/bin/sros --version-number");
        if (version_number.size() == 7) {
            if (std::stoi(version_number) < 4010001) {
                std::cout << "Current SROS version must than v4.10.1!" << std::endl;
                return -1;
            }
        } else { // 还不支持--version-number的情况
            std::cout << version_number << std::endl;
            return -1;
        }
        std::cout << "sros_version " << GIT_VERSION_STR << std::endl;
        return 0;
    }
    if (argc == 2 && 0 == strncmp(argv[1], "--version-number", 16)) {
        std::cout << VERSION(SROS_MAJOR_VERSION, SROS_MINOR_VERSION, SROS_PATCH_VERSION) << std::endl;
        return 0;
    }
    if (argc == 2 && 0 == strncmp(argv[1], "--version", 9)) {
        std::cout << "sros version " << SROS_VERSION_STR << std::endl;
        return 0;
    }

    gflags::SetVersionString(SROS_VERSION_STR);
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_log_dir = FLAGS_log_path;  // hack for glog, glog not support gflags when build
    FLAGS_v = sros::core::Settings::getInstance().getValue<int>("debug.vlog_v", -10);

    sros::core::LogHelper logHelper(argv[0]);

    Logger::init();
    LOGGER(INFO, SROS) << "SROS start! Version: " << SROS_VERSION_STR << " (" << GIT_VERSION_STR << ")";
    LOG(INFO) << "/etc/issue is :" << std::endl << execShell("cat /etc/issue");
    LOG(INFO) << "ifconfig is: " << std::endl << execShell("ifconfig -a");

    init();

    print_runtime_info();

    runAllModule();

    return 0;
}

void runAllModule() {
    sros::core::ModuleManager module_manager;

    module_manager.registerModule(new sros::TimerModule);
    module_manager.registerModule(new sros::MainModule);
    module_manager.registerModule(new sros::RegistersModule);

    Canbus::getInstance();

    module_manager.registerModule(new hmi::HMIModule);
    module_manager.registerModule(new tscreen::ScreenModule);

#ifdef ENABLE_VISION
    module_manager.registerModule(new camera::CameraModule);
    module_manager.registerModule(new vision::VisionModule);
#endif

#ifdef ENABLE_SROSBAG
    module_manager.registerModule(new srosbag::BagModule);
#endif
    module_manager.registerModule(new network::NetworkModule);

    module_manager.registerModule(new mapping::SlamModule);
    module_manager.registerModule(new sros::pose_filter::PoseManagerModule);
    module_manager.registerModule(new location::LocationModule);

    module_manager.registerModule(new nav::NavigationModule);

    module_manager.registerModule(new sros::StationRecognitionModule);

    module_manager.registerModule(new ac::ActionControllerModule);

    module_manager.registerModule(new usart::UsartModule);
    module_manager.registerModule(new vsc::VSCModule);
    module_manager.registerModule(new spu::SPUModule);
    module_manager.registerModule(new security::SecurityModule);
    module_manager.registerModule(new device::DeviceModule);
    module_manager.registerModule(new laser::StandardLaserModule);

    module_manager.registerModule(new monitor::MonitorModule);
    module_manager.registerModule(new sros::UpgradeModule);
    module_manager.registerModule(new huawei::HuaweiCommModule);
    module_manager.registerModule(new sros::ModbusModule(sros::MODBUS_TCP));
    module_manager.registerModule(new sros::ModbusModule(sros::MODBUS_RTU));

    module_manager.registerModule(new sros::MissionModule);

    module_manager.registerModule(new oba::ObstacleModule);
    module_manager.registerModule(new sros::PipeModule);

    module_manager.registerModule(new rack_query::RackQueryModule);
    module_manager.registerModule(new laser::LivoxMidXXModule);
    module_manager.registerModule(new object_detector::PerceptionManager);

    module_manager.registerModule(new sros::eac::ExtensionActionController);
    module_manager.registerModule(new sros::eac::SimplifyExtensionActionController);

    module_manager.registerModule(new sros::ScheduleModule());

    module_manager.registerModule(new sros::FeatureExtractorModule());
    module_manager.registerModule(new imu::ImuModule());

    // 启动各个Module
    module_manager.runAll();

    // 开始分发msg
    sros::core::MsgBus::getInstance()->dispatch();

    // never return
}
