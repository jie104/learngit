#include <iostream>
#include "ord/ord_driver.h"
#include "ord/lidar_address.h"
#include "ord/ord_types.h"
#include <string.h>
#ifdef __linux__
#include <arpa/inet.h>
#include <unistd.h>
#endif

int main()
{
    printf("this is oradar lidar sdk test demo \n");
    // 雷达出厂默认IP为192.168.1.100, 监听的端口号Port为2007
    in_addr_t ip_addr = inet_addr("192.168.1.100");
    in_port_t port = htons(2007);
    ord_sdk::LidarAddress sensor(ip_addr, port);
    ord_sdk::OrdDriver drv(sensor);
    ord_sdk::ScanFrameData scan_frame_data;
	
	//尝试打开设备
    if (drv.open() != ord_sdk::no_error){
        std::cerr << "unable to open device" << std::endl;
        return -1;
    }
	//尝试连接设备
    if(drv.trackConnect() != ord_sdk::no_error){
        std::cerr << "unable to connect ms500 lidar" << std::endl;
        return -1;
    }
	//启动雷达的数据传输
	drv.enabelDataStream();

	while (true) {
		//读取雷达一圈扫描数据，该数据以ScanFrameData的形式提供给用户
        if (drv.getScanFrameData(scan_frame_data) == ord_sdk::no_error){
            int count =  scan_frame_data.layers[0].ranges.size();
            std::cout << "scan_frame_data count size is " << \
            scan_frame_data.layers[0].ranges.size()  << std::endl;
            std::cout << "scan_frame_data.layers.szie = " << \
            scan_frame_data.layers.size() << std::endl;
            std::cout << "scan_frame_data.layers[0].ranges.szie = " << \
            scan_frame_data.layers[0].ranges.size() << std::endl;
            std::cout << "scan_frame_data.layers[0].intensities.szie = " << \
            scan_frame_data.layers[0].intensities.size() << std::endl;
            std::cout << "timestamp is " << scan_frame_data.timestamp << std::endl;
        }else{
            std::cerr << "unable to get point cloud data\n";
            break;
        }
    }
	//关闭设备
    drv.close();
    return 0;
}
