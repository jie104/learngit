#include "../src/leimou_f30_driver.h"
#include <iostream>
#include <sys/time.h>

int main(int argc, char** argv)
{
    // laser data
    Intelly laser;
    ScanCfg scan_cfg;
    ScanData scan_data;

    scan_cfg.host_ip = "192.168.0.111";
    scan_cfg.data_port = 4001;
    scan_cfg.debug_port = 4002;
    scan_cfg.start_angle = -4500;
    scan_cfg.stop_angle = 22500;
    scan_cfg.reporting_interval = 66;  // ms

    if (laser.init(scan_cfg))
        std::cout << "Intelly intialization succeeded!" << std::endl;
    else
    {
        std::cerr << "Intelly intialization error!" << std::endl;
        return -1;
    }

    if (laser.StartScan())
        std::cout << "Start laser scan succeeded!" << std::endl;
    else
    {
        std::cerr << "Start laser scan error!" << std::endl;
        return -1;
    }
    // Sleep 3 senconds.
    sleep(3);

    int error_cnt = 0;
    int t1, t2;
    float delay;
    while(1)
    {
//        t1 = getms();
        if (laser.GrabScanData(scan_data))
        {

            std::cout << "------Received scan data: ------\n";
            for (int i = 0; i < scan_data.num_values; i++)
                std::cout << scan_data.ranges[i] << " ";
            std::cout << std::endl;
        }
        else
        {
            error_cnt ++;
            std::cout << "Waiting laser data..." << std::endl;
            if (error_cnt >= 100)
            {
                std::cout << "---Grab Laser Data Error, Exit!---" << std::endl;
                return -1;
            }
        }
//        t2 = getms();
//        delay = float(t2-t1);
//        std::cout << "---time delay: ---" << delay << " ms" << std::endl;
    }
    laser.StopScan();

    return 0;
}




/*****************End of File************************************/

