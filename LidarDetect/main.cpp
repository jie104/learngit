#include "widget.h"
#include <QApplication>
#include "standard_lidar_protocol/oradar_laser_protocol.hpp"
#include "standard_lidar_protocol/tcp_scan_data_receiver.hpp"
#include "standard_lidar_protocol/udp_scan_data_receiver.hpp"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pepperl_fuchs_r2000/r2000_node.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;

    w.show();


    return a.exec();
}

