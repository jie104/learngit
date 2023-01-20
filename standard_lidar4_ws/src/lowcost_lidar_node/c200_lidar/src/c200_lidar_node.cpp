#include <c200_lidar/c200_lidar.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "c200_lidar_node");

    C200 lidar;
    
    while (ros::ok()) {
        ROS_INFO_STREAM("Connecting to laser at " << lidar.host_);
        lidar.connect(lidar.host_, lidar.port_);
        if (!lidar.isConnected()) {
            ROS_WARN("Unable to connect, retrying.");
            ros::Duration(1).sleep();
            continue;
        }

        ROS_DEBUG("Logging in to lidar.");
        lidar.login();
        lidar.getDeviceID();
        lidar.getSerialNumber();
        lidar.getFirmwareVersion();
        if(lidar.GetNORSwitchFlag())
        {
           lidar.SetReflectSwitch(1);
        }
        else
        {
           lidar.SetReflectSwitch(0);
        }
        lidar.SetTimeStamp(lidar.NTPswitch_);

        if (lidar.getDeviceState() != free_optics::device_ready) {
            lidar.disconnect();
            ROS_WARN("Device is not ready. Retrying.");
            ros::Duration(1).sleep();
            continue;
        }

        ROS_INFO("Connected to lidar.");

        lidar.getScanAngle();

        lidar.scanContinous(1);

        bool result = true;
        while(ros::ok() && result == true) {
            ros::spinOnce();
            result = lidar.getScanData();

            if (result == false) {
                ROS_ERROR("Lidar timed out on delivering scan, attempting to reinitialize.");
            }
            // ros::Duration(0.04).sleep();
        }

        // lidar.scanContinous(0);
        lidar.disconnect();

        if (ros::isShuttingDown()) {
            break;
        }
    }

    return 0;
}
