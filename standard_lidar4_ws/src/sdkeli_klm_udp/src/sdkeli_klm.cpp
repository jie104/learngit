#include <sdkeli_klm_udp/sdkeli_klm_common_udp.h>
#include <sdkeli_klm_udp/sdkeli_klm_parser.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sdkeli_klm");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /*Check whether hostname is provided*/
    bool isTcpConnection = false;
    std::string strHostName;
    std::string strPort;

    if(pnh.getParam("hostname", strHostName))
    {
        isTcpConnection = true;
        pnh.param<std::string>("port", strPort, "2112");
    }

    /*Get configured time limit*/
    int iTimeLimit = 5;
    pnh.param("timelimit", iTimeLimit, 5);

    bool isDataSubscribed = false;
    pnh.param("subscribe_datagram", isDataSubscribed, false);

    int iDeviceNumber = 0;
    pnh.param("device_number", iDeviceNumber, 0);

    /*Create and initialize parser*/
    sdkeli_klm_udp::CSDKeliKlmParser *pParser = new sdkeli_klm_udp::CSDKeliKlmParser();

    double param;
    std::string frame_id;

    if(pnh.getParam("range_min", param))
    {
        ROS_INFO("range_min: %f", param);
        pParser->SetRangeMin(param);
    }

    if(pnh.getParam("range_max", param))
    {
        ROS_INFO("range_max: %f", param);
        pParser->SetRangeMax(param);
    }

    if(pnh.getParam("time_increment", param))
    {
        ROS_INFO("time_increment: %f", param);
        pParser->SetTimeIncrement(param);
    }

    if(pnh.getParam("frame_id", frame_id))
    {
        ROS_INFO("frame_id: %s", frame_id.c_str());
        pParser->SetFrameId(frame_id);
    }

    /*Setup UDP connection and attempt to connect/reconnect*/
    sdkeli_klm_udp::CSDKeliKlmCommon *pSDKeliKlm = NULL;
    int result = sdkeli_klm_udp::ExitError;
    while(ros::ok())
    {
        if(pSDKeliKlm != NULL)
        {
            delete pSDKeliKlm;
        }

        ROS_DEBUG("pSDKeliKlm");

        pSDKeliKlm = new sdkeli_klm_udp::CSDKeliKlmCommonUdp(strHostName, strPort, iTimeLimit, pParser, nh, pnh);
        result = pSDKeliKlm->Init();


        /*Device has been initliazed successfully*/
        while(ros::ok() && (result == sdkeli_klm_udp::ExitSuccess))
        {
            ros::spinOnce();
            result = pSDKeliKlm->LoopOnce();
        }

        if(result == sdkeli_klm_udp::ExitFatal)
        {
            return result;
        }
    }

    if(pSDKeliKlm != NULL)
    {
        delete pSDKeliKlm;
    }

    if(pParser != NULL)
    {
        delete pParser;
    }

    return result;
}
