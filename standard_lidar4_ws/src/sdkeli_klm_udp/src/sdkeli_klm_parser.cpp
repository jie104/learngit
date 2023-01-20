#include <sdkeli_klm_udp/sdkeli_klm_parser.h>
#include <sdkeli_klm_udp/sdkeli_klm_sensor_frame.h>
#include <ros/ros.h>
#include <stdio.h>

namespace sdkeli_klm_udp
{
CSDKeliKlmParser::CSDKeliKlmParser() :
    CParserBase(),
    fRangeMin(0.05),
    fRangeMax(10.0),
    fTimeIncrement(-1.0),
    fFrame_id("laser")
{
    // Do Nothing...
}

CSDKeliKlmParser::~CSDKeliKlmParser()
{
    // Do Nothing...
}

int CSDKeliKlmParser::Parse(unsigned short *data, size_t data_length,
                            unsigned char *info, size_t info_length,
                            SDKeliKlmConfig &config, sensor_msgs::LaserScan &msg)
{
    CSDKeliKlmSensFrame *pSensFrame = new CSDKeliKlmSensFrame();
    if(!pSensFrame->InitFromSensBuff(data, data_length, info, info_length))
    {
        ROS_INFO("Invalid frame data!");
        return ExitSuccess;
    }

    int dataCount = pSensFrame->GetSensDataCount();
    int angleRes = pSensFrame->GetSensInfoAngleRes();
    int sampleNum = pSensFrame->GetSensInfoSampleNum();
    int addIntensity = pSensFrame->GetSensInfoAddIntensity();
    int timeSample = pSensFrame->GetSensInfoTimeSample();
    int scanTime = pSensFrame->GetSensInfoScanTime();
    int expectNum = 360 * 1000 / angleRes;

    ROS_DEBUG("expectNum: %d - sampleNum: %d", expectNum,sampleNum);

    /*Fill sensor message struct*/
    //msg.header.frame_id = config.frame_id;
    msg.header.frame_id = fFrame_id;
    //ROS_DEBUG("Publishing with frame id: %s", config.frame_id.c_str());
    ROS_DEBUG("Publishing with frame id: %s", fFrame_id.c_str());

    /*1: Scan time: The time for every frame.*/
    ros::Time start_time = ros::Time::now();
    msg.scan_time = scanTime / 10000000.0f; /* 0.1us ->s  */
    int scanning_freq = 1.0 / msg.scan_time;
    ROS_DEBUG("scanning freq: %d, scan_time: %f", scanning_freq, msg.scan_time);

    /*2: Time increment: Time interval for between each data.*/
    /*Time increment has been overriden*/
    fTimeIncrement = scanTime / 10000000.0f / expectNum;
    msg.time_increment = fTimeIncrement;
    ROS_DEBUG("time_increment: %f", msg.time_increment);

    /*3: Angle Min: Starting angle of current scanning data.*/
    int starting_angle = 0xFFF8AD00; /* -48 */
    if(sampleNum == expectNum)
    {
        starting_angle = 0x00; /* 0 */
    }

    msg.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2;
    ROS_DEBUG("starting_angle: %d, angle_min: %f", starting_angle, msg.angle_min);

    /*4: Angle step width: anguler between each scanning data.*/
    unsigned short angular_step_width = angleRes * 10; /* xxx */
    msg.angle_increment = (angular_step_width / 10000.0f) / 180.0 * M_PI;

    /*5: Angle Max: Ending angle of current scanning data.*/
    msg.angle_max = msg.angle_min + (dataCount - 1) * msg.angle_increment;

    /* calculate statring data index and adjust angle_min to min_ang config param */
    int index_min = 0;
    while (msg.angle_min + msg.angle_increment < config.min_ang)
    {
        msg.angle_min += msg.angle_increment;
        index_min++;
    }
    ROS_DEBUG("index_min: %d, angle_min: %f", index_min, msg.angle_min);

    /* calculate ending data index and adjust angle_max to max_ang config param */
    int index_max = dataCount - 1;
    while (msg.angle_max - msg.angle_increment > config.max_ang)
    {
        msg.angle_max -= msg.angle_increment;
        index_max--;
    }
    ROS_DEBUG("index_max: %i, angle_max: %f", index_max, msg.angle_max);

    /*5: Fill data range*/
    msg.ranges.resize(index_max - index_min + 1);
    msg.ranges.assign(index_max - index_min + 1, std::numeric_limits<double>::infinity());
    ROS_DEBUG("Fill sensor data. index_min = %d, index_max = %d.", index_min, index_max);

    int check_all_num = 0;
    int check_fault_cnt = 0;
    int check_fault_cnt2 = 0;
    for (int j = index_min; j <= index_max; ++j)
    {
        check_all_num++;

        if(config.debug_mode)
        {
            if((j - index_min + 1) % 48 == 0)
            {
                printf("\n");
            }
        }

        unsigned short range = pSensFrame->GetSensDataOfIndex(j);
        /*unsigned short range = 3000; */ /*For testing....*/
        //  ROS_WARN("fRangeMax=%f,fRangeMin=%f",fRangeMax,fRangeMin);
        if(0x01 == range)
        {
            check_fault_cnt++;
        }

        float meter_value = range / 500.0;
        if(meter_value > fRangeMin && meter_value < fRangeMax)
        {
            msg.ranges[j - index_min] = meter_value;
        }
        else
        {
            check_fault_cnt2++;
        }

        if(config.debug_mode)
        {
            printf("%.2f ", msg.ranges[j - index_min]);
        }
    }
    if(config.debug_mode)
    {
        printf("\n");
    }

    if((check_all_num == check_fault_cnt) || (check_all_num == check_fault_cnt2))
    {
        return ExitError;
    }

    if(config.intensity)
    {
        msg.intensities.resize(index_max - index_min + 1);
        for (int j = index_min; j <= index_max; ++j)
        {
            unsigned short intensity = pSensFrame->GetSensIntensityOfIndex(j);

            if(intensity > 55000)
            {
                intensity = 600;
            }

            if(intensity > 5000)
            {
                intensity = 200 + (intensity - 5000) / 1200;

            }
            else
            {
                intensity = intensity / 25;
            }

            msg.intensities[j - index_min] = intensity;
        }
    }

    /*Override range*/
    msg.range_min = fRangeMin;
    msg.range_max = fRangeMax;

    /*6: Setting starting time*/
    /* - last scan point = now ==> first scan point = now - data count * time increment*/
    msg.header.stamp = start_time - ros::Duration().fromSec(dataCount * msg.time_increment);
    /* - shit forward to time of first published scan point*/
    msg.header.stamp += ros::Duration().fromSec((double)index_min * msg.time_increment);
    /* - add time offset (to account for USB latency etc.)*/
    msg.header.stamp += ros::Duration().fromSec(config.time_offset);

    /*Consistency Check*/
    float expected_time_increment = msg.scan_time * msg.angle_increment / (2.0 * M_PI);
    if (fabs(expected_time_increment - msg.time_increment) > 0.00001)
    {
        ROS_DEBUG_THROTTLE(60,
                           "The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
                           "Expected time_increment: %.9f, reported time_increment: %.9f. "
                           "Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.",
                           expected_time_increment,
                           msg.time_increment);
    }

    if(pSensFrame)
    {
        delete pSensFrame;
    }

    return ExitSuccess;
}

void CSDKeliKlmParser::SetRangeMin(float minRange)
{
    fRangeMin = minRange;
}

void CSDKeliKlmParser::SetRangeMax(float maxRange)
{
    fRangeMax = maxRange;
}

void CSDKeliKlmParser::SetTimeIncrement(float time)
{
    fTimeIncrement = time;
}

void CSDKeliKlmParser::SetFrameId(std::string frame_id)
{
    fFrame_id = frame_id;
}

} /*namespace sdkeli_klm_udp*/
