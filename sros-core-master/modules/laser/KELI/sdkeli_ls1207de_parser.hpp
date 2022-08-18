#ifndef SDKELI_LS1207DE_PARSER__
#define SDKELI_LS1207DE_PARSER__

//#include <sdkeli_lspdm_udp/parser_base.h>
#include <boost/asio/detail/shared_ptr.hpp>
//#include "sdkeli_lspdm_udp/SDKeliLspdmConfig.h"
#include "sdkeli_ls_sensor_frame.hpp"
#include <math.h>
//#include "sensor_msgs/LaserScan.h"
#include <limits>

namespace sdkeli_lspdm_udp
{
    enum ExitCode
    {
        ExitSuccess = 0,
        ExitError   = 1,
        ExitFatal   = 2
    };
    struct LidarConfig{
        double min_ang;
        double max_ang;
        bool intensity;
        bool debug_mode;
        double time_offset;
        int skip;
        bool auto_reboot;
        LidarConfig(){
            min_ang = -2.35619449019;
            max_ang = 2.35619449019;
            intensity = true;
            debug_mode = 0;
            time_offset = -0.001;
            skip = 0;
            auto_reboot = 0;
        }
    };
class CSDKeliLs1207DEParser
{
public:
    CSDKeliLs1207DEParser():fRangeMin(0.05),
                            fRangeMax(30.0),
                            fTimeIncrement(-1.0),
                            fangleResolution(0.25),
                            fFrame_id("laser"){}
    virtual ~CSDKeliLs1207DEParser(){}
//    template <class ScanMsg>
//    int Parse(char *data, size_t data_length, SDKeliLspdmConfig &config, ScanMsg &msg);
    template <class ScanMsg>
    int Parse(char *data, size_t data_length, LidarConfig &config, ScanMsg &msg)
    {
        CSDKeliLsSensFrame *pSensFrame = new CSDKeliLsSensFrame();

        if(fangleResolution == 0.25)
        {
            mSamplePointNum = 270 * 4 + 1;
        }
        else if(fangleResolution == 0.5)
        {
            mSamplePointNum = 270 * 2 + 1;
        }
        else if(fangleResolution == 0.33)
        {
            mSamplePointNum = 270 * 3 + 1;
        }
        else
        {
            mSamplePointNum = 270 * 4 + 1;
        }

        pSensFrame->SetAngleResolution(fangleResolution);
        pSensFrame->SetNumSampePoint(mSamplePointNum);

        if(!pSensFrame->InitFromSensBuff(data, data_length))
        {
//            ROS_INFO("Invalid frame data!");
            return ExitSuccess;
        }

        int dataCount = pSensFrame->GetSensDataCount();
//    LOG(INFO) << "dataCOunt: " << dataCount;
        float lidarDistenPara = 100.0f;
        if(pSensFrame->GetHigePrecisionFlag() == true)
        {
            lidarDistenPara = 1000.0f;
        }
        else
        {
            lidarDistenPara = 100.0f;
        }

        /*Fill sensor message struct*/
        //msg.header.frame_id = config.frame_id;
//        msg.header.frame_id = fFrame_id;
        //ROS_DEBUG("Publishing with frame id: %s", config.frame_id.c_str());
//        ROS_DEBUG("Publishing with frame id: %s", fFrame_id.c_str());

        /*1: Scan time: The time for every frame.*/
//    ros::Time start_time = ros::Time::now();
        unsigned short scanning_freq = 1000 / 43 * 100; /*For dev borad, the device will send data every 50ms*/
        msg.scan_time = 1.0 / (scanning_freq / 100.0);
//        ROS_DEBUG("scanning freq: %d, scan_time: %f", scanning_freq, msg.scan_time);

        /*2: Time increment: Time interval for between each data.*/
        /*Time increment has been overriden*/
        fTimeIncrement = 0.000040;
        msg.time_increment = fTimeIncrement;
//        ROS_DEBUG("time_increment: %f", msg.time_increment);

        /*3: Angle Min: Starting angle of current scanning data.*/
        int starting_angle = 0xFFF92230; /* -45 degree * 10000 */
        msg.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2;
//        ROS_DEBUG("starting_angle: %d, angle_min: %f", starting_angle, msg.angle_min);

        /*4: Angle step width: anguler between each scanning data.*/
        unsigned short angular_step_width = 0x9c4; /* 2500 */
        if(fangleResolution == 0.25)
        {
            angular_step_width = 0x9c4;     /* 2500 */
        }
        else if(fangleResolution == 0.5)
        {
            angular_step_width = 0x1388;    /* 5000 */
        }
        else if(fangleResolution == 0.33)
        {
            angular_step_width = 0xD05;     /* 3333 */
        }

        msg.angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;

        /*5: Angle Max: Ending angle of current scanning data.*/
        msg.angle_max = msg.angle_min + (dataCount - 1) * msg.angle_increment;

        /* calculate statring data index and adjust angle_min to min_ang config param */
        int index_min = 0;
        while (msg.angle_min + msg.angle_increment < config.min_ang)
        {
            msg.angle_min += msg.angle_increment;
            index_min++;
        }
//    LOG(INFO) << "index_min: " << index_min << ", angle_min: " << msg.angle_min;
//        ROS_DEBUG("index_min: %d, angle_min: %f", index_min, msg.angle_min);

        /* calculate ending data index and adjust angle_max to max_ang config param */
        int index_max = dataCount - 1;
        while (msg.angle_max - msg.angle_increment > config.max_ang)
        {
            msg.angle_max -= msg.angle_increment;
            index_max--;
        }
//    LOG(INFO) << "index_max: " << index_max << ", angle_max: " << msg.angle_max;
//        ROS_DEBUG("index_max: %i, angle_max: %f", index_max, msg.angle_max);

        /*5: Fill data range*/
//    LOG(INFO) << "index_max - index_min + 1" << index_max - index_min + 1;
        msg.ranges.resize(index_max - index_min + 1);
        msg.ranges.assign(index_max - index_min + 1, std::numeric_limits<double>::infinity());
//        ROS_DEBUG("Fill sensor data. index_min = %d, index_max = %d.", index_min, index_max);
        for (int j = index_min; j <= index_max; ++j)
        {
            if(config.debug_mode)
            {
                if((j - index_min + 1) % 48 == 0)
                {
                    printf("\n");
                }
            }

            unsigned short range = pSensFrame->GetSensDataOfIndex(j);

            float meter_value = range / lidarDistenPara;
            if(meter_value > fRangeMin && meter_value < fRangeMax)
            {
                msg.ranges[j - index_min] = meter_value;
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

        if(config.intensity && pSensFrame->GetIntensityEnable())
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
//    msg.header.stamp = start_time - ros::Duration().fromSec(dataCount * msg.time_increment);
        /* - shit forward to time of first published scan point*/
//    msg.header.stamp += ros::Duration().fromSec((double)index_min * msg.time_increment);
        /* - add time offset (to account for USB latency etc.)*/
//    msg.header.stamp += ros::Duration().fromSec(config.time_offset);

        /*Consistency Check*/
        float expected_time_increment = msg.scan_time * msg.angle_increment / (2.0 * M_PI);
        if (fabs(expected_time_increment - msg.time_increment) > 0.00001)
        {
//            ROS_DEBUG_THROTTLE(60,
//                               "The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
//                               "Expected time_increment: %.9f, reported time_increment: %.9f. "
//                               "Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.",
//                               expected_time_increment,
//                               msg.time_increment);
        }

        if(pSensFrame)
        {
            delete pSensFrame;
        }

        return ExitSuccess;
    }

    void SetRangeMin(float minRange){
        fRangeMin = minRange;
    }
    void SetRangeMax(float maxRange){
        fRangeMax = maxRange;
    }
    void SetTimeIncrement(float time){
        fTimeIncrement = time;
    }
    void SetAngleResolution(float resolution){
        fangleResolution = resolution;
    }
    void SetFrameId(std::string str){
        fFrame_id = str;
    }

private:
    float fRangeMin;
    float fRangeMax;
    float fTimeIncrement;
    float fangleResolution;
    int   mSamplePointNum;
    std::string fFrame_id;
};
//typedef boost::shared_ptr<CSDKeliLs1207DEParser> CSDKeliLs1207DEParser_ptr;
// using CSDKeliLs1207DEParser_ptr = boost::shared_ptr<CSDKeliLs1207DEParser>;
} /*namespace sdkeli_lspdm_udp*/

#endif /*SDKELI_LS1207DE_PARSER__*/
