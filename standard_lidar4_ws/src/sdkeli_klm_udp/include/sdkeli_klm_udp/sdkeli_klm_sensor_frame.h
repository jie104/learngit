#ifndef SDKELI_KLM_SENSOR_FRAME__
#define SDKELI_KLM_SENSOR_FRAME__

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>

namespace sdkeli_klm_udp
{
class CSDKeliKlmSensFrame
{

public:
    CSDKeliKlmSensFrame();
    ~CSDKeliKlmSensFrame();

    bool     InitFromSensBuff(unsigned short *buff, int length, unsigned char *info, int info_length);

    /*Get sensor data count*/
    int      GetSensDataCount();

    int      GetSensInfoAngleRes();

    int     GetSensInfoSampleNum();

    int     GetSensInfoAddIntensity();

    int     GetSensInfoTimeSample();

    int     GetSensInfoScanTime();

    /*Get sensor data of index*/
    uint16_t GetSensDataOfIndex(int index);

    /*Get sensor intensity of index*/
    uint16_t GetSensIntensityOfIndex(int index);

private:
    unsigned short *m_pSensData;
    unsigned short *m_pSensData2;
    int            m_angleRes;
    int            m_sampleNum;
    int            m_addIntensity;
    int            m_timeSample;
    int            m_scanTime;

    int            mSensDataLength;
};
}

#endif /*SDKELI_KLM_SENSOR_FRAME__*/
