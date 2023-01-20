#include <sdkeli_klm_udp/sdkeli_klm_sensor_frame.h>
#include <sdkeli_klm_udp/sdkeli_klm_constants.h>

namespace sdkeli_klm_udp
{
CSDKeliKlmSensFrame::CSDKeliKlmSensFrame()
{
    mSensDataLength = 0;

    m_pSensData  = NULL;
    m_pSensData2  = NULL;

    m_angleRes = 0;
    m_sampleNum = 0;
    m_addIntensity = 0;
    m_timeSample = 0;
}

CSDKeliKlmSensFrame::~CSDKeliKlmSensFrame()
{
    if(m_pSensData != NULL)
    {
        delete m_pSensData;
    }

    if(m_pSensData2 != NULL)
    {
        delete m_pSensData2;
    }
}

int  CSDKeliKlmSensFrame::GetSensDataCount()
{
    return mSensDataLength;
}

int  CSDKeliKlmSensFrame::GetSensInfoAngleRes()
{
    return m_angleRes;
}

int  CSDKeliKlmSensFrame::GetSensInfoSampleNum()
{
    return m_sampleNum;
}

int  CSDKeliKlmSensFrame::GetSensInfoAddIntensity()
{
    return m_addIntensity;
}

int  CSDKeliKlmSensFrame::GetSensInfoTimeSample()
{
    return m_timeSample;
}

int  CSDKeliKlmSensFrame::GetSensInfoScanTime()
{
    return m_scanTime;
}

uint16_t CSDKeliKlmSensFrame::GetSensDataOfIndex(int index)
{
    if(index < 0 || index > mSensDataLength)
    {
        ROS_ERROR("Fail to get of index %d.", index);
        return 0;
    }

    return m_pSensData[index];
}

uint16_t CSDKeliKlmSensFrame::GetSensIntensityOfIndex(int index)
{
    if(index < 0 || index > mSensDataLength)
    {
        ROS_ERROR("Fail to get of index %d.", index);
        return 0;
    }

    return m_pSensData2[index];
}

bool CSDKeliKlmSensFrame::InitFromSensBuff(unsigned short *buff, int length,
        unsigned char *info, int info_length)
{
    if(buff == NULL)
    {
        ROS_ERROR("Invalide input buffer!");
        return false;
    }

    m_pSensData = new unsigned short[length];
    if(m_pSensData == NULL)
    {
        ROS_ERROR("Insufficiant memory!");
        return NULL;
    }

    m_pSensData2 = new unsigned short[length];
    if(m_pSensData2 == NULL)
    {
        ROS_ERROR("Insufficiant memory!");
        return NULL;
    }

    mSensDataLength = length;
    unsigned short *p_data = m_pSensData;
    for(int i = 0; i < length; i++)
    {
        *p_data++ = *buff++;
    }

    p_data = m_pSensData2;
    for(int i = 0; i < length; i++)
    {
        *p_data++ = *buff++;
    }

    m_angleRes = info[72] | info[73] << 8 | info[74] << 16 | info[75] << 24;
    m_sampleNum = info[76] | info[77] << 8;
    m_addIntensity = info[86];
    m_timeSample = info[100] | info[99] << 8 | info[98] << 16 | info[97] << 24;
    m_scanTime = info[94] | info[93] << 8 | info[92] << 16 | info[91] << 24;

    if(0x00 == m_scanTime)
    {
        m_scanTime = 400000;
    }

    ROS_DEBUG("m_angleRes = %d - %d -%d - %d -%d", m_angleRes, m_sampleNum, m_addIntensity, m_timeSample, m_scanTime);

    return true;
}
} /*namespace sdkeli_klm_udp*/
