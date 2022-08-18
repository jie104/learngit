#ifndef SDKELI_LS_SENSOR_FRAME__
#define SDKELI_LS_SENSOR_FRAME__

#include <stdio.h>
#include <stdlib.h>

//#include <ros/ros.h>

namespace sdkeli_lspdm_udp
{
class CSDKeliLsSensFrame
{
    struct SensData
    {
        uint8_t  header;
        uint8_t  cmd_id;
        uint16_t range_start;
        uint16_t range_end;
        uint8_t check_value;
        uint8_t add_info[128];
        uint16_t  sens_data[0];
    } __attribute__ ((packed));

public:
    CSDKeliLsSensFrame(){
        mSensDataLength = 0;
        m_pSensData  = NULL;
        mAngleResolution = 0.25;
        mNumSampePoint = 270 * 4 + 1;
        mHigePrecisionFlag = false;
    }
    ~CSDKeliLsSensFrame(){
        if(m_pSensData != NULL)
        {
            delete m_pSensData;
        }
    }

    bool InitFromSensBuff(char *buff, int length){
        int index = 0;
        uint16_t *tmp_buf = (uint16_t *)buff;
        if(buff == NULL)
        {
//            ROS_ERROR("Invalide input buffer!");
            return false;
        }

        char *pData = new char[length + 100];
        if(pData == NULL)
        {
//            ROS_ERROR("Insufficiant memory!");
            return NULL;
        }

        //memcpy(pData, buff, length);
        m_pSensData = new(pData) CSDKeliLsSensFrame::SensData;
        mSensDataLength = length;

        if(mSensDataLength == mNumSampePoint * 2)
        {
            mIntensityEnable = false;
            mAddInfoEnable = false;
        }
        else if(mSensDataLength == mNumSampePoint * 2 * 2)
        {
            mIntensityEnable = true;
            mAddInfoEnable = false;
        }
        else if(mSensDataLength == mNumSampePoint * 2 * 2 + 128)
        {
            mIntensityEnable = true;
            mAddInfoEnable = true;
        }
        else
        {
            mIntensityEnable = false;
            mAddInfoEnable = false;

//            ROS_ERROR("config angle_resolution in launch file err !");
            return NULL;
        }

        m_pSensData->range_start = 0;
        m_pSensData->range_end   = mNumSampePoint - 1;

        /*Switch sensor data*/
        if(mAddInfoEnable)
        {
            for(int k = 0; k < 128; k++)
            {
                m_pSensData->add_info[k] = buff[k];
            }

            if(m_pSensData->add_info[66])
            {
                mHigePrecisionFlag = true;
            }
            else
            {
                mHigePrecisionFlag = false;
            }

            tmp_buf = (uint16_t *)(buff + 128);
        }
        else
        {
            mHigePrecisionFlag = false;
            tmp_buf = (uint16_t *)buff;
        }

        index = 0;
        while(index < mNumSampePoint)
        {
            m_pSensData->sens_data[index] = SWITCH_UINT16(tmp_buf[index]);
            index ++;
        }

        if(mIntensityEnable)
        {
            while(index < mNumSampePoint * 2)
            {
                m_pSensData->sens_data[index] = SWITCH_UINT16(tmp_buf[index]);
                index ++;
            }
        }

        return true;
    }

    /*Get Frame Header*/
    uint8_t  GetFrameHeader(){
        return m_pSensData->header;
    }

    /*Get command Id*/
    uint8_t  GetCommandId(){
        return m_pSensData->cmd_id;
    }

    /*Get Range Start and Range End*/
    uint16_t GetRangeStart(){
        return m_pSensData->range_start;
    }
    uint16_t GetRangeEnd(){
        return m_pSensData->range_end;

    }

    /*Get sensor data count*/
    int      GetSensDataCount(){
        return m_pSensData->range_end - m_pSensData->range_start + 1;
    }

    /*Get sensor data of index*/
    uint16_t GetSensDataOfIndex(int index){
        if(index < 0 || index > (m_pSensData->range_end - m_pSensData->range_start))
        {
//            ROS_ERROR("Fail to get of index %d.", index);
            return 0;
        }

        return m_pSensData->sens_data[index];
    }

    /*Get sensor intensity of index*/
    uint16_t GetSensIntensityOfIndex(int index){
        uint16_t offsetAddr = m_pSensData->range_end - m_pSensData->range_start + 1 + index;

        if(index < 0 || index > (m_pSensData->range_end - m_pSensData->range_start))
        {
//            ROS_ERROR("Fail to get of index %d.", index);
            return 0;
        }

        return m_pSensData->sens_data[offsetAddr];
    }

    /*For debug only: Dump frame header.*/
    void     DumpFrameHeader(){
        if(m_pSensData == NULL || mSensDataLength == 0)
        {
            return;
        }

//        ROS_DEBUG("Frame Header: 0x%02X", this->GetFrameHeader());
//        ROS_DEBUG("Command   ID: 0x%02X", this->GetCommandId());
//        ROS_DEBUG("Angle  START: 0x%04X", this->GetRangeStart());
//        ROS_DEBUG("Angle    END: 0x%04X", this->GetRangeEnd());
    }

    /*For debug only: Dump frame data.*/
    void DumpFrameData(){
        if(m_pSensData == NULL || mSensDataLength == 0)
        {
            return;
        }

        int dataCount = this->GetSensDataCount();
//        ROS_DEBUG("Data   Count: %d", dataCount);

        int idx = 1;
        while(idx <= dataCount)
        {
            printf("%u ", static_cast<unsigned int>(this->GetSensDataOfIndex(idx - 1)));

            idx++;
            if(idx % 48 == 0)
            {
                printf("\n");
            }
        }
        printf("\n");
    }

    void SetAngleResolution(float angle_resolution){
        if(0.33 == angle_resolution || 0.25 == angle_resolution || 0.5 == angle_resolution)
        {
            mAngleResolution = angle_resolution;
        }
        else
        {
            mAngleResolution = 0.25;
        }
    }

    float GetAngleResolution(void){
        return mAngleResolution;
    }

    void SetNumSampePoint(int num){
        mNumSampePoint = num;
    }

    bool GetIntensityEnable(void){
        return mIntensityEnable;
    }

    bool GetHigePrecisionFlag(void){
        return mHigePrecisionFlag;
    }

private:
    SensData *m_pSensData;
    int       mSensDataLength;
    float     mAngleResolution;
    int       mNumSampePoint;
    bool      mIntensityEnable;
    bool      mAddInfoEnable;
    bool      mHigePrecisionFlag;

    bool CheckFrame(char *buff, int length, uint8_t value){
        int i = 0;
        uint8_t result = 0;
        bool checkframe;

        /* Get configure form launch script */
//        ros::param::get("~checkframe", checkframe);
        checkframe = true;
        if (checkframe == false)
        {
            /* Disable check frame function, default check true*/
            return true;
        }

        if (buff == NULL || length <= 0)
        {
            printf("CheckFrame: parameter failed\n");
            return false;
        }

        for (i = 0; i < length; i++)
        {
            result += (*buff++);
        }

        if (result == value)
        {
            return true;
        }

        printf("CheckFrame: check failed, length = %d, result = 0x%X, value = 0x%X\n",
               length, result, value);

        return false;
    }
};
}

#endif /*SDKELI_LS_SENSOR_FRAME__*/
