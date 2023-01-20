#include <sdkeli_klm_udp/sdkeli_klm_common.h>

#include <cstdio>
#include <cstring>

namespace sdkeli_klm_udp
{
CSDKeliKlmCommon::CSDKeliKlmCommon(CParserBase *parser, ros::NodeHandle &nh, ros::NodeHandle &pnh) :
    mDiagPublisher(NULL),
    dExpectedFreq(15.0), /* Default frequency */
    mParser(parser),
    mNodeHandler(nh),
    mPrivNodeHandler(pnh)
{
    /*Initialize receive buffer*/
    memset(mRecvBuffer, 0, CMD_FRAME_MAX_LEN);

    /*Set reconfigure callback*/
    dynamic_reconfigure::Server<sdkeli_klm_udp::SDKeliKlmConfig>::CallbackType f;
    f = boost::bind(&sdkeli_klm_udp::CSDKeliKlmCommon::UpdateConfig, this, _1, _2);
    mDynaReconfigServer.setCallback(f);

    /*Set data publisher (used for debug)*/
    mPrivNodeHandler.param<bool>("publish_datagram", mPublishData, false);
    if(mPublishData)
    {
        /*datagram publish is enabled*/
        mDataPublisher = mNodeHandler.advertise<std_msgs::String>("datagram", 1000);
    }

    /*Set scan publisher*/
    mScanPublisher = mNodeHandler.advertise<sensor_msgs::LaserScan>("scan", 1000);

    mDiagUpdater.setHardwareID("none");
    mDiagPublisher = new diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan>(mScanPublisher,
            mDiagUpdater,
            /* frequency should be target +- 10% */
            diagnostic_updater::FrequencyStatusParam(&dExpectedFreq, &dExpectedFreq, 0.1, 10),
            /*timestamp delta can be from 1.1 to 1.3x what it ideally is*/
            diagnostic_updater::TimeStampStatusParam(-1, 1.3 * 1.0 / dExpectedFreq - mConfig.time_offset));

    ROS_ASSERT(mDiagPublisher);
}

int CSDKeliKlmCommon::StopScanner()
{
    int result = 0;
    result = SendDeviceReq(CMD_STOP_STREAM_DATA, NULL);
#ifdef CMD_STOP_STREAM_DATA /* TODO: Enable following code block when stop command defined. */
    result = SendDeviceReq(CMD_STOP_STREAM_DATA, NULL);
    if(0 != result)
    {
        // use printf because we couldn't use ROS_ERROR from destructor
        printf("STOP Scan ERROR!\n");
    }
    else
    {
        printf("Streaming scan data stopped.\n");
    }
#endif
    return result;
}

bool CSDKeliKlmCommon::RebootDevice()
{
#ifdef CMD_REBOOT_DEVICE /*TODO: Enable following code block when commands defined.*/
    /*Set maintenance access mode to allow reboot to be sent*/
    std::vector<unsigned char> respAccess;
    int result = SendDeviceReq(CMD_SET_MAINTENANCE_ACCESS_MODE, &respAccess);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_KLM - Error setting access mode");
        mDiagUpdater.broadcast(diagnostic_msgs::DisgnosticStatus::ERROR,
                               "SDKELI_KLM - Error setting access mode");

        return false;
    }

    std::string strAccessResp = StringResp(respAccess);
    if(strAccessResp != "sAN SetAccessMode 1")
    {
        ROS_ERROR_STREAM("SDKELI_KLM - Error setting access mode, unexpected response : " << strAccessResp);
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI - Error setting access mode.");

        return false;
    }

    /*send reboot command*/
    std::vector<unsigned char> respReboot
    result = SendDeviceReq(CMD_REBOOT, &respReboot);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_KLM - Error rebooting device");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_KLM - Error rebooting device");

        return false;
    }

    std::string strRebootResp = StringResp(respReboot);
    if(strRebootResp != "sAN mSCreboot")
    {
        ROS_ERROR_STREAM("SDKELI_KLM - Error setting access mode, unexpected response : " << strRebootResp);
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_KLM - Error rebooting device");

        return false;
    }

    ROS_INFO("SDKELI_KLM - Rebooted scanner");
#endif
    return true;
}

CSDKeliKlmCommon::~CSDKeliKlmCommon()
{
    delete mDiagPublisher;
    ROS_INFO("sdkeli_klm_udp drvier exiting.\n");
}

int CSDKeliKlmCommon::Init()
{
    int result = InitDevice();
    if(0 != result)
    {
        ROS_FATAL("Failed to init device: %d", result);
        return result;
    }

    result = InitScanner();
    if(0 != result)
    {
        ROS_FATAL("Failed to init scanner: %d", result);
    }

    return result;
}

int CSDKeliKlmCommon::InitScanner()
{
    SendDeviceReq(CMD_START_STREAM_DATA, NULL);
#ifdef CMD_DEVICE_INFO /*TODO: Enable following code block when command defined*/
    /*Read device identify*/
    std::vector<unsigned char> respIdentify;
    int result = SendDeviceReq(CMD_READ_IDENTIFY, &respIdentify);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_KLM - Error reading variable 'DeviceIdent'.");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_KLM - Error reading variable 'DeviceIdent'.");
    }

    /*Read device variable 'SerialNumber' by name.*/
    std::vector<unsigned char> respSerialNumber;
    result = SendDeviceReq(CMD_READ_SERIAL_NUMBER, &respSerialNumber);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_KLM - Error reading variable 'SerialNumber'.");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_KLM - Error reading variable 'SerialNumber'.");
    }

    /*Set hardware ID based on device identify and serial number*/
    std::string strIdentify     = StringResponse(respIdentify);
    std::string strSerialNumber = StringResponse(respSerialNumber);
    mDiagUpdater.setHardwareID(strIdentify + " " + strSerialNumber);

    if(!IsCompatibleDevice(strIdentify))
    {
        ROS_ERROR("SDKELI_KLM - Error Unsuppored identify %s", strIdentify);
        return ExitFatal;
    }

    /*Read device variable 'FirmwareVersion' by name.*/
    result = SendDeviceReq(CMD_READ_FIRMWARE_VERSION, NULL);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_KLM - Error reading variable 'FirmwareVersion'.");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_LS - Error reading variable 'FirmwareVersion'.");
    }

    /*Read Device State*/
    std::vector<unsigned char> respDeviceState;
    result = SendDeviceReq(CMD_READ_DEVICE_STATE, &respDeviceState);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_KLM - Error reading variable 'devicestate'.");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_KLM - Error reading variable 'devicestate'.");
    }
    std::string strDeviceState = StringResponse(respDeviceState);

    /*Check device state:
     * 0: Busy,
     * 1: Ready,
     * 2: Error */
    if(strDeviceState == "sRA SCdevicestate 0")
    {
        ROS_WARN("Laser scanner is busy.");
    }
    else if(strDeviceState == "sRA SCdevicestate 1")
    {
        ROS_DEBUG("Laser scanner is ready.");
    }
    else if(strDeviceState == "sRA SCdevicedstate 2")
    {
        ROS_ERROR_STREAM("Laser scanner error state: " << strDeviceState);
        if(mConfig.auto_reboot)
        {
            rebootDevice();
        }
    }
    else
    {
        ROS_WARN_STREAM("Laser scanner reports unknown devicestate: " << strDeviceState);
    }

    /*Start data streaming*/
    result = SendDeviceReq(CMD_START_STREAM_DATA, NULL);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_LS - Error when starting streaming 'LMDscandata'.");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_KLM - Error when starting streaming 'LMDscandata'.");

        return ExitError;
    }
#endif
    return ExitSuccess;
}

std::string CSDKeliKlmCommon::StringResp(const std::vector<unsigned char> &resp)
{
    std::string strResp;
    for(std::vector<unsigned char>::const_iterator it = resp.begin();
            it != resp.end();
            it++)
    {
        if(*it > 13)
        {
            strResp.push_back(*it);
        }
    }

    return strResp;
}

bool CSDKeliKlmCommon::IsCompatibleDevice(const std::string strIdentify) const
{
    // TODO: Always return true
    return true;
}

int CSDKeliKlmCommon::LoopOnce()
{
    unsigned char header[4] = {0xFA, 0x5A, 0xA5, 0xAA};
    static unsigned int lastFrameTotalIndex = 0xFFFFFFFF;
    unsigned int n, totalDataLen, addInfoLen;
    unsigned char *p_data, *p_info;


    mDiagUpdater.update();

    int dataLength = 0;
    static unsigned int iteration_count = 0;

    int result = GetDataGram(mRecvBuffer, CMD_FRAME_MAX_LEN, &dataLength);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_KLM - Read Error when getting datagram: %d", result);
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_KLM - Read Error when getting datagram.");

        return ExitError;
    }
    else
    {
        ROS_DEBUG("SDKELI_KLM - Received data gram. Data Length %d", dataLength);

        //ROS_INFO_ONCE("SDKELI_LS - Successfully connected !!");
        if(!mConnectFlag)
        {
            mConnectFlag = 1;
            ROS_INFO("SDKELI_KLM - Successfully connected !!");
        }

        if(memcmp(mRecvBuffer, header, sizeof(header)) == 0)    //compare the header                     //compare the header
        {
#if(0)
            ROS_INFO("%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", mRecvBuffer[0], mRecvBuffer[1], \
                     mRecvBuffer[2], mRecvBuffer[3], mRecvBuffer[4], mRecvBuffer[5], mRecvBuffer[6], mRecvBuffer[7], mRecvBuffer[8], mRecvBuffer[9], \
                     mRecvBuffer[10], mRecvBuffer[11], mRecvBuffer[12]);
#endif
            int rawDatalen = ((*(mRecvBuffer + CMD_FRAME_HEADER_LENGTH_H) << 8) | (*(mRecvBuffer + CMD_FRAME_HEADER_LENGTH_L)))
                             - (CMD_FRAME_DATA_START - CMD_FRAME_HEADER_CHECK_L);                            //raw data length

            unsigned int frameTotalIndex = (*(mRecvBuffer + CMD_FRAME_HEADER_TOTAL_INDEX_H) << 8)
                                           | (*(mRecvBuffer + CMD_FRAME_HEADER_TOTAL_INDEX_L));          //current totalIndex

            unsigned char subPkgNum = *(mRecvBuffer + CMD_FRAME_HEADER_SUB_PKG_NUM);
            unsigned char subPkgIndex = *(mRecvBuffer + CMD_FRAME_HEADER_SUB_INDEX);  //current subPkgIndex
            unsigned char subPkgType = *(mRecvBuffer + CMD_FRAME_HEADER_SUB_TYPE);    //current subPkgType

            unsigned short checkSum = 0;    //checkSunm
            for(int i = 0; i < rawDatalen + (CMD_FRAME_DATA_START - CMD_FRAME_HEADER_TYPE); i++)  //add sum
            {
                checkSum += *(mRecvBuffer + CMD_FRAME_HEADER_TYPE + i);
            }
            if(checkSum != ((*(mRecvBuffer + CMD_FRAME_HEADER_CHECK_H) << 8) | (*(mRecvBuffer + CMD_FRAME_HEADER_CHECK_L))))    //check sum
            {
                memset(mRecvBuffer, 0, CMD_FRAME_MAX_LEN);
                ROS_WARN("checkSum error");
                return ExitSuccess;
            }

            if(dataLength != (rawDatalen + CMD_FRAME_DATA_START) || dataLength > CMD_FRAME_MAX_LEN)     //the datalength received is not the same as the package length.
            {
                memset(mRecvBuffer, 0, CMD_FRAME_MAX_LEN);
                ROS_WARN("dataLength is error");
                return ExitSuccess;
            }

            if(subPkgNum > CMD_FRAME_MAX_SUB_PKG_NUM || subPkgNum < CMD_FRAME_MIN_SUB_PKG_NUM || subPkgIndex > CMD_FRAME_MAX_SUB_PKG_NUM - 1)
            {
                memset(mRecvBuffer, 0, CMD_FRAME_MAX_LEN);
                ROS_WARN("dataLength is error");
                return ExitSuccess;
            }

            ROS_DEBUG("%d-%d%d%d%d", frameTotalIndex, subPkgNum, subPkgIndex, subPkgType, rawDatalen);

            mDataSaveSt[subPkgIndex].totaIndexlCount = frameTotalIndex;
            mDataSaveSt[subPkgIndex].subPkgNum = subPkgNum;
            mDataSaveSt[subPkgIndex].subPkgIndex = subPkgIndex;
            mDataSaveSt[subPkgIndex].subPkgType = subPkgType;
            mDataSaveSt[subPkgIndex].rawDataLen = rawDatalen;
            memcpy(mDataSaveSt[subPkgIndex].sens_data, mRecvBuffer + CMD_FRAME_DATA_START, rawDatalen);

            bool checkResult = false;
            for(n = 0; n < subPkgNum - 1; n++)
            {
                if(mDataSaveSt[n].totaIndexlCount != mDataSaveSt[n + 1].totaIndexlCount || \
                        mDataSaveSt[n].subPkgIndex != mDataSaveSt[n + 1].subPkgIndex - 1)
                {
                    checkResult = true;
                    break;
                }
            }

            if(checkResult == true)
            {
                //ROS_WARN("data rev not complete !!");
                return ExitSuccess;
            }

            totalDataLen = 0;
            addInfoLen = 0;
            p_data = (unsigned char *)mStoreBuffer;
            p_info = (unsigned char *)mStoreBuffer2;
            for(n = 0; n < subPkgNum; n++)
            {
                if((0x01 == mDataSaveSt[n].subPkgType) || (0x02 == mDataSaveSt[n].subPkgType))
                {
                    memcpy((unsigned char *)(p_data + totalDataLen), mDataSaveSt[n].sens_data, mDataSaveSt[n].rawDataLen);
                    totalDataLen += mDataSaveSt[n].rawDataLen;
                }
                else if(0x03 == mDataSaveSt[n].subPkgType)
                {
                    memcpy((unsigned char *)(p_info + addInfoLen), mDataSaveSt[n].sens_data, mDataSaveSt[n].rawDataLen);
                    addInfoLen += mDataSaveSt[n].rawDataLen;
                }
            }

            ROS_DEBUG("totalDataLen = %d- addInfoLen = %d", totalDataLen, addInfoLen);

            if(frameTotalIndex != lastFrameTotalIndex + 1 && frameTotalIndex != 0)
            {
                ROS_WARN("frameTotalIndex:%d is out-of-order, last is:%d", frameTotalIndex, lastFrameTotalIndex);
            }

            lastFrameTotalIndex = frameTotalIndex;
            memset(mRecvBuffer, 0, CMD_FRAME_MAX_LEN);
        }
        else    //header error
        {
            memset(mRecvBuffer, 0, CMD_FRAME_MAX_LEN);
            ROS_WARN("command header is error!!");
            return ExitSuccess;
        }
    }

#if 0
    if(totalDataLen < FRAME_LENGTH || totalDataLen % FRAME_LENGTH) /*Fixed data length of 1630*/
    {
        ROS_ERROR("SDKELI_LS - Invalid data length!");
        memset(mDataSaveSt, 0, sizeof(mDataSaveSt));
        return ExitSuccess; /*return success to continue looping*/
    }
#endif

    /*Data requested, skip frames*/
    if(iteration_count++ % (mConfig.skip + 1) != 0)
    {
        ROS_DEBUG("SDKELI_KLM - Skip frame");
        return ExitSuccess;
    }

#if(0)
    /*One full frame received. Start Data processing...*/
    if(mPublishData)
    {
        std_msgs::String data_msg;
        data_msg.data = std::string(reinterpret_cast<char *>(mRecvBuffer));
        mDataPublisher.publish(data_msg);
    }
#endif

    sensor_msgs::LaserScan msg;
    int success = mParser->Parse(mStoreBuffer, (totalDataLen >> 2), mStoreBuffer2, addInfoLen, mConfig, msg);
    if(ExitSuccess == success)
    {
        if(mConfig.debug_mode)
        {
            DumpLaserMessage(msg);
        }
        mDiagPublisher->publish(msg);
    }

    memset(mStoreBuffer, 0, RECV_BUFFER_SIZE);

    return ExitSuccess; // return success to continue
}

void CSDKeliKlmCommon::CheckAngleRange(sdkeli_klm_udp::SDKeliKlmConfig &config)
{
    if(config.min_ang > config.max_ang)
    {
        ROS_WARN("Minimum angle must be greater than maxmum angle. Adjusting min_ang");
        config.min_ang = config.max_ang;
    }
}

void CSDKeliKlmCommon::UpdateConfig(sdkeli_klm_udp::SDKeliKlmConfig &newConfig, uint32_t level)
{
    CheckAngleRange(newConfig);
    mConfig = newConfig;
}

void CSDKeliKlmCommon::DumpLaserMessage(sensor_msgs::LaserScan &msg)
{
    ROS_DEBUG("Laser Message to send:");
    ROS_DEBUG("Header  frame_id: %s", msg.header.frame_id.c_str());
    //ROS_DEBUG("Header timestamp: %ld", msg.header.stamp);
    ROS_DEBUG("angle_min: %f", msg.angle_min);
    ROS_DEBUG("angle_max: %f", msg.angle_max);
    ROS_DEBUG("angle_increment: %f", msg.angle_increment);
    ROS_DEBUG("time_increment: %f", msg.time_increment);
    ROS_DEBUG("scan_time: %f", msg.scan_time);
    ROS_DEBUG("range_min: %f", msg.range_min);
    ROS_DEBUG("range_max: %f", msg.range_max);
}

void CSDKeliKlmCommon::ClearConnectFlag(void)
{
    mConnectFlag = 0;
}

} // sdkeli_klm_udp
