#ifndef SDKELI_LS_COMMON_H_
#define SDKELI_LS_COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <vector>

//#include <ros/ros.h>
//#include <sensor_msgs/LaserScan.h>
//#include <std_msgs/String.h>

//#include <diagnostic_updater/diagnostic_updater.h>
//#include <diagnostic_updater/publisher.h>
//
//#include <dynamic_reconfigure/server.h>
//#include <sdkeli_lspdm_udp/SDKeliLspdmConfig.h>

#include "sdkeli_ls_constants.h"
//#include "parser_base.h"
#include "sdkeli_ls1207de_parser.hpp"
#include "glog/logging.h"

/*Fixed received buffer size*/
#define RECV_BUFFER_SIZE                65536

#define CMD_FRAME_HEADER_START          0

#define CMD_FRAME_HEADER_LENGTH_H       4
#define CMD_FRAME_HEADER_LENGTH_L       5
#define CMD_FRAME_HEADER_CHECK_SUM      6
#define CMD_FRAME_HEADER_TYPE           7
#define CMD_FRAME_HEADER_TOTAL_INDEX_H  8
#define CMD_FRAME_HEADER_TOTAL_INDEX_L  9
#define CMD_FRAME_HEADER_SUB_PKG_NUM    10  /* �������Ӱ���Ŀ */
#define CMD_FRAME_HEADER_SUB_INDEX      11
#define CMD_FRAME_DATA_START            12

#define INDEX_RANGE_MAX                 65415       //65535-(20*60)
#define INDEX_RANGE_MIN                 (20*60)     //60 sencod

#define CMD_FRAME_MAX_LEN               1500
#define CMD_FRAME_MAX_SUB_PKG_NUM       4
#define CMD_FRAME_MIN_SUB_PKG_NUM       2


namespace sdkeli_lspdm_udp
{
class CSDKeliLsCommon
{
    struct dataSaveSt
    {
        unsigned int   totaIndexlCount;
        unsigned char  subPkgNum;
        unsigned char  subPkgIndex;
        unsigned int   rawDataLen;
        unsigned char  sens_data[CMD_FRAME_MAX_LEN];
    };

public:
    CSDKeliLsCommon(CSDKeliLs1207DEParser *parser):
            dExpectedFreq(15.0), /* Default frequency */
            mParser(parser) {
        /*Initialize receive buffer*/
        memset(mRecvBuffer, 0, RECV_BUFFER_SIZE);
        mDataLength = 0;

        sdkeli_lspdm_udp::LidarConfig config;
        UpdateConfig(config, -1);//level在函数中没有用到
    }

    virtual ~CSDKeliLsCommon(){
//        ROS_INFO("sdkeli_lspdm_udp drvier exiting.\n");
    }
    virtual bool Init(){
        int result = InitDevice();//虚函数，初始化socket, address之类
        if(0 != result)
        {
//            ROS_FATAL("Failed to init device: %d", result);
            LOG(INFO) << "Failed to init device";
            return false;
        }else{
            LOG(INFO) << "Init device successfully!";
        }
        result = InitScanner();//向设备发送初始化指令，开始接受stream
        if(0 != result)
        {
            setConnectFlag(false);
            LOG(INFO) << "Failed to init scanner";
//            ROS_FATAL("Failed to init scanner: %d", result);
        }else{
            setConnectFlag(true);
            LOG(INFO) << "Init scanner successfully!";
        }
        return mConnectFlag;
    }
//    template<class ScanMsg>
//    int          LoopOnce(ScanMsg& scan);
    template <class ScanMsg>
    int LoopOnce(ScanMsg& msg)
    {

//        LOG(INFO) << "Loop Once";
        unsigned char header[4] = {0xFA, 0x5A, 0xA5, 0xAA};
        static unsigned int lastFrameTotalIndex = 0xFFFFFFFF;
        unsigned int n, totalDataLen;

//    mDiagUpdater.update();

        int dataLength = 0;
        static unsigned int iteration_count = 0;
        //纯虚函数，由继承类实现
        int result = GetDataGram(mRecvBuffer, RECV_BUFFER_SIZE, &dataLength);//从设备获取报文，dataLength为实际报文的大小
        if(0 != result)
        {
            LOG(INFO) << "SDKELI_LS - Read Error when getting datagram: " << result;
//            ROS_ERROR("SDKELI_LS - Read Error when getting datagram: %d", result);
//        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
//                               "SDKELI_LS - Read Error when getting datagram.");

            return ExitError;
        }
        else
        {
//            LOG(INFO) << "SDKELI_LS - Received data gram. Data Length " << dataLength;
//            ROS_DEBUG("SDKELI_LS - Received data gram. Data Length %d", dataLength);

            //ROS_INFO_ONCE("SDKELI_LS - Successfully connected !!");
//            if(!mConnectFlag)
//            {
//                mConnectFlag = 1;
//                LOG(INFO) << "SDKELI_LS - Successfully connected !!";
////                ROS_INFO("SDKELI_LS - Successfully connected !!");
//            }

            if(memcmp(mRecvBuffer, header, sizeof(header)) == 0) //两个地址开始的固定字节长度的区域进行比较   //compare the header                     //compare the header
            {
#if(0)
                ROS_INFO("%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-", *(mRecvBuffer + mDataLength + 0), *(mRecvBuffer + mDataLength + 1), *(mRecvBuffer + mDataLength + 2),   \
                     * (mRecvBuffer + mDataLength + 3), *(mRecvBuffer + mDataLength + 4), *(mRecvBuffer + mDataLength + 5), *(mRecvBuffer + mDataLength + 6), *(mRecvBuffer + mDataLength + 7),          \
                     * (mRecvBuffer + mDataLength + 8), *(mRecvBuffer + mDataLength + 9), *(mRecvBuffer + mDataLength + 10), *(mRecvBuffer + mDataLength + 11));
#endif
                //从接收到的数据中，读取他的原始数据，帧序号，总子packagea数，当前子package序号
                int rawDatalen = ((*(mRecvBuffer + CMD_FRAME_HEADER_LENGTH_H) << 8) | (*(mRecvBuffer + CMD_FRAME_HEADER_LENGTH_L)))
                                 - (CMD_FRAME_DATA_START - CMD_FRAME_HEADER_CHECK_SUM);                            //raw data length

                unsigned int frameTotalIndex = (*(mRecvBuffer + CMD_FRAME_HEADER_TOTAL_INDEX_H) << 8)
                                               | (*(mRecvBuffer + CMD_FRAME_HEADER_TOTAL_INDEX_L));          //current totalIndex

                unsigned char subPkgNum = *(mRecvBuffer + CMD_FRAME_HEADER_SUB_PKG_NUM);
                unsigned char subPkgIndex = *(mRecvBuffer + CMD_FRAME_HEADER_SUB_INDEX);    //current subPkgIndex
                //校验，各种异常的处理
                unsigned char checkSum = 0;                                                         //checkSunm
                for(int i = 0; i < rawDatalen + (CMD_FRAME_DATA_START - CMD_FRAME_HEADER_TYPE); i++)       //add sum
                {
                    checkSum += *(mRecvBuffer + CMD_FRAME_HEADER_TYPE + i);
                }
                if(checkSum != *(mRecvBuffer + CMD_FRAME_HEADER_CHECK_SUM))                 //check sum
                {
                    memset(mRecvBuffer, 0, RECV_BUFFER_SIZE);
                    //mDataLength = 0;
                    LOG(INFO) << "checkSum error";
//                    ROS_WARN("checkSum error");
                    return ExitSuccess;
                }

                if(dataLength != (rawDatalen + CMD_FRAME_DATA_START) || dataLength > CMD_FRAME_MAX_LEN)                                      //the datalength received is not the same as the package length.
                {
                    memset(mRecvBuffer, 0, RECV_BUFFER_SIZE);
                    //mDataLength = 0;
                    LOG(INFO) << "dataLength is error";
//                    ROS_WARN("dataLength is error");
                    return ExitSuccess;
                }

                if(subPkgNum > CMD_FRAME_MAX_SUB_PKG_NUM || subPkgNum < CMD_FRAME_MIN_SUB_PKG_NUM || subPkgIndex > CMD_FRAME_MAX_SUB_PKG_NUM - 1)
                {
                    memset(mRecvBuffer, 0, RECV_BUFFER_SIZE);
                    //mDataLength = 0;
//                LOG(INFO) << "dataLength is error";
//                    ROS_WARN("dataLength is error");
                    return ExitSuccess;
                }
//            LOG(INFO) << "SubPkgIndex: " << int(subPkgIndex);//四个包合成一个scan
                mDataSaveSt[subPkgIndex].totaIndexlCount = frameTotalIndex;//帧的编号
//            LOG(INFO) << "totaIndexlCount" << frameTotalIndex;
                mDataSaveSt[subPkgIndex].subPkgNum = subPkgNum;//4
                mDataSaveSt[subPkgIndex].subPkgIndex = subPkgIndex;//subPkgIndex标志了这个数据包0~3
                mDataSaveSt[subPkgIndex].rawDataLen = rawDatalen;
                memcpy(mDataSaveSt[subPkgIndex].sens_data, mRecvBuffer + CMD_FRAME_DATA_START, rawDatalen);

                bool checkResult = false;

                for(n = 0; n < subPkgNum - 1; n++)
                {
                    if(mDataSaveSt[n].totaIndexlCount != mDataSaveSt[n + 1].totaIndexlCount || \
                        mDataSaveSt[n].subPkgIndex != mDataSaveSt[n + 1].subPkgIndex - 1)//如果四个位置的帧编号不一致或者子package序号不是依次递增的，说明旧的还没有被覆盖掉，还没有凑够四个
                    {
                        checkResult = true;
                        break;
                    }
                }

                if(checkResult == true)
                {
                    //ROS_WARN("data rev not complete !!");
//                    LOG(INFO) << "data rev not complete： subPkgIndex " << int(subPkgIndex);
                    return ExitSuccess;
                }

                totalDataLen = 0;//所有原始数据的总大小
                for(n = 0; n < subPkgNum; n++)//把原始数据从datasave拷贝到storebuffer
                {
                    memcpy(mStoreBuffer + totalDataLen, mDataSaveSt[n].sens_data, mDataSaveSt[n].rawDataLen);//把raw数据提出来保存到mStoreBuffer
                    totalDataLen += mDataSaveSt[n].rawDataLen;
                }

                if(frameTotalIndex != lastFrameTotalIndex + 1 && frameTotalIndex != 0)
                {
//                    ROS_DEBUG("frameTotalIndex:%d is out-of-order, last is:%d", frameTotalIndex, lastFrameTotalIndex);
                }

                lastFrameTotalIndex = frameTotalIndex;

                memset(mRecvBuffer, 0, RECV_BUFFER_SIZE);
            }
            else//head不对                                                                                    //header error
            {
                memset(mRecvBuffer, 0, RECV_BUFFER_SIZE);
                //mDataLength = 0;
//                ROS_WARN("command header is error!!");
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
            LOG(INFO) << "SDKELI_LS - Skip frame";
//            ROS_INFO("SDKELI_LS - Skip frame");
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
//    sensor_msgs::LaserScan msg;

        /* data length is fixed size: 1622* N */
        char *posBuffer = (char *)mStoreBuffer;
        int  startIndex = 0;
        char *start = (char *)mStoreBuffer;  //strchr(posBuffer + startIndex, STX);
        if(start == NULL)
        {
//            ROS_ERROR("SDKELI_LS - Invalide data! Header NOT found!");
        }

        char *end   = NULL;
//    static int test = 0;
        //解析数据
        while(start != NULL && (startIndex + totalDataLen <= totalDataLen))//只可能执行一次？？是的
        {
//            LOG(INFO) << "Start sparsing!" << std::endl;
//        test++;
//        LOG(INFO) << "TotalDataLen: " << totalDataLen;
            size_t length = totalDataLen;
            end = start + length; /*Fixed total length 1631*/
            *end = '\0';
            int success = mParser->Parse(start, length, mConfig, msg);//从start开始解析length数据成msg
//            LOG(INFO) << "Parse status: " << success << ", msg size: " << msg.ranges.size() << std::endl;
            if(ExitSuccess == success)
            {
                if(mConfig.debug_mode)
                {
                    DumpLaserMessage(msg);
                }
//            mDiagPublisher->publish(msg);//四次loop发布一次msg
//            std::cout << "scan_size: " << msg.ranges.size() << std::endl;
//            std::cout << "angle_increment: " << msg.angle_increment/M_PI*180 << std::endl;
//            std::cout << "angle_min: " << msg.angle_min/M_PI*180 << std::endl;
//            std::cout << "time_increment: " << msg.time_increment << std::endl;
//            std::cout << "range_max: " << msg.range_max << std::endl;
//            std::cout << msg.ranges[541] << std::endl;
//            static std::vector<float> ranges;
//            static int count = 0;
//            if(count < 10){
//                ranges.push_back(msg.ranges[541]);
//                count++;
//            }else{
//                for(int i = 0; i < ranges.size(); ++i){
//                    std::cout << ranges[i] << std::endl;
//                }
//                ranges.clear();
//                count = 0;
//            }
            }

            //posBuffer = end + 1;
            //start     = strchr(posBuffer, STX);
            startIndex += totalDataLen;
//        LOG(INFO) << "StartIndex: " << startIndex;
        }

        memset(mStoreBuffer, 0, RECV_BUFFER_SIZE);//被解析器用来解析数据的空间
        //mDataLength = 0;
        return ExitSuccess; // return success to continue
    }
    void CheckAngleRange(sdkeli_lspdm_udp::LidarConfig &config){
        if(config.min_ang > config.max_ang)
        {
//            ROS_WARN("Minimum angle must be greater than maxmum angle. Adjusting min_ang");
            config.min_ang = config.max_ang;
        }
    }
    void UpdateConfig(sdkeli_lspdm_udp::LidarConfig &newConfig, uint32_t level = 0){
        CheckAngleRange(newConfig);
        mConfig = newConfig;
    }
    virtual bool RebootDevice(){
#ifdef CMD_REBOOT_DEVICE /*TODO: Enable following code block when commands defined.*/
        /*Set maintenance access mode to allow reboot to be sent*/
    std::vector<unsigned char> respAccess;
    int result = SendDeviceReq(CMD_SET_MAINTENANCE_ACCESS_MODE, &respAccess);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_LS - Error setting access mode");
        mDiagUpdater.broadcast(diagnostic_msgs::DisgnosticStatus::ERROR,
                               "SDKELI_LS - Error setting access mode");

        return false;
    }

    std::string strAccessResp = StringResp(respAccess);
    if(strAccessResp != "sAN SetAccessMode 1")
    {
        ROS_ERROR_STREAM("SDKELI_LS - Error setting access mode, unexpected response : " << strAccessResp);
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI - Error setting access mode.");

        return false;
    }

    /*send reboot command*/
    std::vector<unsigned char> respReboot
    result = SendDeviceReq(CMD_REBOOT, &respReboot);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_LS - Error rebooting device");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_LS - Error rebooting device");

        return false;
    }

    std::string strRebootResp = StringResp(respReboot);
    if(strRebootResp != "sAN mSCreboot")
    {
        ROS_ERROR_STREAM("SDKELI_LS - Error setting access mode, unexpected response : " << strRebootResp);
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_LS - Error rebooting device");

        return false;
    }

    ROS_INFO("SDKELI_LS - Rebooted scanner");
#endif
        return true;
    }

    double       GetExpectedFreq() const
    {
        return dExpectedFreq;
    }

protected:
    virtual int InitDevice() = 0;
    virtual int InitScanner(){
        SendDeviceReq(CMD_START_STREAM_DATA, NULL);
#ifdef CMD_DEVICE_INFO /*TODO: Enable following code block when command defined*/
        /*Read device identify*/
    std::vector<unsigned char> respIdentify;
    int result = SendDeviceReq(CMD_READ_IDENTIFY, &respIdentify);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_LS - Error reading variable 'DeviceIdent'.");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_LS - Error reading variable 'DeviceIdent'.");
    }

    /*Read device variable 'SerialNumber' by name.*/
    std::vector<unsigned char> respSerialNumber;
    result = SendDeviceReq(CMD_READ_SERIAL_NUMBER, &respSerialNumber);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_LS - Error reading variable 'SerialNumber'.");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_LS - Error reading variable 'SerialNumber'.");
    }

    /*Set hardware ID based on device identify and serial number*/
    std::string strIdentify     = StringResponse(respIdentify);
    std::string strSerialNumber = StringResponse(respSerialNumber);
    mDiagUpdater.setHardwareID(strIdentify + " " + strSerialNumber);

    if(!IsCompatibleDevice(strIdentify))
    {
        ROS_ERROR("SDKELI_LS - Error Unsuppored identify %s", strIdentify);
        return ExitFatal;
    }

    /*Read device variable 'FirmwareVersion' by name.*/
    result = SendDeviceReq(CMD_READ_FIRMWARE_VERSION, NULL);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_LS - Error reading variable 'FirmwareVersion'.");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_LS - Error reading variable 'FirmwareVersion'.");
    }

    /*Read Device State*/
    std::vector<unsigned char> respDeviceState;
    result = SendDeviceReq(CMD_READ_DEVICE_STATE, &respDeviceState);
    if(0 != result)
    {
        ROS_ERROR("SDKELI_LS - Error reading variable 'devicestate'.");
        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "SDKELI_LS - Error reading variable 'devicestate'.");
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
                               "SDKELI_LS - Error when starting streaming 'LMDscandata'.");

        return ExitError;
    }
#endif
        return ExitSuccess;
    }
public:
    virtual int StopScanner(){
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
        LOG(INFO) << "Stop Scanner!" << std::endl;
        return result;
    }
    virtual int CloseDevice() = 0;

    /*Send command/message to the device and print out the response to the console*/
    /**
     * \param [in]  req the command to send
     * \param [out] resp if not NULL, will be filled with the response package to the command.
     */
    virtual int SendDeviceReq(const unsigned char *req, std::vector<unsigned char> *resp) = 0;

    /*Read a datagram from the device*/
    /*
     * \param [in]  receiveBuffer data buffer to fill.
     * \param [in]  bufferSize max data size to buffer (0 terminated).
     * \param [out] length the actual amount of data written.
     * */
    virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize, int *length) = 0;

    /*Helper function. Conver response of SendDeviceReq to string*/
    /*
     * \param [in] resp the response from SendDeviceReq
     * \returns response as string with special characters stripped out
     * */
    static std::string StringResp(const std::vector<unsigned char> &resp);

    /*Check the given device identify is supported by this driver*/
    /*
     * \param [in] strIdentify the identifier of the dvice.
     * \return indicate wether it's supported by this driver
     * */
    bool IsCompatibleDevice(const std::string strIdentify) const;

    template <class Scan>
    void DumpLaserMessage(const Scan &msg){
//        ROS_DEBUG("Laser Message to send:");
//        ROS_DEBUG("Header  frame_id: %s", msg.header.frame_id.c_str());
//        //ROS_DEBUG("Header timestamp: %ld", msg.header.stamp);
//        ROS_DEBUG("angle_min: %f", msg.angle_min);
//        ROS_DEBUG("angle_max: %f", msg.angle_max);
//        ROS_DEBUG("angle_increment: %f", msg.angle_increment);
//        ROS_DEBUG("time_increment: %f", msg.time_increment);
//        ROS_DEBUG("scan_time: %f", msg.scan_time);
//        ROS_DEBUG("range_min: %f", msg.range_min);
//        ROS_DEBUG("range_max: %f", msg.range_max);
    }
    bool isConnected(){
        return mConnectFlag;
    }
    void setConnectFlag(bool flag){
        mConnectFlag = flag;
    }

private:

    // Parser
    CSDKeliLs1207DEParser    *mParser;

    bool            mPublishData;
    double          dExpectedFreq;

    struct dataSaveSt mDataSaveSt[CMD_FRAME_MAX_SUB_PKG_NUM];

    unsigned char   mStoreBuffer[RECV_BUFFER_SIZE];
    unsigned char   mRecvBuffer[RECV_BUFFER_SIZE];
    int             mDataLength;

    bool            mConnectFlag;

    // Diagnostics
//    diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> *mDiagPublisher;//诊断

    // Dynamic Reconfigure
    sdkeli_lspdm_udp::LidarConfig  mConfig;
//    dynamic_reconfigure::Server<sdkeli_lspdm_udp::SDKeliLspdmConfig> mDynaReconfigServer;//节点参数动态调整
};
} // sdkeli_lspdm_udp

#endif // SDKELI_LS_COMMON_H_
