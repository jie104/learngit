#ifndef SDKELI_KLM_COMMON_UDP__
#define SDKELI_KLM_COMMON_UDP__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkeli_klm_common.h"

namespace sdkeli_klm_udp
{
class CSDKeliKlmCommonUdp : public CSDKeliKlmCommon
{
public:
    CSDKeliKlmCommonUdp(const std::string &hostname, const std::string &port, int &timelimit, CParserBase *parser, ros::NodeHandle &nh, ros::NodeHandle &pnh);
    virtual ~CSDKeliKlmCommonUdp();

protected:
    /*Override functions*/
    virtual int InitDevice();
    virtual int CloseDevice();

    virtual int SendDeviceReq(const char *req, std::vector<unsigned char> *resp);

    virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize, int *length);

    int SendUdpData2Device(char *buf, int length);
private:
    int                          mSocket;
    size_t                       mBytesReceived;
    std::string                  mHostName;
    std::string                  mPort;
    int                          mTimeLimit;
    struct sockaddr_in           remoteServAddr;

};

} /*namespace sdkeli_klm_udp*/

#endif /*SDKELI_KLM_COMMON_TCP__*/
