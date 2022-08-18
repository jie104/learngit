/**
 * describe: 
 * Created by caoyan on 2021-3-4.
**/

#ifndef SROS_RFID_PROTOCOL_WYUAN_H
#define SROS_RFID_PROTOCOL_WYUAN_H

#include "rfid_protocol_interface.h"

namespace ac {

const int READ_DATA_COUNT = 10; // 读取字符的个数
const int RFID_UID_LEN = 8; // RFID的uid的长度   

class RfidProtWYuan : public RfidProtInterface {
public:
    RfidProtWYuan();
    virtual ~RfidProtWYuan();

    bool init() override;

    void parseMsg(std::vector<uint8_t>& data) override;

    std::string getRfid() override;

private:
    void setQuerySingleLableCmd();

    std::vector<uint8_t> getUID();

    void setReadDataCmd(const std::vector<uint8_t> &uid);

    std::vector<uint8_t> getReadData();

    const std::vector<uint8_t> &getSendRawData(); // 获取生成的原始帧
    bool feedReciveRawData(const std::vector<uint8_t> &data); // 填充收到的数据

private:
    uint16_t uiCrc16Cal(const std::vector<uint8_t> &data);

    enum SENDFIELD {
        SENDFIELD_LEN = 0,
        SENDFIELD_ADDR = 1,
        SENDFIELD_CMD = 2,
        SENDFIELD_DATE = 3,
//        LSB_CRC1,
//        MSB_CRC1,
    };

    enum RECIVEFILD {
        RECIVEFILD_LEN = 0,
        RECIVEFILD_ADDR,
        RECIVEFILD_RECMD,
        RECIVEFILD_STATUS,
        RECIVEFILD_DATA,
//        LSB_CRC1,
//        MSB_CRC1,
    };

    enum CMD {
        QUERY_SINGLE_LABLE = 0x50, // 查询单张命令
        READ_LABLE = 0x52, // 读取数据
    };

    enum ResultStatus { // 返回结果状态状态
        SUCCEED = 0x00, // 成功
        NONE_LABEL = 0xFB, // 查无此标签
    };

};

}


#endif //SROS_RFID_PROTOCOL_WYUAN_H
