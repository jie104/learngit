//
// Created by lfc on 2020/3/28.
//

#ifndef IMU_READER_IMU_RECEIVE_HPP
#define IMU_READER_IMU_RECEIVE_HPP
#include "functional"
#include "glog/logging.h"
extern "C" {
#include "protocol_wrapper.h"
};

typedef std::function<int(void *, u16 length, u16 type)> imu_data_handler;
extern imu_data_handler g_imuCallback;
int imuDataCallback(void *data, u16 length, u16 type);
int imuOutputInformation(const char *data);

class ImuReceive {
 public:
    ImuReceive() {
        g_imu_msg_callback = imuDataCallback;
        g_output_info = imuOutputInformation;
        pBufferStart = (unsigned char *)ringBuf;
        pBufferEnd = pBufferStart + RING_BUFFER_SIZE;
        pCustomer = pBufferStart;
        pProducer = pBufferStart;
    }

    bool open(const char *dev, const int32_t bandrate) {
        if (ATR_Linux_OpenCom(dev, bandrate) > 0) {
            LOG(WARNING) << "err to read! will return false! dev name:" << dev << ",baudrate:" << bandrate;
            return false;
        }
        LOG(INFO) << "successfully to open imu:" << dev;
        Atom_switchModeReq(CONFIG_MODE);
        LOG(INFO) << "config";
        printf("1111\n");
        int ret = 0;
        ret = waitAck(Atom_switchModeReq, CLASS_ID_OPERATION_CMD, CMD_ID_SWITCH_TO_CFG_MODE, CONFIG_MODE);
        if (ret == 0) {
            LOG(ERROR) << "set operation err!";
            return false;
        }
        // Enable packets,  use SET_BIT to enable packet and use CLEAR_BIT to disable packet.
        int bitEnable = 0;
        /*-------------------Uesr Code Begin------------------*/
        // Users can set the packet according to their own needs
        SET_BIT(bitEnable, PACKET_CAL_ACC);
        // SET_BIT(bitEnable, PACKET_CAL_ACC);
        SET_BIT(bitEnable, PACKET_CAL_GYRO);
        // SET_BIT(bitEnable, PACKET_CAL_MAG);
        SET_BIT(bitEnable, PACKET_EULER_DATA);
        // SET_BIT(bitEnable, PACKET_EULER_DATA);
        SET_BIT(bitEnable, PACKET_QUAT_DATA);
        SET_BIT(bitEnable, PACKET_IMU_TIME);
        // SET_BIT(bitEnable, PACKET_QUAT_DATA);

        /*-------------------Uesr Code End-------------------*/
        SelectPackets(bitEnable);

        ret = waitAck(SelectPackets, CLASS_ID_HOSTCONTROL_CMD, CMD_ID_SET_DATA_PAC_CFG, bitEnable);
        if (ret == 0) {
            LOG(ERROR) << "set data pac err!";
            return false;
        }
        /****************************** Select packet***************************************************************/

        /****************************** Set host update
         * rate***************************************************************/
        SetHostUpdateRate(RATE_100);
        ret = waitAck(SetHostUpdateRate, CLASS_ID_ALGORITHMENGINE, CMD_ID_SET_PACKET_UPDATE_RATE, RATE_100);
        if (ret == 0) {
            LOG(ERROR) << "set data update rate err!";
            return false;
        }
        /****************************** Set host update
         * rate***************************************************************/

        /****************************** Switch To measure
         * mode***************************************************************/

        /****************************** Switch To measure
         * mode***************************************************************/
        return true;
    }

    void close() { ATR_Linux_CloseCom(); }

    bool creatReadLoop(const imu_data_handler &imu_callback, uint8_t cid, uint8_t mid) {
        int ret;
        /****************************** Switch To measure
         * mode***************************************************************/
        Atom_switchModeReq(MEASURE_MODE);
        ret = waitAck(Atom_switchModeReq, CLASS_ID_OPERATION_CMD, CMD_ID_SWITCH_TO_MEASURE_MODE, MEASURE_MODE);
        if (ret == 0) {
            LOG(ERROR) << "cannot receive imu data!";
            return false;
        }
        setImuReceiveCallback(imu_callback);
        ReceiveData(cid, mid);
        LOG(INFO) << "loop end!";
        return true;
    }

    void setImuReceiveCallback(const imu_data_handler &imuCallback) { g_imuCallback = imuCallback; }

 private:
    //    int onImuDataReceive(void *data,u16 length,u16 type){ LOG(INFO) << "length:" << length << "," << type; }
};

#endif  // IMU_READER_IMU_RECEIVE_HPP
