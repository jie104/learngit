/*  intelly_driver.h
*   Author: wuji228@163.com
*   Date:2018.10
*/

#ifndef INTELLY_DRIVER_H_
#define INTELLY_DRIVER_H_

#include <string>
#include <cstdint>
#include <time.h>
#include <unistd.h> //uslee
#include <sys/types.h>
#include <sys/socket.h>
// boost lib
#include <boost/bind.hpp>
#include <boost/thread.hpp>

typedef unsigned long uint64;
typedef unsigned int uint32;

/* *********************** 激光头命令号协议************************** */
#define		        FRAME_INTERNAL_DATA_HEAD1		0xAA		//һ֡\BF\AAʼ\B5ĵ\DA1\B8\F6\D7ֽ\DA
#define		        FRAME_INTERNAL_DATA_HEAD2		0x77		//һ֡\BF\AAʼ\B5ĵ\DA2\B8\F6\D7ֽ\DA
#define		        FRAME_INTERNAL_DATA_HEAD3		0x77		//һ֡\BF\AAʼ\B5ĵ\DA3\B8\F6\D7ֽ\DA
#define		        FRAME_INTERNAL_DATA_HEAD4		0xAA		//һ֡\BF\AAʼ\B5ĵ\DA4\B8\F6\D7ֽ\DA
#define	    	    FRAME_INTERNAL_DATA_HEAD_ALL	((FRAME_INTERNAL_DATA_HEAD1<<24)|(FRAME_INTERNAL_DATA_HEAD2<<16)|(FRAME_INTERNAL_DATA_HEAD3<<8)|(FRAME_INTERNAL_DATA_HEAD4))

#define		        FRAME_INTERNAL_DATA_END1		0x77		//һ֡\BD\E1\CA\F8\B5ĵ\DA1\B8\F6\D7ֽ\DA
#define	        	FRAME_INTERNAL_DATA_END2		0xAA		//һ֡\BD\E1\CA\F8\B5ĵ\DA2\B8\F6\D7ֽ\DA
#define		        FRAME_INTERNAL_DATA_END3		0xAA		//һ֡\BD\E1\CA\F8\B5ĵ\DA3\B8\F6\D7ֽ\DA
#define		        FRAME_INTERNAL_DATA_END4		0x77		//һ֡\BD\E1\CA\F8\B5ĵ\DA4\B8\F6\D7ֽ\DA
#define		        FRAME_INTERNAL_DATA_END_ALL		((FRAME_INTERNAL_DATA_END1<<24)|(FRAME_INTERNAL_DATA_END2<<16)|(FRAME_INTERNAL_DATA_END3<<8)|(FRAME_INTERNAL_DATA_END4))

#define             FRAME_DATA_HEAD1            	0xAA//一帧开始的第1个字节
#define             FRAME_DATA_HEAD2            	0x88//一帧开始的第2个字节
#define             FRAME_DATA_HEAD3            	0x88//一帧开始的第3个字节
#define             FRAME_DATA_HEAD4            	0xAA//一帧开始的第4个字节
#define             FRAME_DATA_HEAD_ALL         	((FRAME_DATA_HEAD1<<24) | (FRAME_DATA_HEAD2<<16) | (FRAME_DATA_HEAD3<<8) | FRAME_DATA_HEAD4)//一帧开始的所有数据


#define             FRAME_DATA_END1             	0x88//一帧结束的第1个字节
#define             FRAME_DATA_END2             	0xAA//一帧结束的第2个字节
#define             FRAME_DATA_END3             	0xAA//一帧结束的第3个字节
#define             FRAME_DATA_END4             	0x88//一帧结束的第4个字节
#define             FRAME_DATA_END_ALL          	((FRAME_DATA_END1<<24) | (FRAME_DATA_END2<<16) | (FRAME_DATA_END3<<8) | FRAME_DATA_END4)//一帧开始的所有数据


#define             FRAME_DATA_MAIN_CMD_POS     	4 //主命令对应的位置
#define             FRAME_DATA_SUB_CMD_POS      	5 //从命令对应的位置
#define             FRAME_DATA_START_POS        	10 //协议中数据对应的位置
#define				FRAME_DATA_MIN_LEN				15
#define             FRAME_DATA_MAX_LEN          	7680

/* *********************** SICK 命令号协议************************** */
#define				SICK_BIN_FRAME_HEAD1			0x02	//一帧开始的第1个字节
#define				SICK_BIN_FRAME_HEAD2			0x02	//一帧开始的第2个字节
#define				SICK_BIN_FRAME_HEAD3			0x02	//一帧开始的第3个字节
#define				SICK_BIN_FRAME_HEAD4			0x02	//一帧开始的第4个字节
#define				SICK_BIN_FRAME_HEAD_ALL			((SICK_BIN_FRAME_HEAD1<<24) | (SICK_BIN_FRAME_HEAD2<<16) | (SICK_BIN_FRAME_HEAD3<<8) | SICK_BIN_FRAME_HEAD4)

#define				SICK_BIN_FRAME_NUM_MIN			0x09	//帧头4+长度4+校验1
#define				SICK_BIN_FRAME_DATA_LEN_POS		0x04	//数据长度起始位置
#define				SICK_BIN_FRAME_DATA_POS_START	0x08	//有效数据的起始位

#define		        SICK_BIN_COMMON_DATA_LEN_1				69
//#define		        SICK_BIN_COMMON_DATA_LEN_2				23
//#define		        SICK_BIN_COMMON_DATA_LEN_3				12  
#define		        SICK_BIN_COMMON_DATA_LEN_2				21
#define		        SICK_BIN_COMMON_DATA_LEN_3				10	
#define             SICK_BIN_COMMON_DATA_LEN_4              15

#define				SICK_ASCII_FRAME_HEAD			        0x02
#define		        SICK_ASCII_COMMON_DATA_LEN_1			95
//#define		        SICK_ASCII_COMMON_DATA_LEN_2			12
#define		        SICK_ASCII_COMMON_DATA_LEN_2			10
#define             SICK_ASCII_COMMON_DATA_LEN_3            26

//基本参数
#define             PRO_SET_CMD                             0xA1
#define             PRO_SET_CMD_ECHO                        0xB1
#define             PRO_INQUERY_CMD                         0xA2
#define             PRO_INQUERY_CMD_ECHO                    0xB2


#define             BASIC_CONFIG_PROTOCOL_TYPE              0x01 //协议类型
#define             BASIC_CONFIG_TRANSMIT_MODE              0x02 //传输模式
#define             BASIC_CONFIG_REPORTING_INTERVAL         0x03 //数据上报间隔
#define				BASIC_CONFIG_SCAN_RANGE					0x04 //扫描范围
#define             BASIC_CONFIG_ENABLE_TIMESTAMP           0x05 //时间戳使能

#define             CMD_GET_SOFT_VERSION                    0x10 //查询软件版本号

#define             BASIC_CONFIG_UNLOCK                     0x14  //\BD\E2\CB\F8

//网络参数
#define             NET_MAIN_SET_CMD                        0xA3
#define             NET_MAIN_SET_CMD_ECHO                   0xB3
#define             NET_MAIN_INQUERY_CMD                    0xA4
#define             NET_MAIN_INQUERY_CMD_ECHO               0xB4

#define             SUB_CMD_NET_ALL                         0x00
#define             SUB_CMD_NET_IP                          0x01
#define             SUB_CMD_NET_PORT                        0x02

//距离数据帧
#define             LMS_RESULT_ASK_FOR_RESULT               0xC5        //主动索要数据
#define             LMS_RESULT_MAIN_CMD                     0xD5        //距离数据
#define             LMS_RESULT_SUB_CMD                      0x01

//\D0޸ĽǶȷֱ\E6\C2\CA
#define		        TXCTRL_MAIN_CMD_SET				        0xC3
#define		        TXCTRL_MAIN_CMD_SET_ECHO		        0xD3
#define		        TXCTRL_MAIN_CMD_QUERY			        0xC4
#define		        TXCTRL_MAIN_CMD_QUERY_ECHO		        0xD4

#define		        TXCTRL_SUB_CMD_TX_ANGEL_STEP	        0x0A		//ɨ\C3\E8\BDǶȲ\BD\BD\F8 0.5\D5\FB\CA\FD\B1\B6*100 @20210201 \BDǶȷֱ\E6\C2\CA\D0\DEΪΪ\CB\F7\D2\FD\B7\BDʽ 0\A3\BA0.5\A1\E3 1\A3\BA0.33\A1\E3 2\A3\BA0.25\A1\E3 3\A3\BA0.125\A1\E3 4\A3\BA0.1

#define             SOCKET_QUEUE_SIZE                       7680    //4096
#define             NET_MAX_PACKAGE                         10


//基本配置相关的变量
#define             PROTOCOL_TYPE_INTELLY                   0

//从客户端接收的数据
#define             RECV_DATA_STEP_FINDING_HEAD             0
#define             RECV_DATA_STEP_FINDING_END              1

//接收到的数据协议类型
#define				FRAME_TYPE_INTELLY_DEBUG				0	//调试 内部协议
#define				FRAME_TYPE_SICK_BIN						1	//SICK 二进制协议
#define				FRAME_TYPE_SICK_ASCII					2	//SICK Ascii协议
#define				FRAME_TYPE_INTELLY_FORMAL				3	//工作 公开协议

// Default time out.
#define DEFAULT_TIMEOUT 1000 // 1000ms
/* ****************************************************************/

/*!
* @struct ScanCfg
* @brief Structure containing scan configuration.
*/
struct ScanCfg
{

    /*
    * @brief 协议类型
    * 0-因泰立协议
    */
    int protocol_type;

    /*
    * @brief 数据传输模式
    * 0-主动传输模式， 1-被动传输模式， 2-强度传输模式
    */
    int transmission_mode;

    /*
    * @brief 数据上报间隔
    * byte: 10--13
    * 单位： ms
    */
    int reporting_interval;

    /*
    * @brief 时间戳使能
    * 0-不使能， 1-使能
    */
    bool enable_timestamp;

    /*
    * @brief 激光传感器 IP
    * byte 10--13,高位在前
    */
    std::string host_ip;

    /*
    * @brief 激光传感器 PORT
    * 高位在前
    * 数据: 4001
    * 调试: 4002
    */
    int data_port;
    int debug_port;

    /*!
    * @brief Mac Address.
    * byte: 6
    */
    std::string mac_addr;

    /*!
    * @brief Scanning resolution.
    * 0.5°
    */
    float angle_resolution;

    /*!
    * @brief Start angle.
    * byte: 10--11
    * 无符号数，权值 1/100，如 50->0.5°
    * Default: 0° -> 0000
    */
    int start_angle;

    /*!
    * @brief Stop angle.
    * byte: 12--13
    * 无符号数，权值 1/100，如 150->1.5°
    * Default: 270° -> 010e
    */
    int stop_angle;

    /*!
    * @brief 激光传感器  型号
    * 0：F30  1：F30-C
    */
    int lidar_type;
    

    /*!
    * @brief operator =.
    */
    void operator = (const ScanCfg& cfg)
    {
        protocol_type       =   cfg.protocol_type;
        transmission_mode   =   cfg.transmission_mode;
        reporting_interval  =   cfg.reporting_interval;
        enable_timestamp    =   cfg.enable_timestamp;
        host_ip             =   cfg.host_ip;
        data_port           =   cfg.data_port;
        debug_port          =   cfg.debug_port;
        angle_resolution    =   cfg.angle_resolution;
        start_angle         =   cfg.start_angle;
        stop_angle          =   cfg.stop_angle;
        lidar_type          =   cfg.lidar_type;
    }
};


/*!
* @struct ScanData
* @brief Structure containing scan data.
* @brief Scan data defination:
* byte 10--11: 起始角度
* byte 12--13: 距离总个数 N
* 14-15:  距离 1, cm
*       ...
* N--N+1 距离 N, cm
* @brief if 强度传输模式
* N+2--N+3 相对强度数值 1
*       ...
* 3N--3N+1 相对强度数值 N
*@brief if 时间戳标志
* 3N+3--3N+4:年
* 3N+5:月
* 3N+6: 日
* 3N+7: 时
* 3N+8: 分
* 3N+9: 秒
* 3N+10--3N+13: 毫秒
*/

struct ScanData
{

    /*
    * @brief 距离总个数 N
    * 无符号数
    */
    unsigned int num_values;

    /*
    * @brief 原始数据缓存
    * 无符号短整型， cm
    */
    unsigned int ranges[SOCKET_QUEUE_SIZE];       //接收到的原始数据

    /*
    * @brief 原始数据缓存
    * 无符号短整型， cm
    */
    unsigned int intensities[SOCKET_QUEUE_SIZE];       //接收到的原始数据

}__attribute__((packed));   // 内存对齐


class Intelly
{
public:
    Intelly();  // Constructor.
    virtual ~Intelly(); // Destructor.

    /*!
    * @brief Get status of connection.
    * @returns connected or not.
    */
    bool is_connected() const;

    /*!
    * @brief Get current scan configuration.
    * Get scan configuration :
    * @returns scanCfg structure.
    */
    ScanCfg& get_scan_cfg();

    /*!
    * @brief Initialization intelly laser.
    * @return true: if scan Initialization successfully, false: if error.
    */
    bool init(const ScanCfg& cfg);

    /*!
    * @brief Start laser scan.
    * @return true: if scan Initialization successfully, false: if error.
    */
    bool StartScan();

    /*!
    * @brief Stop laser scan.
    */
    void StopScan();

    /*!
    * @brief Receive single scan message.
    * @param scan_data structure containing scan data.
    * @return true: if scan was read successfully, false if error or timeout.
    *         false: implies that higher level logic should take correct action
    *                such as reopening the connection.
    */
    bool GrabScanData(ScanData& scan_data);

protected:
    /******Protected Functions.**********/

    /*!
    * @brief Connect to Intelly.
    * @param host--Intelly host name or ip address.
    * @param port--Intelly port number.
    * @return true--成功，false--失败.
    */
    bool Connect(std::string host, int port = 4001, uint32 timeout=DEFAULT_TIMEOUT);

    /*!
    * @brief Disconnect from Intelly device.
    */
    void DisConnect();

    /*!
    * @brief Set scan configuration.
    * Get scan configuration :
    * - 协议类型.
    * - 数据传输模式.
    * - 数据上报间隔.
    * - 时间戳使能.
    * - 激光传感器 IP.
    * - 激光传感器 PORT.
    * - Scanning resolution.
    * - start angle.
    * - stop angle.
    * @param cfg--structure containing scan configuration.
    * @return true--成功，false--失败.
    */
    bool SetScanCfg(const ScanCfg& cfg);

    /*!
    * @brief Query scan configuration from Intelly laser.
    * @param cfg--ScanCfg structure containing returned scan config.
    * @return true--成功，false--失败.
    */
    bool QueryScanCfg(ScanCfg& cfg);

    /*!
    * @brief Receive single socket message.
    * @param recv_buf--网络数据缓存.
    * @param recv_len--网络数据缓存长度.
    * @param package_buf--one package buffer.
    * @param package_len--package buffer length.
    */
    unsigned char DealSocketDataInPackage(unsigned char* recv_buf, unsigned int& recv_len,
                                 unsigned char* package_buf, unsigned int& package_len);


    // @brief Find Sick Ascii Pro
    int DecodeSickPkgAscii(unsigned char p_CodeBuf[], unsigned int p_Len);

    
    bool AsciiFrameFindValidData(const unsigned char *p_u8TargetBuf, unsigned int& p_u32TargetPos, signed int& p_s32TargetData);

    /*!
    * @brief Decode single net message.
    * 对接受的数据进行解析（去头尾、转义等操作），验证校验是否正确.
    * @param buf--待解码数据，即收到的原始数据包.
    * @param len--待解码长度.
    * @param target_buf--解码完成数据，去掉了帧头帧尾和校验位，只包括帧数据位.
    * @param target_len--解码后数据长度.
    * @param main_cmd--主命令号
    * @param sub_cmd--子命令号
    * @return true--成功，false--失败.
    */
	//intelly协议
    bool SocketDataDecoderForIntelly(const unsigned char* buf, unsigned int& len,
                           unsigned char* target_buf, unsigned int& target_len,
                           unsigned char& main_cmd, unsigned char& sub_cmd);
									
	//Sick bin 协议
    bool SocketDataDecoderForSickBin(const unsigned char* buf, unsigned int& len,
                           unsigned char* target_buf, unsigned int& target_len,
                           unsigned char& main_cmd, unsigned char& sub_cmd);

    //Sick Ascii协议
    bool SocketDataDecoderForSickAscii(const unsigned char* buf, unsigned int& len,
                                unsigned char* target_buf, unsigned int& target_len,
                                unsigned char& main_cmd, unsigned char& sub_cmd);
	bool SocketInternalDecoderForIntelly(const unsigned char* buf, unsigned int& len,
		                        unsigned char* target_buf, unsigned int& target_len,
								unsigned char& main_cmd, unsigned char& sub_cmd);
    /*!
    * @brief 通用数据编码--帧头是aa aa 77 77，帧尾aa 77 77 aa.
    * 加入帧头、校验位和帧尾，编码发送.
    * @param main_cmd: (输入)主命令号.
    * @param sub_cmd: (输入)从命令号.
    * @param *buf: (输入)待编码的原始数据，不包括帧头（0xff 0xff）、校验位和帧尾.
    * @param len: (输入)buf原始数据的长度.
    * @param targetbuf: (输出)符合协议要求格式的数据.
    * @param targetlen: (输出)target_buf的大小.
    * @return true--成功，false--失败.
    */
    bool SocketDataEncoder(const unsigned char& main_cmd, const unsigned char& sub_cmd,
                           const unsigned char* buf, const unsigned int& len,
                           unsigned char* target_buf, unsigned int& target_len);

	bool SocketInternalEncoder(const unsigned char& main_cmd, const unsigned char& sub_cmd,
		                       const unsigned char* buf, const unsigned int& len, unsigned char* target_buf,
                               unsigned int& target_len);
    /*!
    * @brief socket send cmd and recevie data buffer.
    */
    void SocketSendCmdAndRev(const unsigned char& set_main_cmd, const unsigned char& set_sub_cmd,
                             const unsigned char* cmd_buf, const unsigned int& cmd_len,
                             unsigned char& recv_main_cmd, unsigned char& recv_sub_cmd,
                             unsigned char* data_buf, unsigned int& data_len);

	void SocketInternalSendCmdAndRev(const unsigned char& set_main_cmd, const unsigned char& set_sub_cmd,  // send cmd.
		                             const unsigned char* cmd_buf, const unsigned int& cmd_len,    // set buf.
		                             unsigned char& recv_main_cmd, unsigned char& recv_sub_cmd,  // rev cmd.
                                     unsigned char* data_buf, unsigned int& data_len);  // rev data buf.
    /*!
    * @brief Parse laser scan Data.
    * @param buf: pointer to scan data buffer.
    * @param data: refence to ScanData structure.
    * @return true--成功，false--失败.
    * @brief Scan data defination:
    * byte 10--11: 起始角度
    * byte 12--13: 距离总个数 N
    * 14-15:  距离 1, cm
    *       ...
    * N--N+1 距离 N, cm
    * @brief if 强度传输模式
    * N+2--N+3 相对强度数值 1
    *       ...
    * 3N--3N+1 相对强度数值 N
    *@brief if 时间戳标志
    * 3N+3--3N+4:年
    * 3N+5:月
    * 3N+6: 日
    * 3N+7: 时
    * 3N+8: 分
    * 3N+9: 秒
    * 3N+10--3N+13: 毫秒
    */
    bool ParseScanData(unsigned char* buf, unsigned int& len, ScanData& scan_data);

    /*!
    * @brief cache a package of laser scan data into scan_data_buf_.
    */
    void CacheScanData();


protected:
    // Variables.
    bool is_connected_;    // laser connect flag.
    bool is_scanning_;  // is grabing scan data.
    int socket_fd_;     // socket file descriptor.
    ScanCfg scan_cfg_;  // intelly laser configure.
    //    ScanData scan_data_;    // laser scan data.
    // cache scan data.
    boost::mutex buf_mutex_;
    unsigned char scan_data_buf_[SOCKET_QUEUE_SIZE];    // scan data cache buffer.
    unsigned int scan_data_len_;    // scan data buffer length.
    bool socket_recv_step_; // data receive step.
	unsigned char socket_data_type;	//Intelly Or SICK
    //    boost::thread cache_thread_;
    boost::shared_ptr<boost::thread> cache_thread_ptr_;

};

#endif  //INTELLY_DRIVER_H_

/*****************End of file*********************/
