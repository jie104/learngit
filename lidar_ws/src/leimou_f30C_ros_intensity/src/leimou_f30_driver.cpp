/*  intelly_driver.h
*   Author:wuji228@163.com
*   Date:2018.10
*/
#include "leimou_f30_driver.h"

#include <iostream>
#include <string>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h> // 互联网地址族
#include <arpa/inet.h>  // inet_pton()

#include <cstring>  // memset(), memcpy()
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <errno.h>


// Constructor.
Intelly::Intelly():is_connected_(false), is_scanning_(false)
{
    scan_data_len_ = 0;
    socket_recv_step_ = RECV_DATA_STEP_FINDING_HEAD;    // Recv step.
}
// Destructor.
Intelly::~Intelly()
{
    DisConnect();   // close data port.
    StopScan();
    boost::mutex::scoped_lock lock(buf_mutex_); // 加解锁,一直锁定这个mutex直到对象被销毁.
}

// @brief Get status of connection.
bool Intelly::is_connected() const
{
    return is_connected_;
}

// @brief Get current scan configuration.
ScanCfg& Intelly::get_scan_cfg()
{
    return scan_cfg_;
}

// @brief Initialization intelly laser.
bool Intelly::init(const ScanCfg& cfg)
{
    // Step 1: connect at debug port.
    scan_cfg_ = cfg;    // Struct Assignment.
    if (!Connect(scan_cfg_.host_ip, scan_cfg_.debug_port))   // connect at debug port.
        return false;
    if (!SetScanCfg(scan_cfg_)) // set intelly laser scan config.
        return false;
    // Query Scan config.
	ScanCfg query_cfg;
	if (!QueryScanCfg(query_cfg))
	{
		return false;
	}
	else
	{
		std::cout << "Queried laser scan config is: \n"
				  << "--- Protocol Type: "		<< query_cfg.protocol_type << " ---\n"
				  << "--- Transmission Mode: " 	<< query_cfg.transmission_mode << " ---\n"
				  << "--- Reporting Interval: " << query_cfg.reporting_interval << " ---\n"
				  << "--- Enable Timestamp: " 	<< query_cfg.enable_timestamp << " ---\n"
				  << "--- IP Address: " 		<< query_cfg.host_ip << " ---\n"
				  << "--- Data Port: " 			<< query_cfg.data_port << " ---\n"
				  << "--- Mac Address: " 		<< query_cfg.mac_addr << " ---\n"
				  << "--- Start angle: " 		<< query_cfg.start_angle << " ---\n"
				  << "--- Stop angle: " 		<< query_cfg.stop_angle << " ---\n" 
				  << "--- angle step: " 		<< query_cfg.angle_resolution << " ---"<< std::endl;
	}

   
    DisConnect();   // close debug port.
    // Sleep 1 senconds.
    sleep(1.0);

    // Step 2: connect at data port, receive data.
    if (!Connect(scan_cfg_.host_ip, scan_cfg_.data_port))   // connect at data port.
        return false;
    // Sleep 1 senconds.
    sleep(1.0);

    return true;
}

// @brief Start laser scan.
bool Intelly::StartScan()
{
    // check socket connetion.
    if (!is_connected())
        return false;
    // Now, only auto report transmition mode works.(2018.10)

    // check is scanning.
    if (is_scanning_)   // Already scanning.
        return true;
    // start cache scan data.
    cache_thread_ptr_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Intelly::CacheScanData, this)));
    if (cache_thread_ptr_ == nullptr)
        return false;
    is_scanning_ = true;
    // Sleep 2 senconds.
    // sleep(2.0);
    return true;
}

//@brief Stop laser scan.
void Intelly::StopScan()
{
    is_scanning_ = false;
    if (cache_thread_ptr_ != nullptr)   // Assertion `px != 0' failed.
        cache_thread_ptr_->join();  // 等待一个线程 shutdown.
}

// @brief Receive single scan message.
bool Intelly::GrabScanData(ScanData& scan_data)
{
    static bool flag;
    // one package buffer.
    unsigned char package_buf[scan_data_len_];    // scan data cache buffer.
    if (!scan_data_len_ || !is_scanning_)
    {
        return false;
    }

    buf_mutex_.lock();
    // Scan data
    flag = ParseScanData(scan_data_buf_, scan_data_len_, scan_data);
	buf_mutex_.unlock();

    return flag;
}

/******************Protected Functions.**********************/

// @brief Connect to Intelly.
bool Intelly::Connect(std::string host, int port, uint32 timeout)
{
    static bool first_flag = true;
    bool ret = false;

    if (!is_connected_)
    {
        // int socket(int domain, int type, int protocol);
        socket_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);    //IPPROTO_TCP
        // Not create socket, return.
        if (socket_fd_ < 0)
            return 1;
        // Socket created.
        struct sockaddr_in server_addr;  // client's address information.
        server_addr.sin_family = AF_INET;   //
        server_addr.sin_port = htons(port);  // short, network byte order.
        server_addr.sin_addr.s_addr = inet_addr(host.c_str());  ///服务器ip.

        // First time connection.
        // Deal the case that network cable unplugged.
        if (first_flag)
        {
            // Set to non-blocking mode.
            unsigned long ul = 1;
            ioctl(socket_fd_, FIONBIO, &ul); //设置为非阻塞模式.
            fd_set set;
            FD_ZERO(&set);  // 把可读文件描述符的集合清空.
            FD_SET(socket_fd_, &set);   // 把当前连接的文件描述符加入到集合中.
            // timeout
            struct timeval tv;    //设置超时时间.
            tv.tv_sec = 0;
            tv.tv_usec = timeout*3*1000;    // us.
            int error=-1, len = sizeof(int);

            if( connect(socket_fd_, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
            {
                std::cout<<"socket error:"<<strerror(errno)<<std::endl;
                if(select(socket_fd_+1, NULL, &set, NULL, &tv) > 0)
                {
                    getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &error, (socklen_t *)&len);
                    if(error == 0)  // succeed.
                        ret = true;
                    else    // Failed.
                        ret = false;
                }
                else
                {
                    std::cout << "--- Please check your network cable!!! ---" << std::endl;
                    ret = false;
                }

            }
            else
                ret = true;
            // Set back to blocking mode.
            ul = 0;
            ioctl(socket_fd_, FIONBIO, &ul); //设置成阻塞模式.

            if(ret == true)  // succeed.
            {
                is_connected_ = true;
                std::cout << "Connected to host: " << host << " ,at port: " << port << " succeeded.\n";
                return true;
            }
            else    // Failed.
            {
                std::cerr << "Connected to host: " << host << " ,at port: " << port << " Failed!\n";
                return false;
            }
            first_flag = false;
        }
        // Blocking mode.
        else
        {
            if(connect(socket_fd_, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == 0)
            {
                is_connected_ = true;
                std::cout << "Connected to host: " << host << ", at port: " << port << " succeeded.\n";
                return true;
            }
            else
            {
                std::cerr << "Connected to host: " << host << ", at port: " << port << " Failed!\n";
                return false;
            }
        }

    }
    // Already connected.
    return true;
}

// @brief Disconnect from Intelly device.
void Intelly::DisConnect()
{
    if (is_connected_)
    {
        close(socket_fd_);
        is_connected_ = false;
        std::cout << "Disconnet socket!" << std::endl;
    }
}


// @brief Set scan configuration.
bool Intelly::SetScanCfg(const ScanCfg& cfg)
{
    unsigned char   recv_buf[50];
    unsigned int    recv_len = 0;
    unsigned char   set_buf[10]; // data buffer.
    unsigned char   recv_main_cmd, recv_sub_cmd;
	unsigned int    ip[4];
    unsigned int    l_u32MinInterval;
	unsigned int    l_u32AngleStep;

    // 0-SICK_BIN协议
    set_buf[0] = cfg.protocol_type;
    SocketSendCmdAndRev(PRO_SET_CMD, BASIC_CONFIG_PROTOCOL_TYPE, set_buf, 1,
                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd != PRO_SET_CMD_ECHO) 
	|| (recv_sub_cmd!=BASIC_CONFIG_PROTOCOL_TYPE)
    || (recv_len != 1) 
	|| (recv_buf[0] != 0x00))
    {
        std::cerr << "Set Protocol Type Error!" << std::endl;
        return false;
    }

    // 0-主动传输模式， 1-被动传输模式
    set_buf[0] = cfg.transmission_mode;
    SocketSendCmdAndRev(PRO_SET_CMD, BASIC_CONFIG_TRANSMIT_MODE, set_buf, 1,
                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd != PRO_SET_CMD_ECHO) 
	|| (recv_sub_cmd!=BASIC_CONFIG_TRANSMIT_MODE)
    || (recv_len != 1) 
	|| (recv_buf[0] != 0x00))
    {
        std::cerr << "Set Transmission Mode Error!" << std::endl;
        return false;
    }

    // 数据上报间隔, byte: 10--13, 单位： ms
    set_buf[0] = (cfg.reporting_interval>>24);
    set_buf[1] = (cfg.reporting_interval>>16);
    set_buf[2] = (cfg.reporting_interval>>8);
    set_buf[3] = cfg.reporting_interval;
    SocketSendCmdAndRev(PRO_SET_CMD, BASIC_CONFIG_REPORTING_INTERVAL, set_buf, 4,
                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd != PRO_SET_CMD_ECHO) 
	|| (recv_sub_cmd != BASIC_CONFIG_REPORTING_INTERVAL)
    || (recv_len < 1) 
	|| (recv_buf[0] != 0x00))
    {
        l_u32MinInterval = (recv_buf[1] << 24) | (recv_buf[2] << 16) | (recv_buf[3] << 8) | (recv_buf[4] << 0);
        std::cerr << "Set Reporting Interval Error! Curr Set:" << cfg.reporting_interval << "Can Set Min: " << l_u32MinInterval << std::endl;
        return false;
    }

    // 时间戳使能, 0-不使能， 1-使能
    /**** Not used now.****/

    // 激光传感器 IP, byte 10--13,高位在前
    /**** Not used now.****/

	// Start & Stop angle, byte: 10-11, 12-13.
    set_buf[0] = (cfg.start_angle >> 8);
    set_buf[1] = (cfg.start_angle >> 0);
    set_buf[2] = (cfg.stop_angle >> 8);
    set_buf[3] = (cfg.stop_angle >> 0);
    SocketSendCmdAndRev(PRO_SET_CMD, BASIC_CONFIG_SCAN_RANGE, set_buf, 4,
                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd != PRO_SET_CMD_ECHO) 
	|| (recv_sub_cmd!=BASIC_CONFIG_SCAN_RANGE)
    || (recv_len != 1) 
	|| (recv_buf[0] != 0x00))
    {
        std::cerr << "Set Start & Stop Angle Error!" << std::endl;
        return false;
    }
std::cerr << "Set Start & Stop Angle success!" << std::endl;
	//\C9\E8\D6ýǶȷֱ\E6\C2\CA
	//1.\BD\E2\CB\F8
	set_buf[0] = 0xBB;
    set_buf[1] = 0x44;
    set_buf[2] = 0x44;
    set_buf[3] = 0xBB;
	set_buf[4] = 0x00;
	set_buf[5] = 0x44;
    set_buf[6] = 0xBB;
    set_buf[7] = 0xBB;
    set_buf[8] = 0x44;
    SocketInternalSendCmdAndRev(PRO_SET_CMD, BASIC_CONFIG_UNLOCK, set_buf, 9,
		                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd != PRO_SET_CMD_ECHO) 
	|| (recv_sub_cmd!=BASIC_CONFIG_UNLOCK)
	|| (recv_len != 1) 
	|| (recv_buf[0] != 0x00))
    {
        std::cerr << "unlock Error!" << std::endl;
        return false;
    }

	//2.\C9\E8\D6÷ֱ\E6\C2\CA
	if(fabs(cfg.angle_resolution - 0.5) < 0.001)
	{
		l_u32AngleStep = 0;
	}
	else if(fabs(cfg.angle_resolution - 0.33) < 0.001)
	{
		l_u32AngleStep = 1;
	}
	else if(fabs(cfg.angle_resolution - 0.25) < 0.001)
	{
		l_u32AngleStep = 2;
	}
	
    set_buf[0] = 0x00;
    set_buf[1] = l_u32AngleStep;
    SocketInternalSendCmdAndRev(TXCTRL_MAIN_CMD_SET, TXCTRL_SUB_CMD_TX_ANGEL_STEP, set_buf, 2,
		                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd != TXCTRL_MAIN_CMD_SET_ECHO) 
	|| (recv_sub_cmd!=TXCTRL_SUB_CMD_TX_ANGEL_STEP)
	|| (recv_len != 1) 
	|| (recv_buf[0] != 0x00))
    {
        std::cerr << "set angle step Error!" <<  recv_buf[0]  << std::endl;
        //return false;     //\D5\E2\B8\F6\D7\EE\BAò\BB\BA÷\B5\BBأ\AC\B7\F1\D5\DF\D5\E2ʱ\BA\F2\C0״ﴦ\D3ڽ\E2\CB\F8״̬
    }

	//3.\CB\F8\B6\A8
    set_buf[0] = 0xBB;
    set_buf[1] = 0x44;
    set_buf[2] = 0x44;
    set_buf[3] = 0xBB;
    set_buf[4] = 0x01;
    set_buf[5] = 0x44;
    set_buf[6] = 0xBB;
    set_buf[7] = 0xBB;
    set_buf[8] = 0x44;
    SocketInternalSendCmdAndRev(PRO_SET_CMD, BASIC_CONFIG_UNLOCK, set_buf, 9,
		recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd != PRO_SET_CMD_ECHO) 
		|| (recv_sub_cmd!=BASIC_CONFIG_UNLOCK)
		|| (recv_len != 1) 
		|| (recv_buf[0] != 0x00))
    {
        std::cerr << "lock Error!" << std::endl;
        return false;
    }
	
    std::cout << "Set laser scan config succeed!" << std::endl;
    return true;
}


// @brief Query scan configuration from laser.
bool Intelly::QueryScanCfg(ScanCfg& cfg)
{
    unsigned char recv_main_cmd, recv_sub_cmd;
    unsigned char set_buf[0]; // set buffer.
    unsigned char recv_buf[30];
    unsigned int recv_len = 0;
    static const char* hex[16]={"0","1","2","3","4","5","6","7","8","9","A","B","C","D","E","F"};
    unsigned short l_u16StepAngle;

    /*******基本参数查询类指令-0xA2********/
    // 查询协议类型, 0-SICK_BIN协议
    SocketSendCmdAndRev(PRO_INQUERY_CMD, BASIC_CONFIG_PROTOCOL_TYPE, set_buf, 0,
                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd == PRO_INQUERY_CMD_ECHO) 
	&& (recv_sub_cmd == BASIC_CONFIG_PROTOCOL_TYPE)
    && (recv_len == 1))
	{
        cfg.protocol_type = recv_buf[0];
    }
	else
    {
        std::cerr << "Query Protocol Type Error!" << std::endl;
        return false;
    }

    // 查询数据传输模式: 0-主动传输模式， 1-被动传输模式
    SocketSendCmdAndRev(PRO_INQUERY_CMD, BASIC_CONFIG_TRANSMIT_MODE, set_buf, 0,
                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd == PRO_INQUERY_CMD_ECHO) 
	&& (recv_sub_cmd == BASIC_CONFIG_TRANSMIT_MODE)
    && (recv_len == 1))
	{
        cfg.transmission_mode = recv_buf[0];
    }
	else
    {
        std::cerr << "Query Transmission Mode Error!" << std::endl;
        return false;
    }

    // 查询数据上报间隔, Byte 10--13, in ms.
    SocketSendCmdAndRev(PRO_INQUERY_CMD, BASIC_CONFIG_REPORTING_INTERVAL, set_buf, 0,
                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd == PRO_INQUERY_CMD_ECHO) 
	&& (recv_sub_cmd == BASIC_CONFIG_REPORTING_INTERVAL)
    && (recv_len == 4))
    {
		cfg.reporting_interval = (recv_buf[0]<<24) + (recv_buf[1]<<16) + (recv_buf[2]<<8) + recv_buf[3];
    }
	else
    {
        std::cerr << "Query Reporting Interval Error!" << std::endl;
        return false;
    }
std::cerr << "Query Reporting Interval success!" << std::endl;

    /**** Not used now.****/
    // 查询时间戳使能, Byte 10--13, in ms.
//    SocketSendCmdAndRev(PRO_INQUERY_CMD, BASIC_CONFIG_ENABLE_TIMESTAMP, set_buf, 0,
//                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
//    if (recv_main_cmd == PRO_INQUERY_CMD_ECHO && recv_sub_cmd == BASIC_CONFIG_ENABLE_TIMESTAMP
//            && recv_len == 1)
//        cfg.enable_timestamp = recv_buf[0];
//    else
//    {
//        std::cerr << "Query Enable Timestamp Error!" << std::endl;
//        return false;
//    }

    /*******网络参数查询类指令-0xA4********/
    // 网络参数一键查询, Byte 10--13 激光传感器 IP ,14--17 激光传感器 PORT ,18--23 激光传感器 MAC, 高位在前.
    SocketSendCmdAndRev(NET_MAIN_INQUERY_CMD, SUB_CMD_NET_ALL, set_buf, 0,
                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd == NET_MAIN_INQUERY_CMD_ECHO) 
	&& (recv_sub_cmd == SUB_CMD_NET_ALL))
    {
        unsigned int ip0, ip1, ip2, ip3;
        ip0 = recv_buf[0];
        ip1 = recv_buf[1];
        ip2 = recv_buf[2];
        ip3 = recv_buf[3];
        cfg.host_ip = std::to_string(ip0) + "." + std::to_string(ip1) + "."
                      + std::to_string(ip2) + "." + std::to_string(ip3);
        cfg.data_port = (recv_buf[4]<<24) + (recv_buf[5]<<16) + (recv_buf[6]<<8) + recv_buf[7];
    }
    else
    {
        std::cerr << "Query IP & Port & MAC Error!" << std::endl;
        return false;
    }

    /*******扫描参数查询类指令-0xC4********/
    // 查询扫描范围, start_angle & stop_angle.
    SocketSendCmdAndRev(PRO_INQUERY_CMD, BASIC_CONFIG_SCAN_RANGE, set_buf, 0,
                        recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd == PRO_INQUERY_CMD_ECHO) 
	&& (recv_sub_cmd == BASIC_CONFIG_SCAN_RANGE)
    && (recv_len == 4))
    {
        // Negative value.
        if ((recv_buf[0]&0x80))
        {
            char buf[2];
            for (int i = 0; i < 2; i++)
                buf[i] = static_cast<char>(recv_buf[i]);
            cfg.start_angle = (buf[0]<<8) + buf[1];
        }
        else
            cfg.start_angle = (recv_buf[0]<<8) + recv_buf[1];
        // Negative value.
        if ((recv_buf[2]&0x80))
        {
            char buf[2];
            for (int i = 0; i < 2; i++)
                buf[i] = static_cast<char>(recv_buf[i+2]);
            cfg.stop_angle = (buf[2]<<8) + buf[3];
        }
        else
            cfg.stop_angle = (recv_buf[2]<<8) + recv_buf[3];
    }
    else
    {
        std::cerr << "Query Start & Stop Angle Error!" << std::endl;
        return false;
    }

	//\B2\E9ѯ\BDǶȷֱ\E6\C2\CA
	//1.\BD\E2\CB\F8
	set_buf[0] = 0xBB;
    set_buf[1] = 0x44;
    set_buf[2] = 0x44;
    set_buf[3] = 0xBB;
	set_buf[4] = 0x00;
	set_buf[5] = 0x44;
    set_buf[6] = 0xBB;
    set_buf[7] = 0xBB;
    set_buf[8] = 0x44;
    SocketInternalSendCmdAndRev(PRO_SET_CMD, BASIC_CONFIG_UNLOCK, set_buf, 9,
		recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd != PRO_SET_CMD_ECHO) 
		|| (recv_sub_cmd!=BASIC_CONFIG_UNLOCK)
		|| (recv_len != 1) 
		|| (recv_buf[0] != 0x00))
    {
        std::cerr << "unlock Error!" << std::endl;
        return false;
    }
	
	//2.\B6\C1ȡ\B7ֱ\E6\C2\CA
    SocketInternalSendCmdAndRev(TXCTRL_MAIN_CMD_QUERY, TXCTRL_SUB_CMD_TX_ANGEL_STEP, set_buf, 0,
		recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd == TXCTRL_MAIN_CMD_QUERY_ECHO) 
	&& (recv_sub_cmd==TXCTRL_SUB_CMD_TX_ANGEL_STEP)
	&& (recv_len == 2))
    {
        l_u16StepAngle = recv_buf[0]<< 8 | recv_buf[1];
	switch(l_u16StepAngle)
		{
		case 0:
			cfg.angle_resolution = 0.5;
			break;
		case 1:
			cfg.angle_resolution = 0.33;
			break;
		case 2:
			cfg.angle_resolution = 0.25;
			break;
		case 3:
			cfg.angle_resolution = 0.125;
			break;
		case 4:
			cfg.angle_resolution = 0.1;
			break;
		case 5:
			cfg.angle_resolution = 1;
			break;
		default:
                       std::cerr << "read angle step Error1111!!" << std::endl;
			break;
		}
    }
	else
	{
		std::cerr << "read angle step Error!"<<recv_len << std::endl;
        //return false;     //\D5\E2\B8\F6\D7\EE\BAò\BB\BA÷\B5\BBأ\AC\B7\F1\D5\DF\D5\E2ʱ\BA\F2\C0״ﴦ\D3ڽ\E2\CB\F8״̬
	}
	
	//3.\CB\F8\B6\A8
	set_buf[0] = 0xBB;
    set_buf[1] = 0x44;
    set_buf[2] = 0x44;
    set_buf[3] = 0xBB;
	set_buf[4] = 0x01;
	set_buf[5] = 0x44;
    set_buf[6] = 0xBB;
    set_buf[7] = 0xBB;
    set_buf[8] = 0x44;
    SocketInternalSendCmdAndRev(PRO_SET_CMD, BASIC_CONFIG_UNLOCK, set_buf, 9,
		recv_main_cmd, recv_sub_cmd, recv_buf, recv_len);
    if((recv_main_cmd != PRO_SET_CMD_ECHO) 
		|| (recv_sub_cmd!=BASIC_CONFIG_UNLOCK)
		|| (recv_len != 1) 
		|| (recv_buf[0] != 0x00))
    {
        std::cerr << "lock Error!" << std::endl;
        return false;
    }

    return true;
}

// @brief Receive single net message.
unsigned char Intelly::DealSocketDataInPackage(unsigned char* recv_buf, unsigned int& recv_len,
                                      unsigned char* package_buf, unsigned int& package_len)
{
	unsigned char	l_u8Ret;
    static unsigned short  frame_len;
    unsigned int    index;
    static unsigned int s_u32FrameFindWord = 0;

	
	l_u8Ret = 0xFF;
    for(index = 0; index < recv_len;)
    {
        //寻找帧头，支持同一帧数据分为两次中断来传输
        if(socket_recv_step_ == RECV_DATA_STEP_FINDING_HEAD)
        {
            //从收到的数据中找出完整的帧
            //A、以0xAA/0x88/0x88/0xAA开头、0x88/0xAA/0xAA/0x88结尾的数据
            if((recv_buf[index+0] == FRAME_DATA_HEAD1)
			&& (recv_buf[index+1] == FRAME_DATA_HEAD2)
			&& (recv_buf[index+2] == FRAME_DATA_HEAD3)
			&& (recv_buf[index+3] == FRAME_DATA_HEAD4))
            {
                //Intelly Pro
                memset(package_buf, 0, package_len);
				package_len = 0;
                memcpy(&package_buf[package_len], &recv_buf[index], 4);
                package_len     = 4;
                index               = index + 4;
                socket_recv_step_	= RECV_DATA_STEP_FINDING_END;
				socket_data_type	= FRAME_TYPE_INTELLY_FORMAL;	//intelly pro
            }
			else if((recv_buf[index+0] == FRAME_INTERNAL_DATA_HEAD1)
				 && (recv_buf[index+1] == FRAME_INTERNAL_DATA_HEAD2)
				 && (recv_buf[index+2] == FRAME_INTERNAL_DATA_HEAD3)
				 && (recv_buf[index+3] == FRAME_INTERNAL_DATA_HEAD4))
            {
                //Intelly Pro
                memset(package_buf, 0, package_len);
				package_len = 0;
                memcpy(&package_buf[package_len], &recv_buf[index], 4);
                package_len     = 4;
                index               = index + 4;
                socket_recv_step_	= RECV_DATA_STEP_FINDING_END;
				socket_data_type	= FRAME_TYPE_INTELLY_DEBUG;	//intelly pro
            }
            else if((recv_buf[index+0] == SICK_BIN_FRAME_HEAD1)
				 && (recv_buf[index+1] == SICK_BIN_FRAME_HEAD2)
				 && (recv_buf[index+2] == SICK_BIN_FRAME_HEAD3)
				 && (recv_buf[index+3] == SICK_BIN_FRAME_HEAD4))
			{
				//Sick Bin
                memset(package_buf, 0, package_len);
				package_len = 0;
                memcpy(&package_buf[package_len], &recv_buf[index], 4);
                package_len     	= 4;
                index               = index + 4;
                socket_recv_step_	= RECV_DATA_STEP_FINDING_END;
				socket_data_type	= FRAME_TYPE_SICK_BIN;			//SICK_BIN
			}
            else if(DecodeSickPkgAscii(&recv_buf[index], (recv_len-index)) == 0)
            {
                //Sick Ascii
                memset(package_buf, 0, package_len);
				package_len = 0;
                memcpy(&package_buf[package_len], &recv_buf[index], (recv_len-index));
                package_len = recv_len-index;
                socket_data_type	= FRAME_TYPE_SICK_ASCII;
                
                l_u8Ret = FRAME_TYPE_SICK_ASCII;
				socket_recv_step_ = RECV_DATA_STEP_FINDING_HEAD;
                index = recv_len;
                break;
            }
            else
            {
                index++;    //说明不是以正常的数据帧头作为开始
            }
        }
        else        //寻找帧尾
        {
            if(package_len < SOCKET_QUEUE_SIZE)
            {
                package_buf[package_len] = recv_buf[index];
                package_len++;
                index++;
				
				if(socket_data_type == FRAME_TYPE_INTELLY_FORMAL
				|| socket_data_type == FRAME_TYPE_INTELLY_DEBUG)
				{
					if(package_len == FRAME_DATA_START_POS) //4个帧头+1主命令号+1从命令号+4数据帧长度=10
					{
						//Frame len
						frame_len = (package_buf[package_len-4] << 24)
								  + (package_buf[package_len-3] << 16)
								  + (package_buf[package_len-2] << 8)
								  + (package_buf[package_len-1]);
                        
                        if(frame_len >= SOCKET_QUEUE_SIZE)
                        {
                            socket_recv_step_ = RECV_DATA_STEP_FINDING_HEAD;
                            continue;
                        }
					}
					else if (package_len > FRAME_DATA_START_POS)
					{
						if(package_len == frame_len)
						{
							if((package_buf[package_len-4] == FRAME_DATA_END1)
							&& (package_buf[package_len-3] == FRAME_DATA_END2)
							&& (package_buf[package_len-2] == FRAME_DATA_END3)
							&& (package_buf[package_len-1] == FRAME_DATA_END4))
							{
								//至此，得到一帧完整数据
								l_u8Ret = FRAME_TYPE_INTELLY_FORMAL;
								socket_recv_step_ = RECV_DATA_STEP_FINDING_HEAD;
								break;
							}
							else if((package_buf[package_len-4] == FRAME_INTERNAL_DATA_END1)
								 && (package_buf[package_len-3] == FRAME_INTERNAL_DATA_END2)
								 && (package_buf[package_len-2] == FRAME_INTERNAL_DATA_END3)
								 && (package_buf[package_len-1] == FRAME_INTERNAL_DATA_END4))
							{
								//Intelly Pro
								l_u8Ret	= FRAME_TYPE_INTELLY_DEBUG;	//intelly pro
								socket_recv_step_	= RECV_DATA_STEP_FINDING_END;
								break;
							}
						}
                        else if(package_len > frame_len )
                        {
                            socket_recv_step_ = RECV_DATA_STEP_FINDING_HEAD;
                        }
					}				
				}
				else if(socket_data_type == FRAME_TYPE_SICK_BIN)
				{
					if(package_len == SICK_BIN_FRAME_DATA_POS_START)
					{
						//当前帧总长度：包括帧头帧尾所有字节
						frame_len = (package_buf[package_len-4] << 24)
								  + (package_buf[package_len-3] << 16)
								  + (package_buf[package_len-2] << 8)
								  + (package_buf[package_len-1]);
                        if((frame_len-SICK_BIN_FRAME_NUM_MIN) >= SOCKET_QUEUE_SIZE)
                        {
                            socket_recv_step_ = RECV_DATA_STEP_FINDING_HEAD;
                            continue; 
                        }
					}
					else if(package_len > SICK_BIN_FRAME_DATA_POS_START)
					{
						if(package_len == (frame_len+SICK_BIN_FRAME_NUM_MIN))
						{
							//至此，得到一帧完整数据
							l_u8Ret = FRAME_TYPE_SICK_BIN;
							socket_recv_step_ = RECV_DATA_STEP_FINDING_HEAD;
							break;
						}
                        else if(package_len > (frame_len+SICK_BIN_FRAME_NUM_MIN))
                        {
                            socket_recv_step_ = RECV_DATA_STEP_FINDING_HEAD;
                        }
					}
				}
            }
            else            //超过最大范围都没找到帧尾，放弃
            {
                socket_recv_step_ = RECV_DATA_STEP_FINDING_HEAD;             
            }
        }
    }
	
	return l_u8Ret;
}

// @brief Find Sick Ascii Pro
int Intelly::DecodeSickPkgAscii(unsigned char p_CodeBuf[], unsigned int p_Len)
{
    //Frame Head sRA LMDscandata 4C 4D 44 73 63 61 6E 64 61 74 61 20
	if(p_Len < 17)
	{
		return 1;
	}

	if ((p_CodeBuf[0] != 0x02) || (p_CodeBuf[p_Len-1] != 0x03))
	{
		return 1;
	}

	if ((p_CodeBuf[1] != 0x73) || (p_CodeBuf[4] != 0x20) || (p_CodeBuf[5] != 0x4C) ||
		(p_CodeBuf[6] != 0x4D) || (p_CodeBuf[7] != 0x44) || (p_CodeBuf[8] != 0x73) ||
		(p_CodeBuf[9] != 0x63) || (p_CodeBuf[10] != 0x61) || (p_CodeBuf[11] != 0x6E) ||
		(p_CodeBuf[12] != 0x64) || (p_CodeBuf[13] != 0x61) || (p_CodeBuf[14] != 0x74) || (p_CodeBuf[15] != 0x61))
	{
		return 1;
	}

	return 0;
}

// @brief Decode single net message.
bool Intelly::SocketDataDecoderForIntelly(const unsigned char* buf, unsigned int& len,
                                unsigned char* target_buf, unsigned int& target_len,
                                unsigned char& main_cmd, unsigned char& sub_cmd)
{
	signed int		i, j;
    unsigned int	pos=0;
    unsigned char	check=0; // 校验位
	unsigned char	l_u8FrameHeadAndEndCheck;

    //解析后的参数
    unsigned int frame_len;//数据帧长度

    //4字节帧头+1主命令+1从命令+4字节长度+1字节chk+4字节帧尾=15，至少15个字节
    if(len < FRAME_DATA_MIN_LEN)
    {
		return false;
    }
	
	l_u8FrameHeadAndEndCheck = 0;
	// Head check
    if((FRAME_DATA_HEAD1 != buf[0]) || (FRAME_DATA_HEAD2 != buf[1]) 
	|| (FRAME_DATA_HEAD3 != buf[2]) || (FRAME_DATA_HEAD4 != buf[3]))
	{
		l_u8FrameHeadAndEndCheck = 1;
	}

	if((FRAME_DATA_END1 != buf[len-4]) || (FRAME_DATA_END2 != buf[len-3])
	|| (FRAME_DATA_END3 != buf[len-2]) || (FRAME_DATA_END4 != buf[len-1]))
    {
		l_u8FrameHeadAndEndCheck = 2;
    }// Skip frame headers.
	
	if(l_u8FrameHeadAndEndCheck != 0)
	{
		return false;
	}

	pos = FRAME_DATA_MAIN_CMD_POS;

    //命令号
    main_cmd = buf[pos++];      //主命令号
    sub_cmd = buf[pos++];       //从命令号

    //数据帧长度,包含帧头、帧尾所有字节
    frame_len = (buf[pos] << 24) | (buf[pos+1] << 16) | (buf[pos+2] << 8) |buf[pos+3];
    pos += 4;

    //校验数据帧长度是否正确
    if(len != frame_len)  //总长度减去帧头和帧尾
    {
		return false;
    }
	else
    {
//		for(i=4; i<len-5; i++)//len-1、len-2、len-3、len-4这是4个帧尾标志，len-5这是chk位
//		{
//			check ^= buf[i];
//		}
//        
//		// 校验
//		if(check != buf[len-5])
//		{
//			return false;   //校验不正确
//		}
		
        // data
        for(i=10, j=0; i<(len-5); i++,j++)
        {
			target_buf[j] = buf[i];
        }
		target_len = len-15;

        return true;
    }

    return true;
}

// @brief Decode single net message.
bool Intelly::SocketInternalDecoderForIntelly(const unsigned char* buf, unsigned int& len,
										      unsigned char* target_buf, unsigned int& target_len,
										      unsigned char& main_cmd, unsigned char& sub_cmd)
{
	signed int		i, j;
    unsigned int	pos=0;
    unsigned char	check=0; // 校验位
	unsigned char	l_u8FrameHeadAndEndCheck;
	
    //解析后的参数
    unsigned int frame_len;//数据帧长度
	
    //4字节帧头+1主命令+1从命令+4字节长度+1字节chk+4字节帧尾=15，至少15个字节
    if(len < FRAME_DATA_MIN_LEN)
    {
		return false;
    }
	
	l_u8FrameHeadAndEndCheck = 0;
	// Head check
    if((FRAME_INTERNAL_DATA_HEAD1 != buf[0]) || (FRAME_INTERNAL_DATA_HEAD2 != buf[1]) 
		|| (FRAME_INTERNAL_DATA_HEAD3 != buf[2]) || (FRAME_INTERNAL_DATA_HEAD4 != buf[3]))
	{
		l_u8FrameHeadAndEndCheck = 1;
	}
	
	if((FRAME_INTERNAL_DATA_END1 != buf[len-4]) || (FRAME_INTERNAL_DATA_END2 != buf[len-3])
		|| (FRAME_INTERNAL_DATA_END3 != buf[len-2]) || (FRAME_INTERNAL_DATA_END4 != buf[len-1]))
    {
		l_u8FrameHeadAndEndCheck = 2;
    }// Skip frame headers.
	
	if(l_u8FrameHeadAndEndCheck != 0)
	{
		return false;
	}
	
	pos = FRAME_DATA_MAIN_CMD_POS;
	
    //命令号
    main_cmd = buf[pos++];      //主命令号
    sub_cmd = buf[pos++];       //从命令号
	
    //数据帧长度,包含帧头、帧尾所有字节
    frame_len = (buf[pos] << 24) | (buf[pos+1] << 16) | (buf[pos+2] << 8) |buf[pos+3];
    pos += 4;
	
    //校验数据帧长度是否正确
    if(len != frame_len)  //总长度减去帧头和帧尾
    {
		return false;
    }
	else
    {
		//		for(i=4; i<len-5; i++)//len-1、len-2、len-3、len-4这是4个帧尾标志，len-5这是chk位
		//		{
		//			check ^= buf[i];
		//		}
		//        
		//		// 校验
		//		if(check != buf[len-5])
		//		{
		//			return false;   //校验不正确
		//		}
		
        // data
        for(i=10, j=0; i<(len-5); i++,j++)
        {
			target_buf[j] = buf[i];
        }
		target_len = len-15;
		
        return true;
    }
	
    return true;
}

// @brief Decode single net message.
bool Intelly::SocketDataDecoderForSickBin(const unsigned char* buf, unsigned int& len,
                                unsigned char* target_buf, unsigned int& target_len,
                                unsigned char& main_cmd, unsigned char& sub_cmd)
{
	signed int		i, j;
    unsigned int	pos=0;
    unsigned char	l_u8CheckRes=0; // 校验位
	unsigned char	l_u8FrameHeadAndEndCheck;

    //解析后的参数
    unsigned int	frame_len;//数据帧长度
	unsigned int	l_u32SickDataLen, l_u32RealDataLen;

    //帧头4+长度4+校验1
    if(len < SICK_BIN_FRAME_NUM_MIN)
    {
		return false;
    }
	
	l_u8FrameHeadAndEndCheck = 0;
	// Head check
    if((SICK_BIN_FRAME_HEAD1 != buf[0]) 
	|| (SICK_BIN_FRAME_HEAD2 != buf[1]) 
	|| (SICK_BIN_FRAME_HEAD3 != buf[2])
    || (SICK_BIN_FRAME_HEAD4 != buf[3]))
    {
		l_u8FrameHeadAndEndCheck = 1;
    }// Skip frame headers.
    
	if(l_u8FrameHeadAndEndCheck != 0)
	{
		return false;
	}
	
	pos = SICK_BIN_FRAME_DATA_LEN_POS;

    //数据帧长度,包含帧头、帧尾所有字节
    l_u32RealDataLen = (buf[pos] << 24) | (buf[pos+1] << 16) | (buf[pos+2] << 8) |buf[pos+3];
	l_u32SickDataLen = l_u32RealDataLen + SICK_BIN_FRAME_NUM_MIN;
	pos += 4;
	
	if(l_u32SickDataLen > len)
	{
		return false;
	}
    
//	l_u8CheckRes = 0;
//	for(i = SICK_BIN_FRAME_DATA_POS_START; i < (l_u32SickDataLen - 1); i++)
//	{
//		l_u8CheckRes ^= buf[i];
//	}
//	
//	if(l_u8CheckRes != buf[l_u32SickDataLen-1])
//	{
//		return false;
//	}
	
	memcpy(&target_buf[0], &buf[SICK_BIN_FRAME_DATA_POS_START], l_u32RealDataLen);
	target_len = l_u32RealDataLen;
	
    return true;
}

// @brief Decode single net message.
bool Intelly::SocketDataDecoderForSickAscii(const unsigned char* buf, unsigned int& len,
                                unsigned char* target_buf, unsigned int& target_len,
                                unsigned char& main_cmd, unsigned char& sub_cmd)
{
    unsigned int    l_u32RealDataLen;

	//check
    if(buf[0] != 0x02 || buf[len-1] != 0x03)
    {
        return false;
    }
	
    l_u32RealDataLen = len - 2;

	memcpy(&target_buf[0], &buf[1], l_u32RealDataLen);
	target_len = l_u32RealDataLen;

    return true;
}

bool Intelly::AsciiFrameFindValidData(const unsigned char *p_u8TargetBuf, unsigned int& p_u32TargetPos, signed int& p_s32TargetData)
{
    signed int      l_s32ResultData;    
    unsigned char   l_u8TempBuf;  
    unsigned int    l_u32ErrorCount, l_u32Index;
    
    l_u32Index = 0;
    l_u32ErrorCount = 0;
    l_s32ResultData = 0;

    while(p_u8TargetBuf[l_u32Index] != 0x20)
    {
	    if(p_u8TargetBuf[l_u32Index] >= 48 && p_u8TargetBuf[l_u32Index] <= 57)
	    {
		    l_u8TempBuf = p_u8TargetBuf[l_u32Index] - 48;
	    }
	    else if(p_u8TargetBuf[l_u32Index] > 58)
	    {
		    l_u8TempBuf = p_u8TargetBuf[l_u32Index] - 55;
	    }
        
        l_s32ResultData = l_u8TempBuf | (l_s32ResultData << 4);
        l_u32Index++;

	    l_u32ErrorCount++;
	    if(l_u32ErrorCount > 0xFF)
	    {
		    return false;
	    }
    }
    l_u32Index += 1;

    p_u32TargetPos = l_u32Index;
    p_s32TargetData = l_s32ResultData;
    
    return true;
}

// @brief 通用数据编码--帧头是aa aa 77 77，帧尾aa 77 77 aa.
bool Intelly::SocketDataEncoder(const unsigned char& main_cmd, const unsigned char& sub_cmd,
                                const unsigned char* buf, const unsigned int& len, unsigned char* target_buf,
                                unsigned int& target_len)
{
    unsigned int    slen=0,pos_frame_lenth = 0;
    unsigned int    total_len;
    unsigned char   buf_check = 0;

    target_len = 0 ;

    if(len<0)
    {
		return false;
	}
	
    /*********************第一步：组包，除去开头、校验和结尾的所有内容***********************/;
    //主节点ID
    slen = 0;
    // frame head
    target_buf[slen++] = FRAME_DATA_HEAD1;
    target_buf[slen++] = FRAME_DATA_HEAD2;
    target_buf[slen++] = FRAME_DATA_HEAD3;
    target_buf[slen++] = FRAME_DATA_HEAD4;

    //命令号
    target_buf[slen++] = main_cmd;
    target_buf[slen++] = sub_cmd;
    //跳过数据帧长度
    pos_frame_lenth = slen;
    slen += 4;

    //数据内容
    memcpy(&target_buf[slen],buf,len);
    slen += len;

    //长度信息
    total_len = slen+1+4;//需再加上1个字节的校验位、4个字节的结束位
    target_buf[pos_frame_lenth]     = (total_len >> 24) & 0xff;
    target_buf[pos_frame_lenth+1]   = (total_len >> 16) & 0xff;
    target_buf[pos_frame_lenth+2]   = (total_len >> 8) & 0xff;
    target_buf[pos_frame_lenth+3]   = (total_len >> 0) & 0xff;

    /*********************第二步：转义，加上开头、校验和结尾，(包括校验位)***********************/;
    for (int i = 4; i < slen; i++)
    {
        buf_check ^= target_buf[i]; // XOR
    }
    target_buf[slen++] = buf_check;
    // Frame end.
    target_buf[slen++] = FRAME_DATA_END1;
    target_buf[slen++] = FRAME_DATA_END2;
    target_buf[slen++] = FRAME_DATA_END3;
    target_buf[slen++] = FRAME_DATA_END4;
    target_len = slen;

    return true;
}




// @brief socket send cmd and recevie data buffer.
void Intelly::SocketSendCmdAndRev(const unsigned char& set_main_cmd, const unsigned char& set_sub_cmd,  // send cmd.
                                  const unsigned char* cmd_buf, const unsigned int& cmd_len,    // set buf.
                                  unsigned char& recv_main_cmd, unsigned char& recv_sub_cmd,  // rev cmd.
                                  unsigned char* data_buf, unsigned int& data_len)  // rev data buf.
{

    unsigned char package_buf[256];  // one package buffer.
    unsigned int package_len = 0;    // package buffer len.
    unsigned char recv_buf[50];
    unsigned int recv_len = 0;
    unsigned char target_buf[256];  // rev package buffer.
    unsigned int target_len = 0;    // socket buffer len.
	unsigned char l_u8RecvDataType = 0;

    // buffer data encode.
    if (!SocketDataEncoder(set_main_cmd, set_sub_cmd, cmd_buf, cmd_len, package_buf, package_len))
        return;
    // send to socket.
    write(socket_fd_, package_buf, package_len);
    // Receive intelly socket back.
    recv_len = read(socket_fd_, recv_buf, 50);
    // Deal socket data in package.
    l_u8RecvDataType = DealSocketDataInPackage(recv_buf, recv_len, target_buf, target_len);
    // Decode single net message.
    
	if(l_u8RecvDataType == FRAME_TYPE_INTELLY_FORMAL)
	{
		if (!SocketDataDecoderForIntelly(target_buf, target_len, data_buf, data_len, recv_main_cmd, recv_sub_cmd))
			return;		
	}
	else if(l_u8RecvDataType == FRAME_TYPE_SICK_BIN)
	{
		if (!SocketDataDecoderForSickBin(target_buf, target_len, data_buf, data_len, recv_main_cmd, recv_sub_cmd))
			return;	
	}
    else if(l_u8RecvDataType == FRAME_TYPE_SICK_ASCII)
    {
		if (!SocketDataDecoderForSickAscii(target_buf, target_len, data_buf, data_len, recv_main_cmd, recv_sub_cmd))
			return;	
    }
	else if(l_u8RecvDataType == FRAME_TYPE_INTELLY_DEBUG)
	{
		if (!SocketInternalDecoderForIntelly(target_buf, target_len, data_buf, data_len, recv_main_cmd, recv_sub_cmd))
			return;	
	}
}


// @brief 通用数据编码--帧头是aa aa 77 77，帧尾aa 77 77 aa.
bool Intelly::SocketInternalEncoder(const unsigned char& main_cmd, const unsigned char& sub_cmd,
                                    const unsigned char* buf, const unsigned int& len, unsigned char* target_buf,
                                    unsigned int& target_len)
{
    unsigned int    slen=0,pos_frame_lenth = 0;
    unsigned int    total_len;
    unsigned char   buf_check = 0;
	
    target_len = 0 ;
	
    if(len<0)
    {
		return false;
	}
	
    /*********************第一步：组包，除去开头、校验和结尾的所有内容***********************/;
    //主节点ID
    slen = 0;
    // frame head
    target_buf[slen++] = FRAME_INTERNAL_DATA_HEAD1;
    target_buf[slen++] = FRAME_INTERNAL_DATA_HEAD2;
    target_buf[slen++] = FRAME_INTERNAL_DATA_HEAD3;
    target_buf[slen++] = FRAME_INTERNAL_DATA_HEAD4;
	
    //命令号
    target_buf[slen++] = main_cmd;
    target_buf[slen++] = sub_cmd;
    //跳过数据帧长度
    pos_frame_lenth = slen;
    slen += 4;
	
    //数据内容
    memcpy(&target_buf[slen],buf,len);
    slen += len;
	
    //长度信息
    total_len = slen+1+4;//需再加上1个字节的校验位、4个字节的结束位
    target_buf[pos_frame_lenth]     = (total_len >> 24) & 0xff;
    target_buf[pos_frame_lenth+1]   = (total_len >> 16) & 0xff;
    target_buf[pos_frame_lenth+2]   = (total_len >> 8) & 0xff;
    target_buf[pos_frame_lenth+3]   = (total_len >> 0) & 0xff;
	
    /*********************第二步：转义，加上开头、校验和结尾，(包括校验位)***********************/;
    for (int i = 4; i < slen; i++)
    {
        buf_check ^= target_buf[i]; // XOR
    }
    target_buf[slen++] = buf_check;
    // Frame end.
    target_buf[slen++] = FRAME_INTERNAL_DATA_END1;
    target_buf[slen++] = FRAME_INTERNAL_DATA_END2;
    target_buf[slen++] = FRAME_INTERNAL_DATA_END3;
    target_buf[slen++] = FRAME_INTERNAL_DATA_END4;
    target_len = slen;
	
    return true;
}

// @brief socket send cmd and recevie data buffer.
void Intelly::SocketInternalSendCmdAndRev(const unsigned char& set_main_cmd, const unsigned char& set_sub_cmd,  // send cmd.
                                  const unsigned char* cmd_buf, const unsigned int& cmd_len,    // set buf.
                                  unsigned char& recv_main_cmd, unsigned char& recv_sub_cmd,  // rev cmd.
                                  unsigned char* data_buf, unsigned int& data_len)  // rev data buf.
{
	
    unsigned char package_buf[256];  // one package buffer.
    unsigned int package_len = 0;    // package buffer len.
    unsigned char recv_buf[50];
    unsigned int recv_len = 0;
    unsigned char target_buf[256];  // rev package buffer.
    unsigned int target_len = 0;    // socket buffer len.
	unsigned char l_u8RecvDataType = 0;
	
    // buffer data encode.
    if (!SocketInternalEncoder(set_main_cmd, set_sub_cmd, cmd_buf, cmd_len, package_buf, package_len))
        return;
    // send to socket.

    write(socket_fd_, package_buf, package_len);
    // Receive intelly socket back.

    recv_len = read(socket_fd_, recv_buf, 50);
    // Deal socket data in package.
    l_u8RecvDataType = DealSocketDataInPackage(recv_buf, recv_len, target_buf, target_len);
    // Decode single net message.
    
	if(l_u8RecvDataType == FRAME_TYPE_INTELLY_FORMAL)
	{
		if (!SocketDataDecoderForIntelly(target_buf, target_len, data_buf, data_len, recv_main_cmd, recv_sub_cmd))
			return;		
	}
	else if(l_u8RecvDataType == FRAME_TYPE_SICK_BIN)
	{
		if (!SocketDataDecoderForSickBin(target_buf, target_len, data_buf, data_len, recv_main_cmd, recv_sub_cmd))
			return;	
	}
    else if(l_u8RecvDataType == FRAME_TYPE_SICK_ASCII)
    {
		if (!SocketDataDecoderForSickAscii(target_buf, target_len, data_buf, data_len, recv_main_cmd, recv_sub_cmd))
			return;	
    }
	else if(l_u8RecvDataType == FRAME_TYPE_INTELLY_DEBUG)
	{
		if (!SocketInternalDecoderForIntelly(target_buf, target_len, data_buf, data_len, recv_main_cmd, recv_sub_cmd))
			return;	
	}
}

// @brief Parse laser scan Data.
// @return true--成功，false--失败.
bool Intelly::ParseScanData(unsigned char *buf, unsigned int& len, ScanData& scan_data)
{
	signed int      i;
    unsigned char   target_buf[SOCKET_QUEUE_SIZE];  // socket buffer.
    unsigned int    target_len = 0;    // socket buffer len.
    unsigned char   main_cmd, sub_cmd;
	unsigned char   l_u8DecoderCheck, l_u8TimeStamp;
	unsigned int    l_u32Index = 0, l_u32AngelStep = 0;
    signed int      l_s32StartAngel = 0, l_s32FrameFindRes;
    unsigned int    l_u32FindPos, l_u8SickChannel;
    

    // Decode single net message.
    // target_buf:解码完成数据，去掉了帧头帧尾和校验位，只包括帧数据位.
	l_u8DecoderCheck = 0;
    if(SocketDataDecoderForIntelly(buf, len, target_buf, target_len, main_cmd, sub_cmd))
    {
		l_u8DecoderCheck = 1;
    }
	else if(SocketDataDecoderForSickBin(buf, len, target_buf, target_len, main_cmd, sub_cmd))
	{
		l_u8DecoderCheck = 2;
	}
    else if(SocketDataDecoderForSickAscii(buf, len, target_buf, target_len, main_cmd, sub_cmd))
    {
        l_u8DecoderCheck = 3;
    }   
	
	if(l_u8DecoderCheck == 0)
	{
		return false;
	}
	
	if(l_u8DecoderCheck == 1)
	{
		if(main_cmd == 0xd5 && sub_cmd == 0x01)    // Scan data.
		{
			l_u32Index = 0;
			l_u32Index += 2;
			// start angle. byte 2--3
			l_u32Index += 2;
			//Angel Step
			l_u32Index += 2;
			//    if (buf[0] != scan_cfg_.start_angle)
			//        logWarn("Start angle not equal to ScanCfg.start_angle!");
			// 距离总个数 N, byte 6--7
			scan_data.num_values = (target_buf[l_u32Index]<<8) + target_buf[l_u32Index+1];
			l_u32Index += 2;
			// ranges[0:n-1]; 
			for (i = 0; i < scan_data.num_values; i++)
			{
				scan_data.ranges[i] = (target_buf[l_u32Index]<<8) + target_buf[l_u32Index+1];
				l_u32Index += 2;
			}

			// 时间戳标志
			/*****TODO******/

			return true;
		}
        else if(main_cmd == 0xd5 && sub_cmd == 0x03)
        {
			l_u32Index = 0;
			l_u32Index += 2;
			// start angle. byte 2--3
			l_u32Index += 2;
			//Angel Step
			l_u32Index += 2;
			//    if (buf[0] != scan_cfg_.start_angle)
			//        logWarn("Start angle not equal to ScanCfg.start_angle!");
			// 距离总个数 N, byte 6--7
			scan_data.num_values = (target_buf[l_u32Index]<<8) + target_buf[l_u32Index+1];
			l_u32Index += 2;
			// ranges[0:n-1]; 
			for (i = 0; i < scan_data.num_values; i++)
			{
				scan_data.ranges[i] = (target_buf[l_u32Index]<<8) + target_buf[l_u32Index+1];
				l_u32Index += 2;
			}

            for (i = 0; i < scan_data.num_values; i++)
			{
				scan_data.intensities[i] = (target_buf[l_u32Index+1] << 8) + target_buf[l_u32Index];
				l_u32Index += 2;
			}
			// 时间戳标志
			/*****TODO******/
            l_u8TimeStamp = target_buf[l_u32Index++];

			return true;
        }	
	}
	else if(l_u8DecoderCheck == 2)
	{
		l_u32Index = 0;
		l_u32Index += SICK_BIN_COMMON_DATA_LEN_1;	//第一段固定长度
		//Start Angel
		//    if (buf[0] != scan_cfg_.start_angle)
		//        logWarn("Start angle not equal to ScanCfg.start_angle!");
		l_u32Index += 4;
		//Angel Step
		l_u32Index += 2;
		//距离总个数N
		scan_data.num_values = (target_buf[l_u32Index]<<8) + target_buf[l_u32Index+1];
		l_u32Index += 2;
		

		// ranges[0:n-1]; 
		for (i = 0; i < scan_data.num_values; i++)
		{
			scan_data.ranges[i] = (target_buf[l_u32Index]<<8) + target_buf[l_u32Index+1];
			l_u32Index += 2;
		}

        //channels 2 + Output channel 5 + Scale factor 4 + Scale factor offset 4
        l_u8SickChannel = (target_buf[l_u32Index]<<8) + target_buf[l_u32Index+1];
        l_u32Index += SICK_BIN_COMMON_DATA_LEN_4;
        
        if(l_u8SickChannel != 0)
        {
            //Start Angel 4
            l_u32Index += 4;

            //Angel Step 2
            l_u32Index += 2;

            //intensities num same with dist num
            l_u32Index += 2;

            //intensities[0:n-1]
            for (i = 0; i < scan_data.num_values; i++)
		    {
			    scan_data.intensities[i] = target_buf[l_u32Index++];
		    }
        }

		
		return true;
	}
    else if(l_u8DecoderCheck == 3)
    {
        l_u32Index = 0;
        l_u32Index += SICK_ASCII_COMMON_DATA_LEN_1;

        //Start Angel
        if(!AsciiFrameFindValidData(&target_buf[l_u32Index], l_u32FindPos, l_s32FrameFindRes))
        {
            return false;
        }
        else
        {   
            l_s32StartAngel = l_s32FrameFindRes;
            l_u32Index += l_u32FindPos;
        }

        //Angel Step
        if(!AsciiFrameFindValidData(&target_buf[l_u32Index], l_u32FindPos, l_s32FrameFindRes))
        {
            return false;
        }
        else
        {   
            l_u32AngelStep = l_s32FrameFindRes;
            l_u32Index += l_u32FindPos;
        }

        //距离总个数N
        if(!AsciiFrameFindValidData(&target_buf[l_u32Index], l_u32FindPos, l_s32FrameFindRes))
        {
            return false;
        }
        else
        {   
            scan_data.num_values = l_s32FrameFindRes;
            l_u32Index += l_u32FindPos;
        }

        // ranges[0:n-1]; 
		for (i = 0; i < scan_data.num_values; i++)
		{
            if(!AsciiFrameFindValidData(&target_buf[l_u32Index], l_u32FindPos, l_s32FrameFindRes))
            {
                return false;
            }
            else
            {   
                scan_data.ranges[i] = l_s32FrameFindRes;
                l_u32Index += l_u32FindPos;
            }
		}
            
        if(!AsciiFrameFindValidData(&target_buf[l_u32Index], l_u32FindPos, l_s32FrameFindRes))
        {
            return false;
        }
        else
        {   
            l_u8SickChannel = l_s32FrameFindRes;
            l_u32Index += l_u32FindPos;
        }
        l_u32Index += SICK_ASCII_COMMON_DATA_LEN_3;
        
        if(l_u8SickChannel != 0)
        {
            //Start Angel
            if(!AsciiFrameFindValidData(&target_buf[l_u32Index], l_u32FindPos, l_s32FrameFindRes))
            {
                return false;
            }
            else
            {   
                l_s32StartAngel = l_s32FrameFindRes;
                l_u32Index += l_u32FindPos;
            }

            //Angel Step
            if(!AsciiFrameFindValidData(&target_buf[l_u32Index], l_u32FindPos, l_s32FrameFindRes))
            {
                return false;
            }
            else
            {   
                l_u32AngelStep = l_s32FrameFindRes;
                l_u32Index += l_u32FindPos;
            }

            //intensities num same with dist num
            if(!AsciiFrameFindValidData(&target_buf[l_u32Index], l_u32FindPos, l_s32FrameFindRes))
            {
                return false;
            }
            else
            {   
                scan_data.num_values = l_s32FrameFindRes;
                l_u32Index += l_u32FindPos;
            }

            //intensities[0:n-1]
            for (i = 0; i < scan_data.num_values; i++)
		    {
                if(!AsciiFrameFindValidData(&target_buf[l_u32Index], l_u32FindPos, l_s32FrameFindRes))
                {
                    return false;
                }
                else
                {   
                    scan_data.intensities[i] = l_s32FrameFindRes;
                    l_u32Index += l_u32FindPos;
                }
		    }   
        }


        return true;
    }
    else    // Not Scan data.
    {
        std::cerr << "Not scan data, return." << std::endl;
        return false;
    }
}

// @brief cache a package of laser scan data into scan_data_buf_.
void Intelly::CacheScanData()
{
    unsigned char recv_buf[SOCKET_QUEUE_SIZE];
    unsigned char buf[SOCKET_QUEUE_SIZE];
    unsigned int recv_len = 0;
    unsigned char main_cmd, sub_cmd;
    // one package buffer.
    unsigned char package_buf[SOCKET_QUEUE_SIZE];  // socket buffer.
    unsigned int package_len = 0;    // socket buffer len.
    static bool first_flag = true;  // first time in, wait 5s, else, wait 1s.
    unsigned char l_u8Ret;
//    int size = SOCKET_QUEUE_SIZE;
//    int len = 0;
    // After start laser is scanning.
    while(is_scanning_)
    {
//        size = SOCKET_QUEUE_SIZE;

        // 0-主动传输模式
        if (scan_cfg_.transmission_mode == 0x00)
        {
            /**** When the network cable unplugged, Exit.****/
            // Only work when 'SOCKET_QUEUE_SIZE=1101'.
//            uint32 start_time = getms();
//            while (size>0)//剩余部分大于0
//            {
//                std::cout << "---size: " << size << std::endl;
//                sleep_ms(scan_cfg_.reporting_interval);
//                len= recv(socket_fd_, buf, size, MSG_DONTWAIT); // recv data from socket.
//                if(len == -1)   // error, returns -1.
//                {
//                    if ((getms()-start_time) >= (first_flag==true ? 10*DEFAULT_TIMEOUT:DEFAULT_TIMEOUT))   // timeout 5s/2s.
//                    {
//                        is_scanning_ = false;
//                        break;
//                    }
//                    else
//                        continue;
//                }
//                size = size - len;  //
//                memcpy(recv_buf+recv_len, buf, len);
//                //                    for (int i = 0; i < len; i++)
//                //                    {
//                //                        recv_buf[recv_len+i] = buf[i];
//                //                    }
//                recv_len += len;
//            }

//            first_flag = false; // first time in, wait 5s, else, wait 1s.


//            // Receive intelly socket back.
//            //        recv_len = read(socket_fd_, recv_buf, SOCKET_QUEUE_SIZE);
            if ((recv_len = recv(socket_fd_, recv_buf, SOCKET_QUEUE_SIZE, 0)) == -1)     // MSG_WAITALL
            {
                is_scanning_ = false;
                std::cerr << "Receive data error!" << std::endl;
                break;
            }
        }
        else
        {
            if ((recv_len = recv(socket_fd_, recv_buf, SOCKET_QUEUE_SIZE, 0)) == -1)     // MSG_WAITALL
            {
                is_scanning_ = false;
                std::cerr << "Receive data error!" << std::endl;
                break;
            }
        }

        /**** Not used now.****/
        // else
        // 1-被动传输模式.

        // Get single package.
        buf_mutex_.lock();
        l_u8Ret = DealSocketDataInPackage(recv_buf, recv_len, scan_data_buf_, scan_data_len_);
        buf_mutex_.unlock();
        recv_len = 0;   // for circle.

    }

    is_scanning_ = false;


}

/*****************End of file*********************/
