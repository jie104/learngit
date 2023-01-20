#include "c200_lidar/c200_lidar.h"

C200::C200() : connected_(false) , nh_("~")
{
    laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 100);

    nh_.param<std::string>("host", host_, "192.168.1.111");
    nh_.param<int>("port", port_, 2111);
    nh_.param<std::string>("frame_id", frame_id_, "/scan");
    nh_.param<double>("range_min", range_min_, 0.05);
    nh_.param<double>("range_max", range_max_, 30);
    nh_.param<int>("offset_angle", offset_angle_, -90);
    nh_.param<bool>("filterswitch", filterswitch_, true);
    nh_.param<bool>("NTPswitch", NTPswitch_, true);
    //add by zhurui 2022/10/10
    nh_.param<bool>("NORswitch", NORswitch_, true);

    laser_scan.header.frame_id = frame_id_;
    laser_scan.range_min = range_min_;
    laser_scan.range_max = range_max_;
    data.clear();
}


C200::~C200()
{
}

void C200::connect(std::string host, int port)
{
    printf("Creating non-blocking socket.\n");
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(socket_fd_) {
        struct sockaddr_in sock_addr;
        sock_addr.sin_family = PF_INET;
        sock_addr.sin_port = htons(port);
        inet_pton(AF_INET, host.c_str(), &sock_addr.sin_addr);

        printf("Connecting socket to laser.\n");
        // ::abort();
        int ret = ::connect(socket_fd_, (struct sockaddr *) &sock_addr, sizeof(sock_addr));
        if (ret == 0) {
            connected_ = true;
            printf("Connected succeeded.\n");
        }
    }
}

void C200::disconnect()
{
    if (connected_) {
        close(socket_fd_);
        connected_ = false;
    }
}

bool C200::isConnected()
{
    return connected_;
}

void C200::login()
{
    const size_t cmdLength = 14;
    uint8_t buf[50], checkSum = 0;
    int res;

    buf[0] = 0x02;
    buf[1] = 0x02;
    buf[2] = 0x02;
    buf[3] = 0x02;
    buf[4] = (cmdLength & 0xff00) >> 8;
    buf[5] = (cmdLength & 0x00ff);
    buf[6] = METHOD_DOWN;
    buf[7] = LOGIN;
    buf[8] = 0x02;
    buf[9] = 0x20;
    buf[10] = 0x21;
    buf[11] = 0x05;
    buf[12] = 0x18;

    for (size_t  i= 0; i < cmdLength - 1; i++) {
        checkSum += buf[i];
    }
    buf[cmdLength - 1] = checkSum;

    fd_set read_set;
    struct timeval timeout;

    do {
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        write(socket_fd_, buf, cmdLength);

        FD_ZERO(&read_set);
        FD_SET(socket_fd_, &read_set);
        res = select(socket_fd_ + 1, &read_set, NULL, NULL, &timeout);
    }
    while (res <= 0);

    size_t len = read(socket_fd_, buf, 50);
    if (buf[0] != 0x02 || buf[1] != 0x02 || buf[2] != 0x02 || buf[3] != 0x02 || len != 10) {
        printf("invalid packet recieved.\n");
    }

    if (buf[6] != METHOD_UP) {
        printf("opt code error.\n");
    }

    if (buf[7] != LOGIN) {
        printf("cmd code error.\n");
    }

    checkSum = 0;
    for (size_t i = 0; i < len - 1; i++){
        checkSum += buf[i];
    }
    if (buf[len - 1] != checkSum) {
        printf("checksum error.\n");
    }

    printf("RX: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
}


void C200::SetReflectSwitch(uint8_t reflag)
{
    const size_t cmdLength = 10;
    uint8_t buf[50], checkSum = 0;

    buf[0] = 0x02;
    buf[1] = 0x02;
    buf[2] = 0x02;
    buf[3] = 0x02;
    buf[4] = (cmdLength & 0xff00) >> 8;
    buf[5] = (cmdLength & 0x00ff);
    buf[6] = WRITE_DOWN;
    buf[7] = SET_REFLECTSWITCH;
    buf[8] = reflag;

    for (size_t  i= 0; i < cmdLength - 1; i++) {
        checkSum += buf[i];
    }
    buf[cmdLength - 1] = checkSum;

    write(socket_fd_, buf, cmdLength);

    size_t len = read(socket_fd_, buf, sizeof(buf));

    if (buf[0] != 0x02 || buf[1] != 0x02 || buf[2] != 0x02 || buf[3] != 0x02 || len != 10) {
        printf("invalid packet recieved.\n");
    }

    if (buf[6] != WRITE_UP) {
        printf("opt code error.\n");
    }

    if (buf[7] != SET_REFLECTSWITCH) {
        printf("cmd code error.\n");
    }

    checkSum = 0;
    for (size_t i = 0; i < len - 1; i++){
        checkSum += buf[i];
    }
    if (buf[len - 1] != checkSum) {
        printf("checksum error.\n");
    }

    printf("RX: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
}

void C200::getDeviceID()
{
    const size_t cmdLength = 9;
    uint8_t buf[50], checkSum = 0;

    buf[0] = 0x02;
    buf[1] = 0x02;
    buf[2] = 0x02;
    buf[3] = 0x02;
    buf[4] = (cmdLength & 0xff00) >> 8;
    buf[5] = (cmdLength & 0x00ff);
    buf[6] = METHOD_DOWN;
    buf[7] = GET_DEV_ID;

    for (size_t  i= 0; i < cmdLength - 1; i++) {
        checkSum += buf[i];
    }
    buf[cmdLength - 1] = checkSum;

    write(socket_fd_, buf, cmdLength);

    size_t len = read(socket_fd_, buf, sizeof(buf));

    if (buf[0] != 0x02 || buf[1] != 0x02 || buf[2] != 0x02 || buf[3] != 0x02 || len != 10) {
        printf("invalid packet recieved.\n");
    }

    if (buf[6] != METHOD_UP) {
        printf("opt code error.\n");
    }

    if (buf[7] != GET_DEV_ID) {
        printf("cmd code error.\n");
    }

    checkSum = 0;
    for (size_t i = 0; i < len - 1; i++){
        checkSum += buf[i];
    }
    if (buf[len - 1] != checkSum) {
        printf("checksum error.\n");
    }

    printf("RX: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
}

void C200::getSerialNumber()
{
    const size_t cmdLength = 9;
    uint8_t buf[100], checkSum = 0;

    buf[0] = 0x02;
    buf[1] = 0x02;
    buf[2] = 0x02;
    buf[3] = 0x02;
    buf[4] = (cmdLength & 0xff00) >> 8;
    buf[5] = (cmdLength & 0x00ff);
    buf[6] = READ_DOWN;
    buf[7] = GET_SN;

    for (size_t  i= 0; i < cmdLength - 1; i++) {
        checkSum += buf[i];
    }
    buf[cmdLength - 1] = checkSum;

    write(socket_fd_, buf, cmdLength);

    size_t len = read(socket_fd_, buf, sizeof(buf));

    if (buf[0] != 0x02 || buf[1] != 0x02 || buf[2] != 0x02 || buf[3] != 0x02 || len != 13) {
        printf("invalid packet recieved.\n");
    }

    if (buf[6] != READ_UP) {
        printf("opt code error.\n");
    }

    if (buf[7] != GET_SN) {
        printf("cmd code error.\n");
    }

    checkSum = 0;
    for (size_t i = 0; i < len - 1; i++){
        checkSum += buf[i];
    }
    if (buf[len - 1] != checkSum) {
        printf("checksum error.\n");
    }

    printf("RX: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
}

bool C200::GetNORSwitchFlag()
{
    return NORswitch_;
}

void C200::getFirmwareVersion()
{
    const size_t cmdLength = 9;
    uint8_t buf[100], checkSum = 0;

    buf[0] = 0x02;
    buf[1] = 0x02;
    buf[2] = 0x02;
    buf[3] = 0x02;
    buf[4] = (cmdLength & 0xff00) >> 8;
    buf[5] = (cmdLength & 0x00ff);
    buf[6] = METHOD_DOWN;
    buf[7] = GET_FW_VER;

    for (size_t  i= 0; i < cmdLength - 1; i++) {
        checkSum += buf[i];
    }
    buf[cmdLength - 1] = checkSum;

    write(socket_fd_, buf, cmdLength);

    size_t len = read(socket_fd_, buf, sizeof(buf));

    if (buf[0] != 0x02 || buf[1] != 0x02 || buf[2] != 0x02 || buf[3] != 0x02 || len != 13) {
        printf("invalid packet recieved.\n");
    }

    if (buf[6] != METHOD_UP) {
        printf("opt code error.\n");
    }

    if (buf[7] != GET_FW_VER) {
        printf("cmd code error.\n");
    }

    checkSum = 0;
    for (size_t i = 0; i < len - 1; i++){
        checkSum += buf[i];
    }
    if (buf[len - 1] != checkSum) {
        printf("checksum error.\n");
    }

    printf("RX: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
}

status_t C200::getDeviceState()
{
    const size_t cmdLength = 9;
    uint8_t buf[100], checkSum = 0;

    buf[0] = 0x02;
    buf[1] = 0x02;
    buf[2] = 0x02;
    buf[3] = 0x02;
    buf[4] = (cmdLength & 0xff00) >> 8;
    buf[5] = (cmdLength & 0x00ff);
    buf[6] = METHOD_DOWN;
    buf[7] = GET_DEV_STA;

    for (size_t  i= 0; i < cmdLength - 1; i++) {
        checkSum += buf[i];
    }
    buf[cmdLength - 1] = checkSum;

    write(socket_fd_, buf, cmdLength);

    size_t len = read(socket_fd_, buf, sizeof(buf));

    if (buf[0] != 0x02 || buf[1] != 0x02 || buf[2] != 0x02 || buf[3] != 0x02 || len != 10) {
        printf("invalid packet recieved.\n");
        return device_error;
    }

    if (buf[6] != METHOD_UP) {
        printf("opt code error.\n");
        return device_error;
    }

    if (buf[7] != GET_DEV_STA) {
        printf("cmd code error.\n");
        return device_error;
    }

    checkSum = 0;
    for (size_t i = 0; i < len - 1; i++){
        checkSum += buf[i];
    }
    if (buf[len - 1] != checkSum) {
        printf("checksum error.\n");
        return device_error;
    }

    printf("RX: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");

    return (status_t)buf[8];
}

void C200::getScanAngle()
{
    const size_t cmdLength = 9;
    uint8_t buf[100], checkSum = 0;

    buf[0] = 0x02;
    buf[1] = 0x02;
    buf[2] = 0x02;
    buf[3] = 0x02;
    buf[4] = (cmdLength & 0xff00) >> 8;
    buf[5] = (cmdLength & 0x00ff);
    buf[6] = READ_DOWN;
    buf[7] = GET_ANGLE;

    for (size_t  i= 0; i < cmdLength - 1; i++) {
        checkSum += buf[i];
    }
    buf[cmdLength - 1] = checkSum;

    write(socket_fd_, buf, cmdLength);

    size_t len = read(socket_fd_, buf, sizeof(buf));

    if (buf[0] != 0x02 || buf[1] != 0x02 || buf[2] != 0x02 || buf[3] != 0x02 || len != 18) {
        printf("invalid packet recieved.\n");
    }

    if (buf[6] != READ_UP) {
        printf("opt code error.\n");
    }

    if (buf[7] != GET_ANGLE) {
        printf("cmd code error.\n");
    }

    checkSum = 0;
    for (size_t i = 0; i < len - 1; i++){
        checkSum += buf[i];
    }
    if (buf[len - 1] != checkSum) {
        printf("checksum error.\n");
    }

    printf("RX: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");

    int32_t start_angle, stop_angle;

    start_angle = ((int32_t)buf[9] << 24) + ((int32_t)buf[10] << 16) + ((int32_t)buf[11] << 8) + (int32_t)buf[12];
    stop_angle = ((int32_t)buf[13] << 24) + ((int32_t)buf[14] << 16) + ((int32_t)buf[15] << 8) + (int32_t)buf[16];

    angle_min_ = (double)start_angle * M_PI / 1800000 + (double)offset_angle_ * M_PI / 180;
    angle_max_ = (double)stop_angle * M_PI/ 1800000 + (double)offset_angle_ * M_PI / 180;
}

void C200::SetTimeStamp(bool stamp_switch)
{
    const size_t cmdLength = 10;
    uint8_t buf[100], checkSum = 0;

    buf[0] = 0x02;
    buf[1] = 0x02;
    buf[2] = 0x02;
    buf[3] = 0x02;
    buf[4] = (cmdLength & 0xff00) >> 8;
    buf[5] = (cmdLength & 0x00ff);
    buf[6] = WRITE_DOWN;
    buf[7] = SET_TIMESTAMP;
    if(stamp_switch)
    {
      buf[8] = 0x01;
    }
    else
    {
      buf[8] = 0x00;
    }

    for (size_t  i= 0; i < cmdLength - 1; i++) {
        checkSum += buf[i];
    }
    buf[cmdLength - 1] = checkSum;

    write(socket_fd_, buf, cmdLength);

    size_t len = read(socket_fd_, buf, sizeof(buf));

    if (buf[0] != 0x02 || buf[1] != 0x02 || buf[2] != 0x02 || buf[3] != 0x02 || len != 10) {
        printf("invalid packet recieved.\n");
    }

    if (buf[6] != WRITE_UP) {
        printf("opt code error.\n");
    }

    if (buf[7] != SET_TIMESTAMP) {
        printf("cmd code error.\n");
    }

    checkSum = 0;
    for (size_t i = 0; i < len - 1; i++){
        checkSum += buf[i];
    }
    if (buf[len - 1] != checkSum) {
        printf("checksum error.\n");
    }

    printf("RX: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
}

void C200::scanContinous(uint8_t state)
{
    const size_t cmdLength = 10;
    uint8_t buf[100], checkSum = 0;

    buf[0] = 0x02;
    buf[1] = 0x02;
    buf[2] = 0x02;
    buf[3] = 0x02;
    buf[4] = (cmdLength & 0xff00) >> 8;
    buf[5] = (cmdLength & 0x00ff);
    buf[6] = METHOD_DOWN;
    buf[7] = LOOP_SWITCH;
    buf[8] = state;

    for (size_t  i= 0; i < cmdLength - 1; i++) {
        checkSum += buf[i];
    }
    buf[cmdLength - 1] = checkSum;

    write(socket_fd_, buf, cmdLength);

    size_t len = read(socket_fd_, buf, sizeof(buf));

    if (buf[0] != 0x02 || buf[1] != 0x02 || buf[2] != 0x02 || buf[3] != 0x02 || len != 10) {
        printf("invalid packet recieved.\n");
    }

    if (buf[6] != METHOD_UP) {
        printf("opt code error.\n");
    }

    if (buf[7] != LOOP_SWITCH) {
        printf("cmd code error.\n");
    }

    checkSum = 0;
    for (size_t i = 0; i < len - 1; i++){
        checkSum += buf[i];
    }
    if (buf[len - 1] != checkSum) {
        printf("checksum error.\n");
    }

    printf("RX: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
}

void C200::filterHandle(std::vector<float> tempdisx, std::vector<float> tempmk, int pointnum)
{
     //初始化
     float mk[pointnum];
     float DisX1[pointnum];
     float filterbuf[pointnum];
     float filterv = 35;
     for(int i = 0; i < pointnum; i++) {
         DisX1[i] = tempdisx[i]*1000;
         filterbuf[i] = filterv;
//         mk[i] = tempmk[i] * 32;
//	 if (mk[i] >4095){
//	     mk[i] = 4095;
//	 }
     }
     float lenvalue = 20;  

     //高反射率物体的判断，以此增强滤波能力
     for(int i = 0; i < pointnum - 1; i++) {
        if(mk[i] >= 1000 && tempmk[i+1] < 1000) {
           int startindex = i;
           int stopindex;
           if(i + 20 <= pointnum-1) {
             stopindex = i + 20;
           }
           else {
             stopindex = pointnum-1;
           }
           
           for(int j = startindex; j < stopindex; j++) {
              filterbuf[j] = 50;
              if((laser_scan.ranges[j] + 0.05 <= laser_scan.ranges[i]) && laser_scan.ranges[j] > 0.003) {
                 filterbuf[j] = filterv;
              }
           }
        }
        else if(mk[i] < 1000 && tempmk[i+1] >= 1000) {
           int startindex;
           int stopindex = i;
           if((i-20) >= 1) {
              startindex = i - 20;
           }
           else {
              startindex = 1;
           }

           for(int j = startindex; j < stopindex; j++) {
              filterbuf[j] = 50;
              if(((laser_scan.ranges[j] + 0.05) <= laser_scan.ranges[i+1]) && laser_scan.ranges[j] > 0.003) {
                 filterbuf[j] = filterv;
              }
           }
        }
     }


      //下面部分为雷达滤波主体
      for(int i = 0; i < pointnum-5; i++) {
        if( DisX1[i] <= 8000 && DisX1[i+1] <= 8000 && DisX1[i+2] <= 8000 && DisX1[i+3] <= 8000&& DisX1[i+4] <= 8000 && DisX1[i+5] <= 8000) {
            if( DisX1[i] > 3 && (DisX1[i+5] >= DisX1[i] + 20))
            {
               if(filterbuf[i]*(DisX1[i+5] - DisX1[i]) >= DisX1[i])
               {
                  laser_scan.ranges[i+1] = 0.001;
                  laser_scan.ranges[i+2] = 0.001;
				  laser_scan.ranges[i+3] = 0.001;
                  laser_scan.ranges[i+4] = 0.001;
				  laser_scan.ranges[i+5] = 0.001;

               }
            }

         
	     else if(DisX1[i+5] >3 && (DisX1[i] >= DisX1[i+5] + 20)){
                if(filterbuf[i]*(DisX1[i] - DisX1[i+5]) >= DisX1[i+5]){
                  laser_scan.ranges[i] = 0.002;
                  laser_scan.ranges[i+1] = 0.002;
				  laser_scan.ranges[i+2] = 0.002;
				  laser_scan.ranges[i+3] = 0.002;
                  laser_scan.ranges[i+4] = 0.002;
                }
	     }
	}
      }

      //以下部分为雷达补点
      for(int i = 1; i < pointnum-1; i++)
      {
         if(DisX1[i-1] <= 3 && DisX1[i+1] <= 3)
         {
            laser_scan.ranges[i] = 0.001;
         }
      }

      for(int i = 2; i < pointnum-2; i++)
      {
         if(DisX1[i-2]>3 && DisX1[i-1]>3 && DisX1[i+1]>3 && DisX1[i+2]>3 && (DisX1[i]-DisX1[i-2])<0)
         {
            if((DisX1[i]-DisX1[i-2])*(DisX1[i]-DisX1[i+2]) >= 1000)
            {
               laser_scan.ranges[i] = DisX1[i]/1000;
            }
         }
      }

//      for(int i = 0;i <pointnum;i++){
//		laser_scan.intensities[i] = mk[i];
//      }

 //     for(int i = 0; i < pointnum-2; i++)
 //     {
 //        if(abs(DisX1[i]-DisX1[i+1])<=lenvalue && abs(DisX1[i+1]-DisX1[i+2])<=lenvalue && DisX1[i] >= 3 && DisX1[i+1] >= 3 && DisX1[i+2] >= 3)
 //        {
 //           laser_scan.ranges[i] = DisX1[i]/1000;
 //           laser_scan.ranges[i+1] = DisX1[i+1]/1000;
 //           laser_scan.ranges[i+2] = DisX1[i+2]/1000;
 //        }
 //     }
}



bool C200::getScanData()
{
    if(last_scan.ranges.size()==0)
    {
       flag_is_first = true;
    }
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);
    while(ros::ok())
    {
	struct timeval timeout;
	timeout.tv_sec = 5;
	timeout.tv_usec = 10;
	int ret = select(socket_fd_ + 1, &read_set, NULL, NULL, &timeout);
        if(ret > 0)
        {
           ssize_t len = read(socket_fd_, buffer, 65535);
           for(int i=0; i < len; i++)
           {
              data.push_back(buffer[i]);
           }
           //cout << "data.size()" << data.size() << endl;
           while(data.size() >= 6)
           {
              //启动字不满足，下移一个字节
              uint8_t nStart1 = data[0];
              uint8_t nStart2 = data[1];
              uint8_t nStart3 = data[2];
              uint8_t nStart4 = data[3];
              uint32_t headerbyte = (nStart1 << 24) + (nStart2 << 16) + (nStart3 << 8) + nStart4;
              if(headerbyte != START_FLAG)
              {
                 cout << "headertype error!" << endl;
                 data.pop_front();
                 continue;
              }
              //获取一包数据的长度
              uint8_t nHeight = data[4];
              uint8_t nLower = data[5];
              uint16_t nLength = (nHeight << 8) + nLower;
              //cout << "nLength:" << nLength << endl;
              if(nLength < 6 || nLength > data.size())
              {
                 //cout << "length no enough" << endl;
                 break;
              }
              //校验和检测
              uint8_t checkSum = 0;
              for(int sum = 0; sum < nLength - 1; sum++)
              {
                 checkSum += data[sum];
              }
              
              //cout << "checkSum:" << checkSum << "data[nlength-1]:" << (uint8_t)data[nLength - 1] << endl;
              if(checkSum != (uint8_t)data[nLength - 1])
              {
                 //cout << "checksum error" << endl;
                 uint16_t points_per_packet = ((uint16_t)data[20]<<8)+(uint16_t)data[21];
                 for(size_t i = 0; i < points_per_packet; i++)
                 {
                    laser_scan.ranges.push_back(0.0);
                    laser_scan.intensities.push_back(0);
                 }
                 for(int i = 0; i < nLength; i++)
                 {
                    data.pop_front();
                 }
                 //continue;
              }
              else
              {
                               //检测命令字是否正确
              uint8_t nCommand1 = data[6];
              uint8_t nCommand2 = data[7];
              uint16_t CommandType = (nCommand1 << 8) + nCommand2;
              if(CommandType == RSP_MEASUREMENT_DATA)
              {
                 if(data[11] == 0)
                 {
                    laser_scan.ranges.clear();
                    laser_scan.intensities.clear();
                 }
                 uint16_t temp = 0;
                 uint16_t points_per_packet = ((uint16_t)data[20]<<8)+(uint16_t)data[21];
                 for(size_t i = 0; i < points_per_packet; i++)
                 {
                    temp = ((uint16_t)data[22+i*4]<<8)+(uint16_t)data[22+1+i*4];
                    laser_scan.ranges.push_back(((double)temp/1000));
                    temp = ((uint16_t)data[22+2+i*4]<<8) + (uint16_t)data[22+3+i*4];
                    laser_scan.intensities.push_back(temp & 0x0fff);
                 }
                 uint16_t scan_number = ((uint16_t)data[9]<<8)+(uint16_t)data[10];
                 uint16_t scan_frequency = ((uint16_t)data[12]<<8)+(uint16_t)data[13];
                 uint16_t angle_index = ((uint16_t)data[18]<<8)+(uint16_t)data[19];
                 uint16_t points_per_scan = ((uint16_t)data[16]<<8)+(uint16_t)data[17];
                 uint16_t packet_size = (((uint16_t)data[4]<<8)+(uint16_t)data[5]);
                 uint16_t angle_resolution = ((uint16_t)data[14]<<8)+(uint16_t)data[15];
                 if(points_per_packet + angle_index == points_per_scan)
                 {
                    if(laser_scan.ranges.size() < 1080)
                    {
                        cout << "packet_no:" << data[11] << endl;
                    }
                    if(data[packet_size - 11] == 0xbb && data[packet_size - 10] == 0xbb)
                    {
                        ROS_WARN("No NTP data received!");
                    }
                     
                    laser_scan.header.stamp.sec = ((uint32_t)data[packet_size - 9] << 24) + 
                                                  ((uint32_t)data[packet_size - 8] << 16) + 
                                                  ((uint32_t)data[packet_size - 7] << 8) + 
                                                  (uint32_t)data[packet_size - 6];
		    if(laser_scan.header.stamp.sec > 2208988800)
                    {
		      laser_scan.header.stamp.sec-= 2208988800;
		    }
                    uint32_t temp_nsec = ((uint32_t)data[packet_size - 5] << 24) + 
                                         ((uint32_t)data[packet_size - 4] << 16) + 
                                         ((uint32_t)data[packet_size - 3] << 8) + 
                                         (uint32_t)data[packet_size - 2];
		    //std::cout<<std::hex<<laser_scan.header.stamp.sec<<"  "<<temp_nsec<<std::endl;    
                    laser_scan.header.stamp.nsec = (uint32_t)round((double)temp_nsec / pow(2, 32) * pow(10, 9));
                    last_scan = laser_scan;
                    laser_scan.header.seq = scan_number;
                    laser_scan.angle_min = angle_min_;
                    laser_scan.angle_max = angle_max_;
                    laser_scan.angle_increment = 2 * M_PI / (3600000 / angle_resolution);
                    laser_scan.time_increment = (100 / scan_frequency) / points_per_scan;
                    if(laser_scan.ranges.size() > points_per_scan)
                    {
                      laser_scan.ranges.clear();
                      laser_scan.intensities.clear();
                    }
                    if(filterswitch_)
                    {
	               filterHandle(laser_scan.ranges, laser_scan.intensities, laser_scan.ranges.size());
		    }
                    laser_scan_publisher_.publish(laser_scan);
                 }
                 for(int i = 0; i < nLength; i++)
                 {
                    data.pop_front();
                 }
              }
              }
           }
        }
        else if(ret < 0)
        {
           perror("select()");
           break;
        }
        else
        {
           cout << "timeout!~" << endl;
           return false;
        }
    }

    /*struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    int ret = select(socket_fd_ + 1, &read_set, NULL, NULL, &timeout);
    if (ret) 
    {
        ssize_t len = read(socket_fd_, buffer, 65535);
        ssize_t index = 0;

        while (index < len) {
            if (buffer[index] != 0x02 || buffer[index + 1] != 0x02 || buffer[index + 2] != 0x02 || buffer[index + 3] != 0x02) {
                printf("invalid packet recieved.\n");
                return false;
            }

            if (buffer[6] != METHOD_UP) {
                printf("opt code error.\n");
                return false;
            }

            if (buffer[7] != LOOP_DATA) {
                printf("cmd code error.\n");
                return false;
            }

            free_optics::packetHeader *p = (free_optics::packetHeader*)(buffer + index);
            p->packet_size = byteExchange(p->packet_size);
            p->scan_number = byteExchange(p->scan_number);
            p->scan_frequency = byteExchange(p->scan_frequency);
            p->angle_resolution = byteExchange(p->angle_resolution);
            p->angle_index = byteExchange(p->angle_index);
            p->points_per_scan = byteExchange(p->points_per_scan);
            p->points_per_packet = byteExchange(p->points_per_packet);
            // printf("RX: packet_size=%03u, packet_number=%u, scan_number=%u, angle_resolution=%u, angle_index=%04u, points_per_packet=%03u\n", \
            // p->packet_size, p->packet_number, p->scan_number, p->angle_resolution, p->angle_index, p->points_per_packet);

            if (p->packet_number == 0) {
                laser_scan.ranges.clear();
                laser_scan.intensities.clear();
            }
            
            uint16_t temp = 0;
            for (size_t i = 0; i < p->points_per_packet; i++) {
                temp = ((uint16_t)buffer[sizeof(packetHeader) + index + i * 4] << 8) + (uint16_t)buffer[sizeof(packetHeader) + 1 + index + i * 4];
                laser_scan.ranges.push_back(((double)temp/1000));
				temp = ((uint16_t)buffer[sizeof(packetHeader) + index + 2 + i * 4] << 8) + (uint16_t)buffer[sizeof(packetHeader) + 3 + index + i * 4];
                laser_scan.intensities.push_back(temp & 0x0fff);
            }
            if (p->points_per_packet + p->angle_index == p->points_per_scan) {
                //if (buffer[index + p->packet_size - 11] == 0xaa && buffer[index + p->packet_size - 10] == 0xaa) {
                if (buffer[index + p->packet_size - 11] == 0xbb && buffer[index + p->packet_size - 10] == 0xbb)
                {
                   ROS_WARN("No NTP data received!");
                }
           
                    laser_scan.header.stamp.sec = ((uint32_t)buffer[index + p->packet_size - 9] << 24) + 
                                                  ((uint32_t)buffer[index + p->packet_size - 8] << 16) + 
                                                  ((uint32_t)buffer[index + p->packet_size - 7] << 8) + 
                                                  (uint32_t)buffer[index + p->packet_size - 6];
		    if(laser_scan.header.stamp.sec > 2208988800){
		      laser_scan.header.stamp.sec-= 2208988800;
		    }
                    uint32_t temp_nsec = ((uint32_t)buffer[index + p->packet_size - 5] << 24) + 
                                         ((uint32_t)buffer[index + p->packet_size - 4] << 16) + 
                                         ((uint32_t)buffer[index + p->packet_size - 3] << 8) + 
                                         (uint32_t)buffer[index + p->packet_size - 2];
		    //std::cout<<std::hex<<laser_scan.header.stamp.sec<<"  "<<temp_nsec<<std::endl;    
                    laser_scan.header.stamp.nsec = (uint32_t)round((double)temp_nsec / pow(2, 32) * pow(10, 9));
                    //if(!flag_is_first)
                    //{
                        //float diff_inter = (laser_scan.header.stamp.toNSec() - last_scan.header.stamp.toNSec())/1e6;
                        //double diff_between = (ros::Time::now().toNSec()-laser_scan.header.stamp.toNSec())/1e6;
                        //std::cout.precision(5);
                        //std::cout.setf(std::ios::fixed);
			//ROS_INFO_STREAM(" "<<diff_inter<<" "<<diff_between);
			last_scan = laser_scan;
                    //}
		    //else{
		    //	last_scan = laser_scan;
			//flag_is_first = false;
		    //}
		    
                //}
                //else {
                //    laser_scan.header.stamp = ros::Time::now();
                //}
                laser_scan.header.seq = p->scan_number;
                laser_scan.angle_min = angle_min_;
                laser_scan.angle_max = angle_max_;
                laser_scan.angle_increment = 2 * M_PI / (3600000 / p->angle_resolution);
                laser_scan.time_increment = (100 / p->scan_frequency) / p->points_per_scan;

                if (laser_scan.ranges.size() > p->points_per_scan) {
                    laser_scan.ranges.clear();
                    laser_scan.intensities.clear();
                }
				if(filterswitch_){
					//add by zhurui 2021/10/20
					filterHandle(laser_scan.ranges, laser_scan.intensities, laser_scan.ranges.size());
					//printf("open filter.\n");
				}
                laser_scan_publisher_.publish(laser_scan);
               // printf("angle_min=%f,angle_max=%f\n",laser_scan.angle_min,laser_scan.angle_max);
            }

            index += p->packet_size;
        }

        return true;
    }
    else {
        return false;
    }*/
}

void C200::saveConfig()
{

}

uint16_t C200::byteExchange(uint16_t value)
{
    uint16_t temp = value;

    temp = ((temp & 0xff00) >> 8) + ((value & 0x00ff) << 8);

    return temp;
}
