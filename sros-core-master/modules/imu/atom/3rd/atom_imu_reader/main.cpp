//#include <iostream>
extern "C"{
#include "atom_macro.h"
#include "atomprotocol.h"
#include "com.h"
#include "protocol_wrapper.h"
}
#include "glog/logging.h"
#include <stdio.h>
#include "imu_receive.hpp"
int imuMsgCallback(void* data,u16 length,u16 type){ LOG(INFO) << "length:" << length << "," << type; }

int main()
{
  ImuReceive receieve;
  if (receieve.open("/dev/ttyUSB0", 115200)) {
      LOG(INFO) << "receieve data!";
      receieve.creatReadLoop(imuMsgCallback, 0x06, 0x81);
  }
  //  g_imu_msg_callback = imuDataCallback;
  // Create COMthread
  //OpenCom();
//  ATR_Linux_OpenCom("/dev/ttyUSB0",115200);
  /****************************** Switch to config mode ********************************************/
//  ReceiveData(0x06,0x81);

  getchar();
  return 0;
}