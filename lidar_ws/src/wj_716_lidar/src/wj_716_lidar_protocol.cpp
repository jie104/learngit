#include "wj_716_lidar_protocol.h"
#include <iostream>

namespace wj_lidar
{
  bool wj_716_lidar_protocol::setConfig(wj_716_lidar::wj_716_lidarConfig &new_config,uint32_t level)
  {
    config_ = new_config;
    scan.header.frame_id = config_.frame_id;
    scan.angle_min = config_.min_ang;
    scan.angle_max = config_.max_ang; 
    scan.range_min = config_.range_min;
    scan.range_max = config_.range_max;   

    scan.angle_increment = 0.017453 / 3;
    scan.time_increment = 1/15.00000000/811;
    //adjust angle_min to min_ang config param
    index_start = 811-((config_.min_ang+2.35619449)/scan.angle_increment);
    //adjust angle_max to max_ang config param
    index_end =  (2.35619449-config_.max_ang)/ scan.angle_increment;
    resizeNum = index_start-index_end;

    scan.ranges.resize(resizeNum);
    scan.intensities.resize(resizeNum);

    scan_TwoEcho.header.frame_id = config_.frame_id;
    scan_TwoEcho.angle_min = config_.min_ang;
    scan_TwoEcho.angle_max = config_.max_ang;
    scan_TwoEcho.angle_increment = scan.angle_increment;
    scan_TwoEcho.time_increment = scan.time_increment;
    scan_TwoEcho.range_min = config_.range_min;
    scan_TwoEcho.range_max = config_.range_max;

    scan_TwoEcho.ranges.resize(resizeNum);
    scan_TwoEcho.intensities.resize(resizeNum);

    cout << "frame_id:" <<scan.header.frame_id<<endl;
    cout << "min_ang:" <<scan.angle_min<<endl;
    cout << "max_ang:" <<scan.angle_max<<endl;
    cout << "angle_increment:" <<scan.angle_increment<<endl;
    cout << "time_increment:" <<scan.time_increment<<endl;
    cout << "range_min:" <<scan.range_min<<endl;
    cout << "range_max:" <<scan.range_max<<endl;
    cout << "resize:" <<resizeNum<<endl;
    return true;
  }

   wj_716_lidar_protocol::wj_716_lidar_protocol()
   {
     memset(&m_sdata,0,sizeof(m_sdata));
     marker_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
     twoecho_=nh.advertise<sensor_msgs::LaserScan>("scan_TwoEcho", 5);
     ros::Time scan_time = ros::Time::now();      //make a virtual data per sec
     scan.header.stamp = scan_time;

     g_u32PreFrameNo = 0;

     scan.header.frame_id = "wj_716_lidar_frame";
     scan.angle_min = -2.35619449;
     scan.angle_max = 2.35619449;
     scan.angle_increment = 0.00582;
     scan.time_increment = 1/15.00000000/811;
     scan.range_min = 0;
     scan.range_max = 25;
     scan.ranges.resize(811);
     scan.intensities.resize(811);

     scan_TwoEcho.header.frame_id = "wj_716_lidar_frame";
     scan_TwoEcho.angle_min = -2.35619449;
     scan_TwoEcho.angle_max = 2.35619449;
     scan_TwoEcho.angle_increment = 0.00582;
     scan_TwoEcho.time_increment = 1/15.00000000/811;
     scan_TwoEcho.range_min = 0;
     scan_TwoEcho.range_max = 25;
     scan_TwoEcho.ranges.resize(811);
     scan_TwoEcho.intensities.resize(811);
     cout << "wj_716_lidar_protocl start success" << endl;

   }

   bool wj_716_lidar_protocol::dataProcess(unsigned char *data, const int reclen)
   {
     if(reclen > MAX_LENGTH_DATA_PROCESS)
     {
       return false;
     }

     if(m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS)
     {
       memset(&m_sdata,0,sizeof(m_sdata));
       return false;
     }
     memcpy(&m_sdata.m_acdata[m_sdata.m_u32in],data,reclen*sizeof(char));
     m_sdata.m_u32in += reclen;
     while(m_sdata.m_u32out < m_sdata.m_u32in)
     {
       if((m_sdata.m_acdata[m_sdata.m_u32out] == 0x02 && m_sdata.m_acdata[m_sdata.m_u32out+1] == 0x02 &&
          m_sdata.m_acdata[m_sdata.m_u32out+2] == 0x02 && m_sdata.m_acdata[m_sdata.m_u32out+3] == 0x02) ||
          (m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out+1] == 0xAA))
       {
         unsigned l_u32reallen = 0;
         if(m_sdata.m_acdata[m_sdata.m_u32out] == 0x02)
         {
             l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 4] << 24) |
                            (m_sdata.m_acdata[m_sdata.m_u32out + 5] << 16) |
                            (m_sdata.m_acdata[m_sdata.m_u32out + 6] << 8)  |
                            (m_sdata.m_acdata[m_sdata.m_u32out + 7] << 0);
             l_u32reallen = l_u32reallen + 9;
         }
         else
         {
             l_u32reallen = (m_sdata.m_acdata[m_sdata.m_u32out + 2] << 8)  |
                            (m_sdata.m_acdata[m_sdata.m_u32out + 3]);
             l_u32reallen = l_u32reallen + 4;
         }


         if(l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1))
         {
           if(OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out],l_u32reallen))
           {
             m_sdata.m_u32out += l_u32reallen;
           }
           else
           {
             cout << "continuous frame"<<endl;
             int i;
             for(i = 1; i<=l_u32reallen; i++)
             {
               if(((m_sdata.m_acdata[m_sdata.m_u32out + i] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 1] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 2] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 3] == 0x02)) ||
                  (m_sdata.m_acdata[m_sdata.m_u32out] == 0xFF && m_sdata.m_acdata[m_sdata.m_u32out+1] == 0xAA))
               {
                 m_sdata.m_u32out += i;
                 break;
               }
               if(i == l_u32reallen)
               {
                 m_sdata.m_u32out += l_u32reallen;
               }
             }
           }
         }
         else if(l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
         {
           cout << "l_u32reallen >= MAX_LENGTH_DATA_PROCESS"<<endl;
           cout << "reallen: "<<l_u32reallen<<endl;
           memset(&m_sdata,0,sizeof(m_sdata));

         }
         else
         {
           break;
         }
       }
       else
       {
         m_sdata.m_u32out++;
       }
     } //end while(m_sdata.m_u32out < m_sdata.m_u32in)

     if(m_sdata.m_u32out >= m_sdata.m_u32in)
     {
       memset(&m_sdata,0,sizeof(m_sdata));
     }
      return true;
   }

   bool wj_716_lidar_protocol::OnRecvProcess(unsigned char *data, int len)
   {
     if(len > 0)
     {
       if(checkXor(data,len))
       {
         protocl(data,len);
       }
     }
     else
     {
       return false;
     }
     return true;
   }

   bool wj_716_lidar_protocol::protocl(unsigned char *data, const int len)
   {
       if(data[0] == 0x02 && data[1] == 0x02 && data[2] == 0x02 && data[3] == 0x02)
       {
           if((data[8] == 0x73 && data[9] == 0x52)||(data[8] == 0x73 && data[9] == 0x53) )   //command type:0x73 0x52/0X53
           {
             static int s_n32ScanIndex;
             int l_n32PackageNo= (data[50] << 8) + data[51];                                        //shuju bao xu hao
             unsigned int l_u32FrameNo = (data[46]<<24) + (data[47]<<16) + (data[48]<<8) + data[49];        //quan hao

             if(l_n32PackageNo == 1)
             {
               s_n32ScanIndex = 0;
               g_u32PreFrameNo = l_u32FrameNo;
               heartstate = true;
               for(int j = 0; j < 810;j=j+2)
               {
                 scandata[s_n32ScanIndex] = (((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
                 scandata[s_n32ScanIndex] /= 1000.0;
                 if(scandata[s_n32ScanIndex]>=25 || scandata[s_n32ScanIndex]==0)
                 {
                   scandata[s_n32ScanIndex]= NAN;
                 }
                 s_n32ScanIndex++;
               }
             }
             else if(l_n32PackageNo == 2)
             {
               if(g_u32PreFrameNo == l_u32FrameNo)
               {
                 for(int j = 0; j < 812;j=j+2)
                 {
                   scandata[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
                   scandata[s_n32ScanIndex] /= 1000.0;
                   if(scandata[s_n32ScanIndex]>=25 || scandata[s_n32ScanIndex]==0)
                   {
                     scandata[s_n32ScanIndex]= NAN;
                   }
                   scan.intensities[s_n32ScanIndex] = 0;
                   s_n32ScanIndex++;
                 }

               }
               else
               {
                 s_n32ScanIndex = 0;
                 return false;
               }
             }
             else if(l_n32PackageNo == 3)
             {
               s_n32ScanIndex=0;
               if(g_u32PreFrameNo == l_u32FrameNo)
               {
                 for(int j = 0; j < 810;j=j+2)
                 {
                   scaninden[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
                   s_n32ScanIndex++;
                 }
               }
               else
               {
                 s_n32ScanIndex = 0;
                 return false;
               }
             }
             else if(l_n32PackageNo == 4)
             {
               if(g_u32PreFrameNo == l_u32FrameNo)
               {
                 for(int j = 0; j < 812;j=j+2)
                 {
                   scaninden[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
                   s_n32ScanIndex++;
                 }

                 for (int j = index_start, i=0; j >= index_end; --j,i++)
                 {
                     scan.ranges[i] = scandata[j];
                     scan.intensities[i]=scaninden[j];
                 }
               }
               else
               {
                 s_n32ScanIndex = 0;
                 return false;
               }


             }
             else if(l_n32PackageNo == 5)
             {
               s_n32ScanIndex=0;
               if(g_u32PreFrameNo == l_u32FrameNo)
               {
                 for(int j = 0; j < 810;j=j+2)
                 {
                   scandata_te[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
                   scandata_te[s_n32ScanIndex] /= 1000.0;
                   if(scandata_te[s_n32ScanIndex]>=55 || scandata_te[s_n32ScanIndex]==0)
                   {
                     scandata_te[s_n32ScanIndex]= NAN;
                   }
                   s_n32ScanIndex++;
                 }
               }
               else
               {
                 s_n32ScanIndex = 0;
                 return false;
               }
             }
             else if(l_n32PackageNo == 6)
             {
               if(g_u32PreFrameNo == l_u32FrameNo)
               {
                 for(int j = 0; j < 812;j=j+2)
                 {
                   scandata_te[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
                   scandata_te[s_n32ScanIndex] /= 1000.0;
                   if(scandata_te[s_n32ScanIndex]>=55 || scandata_te[s_n32ScanIndex]==0)
                   {
                     scandata_te[s_n32ScanIndex]= NAN;
                   }
                   s_n32ScanIndex++;
                 }

                 for (int j = index_start,i=0; j <= index_end; --j,i++)
                 {
                     scan_TwoEcho.ranges[i] = scandata_te[j];
                     scan_TwoEcho.intensities[i] =0;
                 }

                 ros::Time scan_time = ros::Time::now();
                 scan.header.stamp = scan_time;
                 marker_pub.publish(scan);

                 ros::Time scan_time1 = ros::Time::now();
                 scan_TwoEcho.header.stamp = scan_time1;
                 twoecho_.publish(scan_TwoEcho);
               }
               else
               {
                 s_n32ScanIndex = 0;
                 return false;
               }
             }
             return true;
           }
           else
           {
             return false;
           }
       }
       else if(data[0] == 0xFF && data[1] == 0xAA)
       {
           if((data[22] == 0x06 && data[23] == 0x08))   //command GetMAC
           {
               std::stringstream  sstemp;
               sstemp << std::hex << std::setw(2) << std::setfill('0') << (int)data[53] <<"-";
               sstemp << std::hex << std::setw(2) << std::setfill('0') << (int)data[55] <<"-";
               sstemp << std::hex << std::setw(2) << std::setfill('0') << (int)data[57] <<"-";
               sstemp << std::hex << std::setw(2) << std::setfill('0') << (int)data[59] <<"-";
               sstemp << std::hex << std::setw(2) << std::setfill('0') << (int)data[61] <<"-";
               sstemp << std::hex << std::setw(2) << std::setfill('0') << (int)data[63];
               string maclower = sstemp.str();
               transform(maclower.begin(), maclower.end(), back_inserter(DeviceMAC), ::toupper);
               cout << "MAC is : " << DeviceMAC << endl;
               return true;
           }
           else
           {
               return false;
           }
       }
       else
       {
           return false;
       }
   }

   bool wj_716_lidar_protocol::checkXor(unsigned char *recvbuf,  int recvlen)
   {
     int i = 0;
     unsigned char check = 0;
     unsigned char *p = recvbuf;
     int len;

     if(*p == (char)0x02)
     {
       p = p+8;
       len = recvlen - 9;
       for(i = 0; i < len; i++)
       {
         check ^= *p++;
       }
     }
     else
     {
       p = p+2;
       len = recvlen - 6;
       for(i = 0; i < len; i++)
       {
         check ^= *p++;
       }
       p++;
     }

     if(check == *p)
     {
       return true;
     }
     else
       return false;
   }

}
