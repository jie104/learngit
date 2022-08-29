//
// Created by lfc on 17-2-20.
//

#ifndef GPS_RECEIVER_GPS_MSG_H
#define GPS_RECEIVER_GPS_MSG_H

#include <stdint.h>
#include "base_msg.hpp"
#include <vector>

namespace record {
class GpsMsg : public BaseMsg {
public:
    GpsMsg() : BaseMsg(MSG_GPS) {
        id = 0;
        stamp = 0;
        week_number = 0;
        //从1980年到现在的星期数
        week_time = 0;
        //星期内的秒数
        heading = 0;
        //偏航角
        pitch = 0;
        //俯仰角
        track = 0;
        //地速相对于正北方向夹角
        latitude = 0;
        //纬度
        longitude = 0;
        //经度
        altitude = 0;
        //高度
        ve = 0;
        //东向速度
        vn = 0;
        //北向速度
        vu = 0;
        //天向速度
        baseline = 0;
        //基线长度
        nsv_1 = 0;
        //前天线可用星数
        nsv_2 = 0;
        //后天线可用星数
        status = 0;//系统状态
    }

    virtual ~GpsMsg() { }

    virtual bool encodeBody() override {
        encode_field(id);
        encode_field(stamp);
        encode_field(week_number);
        encode_field(week_time);
        encode_field(heading);
        encode_field(pitch);
        encode_field(track);
        encode_field(latitude);
        encode_field(longitude);
        encode_field(altitude);
        encode_field(ve);
        encode_field(vn);
        encode_field(vu);
        encode_field(baseline);
        encode_field(nsv_1);
        encode_field(nsv_2);
        encode_field(status);
        return true;
    }

    virtual bool decodeBody() override {
        decode_field(id);
        decode_field(stamp);
        decode_field(week_number);
        decode_field(week_time);
        decode_field(heading);
        decode_field(pitch);
        decode_field(track);
        decode_field(latitude);
        decode_field(longitude);
        decode_field(altitude);
        decode_field(ve);
        decode_field(vn);
        decode_field(vu);
        decode_field(baseline);
        decode_field(nsv_1);
        decode_field(nsv_2);
        decode_field(status);
        return true;
    }

    int id;
    int64_t stamp;
    int week_number;
    //从1980年到现在的星期数
    double week_time;
    //星期内的秒数
    double heading;
    //偏航角
    double pitch;
    //俯仰角
    double track;
    //地速相对于正北方向夹角
    double latitude;
    //纬度
    double longitude;
    //经度
    double altitude;
    //高度
    double ve;
    //东向速度
    double vn;
    //北向速度
    double vu;
    //天向速度
    double baseline;
    //基线长度
    int nsv_1;
    //前天线可用星数
    int nsv_2;
    //后天线可用星数
    int status;//系统状态
private:

};

typedef std::shared_ptr<GpsMsg> GpsMsg_ptr;
}
//class GpsMsgVector{
//public:
//    GpsMsgVector(){
//    }
//    virtual ~GpsMsgVector();
//    void push_back(GpsMsg& gps_msg);
//    void clear(GpsMsg &gps_msg);
//    int size();
//
//
//private:
//    uint64_t stamp = 0;
//    std::vector<GpsMsg> gps_vector;
//    //us，时间戳
//};
//class GpsMsgManager{
//
//};
//}


#endif //GPS_RECEIVER_GPS_MSG_H
