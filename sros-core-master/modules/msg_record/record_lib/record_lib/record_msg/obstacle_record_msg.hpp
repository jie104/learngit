//
// Created by lfc on 17-10-27.
//

#ifndef SROS_OBASTACLE_RECORD_MSG_HPP
#define SROS_OBASTACLE_RECORD_MSG_HPP

#include "base_record_msg.hpp"
namespace record{
struct RecordPoint{
    float x;
    float y;
};

class ObstacleRecordMsg :public BaseRecordMsg{
public:
    ObstacleRecordMsg() : BaseRecordMsg(TYPE_RECORD_OBSTACLE){

    }

    virtual ~ObstacleRecordMsg(){

    }

    virtual void encodeBody(std::ostream& record_stream) {//TODO:第三步,子类复写encodeBody虚函数
        writeName(convertToStr(points), record_stream);//将变量名编码
        for (auto &point:points) {
            writeData(point.x, record_stream);
            writeData(point.y, record_stream);
        }//将变量编码
        writeEndl(record_stream);//结束换行符
    }

    virtual void decodeBody(MsgMap& msg_map) {//TODO:第四步,子类复写decodeBody虚函数
        for (auto &iter:msg_map) {
            if (iter.first == convertToStr(points)) {
                auto &datas = iter.second;
                int data_size = datas.size();
                if (data_size % 2 == 0) {
                    for (int i = 0; i < data_size; i = i + 2) {
                        points.emplace_back();
                        points.back().x = datas[i];
                        points.back().y = datas[i+1];
                    }
                }
            }
        }
    }

    std::vector<RecordPoint> points;




};

typedef std::shared_ptr<ObstacleRecordMsg> ObstacleRecordMsg_Ptr; //TODO:需要类型定义一个共享指针

}


#endif //SROS_OBASTACLE_RECORD_MSG_HPP
