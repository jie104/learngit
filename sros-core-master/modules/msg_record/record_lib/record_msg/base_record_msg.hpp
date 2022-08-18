//
// Created by lfc on 17-10-26.
//

#ifndef SROS_BASE_RECORD_MSG_HPP
#define SROS_BASE_RECORD_MSG_HPP

#include <string>
#include <fstream>
#include <glog/logging.h>
#include <map>
#include <memory>

#ifndef convertToStr
#define convertToStr(value_name) #value_name
#endif
namespace record{

typedef std::map<std::string, std::vector<double>> MsgMap;

enum RecordMsgType{
    TYPE_RECORD_OBSTACLE = 1,
    TYPE_RECORD_SCAN = 2,
    TYPE_RECORD_CMD = 3,//TODO:第一步,需要新建一个type类型
    TYPE_RECORD_LMK = 4,
};

class RecordMsgMap{
public:
    RecordMsgMap() {
        type_str_map[int(TYPE_RECORD_OBSTACLE)] = convertToStr(TYPE_RECORD_OBSTACLE);
        type_str_map[int(TYPE_RECORD_SCAN)] = convertToStr(TYPE_RECORD_SCAN);
        type_str_map[int(TYPE_RECORD_CMD)] = convertToStr(TYPE_RECORD_CMD);//TODO:第二步,需要新建一个map类型,实现字符串与type映射
        type_str_map[int(TYPE_RECORD_LMK)] = convertToStr(TYPE_RECORD_LMK);

        for (auto &iter:type_str_map) {
            str_type_map[iter.second] = iter.first;
        }
    }

    static std::string getNameByType(RecordMsgType type);

    static int getTypeByName(std::string name);

    std::string getName(RecordMsgType type){
        auto tmp_iter = type_str_map.find(int(type));
        if (tmp_iter != type_str_map.end()) {
            return tmp_iter->second;
        }else {
            LOG(INFO) << "err to get the type:" << type;
            return std::string();
        }
    }
    int getType(std::string name){
        auto tmp_iter = str_type_map.find(name);
        if (tmp_iter != str_type_map.end()) {
            return tmp_iter->second;
        }else {
            LOG(INFO) << "err to get the type:" << name;
            return 0;
        }
    }

    static std::shared_ptr<RecordMsgMap> getInstance();

private:

    std::map<int,std::string> type_str_map;

    std::map<std::string,int> str_type_map;

    static std::shared_ptr<RecordMsgMap> record_msg_module;

};

class BaseRecordMsg {
public:
    BaseRecordMsg(RecordMsgType type_):type(type_){

    }

    bool encode(std::ostream& record_stream) {
        encodeHeader(record_stream);

        encodeBody(record_stream);//虚函数,由子类实现

        encodeEnd(record_stream);

        return true;
    }

    void decode(MsgMap& msg_map) {

        decodeHeader(msg_map);

        decodeBody(msg_map);
    }

    virtual ~BaseRecordMsg() {


    };

    RecordMsgType getType() {
        return type;
    }


public:
    int64_t stamp;
protected:
    virtual void encodeBody(std::ostream& record_stream){//TODO:第三步,子类复写encodeBody虚函数

    }

    virtual void decodeBody(MsgMap& msg_map){//TODO:第四步,子类复写decodeBody虚函数

    }

    template<typename T_SAVE>
    void writeData(T_SAVE &data,std::ostream& record_stream) {
            record_stream.width(20);
            record_stream << data;
            record_stream << " ";
    }

    void writeName(std::string name,std::ostream& record_stream) {//引用的实现方法就是地址调用,因此,可以用基类引用子类
        writeData(name,record_stream);
    }

    void writeEndl(std::ostream& record_stream) {
        record_stream << std::endl;
    }

    void writeEndStr(std::ostream& record_stream) {
        writeName("END",record_stream);
        writeEndl(record_stream);
    }

private:
    void encodeHeader(std::ostream& record_stream){
        writeType(record_stream);
        writeStamp(record_stream);
    }

    void encodeEnd(std::ostream& record_stream) {
        writeEndl(record_stream);
        writeEndStr(record_stream);
    }

    bool decodeHeader(MsgMap& msg_map){
        auto msg_iter = msg_map.find(RecordMsgMap::getNameByType(type));
        if (msg_iter != msg_map.end()) {
            auto stamp_iter = msg_map.find(convertToStr(stamp));
            if (stamp_iter != msg_map.end()) {
                stamp = stamp_iter->second[0];
            }else {
                LOG(INFO) << "err to get the stamp!";
            }
            return true;
        }else {
            LOG(INFO) << "err to get the header!:" << type;
            return false;
        }
    }

    void writeType(std::ostream& record_stream) {
        writeName(convertToStr(type), record_stream);
        writeData(type, record_stream);
        writeEndl(record_stream);
        writeName(RecordMsgMap::getNameByType(type), record_stream);
        writeData(type, record_stream);
        writeEndl(record_stream);
    }

    void writeStamp(std::ostream& record_stream) {
        writeName(convertToStr(stamp), record_stream);
        writeData(stamp, record_stream);
        writeEndl(record_stream);
    }


    RecordMsgType type;


    BaseRecordMsg(){

    }

};
typedef std::shared_ptr<BaseRecordMsg> BaseRecordMsg_Ptr;

}


#endif //SROS_BASE_RECORD_MSG_HPP
