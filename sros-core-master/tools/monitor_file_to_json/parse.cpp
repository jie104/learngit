//
// Created by huangwuxian on 19-6-18.
//

#include <fstream>

#include "parse.h"
#include <boost/filesystem.hpp>

#include "monitor_protobuf_full.pb.h"
#include "protobuf_transform.h"


#define BUFFER_SIZE 2048

bool Parse::parseMonitorFile(const std::string &file_path, std::string &output) {
    std::cout << "Parse::parseMonitorFile(): Parse monitor file " << file_path << std::endl;
    if (file_path.empty() || !boost::filesystem::exists(file_path))
    {
        std::cerr << "Parse::parseMonitorFile(): File not exist " << file_path << std::endl;
        return false;
    }

    std::ifstream input;

    // 文件名需要转码
    input.open(file_path, std::ifstream::in | std::ifstream::binary);
    if (!input.is_open())
    {
        std::cerr << "Parse::parseMonitorFile(): File " << file_path << " open failed." << std::endl;
        return false;
    }

    input.seekg(0, std::ios_base::end);
    int file_len = input.tellg();
    if (file_len <= 0) {
        std::cerr << "Parse::parseMonitorFile(): File is empty: " << file_path << std::endl;
        input.close();
        return false;
    }
    input.seekg(0, std::ios_base::beg);

    std::vector<char> buff(BUFFER_SIZE);
    size_t len = parseModuleLen(input);
    if (len <= 0 || len > BUFFER_SIZE)
    {
        std::cerr << "Parse::parseMonitorFile(): Invalid header len :" << len << std::endl;
        input.close();
        return false;
    }

    input.read(&buff[0], len);

    monitor::Header header;
    if (!header.ParseFromArray(&buff[0], len))
    {
        std::cerr <<  "Parse::parseMonitorFile(): parse header error" << std::endl;
        input.close();
        return false;
    }

    string str_header;
    if (!ProtobufTransform::toJsonString(header, &str_header)) {
        std::cerr << "Parse::parseMonitorFile(): Parser file header error" << std::endl;
        input.close();
        return false;
    }

//    std::cout << "File header: " << str_header << std::endl;
    nloJson record_list = nloJson::array();
    while (!input.eof()) {
        monitor::Record record;
        len = parseModuleLen(input);
        if (len <= 0 || len > BUFFER_SIZE)
        {
//            std::cerr << "Parse::parseMonitorFile(): invalid block record len" << std::endl;
            break;
        }
        input.read(&buff[0], len);

        // 竟然出现len为0情况
        if (len <= 0)
        {
            break;
        }

        if (!record.ParseFromArray(&buff[0], len))
        {
            std::cerr <<  "Parse::parseMonitorFile(): parse record error" << std::endl;
            input.close();
            continue;
        }

        string str_record;
        if (!ProtobufTransform::toJsonString(record, &str_record)) {
            std::cerr << "Parse::parseMonitorFile(): Parser record error" << std::endl;
            continue;
        }

//        str_header += str_record;

//        std::cout << "Record : " << str_record << std::endl;
        record_list.push_back(nloJson::parse(str_record));
    }
    nloJson json_root;
    nloJson json_header = nloJson::parse(str_header);
    json_root["file_header"] = json_header;
    json_root["records"] = record_list;

    output = json_root.dump(4);

//    output = str_header;
    input.close();
    return true;
}

size_t Parse::parseModuleLen(std::ifstream &file) {
    char low = 0, high = 0;

    file.get(high);
    file.get(low);

    // 下列方式读取，当读取到的是20时会跳过该字节，导致数据异常
//    file >> hight;
//    file >> low;
    return ((size_t) (unsigned char)high << 8) + (unsigned char)low;
}

//void Parse::PbMsg2Json(const ::google::protobuf::Message &src, nloJson &dst, bool enum2str) {
//
//    const ProtobufDescriptor* descriptor = src.GetDescriptor();
//    const ProtobufReflection* reflection = src.GetReflection();
//    if (NULL == descriptor || NULL == descriptor) return;
//
//    int32_t count = descriptor->field_count();
//
//    for (int32_t i = 0; i < count; ++i) {
//        const ProtobufFieldDescriptor *field = descriptor->field(i);
//
//        if (field->is_repeated()) {
//            if (reflection->FieldSize(src, field) > 0)
//                RepeatedMessage2Json(src, field, reflection, dst[field->name()], enum2str);
//            continue;
//        }
//
//
//        if (!reflection->HasField(src, field)) {
//            continue;
//        }
//
//        switch (field->type()) {
//            case ProtobufFieldDescriptor::TYPE_MESSAGE: {
//                const ProtobufMsg &tmp_message = reflection->GetMessage(src, field);
//                if (0 != tmp_message.ByteSize()) PbMsg2Json(tmp_message, dst[field->name()]);
//                break;
//            }
//
//            case ProtobufFieldDescriptor::TYPE_BOOL:
//                dst[field->name()] = reflection->GetBool(src, field) ? true : false;
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_ENUM: {
//                const ::google::protobuf::EnumValueDescriptor *enum_value_desc = reflection->GetEnum(src, field);
//                if (enum2str) {
//                    dst[field->name()] = enum_value_desc->name();
//                } else {
//                    dst[field->name()] = enum_value_desc->number();
//                }
//                break;
//            }
//
//            case ProtobufFieldDescriptor::TYPE_INT32:
//            case ProtobufFieldDescriptor::TYPE_SINT32:
//            case ProtobufFieldDescriptor::TYPE_SFIXED32:
//                dst[field->name()] = Json::Int(reflection->GetInt32(src, field));
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_UINT32:
//            case ProtobufFieldDescriptor::TYPE_FIXED32:
//                dst[field->name()] = Json::UInt(reflection->GetUInt32(src, field));
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_INT64:
//            case ProtobufFieldDescriptor::TYPE_SINT64:
//            case ProtobufFieldDescriptor::TYPE_SFIXED64:
//                dst[field->name()] = Json::Int64(reflection->GetInt64(src, field));
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_UINT64:
//            case ProtobufFieldDescriptor::TYPE_FIXED64:
//                dst[field->name()] = Json::UInt64(reflection->GetUInt64(src, field));
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_FLOAT:
//                dst[field->name()] = reflection->GetFloat(src, field);
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_STRING:
//            case ProtobufFieldDescriptor::TYPE_BYTES:
//                dst[field->name()] = reflection->GetString(src, field);
//                break;
//
//            default:
//                break;
//        }
//    }
//}
//
//void Parse::RepeatedMessage2Json(const ::google::protobuf::Message &message,
//                                 const ::google::protobuf::FieldDescriptor *field,
//                                 const ::google::protobuf::Reflection *reflection, nloJson &json, bool enum2str) {
//
//    if (NULL == field || NULL == reflection)
//    {
//        PbMsg2Json(message, json);
//    }
//
//    for (int32_t i = 0; i < reflection->FieldSize(message, field); ++i)
//    {
//        nloJson tmp_json;
//        switch (field->type())
//        {
//            case ProtobufFieldDescriptor::TYPE_MESSAGE:
//            {
//                const ProtobufMsg& tmp_message = reflection->GetRepeatedMessage(message, field, i);
//                if (0 != tmp_message.ByteSize())
//                {
//                    PbMsg2Json(tmp_message, tmp_json);
//                }
//                break;
//            }
//
//            case ProtobufFieldDescriptor::TYPE_BOOL:
//                tmp_json[field->name()] = reflection->GetRepeatedBool(message, field, i) ? true : false;
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_ENUM:
//            {
//                const ::google::protobuf::EnumValueDescriptor* enum_value_desc = reflection->GetRepeatedEnum(message, field, i);
//                if (enum2str)
//                {
//                    tmp_json = enum_value_desc->name();
//                }
//                else
//                {
//                    tmp_json = enum_value_desc->number();
//                }
//                break;
//            }
//
//            case ProtobufFieldDescriptor::TYPE_INT32:
//            case ProtobufFieldDescriptor::TYPE_SINT32:
//            case ProtobufFieldDescriptor::TYPE_SFIXED32:
//                tmp_json[field->name()] = reflection->GetRepeatedInt32(message, field, i);
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_UINT32:
//            case ProtobufFieldDescriptor::TYPE_FIXED32:
//                tmp_json[field->name()] = reflection->GetRepeatedUInt32(message, field, i);
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_INT64:
//            case ProtobufFieldDescriptor::TYPE_SINT64:
//            case ProtobufFieldDescriptor::TYPE_SFIXED64:
//                tmp_json[field->name()] = (Json::Int64)reflection->GetRepeatedInt64(message, field, i);
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_UINT64:
//            case ProtobufFieldDescriptor::TYPE_FIXED64:
//                tmp_json[field->name()] = Json::UInt64(reflection->GetRepeatedUInt64(message, field, i));
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_FLOAT:
//                tmp_json[field->name()] = reflection->GetRepeatedFloat(message, field, i);
//                break;
//
//            case ProtobufFieldDescriptor::TYPE_STRING:
//            case ProtobufFieldDescriptor::TYPE_BYTES:
//                tmp_json[field->name()] = reflection->GetRepeatedString(message, field, i);
//                break;
//
//            default:
//                break;
//        }
//        json.append(tmp_json);
//    }
//}
