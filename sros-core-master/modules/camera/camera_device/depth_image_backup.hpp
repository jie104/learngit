//
// Created by lfc on 2021/4/19.
//

#ifndef D435_CALIBRATION_DEPTH_IMAGE_BACKUP_HPP
#define D435_CALIBRATION_DEPTH_IMAGE_BACKUP_HPP
#include <Eigen/Dense>
#include <fstream>
#include <glog/logging.h>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

namespace serialization{
class Image{
 public:
    Image(uint16_t length, uint16_t width) : length_(length), width_(width) { points_.resize(length * width); }

    unsigned char &operator()(const uint16_t &coord_x, const uint16_t &coord_y) {
        return points_[coord_y * length_ + coord_x];
    }

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & points_;
        ar & length_;
        ar & width_;
    }

    void recomputePointState() {}

    void computeForBackup(){}

    const std::string &type() const { return data_type_; }

    const unsigned char &operator()(const uint16_t &coord_x, const uint16_t &coord_y) const {
        return points_[coord_y * length_ + coord_x];
    }

    const uint16_t &length() const { return length_; }

    const uint16_t &width() const { return width_; }
 private:
    std::vector<unsigned char> points_;
    uint16_t length_;
    uint16_t width_;
    const std::string data_type_ = "image";
};

class PointImage {
 public:
    struct PointElement {
        Eigen::Vector3f point;
        uint16_t coord_x;
        uint16_t coord_y;
        int16_t x,y,z;
        bool valid = false;
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version) {//对于文本序列化，不会存在32位与64位不兼容问题，但是会增加存储，需要改成16位降低存储
            ar & x;
            ar & y;
            ar & z;
        }
    };

    void recomputePointState(){//将数据从文件恢复以后，需要将值重新恢复，类似于压缩--解压缩过程
        for (int i = 0; i < width_; ++i) {
            for (int j = 0; j < length_; ++j) {
                PointElement &point = points_[i * length_ + j];
                point.point[0] = point.x / 1000.0f;
                point.point[1] = point.y / 1000.0f;
                point.point[2] = point.z / 1000.0f;
                point.valid = point.point.norm() > 0.1f && point.point.norm() < 100.0f;
                point.coord_x = j;
                point.coord_y = i;
            }
        }
    }

    void computeForBackup(){//在存储之前调用一次，目的是压缩数据，减小存储大小。如果没有这个需求，可不用实现该函数
        for (int i = 0; i < width_; ++i) {
            for (int j = 0; j < length_; ++j) {
                PointElement &point = points_[i * length_ + j];
                point.x = roundf(point.point[0] * 1000.0);
                point.y = roundf(point.point[1] * 1000.0);
                point.z = roundf(point.point[2] * 1000.0);
            }
        }
    }

    const std::string &type() const { return data_type_; }

    PointImage() {}

    PointImage(uint16_t length, uint16_t width) : length_(length), width_(width) { points_.resize(length * width); }

    PointElement &operator()(const uint16_t &coord_x, const uint16_t &coord_y) {
        return points_[coord_y * length_ + coord_x];
    }

    const PointElement &operator()(const uint16_t &coord_x, const uint16_t &coord_y) const {
        return points_[coord_y * length_ + coord_x];
    }

    const uint16_t &length() const { return length_; }

    const uint16_t &width() const { return width_; }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & points_;
        ar & length_;
        ar & width_;
    }

 private:
    std::vector<PointElement> points_;
    uint16_t length_;
    uint16_t width_;
    const std::string data_type_ = "depth_image";
};

class ScanRecord {
 public:
    void recomputePointState(){//将数据从文件恢复以后，需要将值重新恢复，类似于压缩--解压缩过程
    }

    void computeForBackup(){//在存储之前调用一次，目的是压缩数据，减小存储大小。如果没有这个需求，可不用实现该函数
    }

    const std::string &type() const { return data_type_; }

    ScanRecord() {}

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & angle_min;
        ar & angle_max;
        ar & angle_increment;
        ar & range_min;
        ar & range_max;
        ar & ranges;
        ar & intensities;
    }

 public:
    std::vector<float> ranges;
    std::vector<float> intensities;
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    const std::string data_type_ = "scan_record";
};

class SerializeFileOperator{
 public:
    SerializeFileOperator(){

    }

    virtual ~SerializeFileOperator(){

    }

    bool isOpen(){ return read_stream_.is_open()||write_stream_.is_open(); }

    void close(){ read_stream_.close(), write_stream_.close(); }

    void openToRead(const std::string& file_name){
        read_stream_.close();
        read_stream_.open(file_name,std::ios_base::in|std::ios_base::binary);
    }

    void openToWrite(const std::string& file_name){
        write_stream_.close();
        write_stream_.open(file_name,std::ios_base::out|std::ios_base::binary);
    }

    template <class DataType>
    int64_t backupData(const DataType &data){
        if (write_stream_.is_open()) {
            int64_t file_position = write_stream_.tellp();
            encodeScan(write_stream_, data);
            return file_position;
        }else {
            LOG(WARNING) << "the position is wrong! will return 0";
            return -1;
        }
    }

    int64_t currFileReadPosition(){ return read_stream_.tellg(); }

    template <class DataType>
    bool recoverData(int64_t file_position,DataType& data){
        if (read_stream_.is_open()) {
            if (read_stream_.tellg() != file_position) {
                read_stream_.seekg(file_position);
            }else{
            }
            if(decodeScan(read_stream_, data)){
                read_stream_.get();
                return true;
            }
        }else {
            LOG(INFO) << "the file is not opened!";
        }
        return false;
    }

    template <class DataType>
    std::string dataType(int64_t file_position){
        if (read_stream_.is_open()) {
            if (read_stream_.tellg() != file_position) {
                read_stream_.seekg(file_position);
            } else {
            }
            std::stringstream read_header;
            readLine(read_stream_, read_header);
            std::string header_name;
            read_header >> header_name;
            read_stream_.seekg(file_position);//恢复到之前的position
            return header_name;
        }
        LOG(INFO) << "cannot open read stream!";
        return "";
    }

    template <class DataType>
    bool encodeScan(std::fstream &write_stream, const DataType &data){
        std::stringstream os;
        boost::archive::text_oarchive oa(os);
        oa << data;

        std::stringstream head_stream;
        encodeData(head_stream, data.type());
        encodeData(head_stream, os.str().size());
        writeLine(write_stream,head_stream);
        writeLine(write_stream, os);
        os.clear();
        return true;
    }

    template <class DataType>
    bool decodeScan(std::fstream &read_stream, DataType &data){
        std::stringstream read_header;
        readLine(read_stream, read_header);
        std::string header_name;
        read_header >> header_name;
        if (header_name == data.type()) {
            int size = 0;
            read_header >> size;
            std::vector<char> datas(size);
            if (read(read_stream, size, datas)) {
                std::stringstream is;
                is.write(datas.data(), size);
                try{
                    boost::archive::text_iarchive ia(is);
                    ia >> data;
                    return true;
                }catch (...) {
                    LOG(WARNING) << "resolve wrong!";
                }
            }else {
                LOG(WARNING) << "read err:" << read_stream.gcount() << "size:" << size << "," << header_name << ","
                             << read_stream.tellg();
            }
        }else {
            LOG(WARNING) << "err to get the header:" << header_name;
        }
        return false;
    }

    template <typename T_SAVE>
    void encodeData(std::stringstream &en_data,const T_SAVE &value) {
        en_data << value << ' ';
    }

    template <typename T_READ>
    void decodeData(std::stringstream &de_data, T_READ &value) {
        de_data >> value;
    }

    void writeLine(std::fstream &write_stream,std::stringstream &data) {
        std::string write_data(data.str() + '\n');
        auto length = write_data.length();
        write_stream.write(write_data.data(), length);
    }

    bool readLine(std::fstream &read_stream, std::stringstream &data) {
        std::string data_str;
        std::getline(read_stream, data_str);
        data << data_str;
        return true;
    }

    template <class DatasType>
    bool read(std::fstream& read_stream,const int &length,DatasType& datas){
        datas.resize(length);
        int sum_length = 0;
        const int max_count = 1024;
        int curr_count = 0;
        while (sum_length != length && curr_count++ < max_count) {
            sum_length += read_stream.readsome(datas.data() + sum_length, length - sum_length);
        }
        return curr_count <= max_count;
    }
 private:
    std::fstream read_stream_;
    std::fstream write_stream_;
};

}
#endif  // D435_CALIBRATION_DEPTH_IMAGE_BACKUP_HPP
