
/**
 * @file NavigationMap.hpp
 *
 * @author lhx
 * @date 2015年12月30日
 *
 * @describe NOTE: 地图单位：长度单位cm，角度单位弧度
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef SROS_MAP_NAVIGATIONMAP_H
#define SROS_MAP_NAVIGATIONMAP_H

#include <libpng12/png.h>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

//#include <Eigen/Dense>
#include <glog/logging.h>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/smart_ptr.hpp>

#include "core/pose.h"
#include "core/state.h"
#include "core/util/json.h"
#include "core/settings.h"

#include "mark/AreaMark.hpp"
#include "mark/DMCodeMark.hpp"
#include "mark/StationMark.hpp"
#include "mark/FeatureMark.hpp"

#include "net/edge.hpp"
#include "net/node.hpp"

using nloJson = nlohmann::json;

namespace sros {
namespace map {

template <class CellType>
struct NavMapCell{
    NavMapCell(int cell_size_l, int cell_size_w) : length_(cell_size_l), width_(cell_size_w) {
        cells_.resize(length_ * width_);
        memset(cells_.data(), 0, length_ * width_);
    }

    virtual ~NavMapCell(){ cells_.clear(); }

    CellType &value(const int& x,const int& y){ return cells_[y * length_ + x]; }

    const CellType &value(const int& x,const int& y)const{ return cells_[y * length_ + x]; }

 private:

    int length_;
    int width_;
    std::vector<CellType> cells_;
};

/**
 * @file NavigationMap.hpp
 * @brief 主要类“PyramidNavMap”的头文件
 * 
 * 将网格地图分为大网格和小网格两层，其中小网格包含于大网格中，如果大网格所有小网格都为空白区域，则将大网格设置为空，减小内存占用
 * 
 * @author lfc
 * @date 文件创建日期：2022-3-7
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

template <class CellType>
struct PyramidNavMap {
    /**
    * @brief 初始化地图网格的个数，分辨率、每个大网格包含的小网格数目、网格以地图左上角为起点（？？？）
    * @param length 地图长度方向小网格个数
    * @param width 地图宽度方向小网格个数
    * @param zero_offset_x
    * @param zero_offset_y
    * @param resolution 小网格的分辨率
    * @param cell_size 大网格宽带和高度方向对应小网格的个数
    */
    PyramidNavMap(int length, int width,int zero_offset_x,int zero_offset_y ,float resolution, int cell_size = 16)
        : length_(length), width_(width), resolution_(resolution),cell_size_(cell_size) {
        int fine_x, fine_y;
        getCoord(length_, coarse_length_, fine_x);
        getCoord(width_, coarse_width_, fine_y);
        //当地图不能分成整数个大网格时，通过增加一行或一列大网格确保将整个地图覆盖
        if (fine_x != 0) {
            coarse_length_ += 1;//确保最后一个cell能取到
        }
        if (fine_y != 0) {
            coarse_width_ += 1;
        }
        map_.resize(coarse_length_ * coarse_width_);
        int zero_x = -zero_offset_x;//offset刚好比正常值小1
        int zero_y = (zero_offset_y) - width + 1;

        //栅格地图左上角(起点)相对于绝对坐标系的位姿
        origin_pose_x_i_ = zero_x;
        origin_pose_y_i_ = zero_y;
        zero_offset_x_ = zero_offset_x;
        zero_offset_y_ = zero_offset_y;
    }

    void clear(){
        map_.clear();
        map_.resize(coarse_length_ * coarse_width_);
    }

    /**
    * @brief 通过小网格在地图的索引位置，查找小网格是否为空白区域
    * @param index 小网格在地图的索引（一维索引）
    */
    bool isFreeInGridIndex(const int index)const{
        int x,y;
        toRightCoord(index, length_, x, y);
        int coarse_x, coarse_y, fine_x, fine_y;
        getCoord(x, coarse_x, fine_x);
        getCoord(y, coarse_y, fine_y);
        return isFreeByCoord(coarse_x, coarse_y, fine_x, fine_y);
    }

    /**
    * @brief 通过小网格在地图的索引位置，查找小网格是否为障碍物区域
    * @param index 小网格在地图的索引（一维索引）
    */
    bool isOccInGridIndex(const int index)const{ return !isFreeInGridIndex(index); }

    /**
    * @brief 坐标系变换
    * @param index 小网格在地图的索引（一维索引）
    * @param length 地图长度方向小网格的数目
    * @param coord_x index对应长度方向的小网格位置
    * @param coord_y index对应宽度方向的小网格位置
    */
    void toRightCoord(const int index, const int &length,int &coord_x,int &coord_y) const {
        auto width_index = index / length;
        auto length_index = index % length;
        coord_x = length_index - zero_offset_x_ - origin_pose_x_i_;
        coord_y = zero_offset_y_ - width_index - origin_pose_y_i_;
    }

    /**
    * @brief 通过小网格在地图的索引位置，设置小网格的值
    * @param index 小网格在地图的索引（一维索引）
    * @param value 设置小网格的值
    */
    void setGridIndexValue(const int index, const CellType &value) {
        int x ,y;
        toRightCoord(index, length_, x, y);
        setRightCoordGridValue(x, y, value);
    }

    /**
    * @brief 通过小网格在地图的位置，查找小网格是否为空白区域
    * @param x 小网格在地图长度方向的索引（小网格的位置）
    * @param y 小网格在地图宽度方向的索引（小网格的位置）
    */
    bool isGridFree(const int &x, const int &y) const { return isFreeInGridIndex(x + y * length_); }

    /**
    * @brief 复制地图数据并设置小网格的值
    * @param data 初始网格地图的数据
    * @param free_value 小网格空白区域的值
    * @param occ_value 小网格障碍物区域的值
    */
    template <class ArrayType>
    void cpToArray(ArrayType* data,const ArrayType& free_value,const ArrayType& occ_value) const {
        int index = 0;    //???
        //遍历大网格长度方向
        for (int i = 0; i < coarse_length_; ++i) {
            //遍历大网格宽度方向
            for (int j = 0; j < coarse_width_; ++j) {
                auto &map_ptr = map_[j * coarse_length_ + i];
                bool have_ptr = !(map_ptr == 0);
                //遍历小网格
                for (int k = 0; k < cell_size_; ++k) {
                    for (int l = 0; l < cell_size_; ++l) {
                        auto x = i * cell_size_ + k;
                        auto y = (j * cell_size_ + l);
                        if (inGridMap(x, y)) {    //判断是否超出地图边界
                            int data_index = (width_ - 1 - y) * length_ + x;    //???，如何保证data不越界
                            if (have_ptr) {
                                data[data_index] = map_ptr->value(k, l) == (CellType)0 ? free_value : occ_value;
                                index = map_ptr->value(k, l) == (CellType)0 ? index : index + 1;
                            }else{
                                data[data_index] = free_value;
                            }
                        }
                    }
                }
            }
        }
    }



    /**
     * @brief 复制网格地图数据
     * @param coarse_map 存放地图的数据
    */
    void cpToCoarseMap(std::shared_ptr<PyramidNavMap<CellType>>& coarse_map){
        coarse_map->clear();
        //遍历大网格长度方向
        for (int i = 0; i < coarse_length_; ++i) {
            //遍历大网格宽度方向
            for (int j = 0; j < coarse_width_; ++j) {
                auto &map_ptr = map_[j * coarse_length_ + i];
                if (map_ptr) {
                    //遍历小网格
                    for (int k = 0; k < cell_size_; ++k) {
                        for (int l = 0; l < cell_size_; ++l) {
                            auto x = i * cell_size_ + k;
                            auto y = width_ -1 - (j * cell_size_ + l);
                            if(inGridMap(x,y)) {    //判断是否超出地图边界
                                auto value = map_ptr->value(k, l);
                                if (value != 0) {
                                    coarse_map->setValue(x * resolution_, y * resolution_, value);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * @brief 设置网格地图小网格的值
     * @param x 小网格在长度方向的位置
     * @param y 小网格在宽度方向的位置
     * @param value 小网格的值
    */
    void setValue(const float &x, const float &y,const CellType& value){
        int x_i = floorf(x / resolution_ + 0.5f);    //采用进一法计算小网格中心的位置
        int y_i = floorf(y / resolution_ + 0.5f);    //采用进一法计算小网格中心的位置
        setGridValue(x_i, y_i, value);
    }

    /**
     * @brief 设置网格地图小网格的值
     * @param x 小网格在长度方向的位置
     * @param y 小网格在宽度方向的位置
     * @param value 小网格的值
    */
    void setGridValue(const int &x, const int &y,const CellType& value){ setGridIndexValue(x + y * length_, value); }

    /**
     * @brief 设置网格地图小网格的值
     * @param x 小网格在长度方向的位置
     * @param y 小网格在宽度方向的位置
     * @param value 小网格的值
    */
    void setRightCoordGridValue(const int &x, const int &y,const CellType& value){
        if (inGridMap(x, y)) {    //判断是否越界
            int coarse_x, coarse_y, fine_x, fine_y;
            getCoord(x, coarse_x, fine_x);
            getCoord(y, coarse_y, fine_y);
//            LOG(INFO) << "co:" << coarse_x << "," << coarse_y << "," << fine_x << "," << fine_y << "," << x << "," << y
//                      << "," << length_ << "," << width_ << "," << coarse_length_ << "," << coarse_width_;
            setValueByCoord(coarse_x, coarse_y, fine_x, fine_y,value);
        }
    }

    /**
     * @brief 获取网格地图小网格的分辨率
     * @return 网格地图小网格的分辨率
    */
    float resolution(){ return resolution_; }
    //    bool inMap

 private:
    /**
     * @brief 通过大网格、小网格的位置确定小网格是否为空白区域
     * @param coarse_x 大网格长度方向的位置
     * @param coarse_y 大网格宽度方向的位置
     * @param fine_x 小网格在大网格中长度方向的位置
     * @param fine_y 小网格在大网格中宽度方向的位置
     * @return 小网格是否为空白区域
    */
    bool isFreeByCoord(const int coarse_x,const int coarse_y,const int fine_x,const int fine_y)const{
        auto &cell_ptr = map_[coarse_y * coarse_length_ + coarse_x];
        return cell_ptr != 0 ? cell_ptr->value(fine_x, fine_y) == (CellType)0 : true;
    }

    /**
     * @brief 通过大网格、小网格的位置，设置大、小网格的值
     * @param coarse_x 大网格长度方向的位置
     * @param coarse_y 大网格宽度方向的位置
     * @param fine_x 小网格在大网格中长度方向的位置
     * @param fine_y 小网格在大网格中宽度方向的位置
    */
    void setValueByCoord(const int coarse_x,const int coarse_y,const int fine_x,const int fine_y,const CellType& value){
        auto &cell_ptr = map_[coarse_y * coarse_length_ + coarse_x];
        if (!cell_ptr) {
            if (value != (CellType)0) {
                cell_ptr.reset(new NavMapCell<CellType>(cell_size_, cell_size_));
            }
        }
        if (cell_ptr) {
            cell_ptr->value(fine_x, fine_y) = value;
        }
    }

    /**
     * @brief 通过小网格在网格地图的位置，判断是否越界
     * @param x 小网格在大网格中长度方向的位置
     * @param y 小网格在大网格中宽度方向的位置
     * @return 是否越界
    */
    bool inGridMap(const int& x,const int &y) const {
        return x >= 0 && x < length_ && y >= 0 && y < width_;
    }

    /**
     * @brief 通过小网格的索引计算宽度或长度方向大网格、小网格的位置
     * @param coord 小网格在宽度或长度方向的位置
     * @param coarse 大网格在宽度或长度方向的位置
     * @param fine 小网格在大网格中宽度或长度方向的位置
    */
    void getCoord(const int &coord,int &coarse,int&fine) const {
        coarse = coord / cell_size_;
        fine = coord % cell_size_;
    }

    /**
     * @brief 通过小网个中心点的坐标计算宽度或长度方向大网格、小网格的位置
     * @param coord 小网格在宽度或长度方向的位置
     * @param coarse 大网格在宽度或长度方向的位置
     * @param fine 小网格在大网格中宽度或长度方向的位置
    */
    void getCoord(const float &coord,int &coarse,int &fine) const {
        int coord_i = floorf(coord/resolution_ + 0.5f);
        getCoord(coord_i, coarse, fine);
    }

    int length_;    //地图长度方向小网格个数
    int width_;    //地图宽度方向小网格个数
    int coarse_length_;    //地图长度方向大网格个数
    int coarse_width_;    //地图宽度方向大网格个数
    int origin_pose_x_i_;
    int origin_pose_y_i_;
    int zero_offset_x_;
    int zero_offset_y_;
    float resolution_;    //小网格的分辨率
    int cell_size_;    //大网格边长由多少个小网格组成
    std::vector<std::shared_ptr<NavMapCell<CellType>>> map_;    //网格地图数据（包含大网格、小网格）
};


using namespace std;

#define NAV_MAGIC_STR "SM5"

typedef unsigned char UChar_t;
typedef unsigned char NavMapNode_t;
typedef boost::shared_array<NavMapNode_t>
    NavMapData_ptr;  // FIXME(pengjiali): 将此处的NavMapNode_t用一位来表示，项目中出现内存过大的问题


typedef std::shared_ptr<PyramidNavMap<NavMapNode_t>> PyramidNavMap_ptr;

static const UChar_t T_VALUE = 0x4F;  // 判定地图上某点是否为障碍的灰度阈值
static const UChar_t FREE = 255;
static const UChar_t OCC = 0;

const string SROS_MAP_DIR = "/sros/map/";

enum MapFileVersion {
    FILE_VER_UNKNOWN = 0,
    FILE_VER_1 = 1,
    FILE_VER_2 = 2,
    FILE_VER_3 = 3,
    FILE_VER_4 = 4,
    FILE_VER_5 = 5,
    FILE_VER_6 = 6,
};

class Point {
 public:
    int x, y;

    Point() : x(0), y(0) {}

    Point(int x, int y) : x(x), y(y) {}

    ~Point() {}

    bool operator<(const Point &p) const {
        if (this->x < p.x)
            return true;
        else if (this->x == p.x)
            return this->y <= p.y;
        else
            return false;
    }

    bool operator==(const Point &p) const { return (this->x == p.x && this->y == p.y); }

    bool operator>(const Point &p) const {
        if (this->x > p.x)
            return true;
        else if (this->x == p.x)
            return this->y >= p.y;
        else
            return false;
    }

    Point &operator+(const Point &p) {
        this->x = this->x + p.x;
        this->y = this->y + p.y;
        return *this;
    }

    Point &operator-(const Point &p) {
        this->x = this->x - p.x;
        this->y = this->y - p.y;
        return *this;
    }

    Point &operator=(const Point &p) {
        this->x = p.x;
        this->y = p.y;
        return *this;
    }
};

static bool convertGrayToNavigation(string gray_map_path, string nav_map_path);
static bool convertNavMapDataToPGM(NavMapData_ptr array, int width, int height, string pgm_path);

/**
 * 处理导航地图文件
 *
 */
class NavigationMap {
 public:
    NavigationMap()
        : size_(0, 0),
          zero_offset_(0, 0),
          resolution_(1),
          modified_timestamp_(0),
          created_timestamp_(0),
          map_file_version_(FILE_VER_5),
          enable_load_map_data_(true) {
        //        mark_group_.reset(new MarkGroup);
    }

    NavigationMap(Point size, Point zero_offset, int resolution)
        : size_(size), zero_offset_(zero_offset), resolution_(resolution) {
        float resolution_f = resolution_ / 100.0f;
        static_map_data_.reset(new PyramidNavMap<NavMapNode_t>(size_.x,size_.y,zero_offset_.x,zero_offset_.y,resolution_f));
    }

    MapFileVersion getMapVersion() { return map_file_version_; }

    Point getMapZeroOffset() { return zero_offset_; }

     void setMapZeroOffset(Point point) {  zero_offset_ = point; }

    int getMapZeroOffsetX() { return zero_offset_.x; }

    int getMapZeroOffsetY() { return zero_offset_.y; }

    Point getMapSize() { return size_; }

    void setMapSize(Point point) {  size_ = point; }

    int getMapSizeX() { return size_.x; }

    int getMapSizeY() { return size_.y; }

    int getMapResolution() { return resolution_; }

    void setMapResolution(int res) { resolution_ = res; }

    std::string getMapPath() { return map_path_; }

    StationMarkGroup *getStationMarkGroup() { return &station_mg_; }

    DMCodeMarkGroup *getDMCodeMarkGroup() { return &dmcode_mg_; }

    AreaMarkGroup *getAreaMarkGroup() { return &area_mg_; }

    FeatureMarkGroup *getFeatureMarkGroup() { return &feature_mg_; }

    net::EdgeGroup *getNetEdgeGroup() { return &edge_group_; }

    net::NodeGroup *getNetNodeGroup() { return &node_group_; }

    PyramidNavMap_ptr getMapData() { return static_map_data_; }

    void setMapData(std::shared_ptr<PyramidNavMap<NavMapNode_t>> mapdata){
        static_map_data_ = mapdata;
    }

    PyramidNavMap_ptr getDisplayMapData() { return display_map_data_; }

    /**
     * 根据站点编码获取站点绑定的特征信息
     */
    bool getFeatureMark(sros::core::StationNo_t station_no, FeatureMark& feature_mark) {
        if(station_no <= 0) {
            FeatureMark temp;
            feature_mark = temp;
            return false;
        }

        auto feature = feature_mg_.getItem(station_no);
        if(feature.station_id == station_no) {
            feature_mark = feature;
            return true;
        } else {
            FeatureMark temp;
            feature_mark = temp;
            return false;
        }
    }

    bool handleLoadSM6File(ifstream &in) {
        uint32_t checksum_crc32 = 0;

        read_field(in, checksum_crc32);

        // 计算校验码

        uint32_t meta_data_len = 0;
        uint32_t gray_data_len = 0;

        read_field(in, meta_data_len);
        read_field(in, gray_data_len);

        if (meta_data_len > 0) {  // 读取并解析meta data
            uint8_t *meta_data_buf = new uint8_t[meta_data_len];

            read_field(in, meta_data_buf, meta_data_len);

            // 解析protobuf

            delete meta_data_buf;
        }

        return false;
    }

    /**
     * 加载指定的地图文件
     */
    bool loadFile(const std::string file_path, bool enable_load_map_data = true) {
        int area_mark_cnt, station_mark_cnt;  // 文件中mark的个数
        // int mark_type, sx, sy, ex, ey, id;
        char name[MAX_MARK_NAME_LEN] = {'\0'};

        MapFileVersion file_version;

        enable_load_map_data_ = enable_load_map_data;

        ifstream in(file_path, ifstream::in | ifstream::binary);
        if (in.fail()) {
            cerr << "Failure to open file:" << file_path << " for intput" << endl;
            return false;
        }
        map_path_ = file_path;

        read_field(in, name, MAGIC_STR_LEN);  // SM1

        file_version = getVersionByStr(name);

        if (file_version == FILE_VER_UNKNOWN) {
            cerr << "File format unknown: " << name << endl;
            return false;
        }

        if (file_version == FILE_VER_6) {
            return handleLoadSM6File(in);
        }

        map_file_version_ = file_version;  // 保存当前打开的文件版本到成员变量

        int zero_x, zero_y, size_x, size_y;

        // 读取文件头
        read_field(in, zero_x);
        read_field(in, zero_y);
        read_field(in, size_x);
        read_field(in, size_y);

        if (file_version >= FILE_VER_4) {
            read_field(in, resolution_);

            read_field(in, created_timestamp_);
            read_field(in, modified_timestamp_);

            read_field(in, unused_parmeter0_);
            read_field(in, unused_parmeter1_);
        } else {
            resolution_ = 1;
            created_timestamp_ = 0;
            modified_timestamp_ = 0;
            unused_parmeter0_ = 0;
            unused_parmeter1_ = 0;
        }

        zero_offset_ = Point(zero_x, zero_y);
        size_ = Point(size_x, size_y);

        read_field(in, area_mark_cnt);
        read_field(in, station_mark_cnt);

        // 读取文件灰度
        if (enable_load_map_data_) {
            float resolution_f = resolution_ / 100.0f;
            static_map_data_.reset(new PyramidNavMap<NavMapNode_t>(size_.x,size_.y,zero_offset_.x,zero_offset_.y,resolution_f));
            display_map_data_.reset(new PyramidNavMap<NavMapNode_t>(size_.x,size_.y,zero_offset_.x,zero_offset_.y,resolution_f));
            LOG(INFO) << "load!"<<resolution_f<<","<<zero_offset_.x<<","<<zero_offset_.y;
            //            display_map_data_.reset(new NavMapNode_t[size_.x * size_.y]);

            UChar_t value;
            int getGray;
            int index = 0;
            for (int i = 0; i < size_.x * size_.y / 8; ++i) {
                in.read(reinterpret_cast<char *>(&value), sizeof(UChar_t));
                for (int j = 0; j < 8; ++j) {
                    getGray = value & (128 >> j);
                    if (getGray == (128 >> j)) {
                        index++;
                        static_map_data_->setGridIndexValue(i * 8 + j,1);
                        display_map_data_->setGridIndexValue(i * 8 + j,1);
                        //                        static_map_data_[i * 8 + j] = 1;
//                        display_map_data_[i * 8 + j] = 1;
                    } else {
//                        static_map_data_[i * 8 + j] = 0;
//                        display_map_data_[i * 8 + j] = 0;
                    }
                }
            }
            LOG(INFO) << "index:" << index << "," << size_.x * size_.y;
        } else {
            int map_data_size = size_.x * size_.y / 8;  // map_data的字节个数
            in.ignore(map_data_size);                   // 跳过map_data区域
        }

        const std::string json_file_path = file_path.substr(0, file_path.size() - 3) + "json";  // 将map后缀替换成json
        if (boost::filesystem::exists(json_file_path)) {  // 若json文件存在，优先存json中读取路径相关信息
            try {
                if (!importJsonFile(json_file_path.c_str())) {
                    LOG(INFO) << "Failed to load json file: " << json_file_path;
                    in.close();

                    return false;
                }
            } catch (std::exception &e) {
                LOG(INFO) << "Failed to load json file: " << json_file_path;
                in.close();

                return false;
            }
        } else {
            area_mg_.clearAllItem();
            AreaMark area_mark;
            for (int i = 0; i < area_mark_cnt; i++) {
                read_field(in, area_mark.id);
                read_field(in, area_mark.name, MAX_MARK_NAME_LEN);
                read_field(in, area_mark.type);
                read_field(in, area_mark.pa);
                read_field(in, area_mark.pb);
                read_field(in, area_mark.pc);
                read_field(in, area_mark.pd);

                area_mg_.addItem(area_mark.id, area_mark);
            }

            station_mg_.clearAllItem();
            StationMark station_mark;
            for (int i = 0; i < station_mark_cnt; i++) {
                read_field(in, station_mark.id);
                read_field(in, station_mark.name, MAX_MARK_NAME_LEN);
                read_field(in, station_mark.type);
                read_field(in, station_mark.pos);
                read_field(in, station_mark.enter_pos);
                read_field(in, station_mark.exit_pos);
                read_field(in, &station_mark.no_rotate, 1);
                read_field(in, &station_mark.enter_backward, 1);
                read_field(in, &station_mark.exit_backward, 1);

                if (file_version >= FILE_VER_2) {
                    read_field(in, station_mark.check_pos);
                    read_field(in, &station_mark.pos_dynamic, 1);
                    read_field(in, station_mark.station_offset);
                    read_field(in, station_mark.param);
                }

                if (file_version >= FILE_VER_3) {
                    read_field(in, station_mark.edge_id);
                }

                station_mg_.addItem(station_mark.id, station_mark);
            }

            if (file_version >= FILE_VER_3) {
                int edge_cnt, node_cnt;
                read_field(in, node_cnt);

                node_group_.clearAllItem();
                net::Nodef node;
                for (int i = 0; i < node_cnt; i++) {
                    read_field(in, node.id);
                    read_field(in, node.x);
                    read_field(in, node.y);
                    read_field(in, node.yaw);

                    node_group_.addItem(node.id, node);
                }

                read_field(in, edge_cnt);

                edge_group_.clearAllItem();
                net::Edgef edge;
                for (int i = 0; i < edge_cnt; i++) {
                    read_field(in, edge.id);
                    read_field(in, edge.type);
                    read_field(in, edge.s_node);
                    read_field(in, edge.e_node);

                    read_field(in, edge.cost);

                    read_field(in, edge.sx);
                    read_field(in, edge.sy);
                    read_field(in, edge.ex);
                    read_field(in, edge.ey);
                    read_field(in, edge.cx);
                    read_field(in, edge.cy);
                    read_field(in, edge.dx);
                    read_field(in, edge.dy);
                    read_field(in, edge.radius);

                    if (file_version >= FILE_VER_5) {
                        read_field(in, edge.limit_v);
                        read_field(in, edge.limit_w);

                        read_field(in, edge.s_facing);
                        read_field(in, edge.e_facing);
                        read_field(in, edge.rotate_direction);
                        read_field(in, edge.direction);

                        read_field(in, edge.execute_type);
                    }

                    edge_group_.addItem(edge.id, edge);
                }
            }
        }

        in.close();

        addNoEnterAreaToStaticMap();

        return true;
    }

    MapFileVersion getVersionByStr(const char *name) const {
        MapFileVersion file_version;
        if (strcmp(name, "SM1") == 0) {
            file_version = FILE_VER_1;
        } else if (strcmp(name, "SM2") == 0) {
            file_version = FILE_VER_2;
        } else if (strcmp(name, "SM3") == 0) {
            file_version = FILE_VER_3;
        } else if (strcmp(name, "SM4") == 0) {
            file_version = FILE_VER_4;
        } else if (strcmp(name, "SM5") == 0) {
            file_version = FILE_VER_5;
        } else {
            file_version = FILE_VER_UNKNOWN;
        }
        return file_version;
    }

    /**
     * 保存地图数据到指定路径
     */
    bool saveFile(const char *file_path) {
        int i, j;
        // char name[MAX_MARK_NAME_LEN] = {'\0'};

        // 使用打开的文件版本作为保存时的版本
        MapFileVersion file_version = map_file_version_;

        if (!enable_load_map_data_) {
            // 如果上次加载地图时没有加载灰度数据, 就不能保存地图文件, 因为没有灰度信息
            return false;
        }

        ofstream out(file_path, ifstream::out | ifstream::binary);
        if (out.fail()) {
            cerr << "Failure to open for output" << endl;
            return false;
        }

        int area_mark_cnt = static_cast<int>(area_mg_.getItemCnt());
        int station_mark_cnt = static_cast<int>(station_mg_.getItemCnt());

        if (file_version == FILE_VER_4) {
            write_field(out, "SM4", MAGIC_STR_LEN);
        } else if (file_version == FILE_VER_3) {
            write_field(out, "SM3", MAGIC_STR_LEN);
        } else if (file_version == FILE_VER_2) {
            write_field(out, "SM2", MAGIC_STR_LEN);
        } else if (file_version == FILE_VER_5) {
            write_field(out, "SM5", MAGIC_STR_LEN);
        } else {
            write_field(out, "SM0", MAGIC_STR_LEN);
        }

        int zero_x = zero_offset_.x;
        int zero_y = zero_offset_.y;
        int size_x = size_.x;
        int size_y = size_.y;

        write_field(out, zero_x);
        write_field(out, zero_y);
        write_field(out, size_x);
        write_field(out, size_y);

        if (file_version >= FILE_VER_4) {
            write_field(out, resolution_);

            write_field(out, created_timestamp_);
            write_field(out, modified_timestamp_);

            write_field(out, unused_parmeter0_);
            write_field(out, unused_parmeter1_);
        }

        write_field(out, area_mark_cnt);
        write_field(out, station_mark_cnt);

        UChar_t value;
        for (i = 0; i < size_.x * size_.y / 8; ++i) {
            value = 0;
            for (j = 0; j < 8; ++j) {
                // value += (static_map_data_[i * 8 + j] << (7 - j));
                int x,y;
                toRightCoord(i * 8 + j, size_x, x, y);
                value = (value << 1) + (NavMapNode_t)display_map_data_->isOccInGridIndex(i * 8 + j);
            }
            out.write(reinterpret_cast<char *>(&value), sizeof(UChar_t));
        }

        std::vector<AreaMark> area_mark_list = area_mg_.getItemList();
        for (auto area_mark : area_mark_list) {
            write_field(out, area_mark.id);
            write_field(out, area_mark.name, MAX_MARK_NAME_LEN);
            write_field(out, area_mark.type);
            write_field(out, area_mark.pa);
            write_field(out, area_mark.pb);
            write_field(out, area_mark.pc);
            write_field(out, area_mark.pd);
        }

        std::vector<StationMark> station_mark_list = station_mg_.getItemList();
        for (auto station_mark : station_mark_list) {
            write_field(out, station_mark.id);
            write_field(out, station_mark.name, MAX_MARK_NAME_LEN);
            write_field(out, station_mark.type);
            write_field(out, station_mark.pos);
            write_field(out, station_mark.enter_pos);
            write_field(out, station_mark.exit_pos);
            write_field(out, &station_mark.no_rotate, 1);
            write_field(out, &station_mark.enter_backward, 1);
            write_field(out, &station_mark.exit_backward, 1);

            if (file_version >= FILE_VER_2) {
                write_field(out, station_mark.check_pos);
                write_field(out, &station_mark.pos_dynamic, 1);
                write_field(out, station_mark.station_offset);
                write_field(out, station_mark.param);
            }

            if (file_version >= FILE_VER_3) {
                write_field(out, station_mark.edge_id);
            }
        }

        if (file_version >= FILE_VER_3) {
            int node_cnt = static_cast<int>(node_group_.getItemCnt());
            write_field(out, node_cnt);

            auto node_list = node_group_.getItemList();
            for (auto node : node_list) {
                write_field(out, node.id);
                write_field(out, node.x);
                write_field(out, node.y);
                write_field(out, node.yaw);
            }

            int edge_cnt = static_cast<int>(edge_group_.getItemCnt());
            write_field(out, edge_cnt);

            auto edge_list = edge_group_.getItemList();
            for (auto edge : edge_list) {
                write_field(out, edge.id);
                write_field(out, edge.type);
                write_field(out, edge.s_node);
                write_field(out, edge.e_node);

                write_field(out, edge.cost);

                write_field(out, edge.sx);
                write_field(out, edge.sy);
                write_field(out, edge.ex);
                write_field(out, edge.ey);
                write_field(out, edge.cx);
                write_field(out, edge.cy);
                write_field(out, edge.dx);
                write_field(out, edge.dy);
                write_field(out, edge.radius);

                if (file_version >= FILE_VER_5) {
                    write_field(out, edge.limit_v);
                    write_field(out, edge.limit_w);

                    write_field(out, edge.s_facing);
                    write_field(out, edge.e_facing);
                    write_field(out, edge.rotate_direction);
                    write_field(out, edge.direction);

                    write_field(out, edge.execute_type);
                }
            }
        }

        out.close();
        return true;
    }

    std::string readFile(const char *json_file) {
        ifstream in(json_file);
        if (!in.is_open()) {
            std::cerr << "Failed to open file " << json_file << std::endl;
            return "";
        }

        std::stringstream buffer;
        buffer << in.rdbuf();
        std::string file_content(buffer.str());
        return file_content;
    }

    // json文件结构：
    // {
    //   "meta": {
    //      "size.x": 1024,
    //      "size.y": 1024,
    //      "resolution": 2,
    //          ...
    //    },
    //   "data": {
    //      "node": [],
    //      "area": [],
    //      "edge": [],
    //      "station": [],
    //      "tag": [],
    //      "barcode": [],
    //      "building": {
    //          "wall": [],
    //          "door": [],
    //          "pillar": [],
    //          "device": [],
    //          "otherArea": []
    //      },
    //    },
    //    "private": {
    //      "cleanArea": []
    //    }
    // }

#define GET_VALUE(p, key, cast_type) getValue<cast_type>(p, key, __FILE__, __LINE__)
    /**
     * [json.exception.type_error.302] type must be number, but is number  key:ex, NavigationMap.hpp:783
     * 上面的提示实际上是没有ex这个字段
     */
    template <class CastType>
    CastType getValue(const nloJson &p, std::string key, const char *file, int line) {
        try {
            return p[key].get<CastType>();
        } catch (std::exception &e) {
            auto str = std::string(e.what()) + std::string("  key:") + key + ", " +
                       boost::filesystem::path(file).filename().c_str() + ":" + std::to_string(line);
            throw std::logic_error(str);
        }
    }

    // 计算车辆在直线上的运动是车头方向，前、后、左、右
    net::VehicleDirection getVehicleDirection(const sros::map::net::Edgef &edge) const {
        // 如果地图不包含s_facing等数据，则不能够计算路径运动方向
        if (map_file_version_ < sros::map::FILE_VER_5) {
            return net::VehicleDirection::FORWARD;
        }

        sros::core::Location tmp;  // 路径从起点到终点的朝向的单位向量
        switch (edge.type) {
            case net::EDGE_LINE: {
                tmp.x() = edge.ex - edge.sx;
                tmp.y() = edge.ey - edge.sy;
                break;
            }
            case net::EDGE_BEZIER: {
                tmp.x() = edge.cx - edge.sx;
                tmp.y() = edge.cy - edge.sy;
                break;
            }
            case net::EDGE_CIRCLE: {
                tmp.x() = edge.cx - edge.sx;
                tmp.y() = edge.cy - edge.sy;
                break;
            }
            default: {
                break;
            }
        }

        double tmp_len = sqrt(pow(tmp.x(), 2) + pow(tmp.y(), 2));
        tmp.x() /= tmp_len;  // 得到单位向量
        tmp.y() /= tmp_len;

        // 起点朝向的单位向量, NOTE: 此处用的是起点朝向
        sros::core::Location s_facing = sros::core::Location(cos(edge.s_facing), sin(edge.s_facing));

        auto normalizeYawFunc = [](double yaw) -> double {
            yaw = fmod(fmod(yaw, 2.0 * M_PI), 2.0 * M_PI);
            do {
                if (yaw >= M_PI) {
                    yaw -= 2.0f * M_PI;
                } else if (yaw < -M_PI) {
                    yaw += 2.0f * M_PI;
                }
            } while (yaw >= M_PI || yaw < -M_PI);

            return yaw;
        };
        double angle = normalizeYawFunc(atan2(s_facing.y(), s_facing.x()) - atan2(tmp.y(), tmp.x()));
        if (angle <= M_PI_4 && angle > -M_PI_4) {
            return net::VehicleDirection::FORWARD;
        } else if (angle <= M_PI_4 + M_PI_2 && angle > M_PI_4) {
            return net::VehicleDirection::LEFT;
        } else if ((angle <= M_PI + M_PI_4 && angle > M_PI_4 + M_PI_2) ||
                   (angle <= -M_PI_4 - M_PI_2 && angle > -M_PI - M_PI_4)) {
            return net::VehicleDirection::BACKWARD;
        } else if (angle <= -M_PI_4 && angle > -M_PI_4 - M_PI_2) {
            return net::VehicleDirection::RIGHT;
        }

        // unreachable
        LOG(INFO) << "unreachable! angle is " << angle;
        return net::VehicleDirection::FORWARD;
    }

    bool importJsonFile(const char *json_file_path) {
        ifstream in(json_file_path);
        nloJson json_map;
        in >> json_map;

        in.close();

        // std::string file_content = readFile(json_file_path);
        // std::cout << file_content << std::endl;

        // nloJson json_map;
        // json_map.parse(file_content);

        // json中长度单位为mm,.map中单位为cm
        // json中弧度单位放大1000倍
        int len_unit = 10.0;
        int rad_unit = 1000.0;
        int feature_pose_unit = 1e6;

        nloJson json_data = json_map["data"];
        if (!json_data.is_object()) {
            throw "Json main data is not object";
        }

        // 不需要更新偏移值,大小,分辨率等数据
        boost::posix_time::ptime ct2(boost::posix_time::second_clock::universal_time());
        // modified_timestamp_ = ct2.time_since_epoch();

        //读取区域的user_defined_params，不同的车型对应的value的名称为 “车型号_value”
        auto &s = sros::core::Settings::getInstance();
        std::string vehicleType = s.getValue<std::string>("main.vehicle_type","");
        std::string valueName="value";
        if(!vehicleType.empty()){
            valueName = vehicleType + "_" + valueName;
        }
        auto findKey = [&](nloJson &obj,std::string&key) {
            auto Iter = obj.find(key);
            if (Iter != obj.end()){
                return true;
            }else{
                return false;
            }
        };

        std::string type = vehicleType.substr(0,4);
        bool isForkType = (type == "gulf" || type == "Gulf");   //叉车系列

        nloJson items;

        items = json_data["area"];
        area_mg_.clearAllItem();
        for (int i = 0; i < items.size(); i++) {
            AreaMark area_mark;
            nloJson p = items[i];
            area_mark.id = GET_VALUE(p, "id", int);
            area_mark.type = GET_VALUE(p, "type", int);
            if (area_mark.type == AreaMark::AREA_TYPE_MULTIPLE_TYPE) {
                nloJson area_groups = p["area_group"];
                for (int j = 0; j < area_groups.size(); j++) {
                    area_mark.user_define_type_list.push_back(area_groups[j]);
                }
            }
            if (p.find("user_defined_params") != p.end()) {
                nloJson user_defined_params = p["user_defined_params"];
                for (int k = 0; k < user_defined_params.size(); k++) {
                    nloJson param_item = user_defined_params[k];
                    //只插入该区域内本车型号的定义参数
                    if(findKey(param_item,valueName)){
                        area_mark.user_define_param.push_back(
                        make_pair(GET_VALUE(param_item, "key", string), GET_VALUE(param_item, valueName, string)));
                    }else{  //兼容O车的value
                        if(!isForkType){    //O车
                            std::string defaultValue="value";
                            if(findKey(param_item,defaultValue)){
                                area_mark.user_define_param.push_back(
                                make_pair(GET_VALUE(param_item, "key", string), GET_VALUE(param_item, defaultValue, string)));
                            }
                        }
                    } 
                }
            }
            if (p.find("z") != p.end()) {
                area_mark.z = GET_VALUE(p, "z", int);
            }
            std::string name = GET_VALUE(p, "name", std::string);
            strcpy(area_mark.name, name.c_str());
            area_mark.pa.x = GET_VALUE(p, "pa.x", float) / len_unit;
            area_mark.pa.y = GET_VALUE(p, "pa.y", float) / len_unit;
            area_mark.pb.x = GET_VALUE(p, "pb.x", float) / len_unit;
            area_mark.pb.y = GET_VALUE(p, "pb.y", float) / len_unit;
            area_mark.pc.x = GET_VALUE(p, "pc.x", float) / len_unit;
            area_mark.pc.y = GET_VALUE(p, "pc.y", float) / len_unit;
            area_mark.pd.x = GET_VALUE(p, "pd.x", float) / len_unit;
            area_mark.pd.y = GET_VALUE(p, "pd.y", float) / len_unit;
            if (p.find("enter_station") != p.end()) {
                area_mark.enter_station = GET_VALUE(p, "enter_station", int);
                area_mark.exit_station = GET_VALUE(p, "exit_station", int);
            }
            if (p.find("is_broadcast") != p.end()) {
                area_mark.is_broadcast = GET_VALUE(p, "is_broadcast", bool);
                area_mark.is_play_music = GET_VALUE(p, "is_play_music", bool);
            }
            if (p.find("param_int") != p.end()) {
                area_mark.param_int = GET_VALUE(p, "param_int", int);
            }
            area_mg_.addItem(area_mark.id, area_mark);
        }

        items = json_data["station"];
        StationMark station_mark;
        station_mg_.clearAllItem();
        for (int i = 0; i < items.size(); i++) {
            nloJson p = items[i];
            station_mark.id = GET_VALUE(p, "id", int);
            station_mark.type = GET_VALUE(p, "type", int);
            std::string name = GET_VALUE(p, "name", std::string);
            strcpy(station_mark.name, name.c_str());

            station_mark.pos.x = GET_VALUE(p, "pos.x", float) / len_unit;
            station_mark.pos.y = GET_VALUE(p, "pos.y", float) / len_unit;
            station_mark.pos.yaw = GET_VALUE(p, "pos.yaw", float) / rad_unit;
            station_mark.enter_pos.x = GET_VALUE(p, "enter_pos.x", float) / len_unit;
            station_mark.enter_pos.y = GET_VALUE(p, "enter_pos.y", float) / len_unit;
            station_mark.enter_pos.yaw = GET_VALUE(p, "enter_pos.yaw", float) / rad_unit;
            station_mark.exit_pos.x = GET_VALUE(p, "exit_pos.x", float) / len_unit;
            station_mark.exit_pos.y = GET_VALUE(p, "exit_pos.y", float) / len_unit;
            station_mark.exit_pos.yaw = GET_VALUE(p, "exit_pos.yaw", float) / rad_unit;
            station_mark.check_pos.x = GET_VALUE(p, "check_pos.x", float) / len_unit;
            station_mark.check_pos.y = GET_VALUE(p, "check_pos.y", float) / len_unit;
            station_mark.check_pos.yaw = GET_VALUE(p, "check_pos.yaw", float) / rad_unit;
            station_mark.edge_id = GET_VALUE(p, "edge_id", int);
            station_mark.enter_backward = GET_VALUE(p, "enter_backward", bool);
            station_mark.exit_backward = GET_VALUE(p, "exit_backward", bool);
            station_mark.no_rotate = GET_VALUE(p, "no_rotate", bool);
            station_mark.param = GET_VALUE(p, "param", int);
            station_mark.pos_dynamic = GET_VALUE(p, "pos_dynamic", bool);
            station_mark.station_offset = GET_VALUE(p, "station_offset", int);
            if (p.find("dmcode_id") != p.end()) {
                station_mark.dmcode_id = GET_VALUE(p, "dmcode_id", std::string);
                station_mark.dmcode_offset.x = GET_VALUE(p, "pgv_offset.x", float) / len_unit;
                station_mark.dmcode_offset.y = GET_VALUE(p, "pgv_offset.y", float) / len_unit;
                station_mark.dmcode_offset.yaw = GET_VALUE(p, "pgv_offset.yaw", float) / rad_unit;
            }

            station_mg_.addItem(station_mark.id, station_mark);
        }

        // 二维码
        dmcode_mg_.clearAllItem();
        if (json_data.find("dmcode") != json_data.end()) {
            items = json_data["dmcode"];
            DMCodeMark dmcode_mark;
            for (int i = 0; i < items.size(); i++) {
                nloJson p = items[i];

                if (p.find("id") != p.end()) {  // 华为二维码时，可以不存在
                    dmcode_mark.id = GET_VALUE(p, "id", int);
                }
                dmcode_mark.no = GET_VALUE(p, "dmcode_id", std::string);
                dmcode_mark.x = GET_VALUE(p, "x", float) / len_unit;
                dmcode_mark.y = GET_VALUE(p, "y", float) / len_unit;
                dmcode_mark.yaw = GET_VALUE(p, "yaw", float) / rad_unit;

                dmcode_mg_.addItem(dmcode_mark.no, dmcode_mark);
            }
        }

        items = json_data["node"];
        node_group_.clearAllItem();
        net::Nodef node_mark;
        for (int i = 0; i < items.size(); i++) {
            nloJson p = items[i];
            node_mark.id = GET_VALUE(p, "id", int);
            node_mark.x = GET_VALUE(p, "x", float) / len_unit;
            node_mark.y = GET_VALUE(p, "y", float) / len_unit;
            node_mark.yaw = GET_VALUE(p, "yaw", float) / rad_unit;

            node_group_.addItem(node_mark.id, node_mark);
        }

        items = json_data["edge"];
        edge_group_.clearAllItem();
        net::Edgef edge_mark;
        for (int i = 0; i < items.size(); i++) {
            nloJson p = items[i];
            edge_mark.id = GET_VALUE(p, "id", int);
//            LOG(INFO) << "edge id: " << edge_mark.id;
            edge_mark.type = (net::EdgeType)GET_VALUE(p, "type", int);
            edge_mark.sx = GET_VALUE(p, "sx", float) / len_unit;
            edge_mark.sy = GET_VALUE(p, "sy", float) / len_unit;
            edge_mark.ex = GET_VALUE(p, "ex", float) / len_unit;
            edge_mark.ey = GET_VALUE(p, "ey", float) / len_unit;
            edge_mark.cx = GET_VALUE(p, "cx", float) / len_unit;
            edge_mark.cy = GET_VALUE(p, "cy", float) / len_unit;
            edge_mark.dx = GET_VALUE(p, "dx", float) / len_unit;
            edge_mark.dy = GET_VALUE(p, "dy", float) / len_unit;
            edge_mark.s_node = GET_VALUE(p, "s_node", int);
            edge_mark.e_node = GET_VALUE(p, "e_node", int);
            edge_mark.s_facing = GET_VALUE(p, "s_facing", float) / rad_unit;
            edge_mark.e_facing = GET_VALUE(p, "e_facing", float) / rad_unit;
            edge_mark.radius = GET_VALUE(p, "radius", float) / len_unit;
            edge_mark.limit_v = GET_VALUE(p, "limit_v", float);
            edge_mark.limit_w = GET_VALUE(p, "limit_w", float);
            edge_mark.direction = (net::EdgeDirection)GET_VALUE(p, "direction", int);
            edge_mark.rotate_direction = (net::EdgeRotateDirection)GET_VALUE(p, "rotate_direction", int);
            edge_mark.execute_type = (net::EdgeExecuteType)GET_VALUE(p, "param", int);
            edge_mark.cost = GET_VALUE(p, "cost", int) / len_unit;
            edge_mark.vehicle_direction = getVehicleDirection(edge_mark);
            edge_group_.addItem(edge_mark.id, edge_mark);
        }

        items = json_data["feature"];
        feature_mg_.clearAllItem();
        FeatureMark feature_mark;
        for (int i = 0; i < items.size(); i++) {
            nloJson p = items[i];
            feature_mark.id = GET_VALUE(p, "id", int);
            feature_mark.station_id = GET_VALUE(p, "station_id", int);
            feature_mark.feature_type = GET_VALUE(p, "feature_type", int);
            feature_mark.feature_name = GET_VALUE(p, "feature_name", std::string);
            feature_mark.sensor_name = GET_VALUE(p, "sensor_name", std::string);
            feature_mark.pos_x = GET_VALUE(p, "pos.x", double) / feature_pose_unit;
            feature_mark.pos_y = GET_VALUE(p, "pos.y", double) / feature_pose_unit;
            feature_mark.pos_z = GET_VALUE(p, "pos.z", double) / feature_pose_unit;
            feature_mark.pos_yaw = GET_VALUE(p, "pos.yaw", double) / feature_pose_unit;
            feature_mark.pos_pitch = GET_VALUE(p, "pos.pitch", double) / feature_pose_unit;
            feature_mark.pos_roll = GET_VALUE(p, "pos.roll", double) / feature_pose_unit;

            feature_mg_.addItem(feature_mark.station_id, feature_mark);
        }

        // std::cout << (int)edge_group_.getItemCnt() << std::endl;
        // std::cout << (int)node_group_.getItemCnt()<< std::endl;
        // std::cout << (int)station_mg_.getItemCnt()<< std::endl;
        // std::cout << (int)area_mg_.getItemCnt()<< std::endl;
#ifdef USE_BOOST_JSON
        boost::property_tree::ptree root;
        boost::property_tree::ptree data;
        boost::property_tree::json_parser::read_json(json_file_path, root);
        data = root.get_child("data");
        // 不需要更新偏移值,大小,分辨率等数据
        boost::posix_time::ptime ct2(boost::posix_time::second_clock::universal_time());
        // modified_timestamp_ = ct2.time_since_epoch();

        boost::property_tree::ptree items;

        items = data.get_child("area");
        area_mg_.clearAllItem();
        AreaMark area_mark;
        for (boost::property_tree::ptree::iterator it = items.begin(); it != items.end(); ++it) {
            boost::property_tree::ptree p = it->second;
            area_mark.id = p.get<int>("id");
            area_mark.type = p.get<int>("type");
            std::string name = p.get<std::string>("name");
            strcpy(area_mark.name, name.c_str());
            area_mark.pa.x = p.get<int>("pa.x");
            area_mark.pa.y = p.get<int>("pa.y");
            area_mark.pb.x = p.get<int>("pb.x");
            area_mark.pb.y = p.get<int>("pb.y");
            area_mark.pc.x = p.get<int>("pc.x");
            area_mark.pc.y = p.get<int>("pc.y");
            area_mark.pd.x = p.get<int>("pd.x");
            area_mark.pd.y = p.get<int>("pd.y");
            area_mark.enter_station = p.get<int>("enter_station");
            area_mark.exit_station = p.get<int>("exit_station");
            area_mark.is_broadcast = p.get<bool>("is_broadcast");
            area_mark.is_play_music = p.get<bool>("is_play_music");
            area_mark.param_int = p.get<int>("param_int");
            std::cout << area_mark.id << std::endl;
            std::cout << area_mark.pa.x << std::endl;
            area_mg_.addItem(area_mark.id, area_mark);
        }

        items = data.get_child("station");
        StationMark station_mark;
        station_mg_.clearAllItem();
        for (boost::property_tree::ptree::iterator it = items.begin(); it != items.end(); ++it) {
            boost::property_tree::ptree p = it->second;
            station_mark.id = p.get<int>("id");
            station_mark.type = p.get<int>("type");
            std::string name = p.get<std::string>("name");
            strcpy(station_mark.name, name.c_str());
            station_mark.pos.x = p.get<int>("pos.x");
            station_mark.pos.y = p.get<int>("pos.y");
            station_mark.pos.yaw = p.get<int>("pos.yaw");
            station_mark.enter_pos.x = p.get<int>("enter_pos.x");
            station_mark.enter_pos.y = p.get<int>("enter_pos.y");
            station_mark.enter_pos.yaw = p.get<int>("enter_pos.yaw");
            station_mark.exit_pos.x = p.get<int>("exit_pos.x");
            station_mark.exit_pos.y = p.get<int>("exit_pos.y");
            station_mark.exit_pos.yaw = p.get<int>("exit_pos.yaw");
            station_mark.check_pos.x = p.get<int>("check_pos.x");
            station_mark.check_pos.y = p.get<int>("check_pos.y");
            station_mark.check_pos.yaw = p.get<int>("check_pos.yaw");
            station_mark.edge_id = p.get<int>("edge_id");
            station_mark.enter_backward = p.get<bool>("enter_backward");
            station_mark.exit_backward = p.get<bool>("exit_backward");
            station_mark.no_rotate = p.get<bool>("no_rotate");
            station_mark.param = p.get<int>("param");
            station_mark.pos_dynamic = p.get<bool>("pos_dynamic");
            station_mark.station_offset = p.get<int>("station_offset");

            station_mg_.addItem(station_mark.id, station_mark);
        }

        items = data.get_child("node");
        node_group_.clearAllItem();
        net::Nodef node_mark;
        for (boost::property_tree::ptree::iterator it = items.begin(); it != items.end(); ++it) {
            boost::property_tree::ptree p = it->second;
            node_mark.id = p.get<int>("id");
            node_mark.x = p.get<int>("x");
            node_mark.y = p.get<int>("y");
            node_mark.yaw = p.get<float>("yaw");

            node_group_.addItem(node_mark.id, node_mark);
        }

        items = data.get_child("edge");
        edge_group_.clearAllItem();
        net::Edgef edge_mark;
        for (boost::property_tree::ptree::iterator it = items.begin(); it != items.end(); ++it) {
            boost::property_tree::ptree p = it->second;
            edge_mark.id = p.get<int>("id");
            edge_mark.type = (net::EdgeType)p.get<int>("type");
            edge_mark.sx = p.get<int>("sx");
            edge_mark.sy = p.get<int>("sy");
            edge_mark.ex = p.get<int>("ex");
            edge_mark.ey = p.get<int>("ey");
            edge_mark.cx = p.get<int>("cx");
            edge_mark.cy = p.get<int>("cy");
            edge_mark.dx = p.get<int>("dx");
            edge_mark.dy = p.get<int>("dy");
            edge_mark.s_node = p.get<int>("s_node");
            edge_mark.e_node = p.get<int>("e_node");
            edge_mark.s_facing = p.get<int>("s_facing");
            edge_mark.e_facing = p.get<int>("e_facing");
            edge_mark.radius = p.get<int>("radius");
            edge_mark.limit_v = p.get<float>("limit_v");
            edge_mark.limit_w = p.get<float>("limit_w");
            edge_mark.direction = (net::EdgeDirection)p.get<int>("direction");
            edge_mark.rotate_direction = (net::EdgeRotateDirection)p.get<int>("rotate_direction");
            edge_mark.param = (net::EdgeExecuteType)p.get<int>("param");
            edge_mark.cost = p.get<int>("cost");

            edge_group_.addItem(edge_mark.id, edge_mark);
        }

        // 清除地图障碍点
        items = data.get_child("cleanArea");
        for (boost::property_tree::ptree::iterator it = items.begin(); it != items.end(); ++it) {
            boost::property_tree::ptree p = it->second;
            int pa_x = p.get<int>("pa.x");
            int pa_y = p.get<int>("pa.y");
            int pc_x = p.get<int>("pc.x");
            int pc_y = p.get<int>("pc.y");

            Point top_left = Point(pa_x / 10, pa_y / 10);
            Point bottom_right = Point(pc_x / 10, pc_y / 10);
            clearArea(top_left, bottom_right);
        }
#endif
        return true;
    }

    // 该函数仅允许绘制完地图后第一次加载显示地图(创建json文件)时调用(code.py)，
    // sros中禁止调用该函数，否则json中很多新增字段会丢失
    bool exportJsonFile(const char *json_file_path) {
        ofstream out(json_file_path, ifstream::out | ifstream::binary);
        if (out.fail()) {
            cerr << "Failure to open for output" << endl;
            return false;
        }

        int len_unit = 10.0;
        int rad_unit = 1000.0;

        out << "{ \r\n";

        out << "\"meta\" : { \r\n";

        out << "\"zero_offset.x\": " << zero_offset_.x << ", \r\n";
        out << "\"zero_offset.y\": " << zero_offset_.y << ", \r\n";
        out << "\"size.x\": " << size_.x << ", \r\n";
        out << "\"size.y\": " << size_.y << ", \r\n";
        out << "\"resolution:\": " << resolution_ << ", \r\n";
        out << "\"length_unit:\": "
            << "\"mm\""
            << ", \r\n";
        out << "\"angle_unit:\": "
            << "\"1/1000 rad\""
            << ", \r\n";
        out << "\"created_timestamp\": " << created_timestamp_ << ", \r\n";
        out << "\"modified_timestamp\": " << modified_timestamp_ << " \r\n";

        out << "},\r\n";

        out << "\"data\" : { \r\n";

        out << " \"area\" : [ \r\n";
        auto area_list = area_mg_.getItemList();
        for (int i = 0; i < area_list.size(); i++) {
            auto it = area_list[i];

            out << "{"
                << "\r\n";
            out << "\"id\": " << it.id << ", \r\n";
            out << "\"type\": " << it.type << ", \r\n";
            out << "\"name\": "
                << "\"" << it.name << "\""
                << ", \r\n";
            out << "\"pa.x\": " << it.pa.x * len_unit << ", \r\n";
            out << "\"pa.y\": " << it.pa.y * len_unit << ", \r\n";
            out << "\"pb.x\": " << it.pb.x * len_unit << ", \r\n";
            out << "\"pb.y\": " << it.pb.y * len_unit << ", \r\n";
            out << "\"pc.x\": " << it.pc.x * len_unit << ", \r\n";
            out << "\"pc.y\": " << it.pc.y * len_unit << ", \r\n";
            out << "\"pd.x\": " << it.pd.x * len_unit << ", \r\n";
            out << "\"pd.y\": " << it.pd.y * len_unit << "\r\n";
            out << "} ";
            if (i < area_list.size() - 1) {
                out << ",";
            }
            out << "\r\n";
        }
        out << " ], \r\n";

        out << " \"station\" : [ \r\n";
        auto station_list = station_mg_.getItemList();
        for (int i = 0; i < station_list.size(); i++) {
            auto it = station_list[i];

            out << "{"
                << "\r\n";
            out << "\"id\": " << it.id << ", \r\n";
            out << "\"type\": " << it.type << ", \r\n";
            out << "\"name\": "
                << "\"" << it.name << "\""
                << ", \r\n";
            out << "\"pos.x\": " << it.pos.x * len_unit << ", \r\n";
            out << "\"pos.y\": " << it.pos.y * len_unit << ", \r\n";
            out << "\"pos.yaw\": " << it.pos.yaw * rad_unit << ", \r\n";
            out << "\"enter_pos.x\": " << it.enter_pos.x * len_unit << ", \r\n";
            out << "\"enter_pos.y\": " << it.enter_pos.y * len_unit << ", \r\n";
            out << "\"enter_pos.yaw\": " << it.enter_pos.yaw * rad_unit << ", \r\n";
            out << "\"exit_pos.x\": " << it.exit_pos.x * len_unit << ", \r\n";
            out << "\"exit_pos.y\": " << it.exit_pos.y * len_unit << ", \r\n";
            out << "\"exit_pos.yaw\": " << it.exit_pos.yaw * rad_unit << ", \r\n";
            out << "\"check_pos.x\": " << it.check_pos.x * len_unit << ", \r\n";
            out << "\"check_pos.y\": " << it.check_pos.y * len_unit << ", \r\n";
            out << "\"check_pos.yaw\": " << it.check_pos.yaw * rad_unit << ", \r\n";
            out << "\"no_rotate\": " << std::boolalpha << it.no_rotate << ", \r\n";
            out << "\"enter_backward\": " << std::boolalpha << it.enter_backward << ", \r\n";
            out << "\"exit_backward\": " << std::boolalpha << it.exit_backward << ", \r\n";
            out << "\"pos_dynamic\": " << std::boolalpha << it.pos_dynamic << ", \r\n";
            out << "\"station_offset\": " << it.station_offset << ", \r\n";
            out << "\"edge_id\": " << it.edge_id << ", \r\n";
            out << "\"param\": " << static_cast<int>(it.param) << "\r\n";
            out << "} ";
            if (i < station_list.size() - 1) {
                out << ",";
            }
            out << "\r\n";
        }
        out << " ], \r\n";

        out << " \"node\" : [\r\n";
        auto node_list = node_group_.getItemList();
        for (int i = 0; i < node_list.size(); i++) {
            auto it = node_list[i];

            out << "{"
                << "\r\n";
            out << "\"id\": " << it.id << ", \r\n";
            out << "\"x\": " << it.x * len_unit << ", \r\n";
            out << "\"y\": " << it.y * len_unit << ", \r\n";
            out << "\"yaw\": " << it.yaw * rad_unit << " \r\n";
            out << "} ";
            if (i < node_list.size() - 1) {
                out << ",";
            }
            out << "\r\n";
        }
        out << " ], \r\n";

        out << " \"edge\" : [ \r\n";
        auto edge_list = edge_group_.getItemList();
        for (int i = 0; i < edge_list.size(); i++) {
            auto it = edge_list[i];

            out << "{"
                << "\r\n";
            out << "\"id\": " << it.id << ", \r\n";
            out << "\"s_node\": " << it.s_node << ", \r\n";
            out << "\"e_node\": " << it.e_node << ", \r\n";
            out << "\"cost\": " << it.cost * len_unit << ", \r\n";
            out << "\"type\": " << it.type << ", \r\n";
            out << "\"sx\": " << it.sx * len_unit << ", \r\n";
            out << "\"sy\": " << it.sy * len_unit << ", \r\n";
            out << "\"ex\": " << it.ex * len_unit << ", \r\n";
            out << "\"ey\": " << it.ey * len_unit << ", \r\n";
            out << "\"cx\": " << it.cx * len_unit << ", \r\n";
            out << "\"cy\": " << it.cy * len_unit << ", \r\n";
            out << "\"dx\": " << it.dx * len_unit << ", \r\n";
            out << "\"dy\": " << it.dy * len_unit << ", \r\n";
            out << "\"radius\": " << (std::isnan(it.radius) ? 0 : it.radius) * len_unit << ", \r\n";
            out << "\"limit_v\": " << it.limit_v << ", \r\n";
            out << "\"limit_w\": " << it.limit_w << ", \r\n";
            out << "\"s_facing\": " << it.s_facing * rad_unit << ", \r\n";
            out << "\"e_facing\": " << it.e_facing * rad_unit << ", \r\n";
            out << "\"rotate_direction\": " << it.rotate_direction << ", \r\n";
            out << "\"direction\": " << it.direction << ", \r\n";
            out << "\"param\": " << it.execute_type << "\r\n";
            out << "} ";
            if (i < edge_list.size() - 1) {
                out << ",";
            }
            out << "\r\n";
        }
        out << " ], \r\n";

        out << " \"feature\" : [ \r\n";
        auto feature_list = feature_mg_.getItemList();
        for (int i = 0; i < feature_list.size(); i++) {
            auto it = feature_list[i];

            out << "{"
                << "\r\n";
            out << "\"id\": " << it.id << ", \r\n";
            out << "\"station_id\": " << it.station_id << ", \r\n";
            out << "\"feature_type\": " << it.feature_type << ", \r\n";
            out << "\"feature_name\": " << "\"" << it.feature_name << "\"" << ", \r\n";
            out << "\"sensor_name\": " << "\"" << it.sensor_name << "\"" << ", \r\n";
            out << "\"pos.x\": " << it.pos_x * len_unit << ", \r\n";
            out << "\"pos.y\": " << it.pos_y * len_unit << ", \r\n";
            out << "\"pos.z\": " << it.pos_z * len_unit << ", \r\n";
            out << "\"pos.yaw\": " << it.pos_yaw * rad_unit << ", \r\n";
            out << "\"pos.pitch\": " << it.pos_pitch * rad_unit << ", \r\n";
            out << "\"pos.roll\": " << it.pos_roll * rad_unit << ", \r\n";
            out << "} ";
            if (i < feature_list.size() - 1) {
                out << ",";
            }
            out << "\r\n";
        }
        out << " ] \r\n";


        out << "}\r\n";
        out << "} \r\n";

        out.close();
        return true;
    }

    bool updateMapJsonMetaFields(const std::string &json_file_path) {
        if (json_file_path == "") {
            return false;
        }
        ifstream in(json_file_path, ifstream::in);
        nloJson json_map;
        in >> json_map;
        in.close();

        if (json_map.find("meta") == json_map.end()) {
            cerr << "Invalid map json file format: " << json_file_path << endl;
            return false;
        }

        if (!json_map["meta"].is_object()) {
            LOG(INFO) << "Map json meta is not object" << json_file_path << endl;
            return false;
        }

        json_map["meta"]["resolution"] = resolution_;
        json_map["meta"]["zero_offset.x"] = zero_offset_.x;
        json_map["meta"]["zero_offset.y"] = zero_offset_.y;
        json_map["meta"]["size.x"] = size_.x;
        json_map["meta"]["size.y"] = size_.y;
        json_map["meta"]["modified_timestamp"] = modified_timestamp_;

        ofstream out(json_file_path, ofstream::out);
        out << json_map.dump(4);
        out.close();
        return true;
    }

    // 向地图json文件meta中添加png数据信息
    void addPngFileToMapMetaField(const std::string &json_file_path, vector<uint8_t> &img_size) {
        if (json_file_path == "") {
            return;
        }
        ifstream in(json_file_path, ifstream::in);
        nloJson json_map;
        in >> json_map;
        in.close();

        if (json_map.find("meta") == json_map.end()) {
            cerr << "Invalid map json file format: " << json_file_path << endl;
            return;
        }

        if (!json_map["meta"].is_object()) {
            LOG(INFO) << "Map json meta is not object" << json_file_path << endl;
            return;
        }

        nloJson images(img_size);
        json_map["meta"]["image_scale"] = images;

        ofstream out(json_file_path, ofstream::out);
        out << json_map.dump(4);
        out.close();
        return;
    }

    bool exportMapPngFiles(const char *map_name) {
        vector<uint8_t> img_size = {1, 2, 4, 8};
        std::map<uint8_t, string>::iterator iter;

        for (int i = 0; i < img_size.size(); i++) {
            uint8_t scale = img_size[i];

            string png_name = string(map_name) + "_" + std::to_string(img_size[i]) + ".png";
            LOG(INFO) << "save png:" << png_name;
            if (scale == 1) {
                png_name = string(map_name) + ".png";
            }
            std::string png_file_path = SROS_MAP_DIR + png_name;
            doExportPngFile(png_file_path.c_str(), scale);
        }
        addPngFileToMapMetaField(SROS_MAP_DIR + "/" + map_name + ".json", img_size);
        return true;
    }

    // 将栅格数据导出为PNG图片
    bool doExportPngFile(const char *file_path, uint32_t scale = 1) {
        uint32_t width = size_.x / scale;
        uint32_t height = size_.y / scale;

        FILE *fp = fopen(file_path, "wb");
        if (!fp) return false;

        png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        if (!png) return false;

        png_infop info = png_create_info_struct(png);
        if (!info) return false;

        if (setjmp(png_jmpbuf(png))) return false;

        png_init_io(png, fp);

        // Output is 8bit depth, GRAY_ALPHA format.
        png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_GRAY_ALPHA, PNG_INTERLACE_NONE,
                     PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
        png_write_info(png, info);

        // To remove the alpha channel for PNG_COLOR_TYPE_RGB format,
        // Use png_set_filler().
        // png_set_filler(png, 0, PNG_FILLER_AFTER);

        png_byte WHITE = 255, BLACK = 0;
        png_byte TRANSPARENT = 0, INTRANSPARENT = 255;

        NavMapData_ptr zip_image_dat = zipPngImageDat(scale);

        //        LOG(INFO) << display_map_data_ << " @ " << zip_image_dat;

        png_bytep one_row_pointers = reinterpret_cast<png_byte *>(malloc(png_get_rowbytes(png, info)));
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width * 2; x += 2) {
                one_row_pointers[x] = zip_image_dat[y * width + x / 2] == 0 ? WHITE : BLACK;
                one_row_pointers[x + 1] = zip_image_dat[y * width + x / 2] == 0 ? TRANSPARENT : INTRANSPARENT;
            }
            png_write_row(png, one_row_pointers);
        }

        png_write_end(png, NULL);

        free(one_row_pointers);

        fclose(fp);

        return true;
    }

    NavMapData_ptr zipPngImageDat(uint32_t scale) {
        uint32_t width = size_.x / scale;
        uint32_t height = size_.y / scale;

        LOG(INFO) << "scale " << scale << " width " << width << " height " << height;

        NavMapData_ptr png_dat;
        png_dat.reset(new NavMapNode_t[width * height]);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int greyVal = calculatePngGrayValue(x, y, scale);
                png_dat[y * width + x] = greyVal;
            }
        }
        return png_dat;
    }

    // 在压缩比为scale(1, 4, 16, 36)下计算(x, y)位置的灰度值
    uint8_t calculatePngGrayValue(uint32_t x, uint32_t y, uint32_t scale) {
        uint32_t width = size_.x;
        uint32_t height = size_.y;

        // 在区块内统计黑白点数目
        uint32_t black_cnt = 0;
        for (int row = 0; row < scale; row++) {
            for (int col = 0; col < scale; col++) {
                uint64_t idx = (row + y * scale) * width + (col + x * scale);
                if (idx >= width * height) {
                    continue;
                }
                int val = display_map_data_->isOccInGridIndex(idx);
                if (val > 0) {
                    black_cnt++;
                }
            }
        }

        // 黑点大于白点(超过半数)时, 放大后该区块为黑点;反之则为白点
        if (black_cnt > scale / 2) {
            return 1;
        }
        return 0;
    }

    uint32_t getPngScale() {
        uint32_t width = size_.x * resolution_ / 100;   // 单位m
        uint32_t height = size_.y * resolution_ / 100;  // 单位m
        uint64_t area = width * height;

        // 按100mx100m为基础压缩图片
        float scale = area / (100 * 100);
        if (scale <= 1) {
            return 1;
        }

        // 向上取整
        uint16_t val = ceil(sqrt(scale));

        // 除了1外需被2整除，合法取值: 2, 4, 6, 8...
        if (val % 2 != 0) {
            val += 1;
        }

        return val;
    }

    // 仅用于转换灰度地图到NavigationMap,如需要导航,则需重新加载NavigationMap
    bool loadGrayMap(const char *file_path) {
        const char *tmp_nav_path = "/tmp/convert-nav.map";
        convertGrayToNavigation(file_path, tmp_nav_path);
        loadFile(tmp_nav_path);

        return true;
    }

    // 覆盖导航地图
    void coverNavMap(const NavigationMap &other) {
        map_file_version_ = other.map_file_version_;

        area_mg_ = other.area_mg_;
        station_mg_ = other.station_mg_;

        edge_group_ = other.edge_group_;
        node_group_ = other.node_group_;
    }

    void addNoEnterAreaToStaticMap() {
        LOG(INFO) << "addNoEnterAreaToStaticMap()";

        int min_x, max_x, min_y, max_y;

        if (!enable_load_map_data_) {
            // 没有加载灰度信息, 无法进行此操作
            LOG(INFO) << "Not enable load map data! ";
            return;
        }

        for (auto area : area_mg_.getItemList()) {
            if (area.isNoEnterAreaType()) {
                LOG(INFO) << "isNoEnterAreaType area id: " << area.id;
                std::vector<int> xs, ys;
                xs.push_back(static_cast<int>(area.pa.x));
                xs.push_back(static_cast<int>(area.pb.x));
                xs.push_back(static_cast<int>(area.pc.x));
                xs.push_back(static_cast<int>(area.pd.x));

                ys.push_back(static_cast<int>(area.pa.y));
                ys.push_back(static_cast<int>(area.pb.y));
                ys.push_back(static_cast<int>(area.pc.y));
                ys.push_back(static_cast<int>(area.pd.y));

                // 此处得到的是在数组中的坐标
                // min_x = *std::min_element(xs.begin(), xs.end()) / resolution_;
                // max_x = *std::max_element(xs.begin(), xs.end()) / resolution_;
                // min_y = *std::max_element(ys.begin(), ys.end()) / resolution_;
                // max_y = *std::min_element(ys.begin(), ys.end()) / resolution_;
                min_x = *std::min_element(xs.begin(), xs.end()) / resolution_ + zero_offset_.x;
                max_x = *std::max_element(xs.begin(), xs.end()) / resolution_ + zero_offset_.x;
                min_y = zero_offset_.y - *std::max_element(ys.begin(), ys.end()) / resolution_;
                max_y = zero_offset_.y - *std::min_element(ys.begin(), ys.end()) / resolution_;


                if (!checkAreaPos(min_x, max_x, min_y, max_y)) {
                    resetAreaPos(min_x, max_x, min_y, max_y);
                }

                int width = size_.x;

                for (int y = min_y; y <= max_y; y++) {
                    for (int x = min_x; x <= max_x; x++) {
                        // 在地图上添加障碍点
                        static_map_data_->setGridValue(x,y, 1);
                        //                        static_map_data_[y * width + x] = 1;
                    }
                }
            }
        }

        //        convertNavMapDataToPGM(static_map_data_, size_.x, size_.y, "/tmp/static.pgm");
    }

    // 返回值为true,表示有清除障碍点; false表示没有清除障碍点
    bool clearArea(const char *json_file_path) {
        ifstream in(json_file_path);
        nloJson json_map;
        in >> json_map;

        in.close();

        // json中长度单位为mm,.map中单位为cm
        // json中弧度单位放大1000倍
        int len_unit = 10.0;
        int rad_unit = 1000.0;

        nloJson json_data = json_map["data"];
        if (!json_data.is_object()) {
            LOG(INFO) << "Json data is not object";
            return false;
        }

        // 不需要更新偏移值,大小,分辨率等数据
        boost::posix_time::ptime ct2(boost::posix_time::second_clock::universal_time());
        // modified_timestamp_ = ct2.time_since_epoch();

        nloJson items;
        if (json_map.find("private") != json_map.end()) {
            nloJson json_private = json_map["private"];
            items = json_private["cleanArea"];
        } else if (json_data.find("cleanArea") != json_data.end()) {
            items = json_data["cleanArea"];
        }
        for (int i = 0; i < items.size(); i++) {
            nloJson p = items[i];
            int pa_x = p["pa.x"].get<float>() / len_unit;
            int pa_y = p["pa.y"].get<float>() / len_unit;
            int pc_x = p["pc.x"].get<float>() / len_unit;
            int pc_y = p["pc.y"].get<float>() / len_unit;

            Point top_left = Point(pa_x, pa_y);
            Point bottom_right = Point(pc_x, pc_y);
            doClearArea(top_left, bottom_right);
        }

        // 如果清除障碍点,则需要重新生成灰度图
        if (items.size() > 0) {
            return true;
        }
        return false;
    }

    // 清除指定矩形区域中的障碍点
    void doClearArea(Point top_left, Point bottom_right) {
        int min_x, max_x, min_y, max_y;

        min_x = top_left.x / resolution_ + zero_offset_.x;
        max_x = bottom_right.x / resolution_ + zero_offset_.x;
        min_y = zero_offset_.y - top_left.y / resolution_;
        max_y = zero_offset_.y - bottom_right.y / resolution_;

        if (!checkAreaPos(min_x, max_x, min_y, max_y)) {
            resetAreaPos(min_x, max_x, min_y, max_y);
        }

        int width = size_.x;

        for (int y = min_y; y <= max_y; y++) {
            for (int x = min_x; x <= max_x; x++) {
                // 清除地图上的障碍点
                display_map_data_->setGridValue(x, y, 0);
            }
        }
    }

    // 检测区域坐标是否在地图范围中
    bool checkAreaPos(int min_x, int max_x, int min_y, int max_y) {
        return (min_x >= 0 && min_y >= 0 && max_x < size_.x && max_y < size_.y);
    }

    // 修改超出地图范围的区域坐标到地图范围之内
    void resetAreaPos(int &min_x, int &max_x, int &min_y, int &max_y) {
        if (min_x < 0) {
            min_x = 0;
        }
        if (min_y < 0) {
            min_y = 0;
        }
        if (max_x >= size_.x) {
            max_x = size_.x - 1;
        }
        if (max_y >= size_.y) {
            max_y = size_.y - 1;
        }
    }

    void toRightCoord(const int index,const int &length,int &coord_x,int &coord_y) const {
        auto width_index = index / length;
        auto length_index = index % length;
        coord_x = length_index - zero_offset_.x;
        coord_y = zero_offset_.y - width_index;
    }

    template <typename T>
    static inline void read_field(std::ifstream &in, T &field) {
        in.read(reinterpret_cast<char *>(&field), sizeof(T));
    }

    template <typename T>
    static inline void read_field(std::ifstream &in, T *field, size_t size) {
        in.read(reinterpret_cast<char *>(field), size);
    }

    template <typename T>
    static inline void write_field(std::ofstream &out, T &field) {
        out.write(reinterpret_cast<const char *>(&field), sizeof(T));
    }

    template <typename T>
    static inline void write_field(std::ofstream &out, T *field, size_t size) {
        out.write(reinterpret_cast<const char *>(field), size);
    }

    static const int MAGIC_STR_LEN = 4;

 private:
    std::string map_path_;  //存储地图路径

    MapFileVersion map_file_version_;  // 当前打开的文件版本, 默认为最新版本

    std::shared_ptr<PyramidNavMap<NavMapNode_t>> static_map_data_;   //静态地图信息,用于导航使用std::shared_ptr<PyramidNavMap<NavMapNode_t>>
    PyramidNavMap_ptr display_map_data_;  // 用于显示与修改的地图,保存地图时使用此数组中的数据

    Point zero_offset_;  // 地图的零点偏移量
    Point size_;         // 地图的大小，没有考虑地图分辨率

    int resolution_;  // 地图分辨率

    int unused_parmeter0_;  // 保留的参数位置
    int unused_parmeter1_;

    int modified_timestamp_;  // 最近一次修改的时间戳
    int created_timestamp_;   // 地图创建时间戳

    AreaMarkGroup area_mg_;        // 存储地图中所有区域的group
    StationMarkGroup station_mg_;  // 存储地图中所有站点的group
    DMCodeMarkGroup dmcode_mg_;    // 存储地图中所有工业二维码的group
    FeatureMarkGroup feature_mg_;  // 存储地图中所有站点特征信息的group

    net::EdgeGroup edge_group_;
    net::NodeGroup node_group_;

    bool enable_load_map_data_;  // 是否加载地图灰度数据, 不加载可节省内存空间
};

typedef std::shared_ptr<NavigationMap> NavigationMap_ptr;

static bool convertGrayToNavigation(string gray_map_path, string nav_map_path) {
    const int MAX_BUFFER_SIZE = 32;
    char buffer[MAX_BUFFER_SIZE] = {'\0'};

    ifstream in(gray_map_path, ifstream::binary);
    if (in.fail()) {
        cerr << "Failure to open for input: " << gray_map_path << endl;
        return false;
    }

    ofstream out(nav_map_path, ofstream::binary);
    if (out.fail()) {
        cerr << "Failure to open for output: " << nav_map_path << endl;
        return false;
    }

    // 开始读取GrayMap文件头
    int zero_offset_x, zero_offset_y;
    int size_x, size_y;
    int resolution;

    // skip "P5\n"
    in.getline(buffer, MAX_BUFFER_SIZE);

    in.getline(buffer, MAX_BUFFER_SIZE);
    zero_offset_x = atoi(buffer + 1);  // +1 是为了跳过行首的“#”

    in.getline(buffer, MAX_BUFFER_SIZE);
    zero_offset_y = atoi(buffer + 1);

    in.getline(buffer, MAX_BUFFER_SIZE);
    if (buffer[0] == '#') {  // 如果开头为'#', 说明该行为resolution， 否则为size
        resolution = atoi(buffer + 1);

        in.getline(buffer, MAX_BUFFER_SIZE);
    } else {
        resolution = 1;
    }

    size_x = atoi(buffer);
    size_y = atoi(strchr(buffer, ' ') + 1);

    int zero = 0;

    in.getline(buffer, MAX_BUFFER_SIZE);  // skip '\n'

    // 写入nav_map文件头
    NavigationMap::write_field(out, NAV_MAGIC_STR, NavigationMap::MAGIC_STR_LEN);
    NavigationMap::write_field(out, zero_offset_x);
    NavigationMap::write_field(out, zero_offset_y);
    NavigationMap::write_field(out, size_x);
    NavigationMap::write_field(out, size_y);
    NavigationMap::write_field(out, resolution);

    NavigationMap::write_field(out, zero);  // created_timestamp = 0
    NavigationMap::write_field(out, zero);  // modified_timestamp = 0
    NavigationMap::write_field(out, zero);  // unused_parameter0 = 0
    NavigationMap::write_field(out, zero);  // unused_parameter1 = 0

    NavigationMap::write_field(out, zero);  // area_mark_cnt = 0
    NavigationMap::write_field(out, zero);  // station_mark_cnt = 0

    assert(size_x * size_y % 8 == 0);  // 地图长宽乘积应该为8的整数倍

    uint8_t in_value, out_value;
    for (int i = 0; i < size_x * size_y / 8; ++i) {
        for (int j = 0; j < 8; j++) {
            NavigationMap::read_field(in, in_value);

            uint8_t x = (in_value > T_VALUE) ? 1 : 0;
            // 判断灰度值是否超过障碍的判定阈值
            out_value = (uint8_t)((out_value << 1) + x);
        }
        out.write(reinterpret_cast<char *>(&out_value), sizeof(UChar_t));
    }

    NavigationMap::write_field(out, zero);  // node_cnt = 0
    NavigationMap::write_field(out, zero);  // edge_cnt = 0

    in.close();
    out.close();

    return true;
}

// 将NavMapData_ptr转为PGM文件，调试使用
static bool convertNavMapDataToPGM(NavMapData_ptr array, int width, int height, string pgm_path) {
    ofstream out(pgm_path);

    out.write("P5\n", 3);
    char size_str[32];

    int n = sprintf(size_str, "%d %d\n", width, height);
    out.write(size_str, n);

    out.write("255\n", 4);

    char FILL = 0xFF;
    char EMPTY = 0x00;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            NavMapNode_t n = array[x + y * width];

            if (n == 0) {
                out.write(&FILL, 1);
            } else {
                out.write(&EMPTY, 1);
            }
        }
    }

    out.flush();
    out.close();
    return true;
}

}  // namespace map
}  // namespace sros

#endif  // SROS_MAP_NAVIGATIONMAP_H
