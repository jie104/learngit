
/**
 * @file GrayMap.hpp
 *
 * @author lhx
 * @date 2015年9月22日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef BASEMAP_H_
#define BASEMAP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <memory>

// 定义Map value类型
// typedef short value_type;

namespace sros {
namespace map {

typedef unsigned char NodeIndex;
typedef unsigned char NodeValue;

typedef struct Node {
    NodeIndex index;
    NodeValue value;
} Node;

using namespace std;

/**
 * StaticGrayMap 和 DynamicGrayMap 的基类
 */
class GrayMap {
 public:
    GrayMap()
        : size_x_(0),
          size_y_(0),
          zero_offset_x_(0),
          zero_offset_y_(0),
          resolution_(1),
          MAGIC_STR("P5\n"),
          MAGIC_STR_LEN(3) {
        DEFAULT_VALUE.index = 0;
        DEFAULT_VALUE.value = 0;
    }

    virtual ~GrayMap() {
        //    cout << "GrayMap::GrayMap()" << endl;
    }

    virtual Node get(int x, int y) = 0;

    virtual Node *getp(int x, int y) = 0;

    virtual bool set(int x, int y, Node value) = 0;

    virtual bool save(const char *path) = 0;

    virtual bool load(const char *path) = 0;

    virtual int getSizeX() = 0;

    virtual int getSizeY() = 0;

    virtual void getLimitX(int &min, int &max) = 0;  // x方向上的极限值
    virtual void getLimitY(int &min, int &max) = 0;  // y方向上的极限值

    virtual void getZeroOffset(int *x, int *y) = 0;

    void setResolution(int resolution) { resolution_ = resolution; }

    int getResolution() const { return resolution_; }

    //    virtual bool getLeftTopPoint(int *x, int *y) = 0;
    //    virtual bool getRightBottomPoint(int *x, int *y) = 0;

    //    virtual int getPGMBufferSize() = 0;
    //    virtual int copyToPGMBuffer(NodeValue *array) = 0;

    //    void setDefaultValue(value_type value);

 protected:
    /**
     * 写入地图文件的头部
     */
    void writeHeader(ofstream &out) {
        char buffer[32] = {'\0'};
        int str_len = 0;

        out.write(MAGIC_STR, MAGIC_STR_LEN);

        out.write("#", 1);
        str_len = sprintf(buffer, "%d", zero_offset_x_);
        out.write(buffer, str_len);
        out.write("\n", 1);

        out.write("#", 1);
        str_len = sprintf(buffer, "%d", zero_offset_y_);
        out.write(buffer, str_len);
        out.write("\n", 1);

        out.write("#", 1);
        str_len = sprintf(buffer, "%d", resolution_);
        out.write(buffer, str_len);
        out.write("\n", 1);

        str_len = sprintf(buffer, "%d", size_x_);
        out.write(buffer, str_len);
        out.write(" ", 1);
        str_len = sprintf(buffer, "%d", size_y_);
        out.write(buffer, str_len);
        out.write("\n", 1);

        str_len = sprintf(buffer, "%d", 255);
        out.write(buffer, str_len);
        out.write("\n", 1);
    }

    /**
     * 读取地图文件的头部
     */
    void readHeader(ifstream &in) {
        char buffer[32] = {'\0'};
        in.read(buffer, MAGIC_STR_LEN);
        // TODO(nobody): check magic_str

        in.getline(buffer, 32);
        zero_offset_x_ = atoi(buffer + 1);
        in.getline(buffer, 32);
        zero_offset_y_ = atoi(buffer + 1);

        in.getline(buffer, 32);
        if (buffer[0] == '#') {  // 如果开头为'#', 说明该行为resolution， 否则为size
            resolution_ = atoi(buffer + 1);

            in.getline(buffer, 32);
        } else {
            resolution_ = 1;
        }

        size_x_ = atoi(buffer);
        size_y_ = atoi(strchr(buffer, ' ') + 1);

        in.getline(buffer, 32);  // skip line
    }

    // 每个元素的默认值
    Node DEFAULT_VALUE;

    int size_x_, size_y_;

    // 原点相对于地图左上角的偏移量
    int zero_offset_x_, zero_offset_y_;

    int resolution_;

    const char *MAGIC_STR;
    const int MAGIC_STR_LEN;
};

typedef std::shared_ptr<GrayMap> GrayMap_ptr;

}  // namespace map
} /* namespace sros */

// 为了兼容旧代码
typedef sros::map::Node node_type;

#endif /* BASEMAP_H_ */
