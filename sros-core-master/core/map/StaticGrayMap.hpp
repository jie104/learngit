
/**
 * @file StaticGrayMap.hpp
 *
 * @author lhx
 * @date 2015年9月22日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef STATICMAP_H_
#define STATICMAP_H_

#include <fstream>
#include <iostream>

#include "GrayMap.hpp"

using namespace std;

namespace sros {
namespace map {

/**
 * @brief 静态地图实现
 *
 * 静态地图需要load()或initialize()之后才能使用
 */
class StaticGrayMap : public GrayMap {
 public:
    StaticGrayMap() {
        size_x_ = 0;
        size_y_ = 0;
        data_ = 0;
    }

    virtual ~StaticGrayMap() {
//        cout << "StaticGrayMap::StaticGrayMapyMap()" << endl;
        if (data_) delete[] data_;
    }

    virtual Node get(int x, int y) {
        Node *p = getp(x, y);
        if (p) {
            return *p;
        } else {
            return DEFAULT_VALUE;
        }
    }

    virtual Node *getp(int x, int y) {
        getRealXY(&x, &y);

        if (data_ && x >= 0 && x < size_x_ && y >= 0 && y < size_y_) {
            return &data_[x + y * size_x_];
        } else {
            return 0;
        }
    }

    virtual bool set(int x, int y, Node value) {
        getRealXY(&x, &y);

        if (data_ != 0 && x >= 0 && x < size_x_ && y >= 0 && y < size_y_) {
            data_[x + y * size_x_] = value;
            return true;
        } else {
            return false;
        }
    }

    virtual bool save(const char *path) {
        ofstream out(path, ofstream::out | ofstream::binary);

        writeHeader(out);

        int yy;
        for (int y = 0; y < size_y_; y++) {
            yy = y * size_x_;
            for (int x = 0; x < size_x_; x++) {
                out.write(reinterpret_cast<char *>(&data_[x + yy].value), sizeof(NodeValue));
            }
        }

        out.close();

        return true;
    }

    virtual bool load(const char *path) {
        ifstream in(path, ifstream::in | ifstream::binary);
        if (!in) {
            return false;
        }

        readHeader(in);

        //        cout << "========> loadFile(): (" << zero_offset_x_ << ", " << zero_offset_y_ << ") -> " << size_x_ <<
        //        "x"
        //             << size_y_ << endl;

        if (data_) {  // release previous data_ memory
            delete[] data_;
        }

        data_ = new Node[size_x_ * size_y_];
        memset(data_, 0, sizeof(Node) * size_x_ * size_y_);  // 将新申请的空间全部置0

        int yy;
        NodeValue value;
        for (int y = 0; y < size_y_; y++) {
            yy = y * size_x_;
            for (int x = 0; x < size_x_; x++) {
                in.read(reinterpret_cast<char *>(&value), sizeof(NodeValue));
                data_[x + yy].value = value;
                //                data_[x + yy].value = 0xFF - value;
                //                if (value > 0x4F) {
                //                    data_[x + yy].value = 0x00;
                //                } else {
                //                    data_[x + yy].value = 0xFF;
                //                }
            }
        }

        in.close();

        return true;
    }

    virtual int getSizeX() { return size_x_; }

    virtual int getSizeY() { return size_y_; }

    virtual void getLimitX(int &min, int &max) {
        min = -zero_offset_x_;
        max = -zero_offset_x_ + (size_x_ - 2);
    }  // x方向上的极限值
    virtual void getLimitY(int &min, int &max) {
        min = -(size_y_ - 2) + zero_offset_y_;
        max = zero_offset_y_;
    }  // y方向上的极限值

    virtual void getZeroOffset(int *x, int *y) {
        *x = zero_offset_x_;
        *y = zero_offset_y_;
    }

    bool initialize(int _size_x, int _size_y, int _zero_offset_x, int _zero_offset_y) {
        size_x_ = _size_x;
        size_y_ = _size_y;
        zero_offset_x_ = _zero_offset_x;
        zero_offset_y_ = _zero_offset_y;

        data_ = new Node[size_x_ * size_y_];
        // 设置默认值
        for (int y = 0; y < size_y_; y++) {
            for (int x = 0; x < size_x_; x++) {
                data_[x + y * size_x_] = DEFAULT_VALUE;
            }
        }

        return data_;
    }

 private:
    Node *data_;

    inline void getRealXY(int *x, int *y) {
        *x = zero_offset_x_ + *x;
        *y = zero_offset_y_ - *y;
    }
};

}  // namespace map
} /* namespace sros */
#endif /* STATICMAP_H_ */
