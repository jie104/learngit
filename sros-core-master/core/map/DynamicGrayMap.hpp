
/**
 * @file DynamicGrapMap.hpp
 *
 * @author lhx
 * @date 2015年9月22日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef DYNAMICMAP_H_
#define DYNAMICMAP_H_

#include "GrayMap.hpp"

#include <cmath>
#include <cstring>

// x方向区块个数
#define MAX_X_BLOCK_NUM 128
// y方向区块个数
#define MAX_Y_BLOCK_NUM 128

namespace sros {
namespace map {

/**
 *
 *                          /\
 *                        Y ||
 *                          ||    MAX_BLOCK_X_SIZE
 *      BLOCK_X_SIZE        ||       /
 *            \             ||      /
 *             \            ||<------------>|
 *   <Q2>    |<-->|    |    ||    |    |    |     <Q1>
 *         --|----|----|----||----|----|----|--
 *           |    |    |    ||    |    |    |
 *         --|----|----|----||----|----|----|--
 *   Block---|->  |    |    ||    |    |    |
 *         --|----|----|----||----|----|----|---
 *           |    |    |    ||    |    |    |
 * =========================||=========================>
 *           |    |    |    ||    |    |    |         X
 *         --|----|----|----||----|----|----|--
 *           |    |    |    ||    |    |    |
 *         --|----|----|----||----|----|----|--
 *   <Q3>    |    |    |    ||    |    |    |     <Q4>
 *                          ||
 *                          ||
 *
 *
 */
class DynamicGrayMap : public GrayMap {
 public:
    DynamicGrayMap() : is_dirty(true) {
        //    value_type* blocks[][][4] = {block_q1, block_q2, block_q3, block_q4};
        for (int i = 0; i < MAX_X_BLOCK_NUM; i++) {
            for (int j = 0; j < MAX_Y_BLOCK_NUM; j++) {
                block_q1[i][j] = nullptr;
                block_q2[i][j] = nullptr;
                block_q3[i][j] = nullptr;
                block_q4[i][j] = nullptr;
            }
        }
    }

    virtual ~DynamicGrayMap() {
        for (int i = 0; i < MAX_X_BLOCK_NUM; i++) {
            for (int j = 0; j < MAX_Y_BLOCK_NUM; j++) {
                if (block_q1[i][j]) delete[] block_q1[i][j];

                if (block_q2[i][j]) delete[] block_q2[i][j];

                if (block_q3[i][j]) delete[] block_q3[i][j];

                if (block_q4[i][j]) delete[] block_q4[i][j];
            }
        }
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
        is_dirty = true;

        int abs_x = std::abs(x);
        int abs_y = std::abs(y);

        int block_x_index = abs_x >> BLOCK_X_SIZE_N;
        int block_y_index = abs_y >> BLOCK_Y_SIZE_N;

        // 超出处理范围
        if (block_x_index >= MAX_X_BLOCK_NUM || block_y_index >= MAX_Y_BLOCK_NUM) {
            return 0;
        }

        BlockQuarter &block_quarter = getBlockQuarter(x, y);

        Node *block = block_quarter[block_x_index][block_y_index];
        if (!block) {
            block = allocateNewBlock();
            block_quarter[block_x_index][block_y_index] = block;
        }

        int index = ((abs_x & BLOCK_X_SIZE_MASK) << BLOCK_Y_SIZE_N) + (abs_y & BLOCK_Y_SIZE_MASK);

        return &block[index];
    }

    virtual bool set(int x, int y, Node value) {
        is_dirty = true;

        int abs_x = std::abs(x);
        int abs_y = std::abs(y);

        int block_x_index = abs_x >> BLOCK_X_SIZE_N;
        int block_y_index = abs_y >> BLOCK_Y_SIZE_N;

        // 超出处理范围
        if (block_x_index >= MAX_X_BLOCK_NUM || block_y_index >= MAX_Y_BLOCK_NUM) {
            return false;
        }

        BlockQuarter &block_quarter = getBlockQuarter(x, y);

        Node *block = block_quarter[block_x_index][block_y_index];
        if (!block) {  // 申请新Block
            block = allocateNewBlock();
            block_quarter[block_x_index][block_y_index] = block;
        }

        int index = ((abs_x & BLOCK_X_SIZE_MASK) << BLOCK_Y_SIZE_N) + (abs_y & BLOCK_Y_SIZE_MASK);
        block[index] = value;

        return true;
    }

    virtual bool save(const char *path) {
        updateSize();

        Node node;
        ofstream out(path, ofstream::out | ofstream::binary);

        writeHeader(out);

        //    cout << "----> x:" << -zero_offset_x_ << ", " << size_x_ - zero_offset_x_ << endl;
        //    cout << "----> y:" << zero_offset_y_ << ", " << zero_offset_y_ - size_y_ << endl;

        //    int min_y, max_y, min_x, max_x;
        //    getLimitX(min_x, max_x);
        //    getLimitY(min_y, max_y);
        //    for (int y = min_y; y <= max_y; y++) {
        //        for (int x = min_x; x <= max_x; x++) {
        // FIXME 获取的x、y范围有问题，会访问到没有set过的node
        int endy = zero_offset_y_ - size_y_;
        int endx = size_x_ - zero_offset_x_;

        for (int y = zero_offset_y_; y > endy; y--) {
            for (int x = -zero_offset_x_; x < endx; x++) {
                if (x == endx - 1 || y == endy + 1) {
                    node.value = 0;
                    out.write(reinterpret_cast<char *>(&node.value), sizeof(NodeValue));
                } else {
                    node = get(x, y);
                    out.write(reinterpret_cast<char *>(&node.value), sizeof(NodeValue));
                }
            }
        }

        out.close();
        return true;
    }

    virtual bool load(const char *path) {
        Node node;
        ifstream in(path, ifstream::in | ifstream::binary);
        if (!in) {
            return false;
        }
        readHeader(in);

        //    cout << "----> x:" << -zero_offset_x_ << ", " << size_x_ - zero_offset_x_ << endl;
        //    cout << "----> y:" << zero_offset_y_ << ", " << zero_offset_y_ - size_y_ << endl;

        //    int min_y, max_y, min_x, max_x;
        //    getLimitX(min_x, max_x);
        //    getLimitY(min_y, max_y);
        //    for (int y = min_y; y <= max_y; y++) {
        //        for (int x = min_x; x <= max_x; x++) {
        // FIXME 获取的x、y范围有问题，会访问到没有set过的node
        int endy = zero_offset_y_ - size_y_;
        int endx = size_x_ - zero_offset_x_;
        int zero_offset_y = zero_offset_y_;
        int zero_offset_x = zero_offset_x_;
        for (int y = zero_offset_y; y > endy; y--) {
            for (int x = -zero_offset_x; x < endx; x++) {
                if (x == endx - 1 || y == endy + 1) {
                    node.value = 0;
                    in.read(reinterpret_cast<char *>(&node.value), sizeof(NodeValue));
                } else {
                    in.read(reinterpret_cast<char *>(&node.value), sizeof(NodeValue));
                    set(x, y, node);
                }
            }
        }

        in.close();
        return true;
    }

    /**
     *
     *
     *                  /\           top_y
     *                Y ||           /
     *      |           ||       |  /
     *    --------------||-------------
     *      |           ||   *   |
     *      |     *     ||       |<-- right_x
     *      |           ||       |
     *  ================||===============>
     *      |<- left_x  ||       |
     *      |           ||       |  bottom_y
     *      | *         ||       |   /
     *      |           ||      *|  /
     *    --------------||-------------
     *      |           ||       |
     *
     */
    void updateSize() {
        if (!is_dirty)  // 如果map没有更新过，直接返回
            return;

        is_dirty = false;

        int x_q[5] = {0};
        int y_q[5] = {0};

        getCornerPoint(block_q1, &x_q[1], &y_q[1]);
        getCornerPoint(block_q2, &x_q[2], &y_q[2]);
        getCornerPoint(block_q3, &x_q[3], &y_q[3]);
        getCornerPoint(block_q4, &x_q[4], &y_q[4]);

        int left_x, right_x, top_y, bottom_y;

        left_x = max(x_q[2], x_q[3]) + 1;
        right_x = max(x_q[1], x_q[4]) + 1;

        top_y = max(y_q[1], y_q[2]) + 1;
        bottom_y = max(y_q[3], y_q[4]) + 1;

        size_x_ = (left_x + right_x) * BLOCK_X_SIZE;
        size_y_ = (top_y + bottom_y) * BLOCK_Y_SIZE;

        zero_offset_x_ = left_x * BLOCK_X_SIZE;
        zero_offset_y_ = top_y * BLOCK_Y_SIZE;

        zero_offset_x_ = zero_offset_x_ == 0 ? 0 : zero_offset_x_ - 1;
        zero_offset_y_ = zero_offset_y_ == 0 ? 0 : zero_offset_y_ - 1;

        //    cout << "========> (" << zero_offset_x_ << ", "<< zero_offset_y_ << ") -> " << size_x_ << "x" << size_y_
        //    << endl;
    }

    /// 获得当前状态下地图X方向的size
    virtual int getSizeX() {
        updateSize();
        return size_x_;
    }

    /// 获得当前状态下地图Y方向的size
    virtual int getSizeY() {
        updateSize();
        return size_y_;
    }

    virtual void getLimitX(int &min, int &max) {
        updateSize();
        min = -zero_offset_x_;
        max = -zero_offset_x_ + (size_x_ - 2);
    }  /// x方向上的极限值
    virtual void getLimitY(int &min, int &max) {
        updateSize();
        min = -(size_y_ - 2) + zero_offset_y_;
        max = zero_offset_y_;
    }  /// y方向上的极限值

    virtual void getZeroOffset(int *x, int *y) {
        updateSize();
        *x = zero_offset_x_;
        *y = zero_offset_y_;
    }

    typedef Node *BlockQuarter[MAX_X_BLOCK_NUM][MAX_Y_BLOCK_NUM];

    /// 区块中x方向元素个数的幂值
    static const int BLOCK_X_SIZE_N = 10;
    /// 区块中y方向元素个数的幂值
    static const int BLOCK_Y_SIZE_N = 10;

    /// 区块中x方向元素个数
    static const int BLOCK_X_SIZE = 2 << (BLOCK_X_SIZE_N - 1);
    /// 区块中y方向元素个数
    static const int BLOCK_Y_SIZE = 2 << (BLOCK_Y_SIZE_N - 1);

 private:
    inline BlockQuarter &getBlockQuarter(int x, int y) {
        if (x >= 0 && y >= 0) {  // 原点在第一象限中
            return block_q1;
        } else if (x <= 0 && y >= 0) {
            return block_q2;
        } else if (x <= 0 && y <= 0) {
            return block_q3;
        } else if (x >= 0 && y <= 0) {
            return block_q4;
        } else {  // 不可能执行到这里，此语句是为了避免编译器警告
            return block_q1;
        }
    }

    void getCornerPoint(BlockQuarter &block_q, int *x, int *y) {
        bool find_x, find_y;
        find_x = false;
        find_y = false;
        *x = -1;
        *y = -1;
        for (int xx = MAX_X_BLOCK_NUM - 1; xx >= 0 && !(find_x && find_y); xx--) {
            for (int yy = MAX_Y_BLOCK_NUM - 1; yy >= 0 && !(find_x && find_y); yy--) {
                if (!find_x && block_q[xx][yy]) {
                    *x = xx;
                    find_x = true;
                }
                if (!find_y && block_q[yy][xx]) {
                    *y = xx;
                    find_y = true;
                }
            }
        }
    }

    Node *allocateNewBlock() {
        Node *block = new Node[BLOCK_X_SIZE * BLOCK_Y_SIZE];
        // 初始化地图中每个Node的值0
        memset(block, 0, sizeof(Node) * BLOCK_X_SIZE * BLOCK_Y_SIZE);
        return block;
    }

    /// 第一象限
    BlockQuarter block_q1;
    /// 第二象限
    BlockQuarter block_q2;
    /// 第三象限
    BlockQuarter block_q3;
    /// 第四象限
    BlockQuarter block_q4;

    static const int BLOCK_X_SIZE_MASK = BLOCK_X_SIZE - 1;
    static const int BLOCK_Y_SIZE_MASK = BLOCK_Y_SIZE - 1;

    bool is_dirty;
};

}  // namespace map
} /* namespace sros */

#endif /* DYNAMICMAP_H_ */
