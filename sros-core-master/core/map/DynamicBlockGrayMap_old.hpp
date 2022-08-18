
/**
 * @file DynamicBlockGrayMap_old.hpp
 *
 * @author lhx
 * @date 2016年10月16日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef DYNAMIC_BLOCK_GRAY_MAP_H_
#define DYNAMIC_BLOCK_GRAY_MAP_H_

#include "GrayMap.hpp"

#include <cmath>
#include <cstring>

// 每个象限x方向区块个数
#define MAX_X_BLOCK_NUM 32
// 每个象限y方向区块个数
#define MAX_Y_BLOCK_NUM 32

namespace sros {
namespace map {

typedef Node *NodeBlock;

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
class DynamicBlockGrayMap_old : public GrayMap {
 public:
    DynamicBlockGrayMap_old() : is_dirty(true) {
        //    value_type* blocks[][][4] = {block_q1, block_q2, block_q3, block_q4};
        initAllBlock();
    }

    void initAllBlock() {
        for (Quarter q = Q1; q <= Q4; q = Quarter(static_cast<int>(q + 1))) {
            for (int i = 0; i < MAX_X_BLOCK_NUM; i++) {
                for (int j = 0; j < MAX_Y_BLOCK_NUM; j++) {
                    block_[q][i][j] = nullptr;
                    block_state_[q][i][j] = EMPTY;
                }
            }
        }
    }

    virtual ~DynamicBlockGrayMap_old() { clearAllBlockData(); }

    void clearAllBlockData() const {
        for (Quarter q = Q1; q <= Q4; q = Quarter(static_cast<int>(q + 1))) {
            for (int i = 0; i < MAX_X_BLOCK_NUM; i++) {
                for (int j = 0; j < MAX_Y_BLOCK_NUM; j++) {
                    if (block_[q][i][j]) delete block_[q][i][j];
                }
            }
        }
    }

    bool createEmptyFile(std::string path, int zero_offset_x, int zero_offset_y, int size_x, int size_y) {
        const int EMPTY_BUFFER_SIZE = 4096;

        zero_offset_x_ = zero_offset_x;
        zero_offset_y_ = zero_offset_y;
        size_x_ = size_x;
        size_y_ = size_y;

        ofstream out(path, ofstream::out | ofstream::binary);

        NodeValue empty_node_buffer[EMPTY_BUFFER_SIZE];
        memset(empty_node_buffer, 0, EMPTY_BUFFER_SIZE * sizeof(NodeValue));

        NodeValue empty_node = 0;

        int64_t total_node_cnt = (int64_t)size_x * (int64_t)size_y;

        int64_t buffer_cnt = total_node_cnt / EMPTY_BUFFER_SIZE;
        int64_t node_cnt = total_node_cnt % EMPTY_BUFFER_SIZE;

        writeHeader(out);

        for (int i = 0; i < buffer_cnt; i++) {
            out.write(reinterpret_cast<char *>(empty_node_buffer), EMPTY_BUFFER_SIZE * sizeof(NodeValue));
        }

        for (int i = 0; i < node_cnt; i++) {
            out.write(reinterpret_cast<char *>(&empty_node), sizeof(NodeValue));
        }

        out.close();

        return true;
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

        Quarter q = getBlockQuarter(x, y);

        BlockState *block_state = &block_state_[q][block_x_index][block_y_index];

        if (*block_state == IN_USE) {
            // nothing to do
        } else if (*block_state == READY) {
            *block_state = IN_USE;
            // TODO(nobody): 异步加载周围的block
            loadBlockAndNeighbor(q, block_x_index, block_y_index);
        } else if (*block_state == NO_VALID) {
            // 同步加载当前block和附近的block
            loadBlockAndNeighbor(q, block_x_index, block_y_index);
        } else if (*block_state == EMPTY) {
            return 0;
        }

        NodeBlock block = block_[q][block_x_index][block_y_index];

        if (!block) {
            block = allocateNewBlock();
            block_[q][block_x_index][block_y_index] = block;
        }

        int index = ((abs_x & BLOCK_X_SIZE_MASK) << BLOCK_Y_SIZE_N) + (abs_y & BLOCK_Y_SIZE_MASK);

        return &block[index];
    }

    /**
     * 加载下标对应的block和其周围的block
     * @param quarter
     * @param block_x_index
     * @param block_y_index
     */
    void loadBlockAndNeighbor(Quarter quarter, int block_x_index, int block_y_index) {
        BlockState block_state = block_state_[quarter][block_x_index][block_y_index];

        if (block_state == NO_VALID || block_state == IN_USE) {
            // 加载当前&周围block
            loadNeighborBlock(quarter, block_x_index, block_y_index);
        }

        // 将状态为READY且不在IN_USE周围的block保存到文件中
    }

    void unloadBlock() {}

    /**
     * 加载以当前block为中心的9个block, 设置新加载的block状态为READY
     * @param quarter
     * @param block_x_index
     * @param block_y_index
     */
    void loadNeighborBlock(Quarter quarter, int block_x_index, int block_y_index) {
        // 在文件坐标系下区块的下标
        int file_x_index = 0;
        int file_y_index = 0;

        getFileBlockIndex(quarter, block_x_index, block_y_index, file_x_index, file_y_index);

        // top left
        doLoadNeighborBlock(file_x_index, file_y_index, -1, -1);

        // top middle
        doLoadNeighborBlock(file_x_index, file_y_index, 0, -1);

        // top right
        doLoadNeighborBlock(file_x_index, file_y_index, 1, -1);

        // middle left
        doLoadNeighborBlock(file_x_index, file_y_index, -1, 0);

        // this
        doLoadNeighborBlock(file_x_index, file_y_index, 0, 0);

        // middle right
        doLoadNeighborBlock(file_x_index, file_y_index, 1, 0);

        // bottom left
        doLoadNeighborBlock(file_x_index, file_y_index, -1, 1);

        // bottom middle
        doLoadNeighborBlock(file_x_index, file_y_index, 0, 1);

        // bottom right
        doLoadNeighborBlock(file_x_index, file_y_index, 1, 1);
    }

    /**
     * 根据参数提供的在文件坐标系下block下标与偏移量加载block
     * @param file_block_x_index
     * @param file_block_y_index
     * @param delta_x
     * @param delta_y
     */
    void doLoadNeighborBlock(int file_block_x_index, int file_block_y_index, int delta_x, int delta_y) {
        int block_x_num = size_x_ / BLOCK_X_SIZE;  // x方向区块总个数
        int block_y_num = size_y_ / BLOCK_Y_SIZE;

        int file_n_x_index;
        int file_n_y_index;
        int n_x_index;
        int n_y_index;
        Quarter n_quarter;

        file_n_x_index = file_block_x_index + delta_x;
        file_n_y_index = file_block_x_index + delta_y;

        // 检查是否越界
        if (file_n_x_index < 0 || file_n_x_index >= block_x_num || file_n_y_index < 0 ||
            file_n_y_index >= block_y_num) {
            return;
        }

        getBlockIndexByFileBlockIndex(file_n_x_index, file_n_y_index, n_quarter, n_x_index, n_y_index);

        BlockState *block_state = &block_state_[n_quarter][n_x_index][n_y_index];

        // 仅当状态为NO_VALID时才需要加载
        if (*block_state != NO_VALID) {
            return;
        }

        NodeBlock block = block_[n_quarter][n_x_index][n_y_index];

        loadBlockByFileIndex(n_quarter, file_n_x_index, file_n_y_index, block);

        *block_state = READY;
    }

    /**
     * 获取(block_x_index, block_y_index)在文件坐标系下对应的区块下标
     * @param quarter
     * @param block_x_index
     * @param block_y_index
     * @param [out] file_block_x_index
     * @param [out] file_block_y_index
     */
    void getFileBlockIndex(const Quarter &quarter, int block_x_index, int block_y_index, int &file_block_x_index,
                           int &file_block_y_index) const {
        // Q1中区块的个数
        int q1_block_x_num = (size_x_ - zero_offset_x_) / BLOCK_X_SIZE;
        int q1_block_y_num = (zero_offset_y_) / BLOCK_Y_SIZE;

        int q2_block_x_num = (zero_offset_x_) / BLOCK_X_SIZE;
        int q2_block_y_num = q1_block_y_num;

        int q3_block_x_num = q2_block_x_num;
        int q3_block_y_num = (size_y_ - zero_offset_y_) / BLOCK_Y_SIZE;

        int q4_block_x_num = q1_block_x_num;
        int q4_block_y_num = q3_block_y_num;

        // 根据象限不同, 得到block在文件坐标系下的下标
        if (quarter == Q1) {
            file_block_x_index = q2_block_x_num + block_x_index - 1;
            file_block_y_index = q1_block_y_num - block_y_index - 1;
        } else if (quarter == Q2) {
            file_block_x_index = q2_block_x_num - block_x_index - 1;
            file_block_y_index = q2_block_y_num - block_y_index - 1;
        } else if (quarter == Q3) {
            file_block_x_index = q3_block_x_num - block_x_index - 1;
            file_block_y_index = q2_block_y_num + block_y_index - 1;
        } else if (quarter == Q4) {
            file_block_x_index = q2_block_x_num + block_x_index - 1;
            file_block_y_index = q2_block_y_num + block_y_index - 1;
        }
    }

    void getBlockIndexByFileBlockIndex(int file_block_x_index, int file_block_y_index, Quarter &quarter,
                                       int &block_x_index, int &block_y_index) {
        // Q1中区块的个数
        int q1_block_x_num = (size_x_ - zero_offset_x_) / BLOCK_X_SIZE;
        int q1_block_y_num = (zero_offset_y_) / BLOCK_Y_SIZE;

        int q2_block_x_num = (zero_offset_x_) / BLOCK_X_SIZE;
        int q2_block_y_num = q1_block_y_num;

        int q3_block_x_num = q2_block_x_num;
        int q3_block_y_num = (size_y_ - zero_offset_y_) / BLOCK_Y_SIZE;

        int q4_block_x_num = q1_block_x_num;
        int q4_block_y_num = q3_block_y_num;

        if (file_block_x_index >= q2_block_x_num && file_block_y_index < q2_block_y_num) {
            quarter = Q1;
            block_x_index = file_block_x_index - q2_block_x_num;
            block_y_index = file_block_y_index;
        }

        if (file_block_x_index < q2_block_x_num && file_block_y_index < q2_block_y_num) {
            quarter = Q2;
            block_x_index = file_block_x_index;
            block_y_index = file_block_y_index;
        }

        if (file_block_x_index < q2_block_x_num && file_block_y_index >= q2_block_y_num) {
            quarter = Q3;
            block_x_index = file_block_x_index;
            block_y_index = file_block_y_index - q2_block_y_num;
        }

        if (file_block_x_index >= q2_block_x_num && file_block_y_index >= q2_block_y_num) {
            quarter = Q4;
            block_x_index = file_block_x_index - q2_block_x_num;
            block_y_index = file_block_y_index - q2_block_y_num;
        }
    }

    /**
     * 根据文件坐标系下区块下标加载数据到block对应的内存地址
     * @param quarter
     * @param file_block_x_index
     * @param file_block_y_index
     * @param [out] block
     * @return
     */
    bool loadBlockByFileIndex(const Quarter &quarter, int file_block_x_index, int file_block_y_index, NodeBlock block) {
        int block_x_num = size_x_ / BLOCK_X_SIZE;  // x方向区块总个数
        int block_y_num = size_y_ / BLOCK_Y_SIZE;

        // 计算文件中的指针位置
        int64_t start_pos =
            MAX_X_BLOCK_NUM * BLOCK_X_SIZE * BLOCK_Y_SIZE * file_block_y_index + BLOCK_X_SIZE * file_block_x_index;

        if (quarter == Q1 || quarter == Q4) {
            start_pos -= 1;  // 去除y轴的影响
        }
        if (quarter == Q3 || quarter == Q4) {
            start_pos -= block_x_num * BLOCK_X_SIZE;  // 去除x轴的影响
        }

        return readBlockFromFile(file_path_, block, start_pos);
    }

    //    /**
    //     * 根据区块下标从文件中加载数据到内存中
    //     * @param quarter 象限
    //     * @param block_x_index
    //     * @param block_y_index
    //     * @return 是否加载成功
    //     */
    //    bool loadBlock(Quarter quarter, int block_x_index, int block_y_index) {
    //        // 在文件坐标系下区块的下标
    //        int file_block_x_index = 0;
    //        int file_block_y_index = 0;
    //
    //        NodeBlock block = block_[quarter][block_x_index][block_y_index];
    //
    //        getFileBlockIndex(quarter, block_x_index, block_y_index, file_block_x_index, file_block_y_index);
    //
    //        return loadBlockByFileIndex(quarter, file_block_x_index, file_block_y_index, block);
    //    }

    /**
     * 从指定的文件指针开始,读取一个区块的数据到block中
     * @param path 文件路径
     * @param block 数据存储的block指针
     * @param start_pos 文件指针
     * @return 是否加载成功
     */
    bool readBlockFromFile(string path, NodeBlock block, int64_t start_pos) {
        ifstream in(path, ifstream::binary);
        readHeader(in);  // skip header

        int64_t file_pos = start_pos + in.tellg();
        in.seekg(file_pos);  // 定位到block开始处

        int yy;
        NodeValue value;
        for (int y = 0; y < BLOCK_Y_SIZE; y++) {
            yy = y * BLOCK_X_SIZE;
            for (int x = 0; x < BLOCK_X_SIZE; x++) {
                in.read(reinterpret_cast<char *>(&value), sizeof(NodeValue));
                block[x + yy].value = value;
            }
            // 跳过一整行, 继续读取下一行
            file_pos += size_x_;
        }

        return true;
    }

    virtual bool set(int x, int y, Node value) {
        Node *p = getp(x, y);
        if (p) {
            *p = value;
        }
        return (p != 0);
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
        file_path_ = path;

        ifstream in(file_path_, ifstream::binary);
        readHeader(in);
        in.close();

        // 根据地图大小设置block状态
        int x_positive_axel_len = size_x_ - zero_offset_x_;
        int x_negative_axel_len = zero_offset_x_;

        int y_positive_axel_len = zero_offset_y_;
        int y_negative_axel_len = size_y_ - zero_offset_y_;

        int q1_x_block_num = static_cast<int>(ceil(1.0 * x_positive_axel_len / BLOCK_X_SIZE));
        int q1_y_block_num = static_cast<int>(ceil(1.0 * y_positive_axel_len / BLOCK_Y_SIZE));

        int q2_x_block_num = static_cast<int>(ceil(1.0 * x_negative_axel_len / BLOCK_X_SIZE));
        int q2_y_block_num = q1_y_block_num;

        int q3_x_block_num = q2_x_block_num;
        int q3_y_block_num = static_cast<int>(ceil(1.0 * y_negative_axel_len / BLOCK_Y_SIZE));

        int q4_x_block_num = q1_x_block_num;
        int q4_y_block_num = q3_y_block_num;

        clearAllBlockData();

        initAllBlock();

        for (int x = 0; x < MAX_X_BLOCK_NUM; x++) {
            for (int y = 0; y < MAX_Y_BLOCK_NUM; y++) {
                if (x < q1_x_block_num && y < q1_y_block_num) {
                    block_state_[Q1][x][y] = NO_VALID;
                }

                if (x < q2_x_block_num && y < q2_y_block_num) {
                    block_state_[Q2][x][y] = NO_VALID;
                }

                if (x < q3_x_block_num && y < q3_y_block_num) {
                    block_state_[Q3][x][y] = NO_VALID;
                }

                if (x < q4_x_block_num && y < q4_y_block_num) {
                    block_state_[Q4][x][y] = NO_VALID;
                }
            }
        }

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

    enum BlockState {
        EMPTY,
        NO_VALID,
        READY,
        IN_USE,
    };

    enum Quarter {
        Q1 = 0,
        Q2 = 1,
        Q3 = 2,
        Q4 = 3,
    };

    static const int QUARTER_NUM = 4;

    typedef NodeBlock BlockQuarter[QUARTER_NUM][MAX_X_BLOCK_NUM][MAX_Y_BLOCK_NUM];
    typedef BlockState BlockQuarterState[QUARTER_NUM][MAX_X_BLOCK_NUM][MAX_Y_BLOCK_NUM];

    /// 区块中x方向元素个数的幂值
    static const int BLOCK_X_SIZE_N = 10;
    /// 区块中y方向元素个数的幂值
    static const int BLOCK_Y_SIZE_N = 10;

    /// 区块中x方向元素个数
    static const int BLOCK_X_SIZE = 2 << (BLOCK_X_SIZE_N - 1);
    /// 区块中y方向元素个数
    static const int BLOCK_Y_SIZE = 2 << (BLOCK_Y_SIZE_N - 1);

 private:
    inline BlockQuarter &getBlockQuarter(int x, int y, BlockQuarter block_quarter,
                                         BlockQuarterState block_quarter_state) {
        if (x >= 0 && y >= 0) {  // 原点在第一象限中
            block_quarter = block_q1;
            block_quarter_state = block_q1_state;
            return block_q1;
        } else if (x <= 0 && y >= 0) {
            block_quarter = block_q2;
            block_quarter_state = block_q2_state;
            return block_q2;
        } else if (x <= 0 && y <= 0) {
            block_quarter = block_q3;
            block_quarter_state = block_q3_state;
            return block_q3;
        } else if (x >= 0 && y <= 0) {
            block_quarter = block_q4;
            block_quarter_state = block_q4_state;
            return block_q4;
        } else {  // 不可能执行到这里，此语句是为了避免编译器警告
            return block_q1;
        }
    }

    inline Quarter getBlockQuarter(int x, int y) {
        if (x >= 0 && y >= 0) {  // 原点在第一象限中
            return Q1;
        } else if (x <= 0 && y >= 0) {
            return Q2;
        } else if (x <= 0 && y <= 0) {
            return Q3;
        } else if (x >= 0 && y <= 0) {
            return Q4;
        }
        return Q1;
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

    NodeBlock allocateNewBlock() {
        NodeBlock block = new Node[BLOCK_X_SIZE * BLOCK_Y_SIZE];
        // 初始化地图中每个Node的值0
        memset(block, 0, sizeof(Node) * BLOCK_X_SIZE * BLOCK_Y_SIZE);
        return block;
    }

    BlockQuarter block_;
    BlockQuarterState block_state_;

    /// 第一象限
    BlockQuarter block_q1;
    BlockQuarterState block_q1_state;
    /// 第二象限
    BlockQuarter block_q2;
    BlockQuarterState block_q2_state;
    /// 第三象限
    BlockQuarter block_q3;
    BlockQuarterState block_q3_state;
    /// 第四象限
    BlockQuarter block_q4;
    BlockQuarterState block_q4_state;

    static const int BLOCK_X_SIZE_MASK = BLOCK_X_SIZE - 1;
    static const int BLOCK_Y_SIZE_MASK = BLOCK_Y_SIZE - 1;

    bool is_dirty;

    string file_path_;
};

}  // namespace map
} /* namespace sros */

#endif /* DYNAMIC_BLOCK_GRAY_MAP_H_ */
