
/**
 * @file DynamicBlockGrayMap.hpp
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
#define D_MAX_X_BLOCK_NUM 32
// 每个象限y方向区块个数
#define D_MAX_Y_BLOCK_NUM 32

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
class DynamicBlockGrayMap : public GrayMap {
 public:
    DynamicBlockGrayMap() : blk_(), blk_state_(), is_dirty(true) {}

    virtual ~DynamicBlockGrayMap() {
        //        clearAllBlockData();
    }

    bool createEmptyFile(std::string path, int zero_offset_x, int zero_offset_y, int size_x, int size_y) {
        file_path_ = path;

        const int EMPTY_BUFFER_SIZE = 4096;

        zero_offset_x_ = zero_offset_x;
        zero_offset_y_ = zero_offset_y;
        size_x_ = size_x;
        size_y_ = size_y;

        blk_x_num_ = size_x_ / BLOCK_X_SIZE;
        blk_y_num_ = size_y_ / BLOCK_Y_SIZE;

        for (int x = 0; x < blk_x_num_; x++) {
            for (int y = 0; y < blk_x_num_; y++) {
                blk_state_[x][y] = INVALID;
            }
        }

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

        int fx, fy;

        fx = x + zero_offset_x_;
        fy = zero_offset_y_ - y;

        assert(fx >= 0);
        assert(fy >= 0);

        // 在文件坐标系下区块下标
        int fx_idx = fx >> BLOCK_X_SIZE_N;
        int fy_idx = fy >> BLOCK_Y_SIZE_N;

        // 超出处理范围
        if (fx_idx >= D_MAX_X_BLOCK_NUM || fy_idx >= D_MAX_Y_BLOCK_NUM) {
            return 0;
        }

        BlockState *block_state = &blk_state_[fx_idx][fy_idx];

        if (*block_state == IN_USE) {
            // nothing to do
        } else if (*block_state == READY) {
            cout << "READY -> IN_USE: " << x << ", " << y << endl;
            *block_state = IN_USE;
            // 异步加载周围的block
             loadNeighborBlock(fx_idx, fy_idx);

            printDebugTable();

            unloadBlocks();

            *block_state = IN_USE;

            printDebugTable();

        } else if (*block_state == INVALID) {
            cout << "INVALID -> IN_USE: " << x << ", " << y << endl;
            // 同步加载当前block和附近的block
            loadNeighborBlock(fx_idx, fy_idx);

            printDebugTable();

            *block_state = IN_USE;

            unloadBlocks();

            *block_state = IN_USE;

            printDebugTable();

        } else if (*block_state == EMPTY) {
            return 0;
        }

        NodeBlock block = blk_[fx_idx][fy_idx];

        if (block == nullptr) {
            cout << "block == nullptr" << endl;
        }

        assert(block != nullptr);

        //        int index = ((fx & BLOCK_X_SIZE_MASK) << BLOCK_Y_SIZE_N)
        //                    + (fy & BLOCK_Y_SIZE_MASK);
        int index = fx - fx_idx * BLOCK_X_SIZE + (fy - fy_idx * BLOCK_Y_SIZE) * BLOCK_X_SIZE;

        return &block[index];
    }

    char getStateChar(int s) {
        if (s == IN_USE) {
            return '*';
        } else if (s == INVALID) {
            return '_';
        } else if (s == READY) {
            return 'R';
        } else {
            return ' ';
        }
    }

    void printDebugTable() {
        cout << "==============================" << endl;

        for (int x = 0; x < blk_x_num_; x++) {
            for (int y = 0; y < blk_x_num_; y++) {
                cout << getStateChar(blk_state_[x][y]);
            }
            cout << endl;
        }

        cout << "==============================" << endl;
    }

    /**
     * 将状态为READY且不在IN_USE周围的block保存到文件中
     */
    void unloadBlocks() {
        cout << "unloadBlocks()" << endl;

        // 二维数组, 用于标记blk是否需要unload
        vector<vector<bool>> blk_unload_(blk_x_num_, vector<bool>(blk_y_num_));

        // 逐个block检查状态是否为READY && 附近是否有IN_USE
        for (int x = 0; x < blk_x_num_; x++) {
            for (int y = 0; y < blk_x_num_; y++) {
                if (blk_state_[x][y] == READY) {
                    // 先假定需要进行unload, 下面进行搜索, 如果在附近搜到了IN_USE的block ,则不需要unload
                    blk_unload_[x][y] = true;
                } else {
                    blk_unload_[x][y] = false;
                    continue;
                }

                // 遍历周围的9个block, 包括自身
                for (int i = -1; i <= 1; i++) {
                    for (int j = -1; j <= 1; j++) {
                        // 检查是否越界
                        if (x + i < 0 || x + i >= blk_x_num_ || y + j < 0 || y + j >= blk_y_num_) {
                            continue;
                        }

                        if (blk_state_[x + i][y + j] == IN_USE) {
                            blk_unload_[x][y] = false;
                            break;
                        }
                    }
                }
            }
        }

        cout << "---------------" << endl;
        for (int x = 0; x < blk_x_num_; x++) {
            for (int y = 0; y < blk_x_num_; y++) {
                cout << static_cast<int>(blk_unload_[x][y]);
            }
            cout << endl;
        }
        cout << "---------------" << endl;

        // 开始unload
        for (int x = 0; x < blk_x_num_; x++) {
            for (int y = 0; y < blk_x_num_; y++) {
                cout << static_cast<int>(blk_unload_[x][y]);
                if (blk_unload_[x][y]) {
                    if (doUnload(x, y)) {
                        blk_state_[x][y] = INVALID;
                        cout << "READY -> INVALID" << endl;
                    }
                }
            }
        }

        for (int x = 0; x < blk_x_num_; x++) {
            for (int y = 0; y < blk_x_num_; y++) {
                if (blk_state_[x][y] == IN_USE) {
                    blk_state_[x][y] = READY;
                }
            }
        }
    }

    bool doUnload(int x, int y) {
        cout << "doUnload() : " << x << ", " << y << endl;

        NodeBlock block = blk_[x][y];

        assert(block != nullptr);

        bool result = false;

        int64_t f_pos = getFileBlockPos(x, y);

        if (f_pos >= 0 && writeBlockToFile(file_path_, block, f_pos)) {
            delete block;  // 释放内存
            blk_[x][y] = nullptr;

            return true;
        } else {
            return false;
        }
    }

    /**
     * 加载以当前block为中心的9个block, 设置新加载的block状态为READY
     * @param quarter
     * @param block_x_index
     * @param block_y_index
     */
    void loadNeighborBlock(int f_x_index, int f_y_index) {
        cout << "loadNeighborBlock: " << f_x_index << ", " << f_y_index << endl;

        // top left
        doLoadNeighborBlock(f_x_index, f_y_index, -1, -1);

        // top middle
        doLoadNeighborBlock(f_x_index, f_y_index, 0, -1);

        // top right
        doLoadNeighborBlock(f_x_index, f_y_index, 1, -1);

        // middle left
        doLoadNeighborBlock(f_x_index, f_y_index, -1, 0);

        // this
        doLoadNeighborBlock(f_x_index, f_y_index, 0, 0);

        // middle right
        doLoadNeighborBlock(f_x_index, f_y_index, 1, 0);

        // bottom left
        doLoadNeighborBlock(f_x_index, f_y_index, -1, 1);

        // bottom middle
        doLoadNeighborBlock(f_x_index, f_y_index, 0, 1);

        // bottom right
        doLoadNeighborBlock(f_x_index, f_y_index, 1, 1);
    }

    /**
     * 根据参数提供的在文件坐标系下block下标与偏移量加载block
     * @param file_block_x_index
     * @param file_block_y_index
     * @param delta_x
     * @param delta_y
     */
    void doLoadNeighborBlock(int file_block_x_index, int file_block_y_index, int delta_x, int delta_y) {
        int f_n_x_index;
        int f_n_y_index;

        f_n_x_index = file_block_x_index + delta_x;
        f_n_y_index = file_block_y_index + delta_y;

        cout << "doLoadNeighborBlock: delta: " << delta_x << ", " << delta_y << endl;
        cout << "doLoadNeighborBlock: index: " << f_n_x_index << ", " << f_n_y_index << endl;

        // 检查是否越界
        if (f_n_x_index < 0 || f_n_x_index >= blk_x_num_ || f_n_y_index < 0 || f_n_y_index >= blk_x_num_) {
            cout << "doLoadNeighborBlock: "
                 << "越界" << endl;
            return;
        }

        BlockState *block_state = &blk_state_[f_n_x_index][f_n_y_index];

        // 仅当状态为NO_VALID时才需要加载
        if (*block_state != INVALID) {
            cout << "doLoadNeighborBlock: state: " << *block_state << ", 不需要加载" << endl;
            return;
        }

        loadBlockByFileIndex(f_n_x_index, f_n_y_index);

        *block_state = READY;
    }

    /**
     * 根据区块下标计算在文件中的指针
     * @param f_blk_x
     * @param f_blk_y
     * @return
     */
    int64_t getFileBlockPos(int f_blk_x, int f_blk_y) const {  // 检查是否越界
        int64_t start_pos;

        if (f_blk_x < 0 || f_blk_x >= blk_x_num_ || f_blk_y < 0 || f_blk_y >= blk_y_num_) {
            return -1;
        }

        // 计算文件中的指针位置
        start_pos = blk_x_num_ * BLOCK_X_SIZE * BLOCK_Y_SIZE * f_blk_y + BLOCK_X_SIZE * f_blk_x;

        cout << "getFileBlockPos: start_pos: " << start_pos << endl;
        return start_pos;
    }

    /**
     * 根据文件坐标系下区块下标加载数据到block对应的内存地址
     * @param quarter
     * @param file_block_x_index
     * @param file_block_y_index
     * @param [out] block
     * @return
     */
    bool loadBlockByFileIndex(int file_block_x_index, int file_block_y_index) {
        cout << "loadBlockByFileIndex: index: " << file_block_x_index << ", " << file_block_y_index << endl;

        NodeBlock block = blk_[file_block_x_index][file_block_y_index];

        if (block == nullptr) {
            block = allocateNewBlock();  // 申请内存
            blk_[file_block_x_index][file_block_y_index] = block;
        }

        int64_t start_pos = getFileBlockPos(file_block_x_index, file_block_y_index);

        if (start_pos >= 0) {
            return readBlockFromFile(file_path_, block, start_pos);
        } else {
            return false;
        }
    }

    bool writeBlockToFile(string path, NodeBlock block, int64_t pos) {
        fstream fs(path, fstream::binary | fstream::in | fstream::out);

        skipFileHeader(fs);

        int64_t file_pos = pos + fs.tellg();

        fs.seekg(file_pos);

        int yy;
        NodeValue value;
        for (int y = 0; y < BLOCK_Y_SIZE; y++) {
            yy = y * BLOCK_X_SIZE;
            for (int x = 0; x < BLOCK_X_SIZE; x++) {
                value = block[x + yy].value;
                fs.write(reinterpret_cast<char *>(&value), sizeof(NodeValue));
            }
            // 跳过一整行, 继续写入下一行
            file_pos += size_x_;
            fs.seekg(file_pos);
        }

        return true;
    }

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
            in.seekg(file_pos);
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

    /**
     * 跳过地图文件的头部
     */
    void skipFileHeader(fstream &in) {
        char buffer[32] = {'\0'};
        in.read(buffer, MAGIC_STR_LEN);
        in.getline(buffer, 32);
        in.getline(buffer, 32);
        in.getline(buffer, 32);
        in.getline(buffer, 32);  // skip line
    }

    virtual bool save(const char *path) {
        assert(file_path_ == string(path));

        for (int x = 0; x < blk_x_num_; x++) {
            for (int y = 0; y < blk_x_num_; y++) {
                if (blk_state_[x][y] == IN_USE) {
                    doUnload(x, y);
                }
            }
        }

        return true;
    }

    virtual bool load(const char *path) {
        file_path_ = path;

        ifstream in(file_path_, ifstream::binary);
        readHeader(in);
        in.close();

        blk_x_num_ = size_x_ / BLOCK_X_SIZE;
        blk_y_num_ = size_y_ / BLOCK_Y_SIZE;

        for (int x = 0; x < blk_x_num_; x++) {
            for (int y = 0; y < blk_x_num_; y++) {
                blk_[x][y] = nullptr;
                blk_state_[x][y] = INVALID;
            }
        }

        return true;
    }

    /// 获得当前状态下地图X方向的size
    virtual int getSizeX() { return size_x_; }

    /// 获得当前状态下地图Y方向的size
    virtual int getSizeY() { return size_y_; }

    virtual void getLimitX(int &min, int &max) {
        min = -zero_offset_x_;
        max = -zero_offset_x_ + (size_x_ - 2);
    }  /// x方向上的极限值

    virtual void getLimitY(int &min, int &max) {
        min = -(size_y_ - 2) + zero_offset_y_;
        max = zero_offset_y_;
    }  /// y方向上的极限值

    virtual void getZeroOffset(int *x, int *y) {
        *x = zero_offset_x_;
        *y = zero_offset_y_;
    }

    enum BlockState {
        EMPTY,
        INVALID,
        READY,
        IN_USE,
    };

    typedef NodeBlock BlockArray[D_MAX_X_BLOCK_NUM][D_MAX_Y_BLOCK_NUM];
    typedef BlockState BlockStateArray[D_MAX_X_BLOCK_NUM][D_MAX_Y_BLOCK_NUM];

    /// 区块中x方向元素个数的幂值
    static const int BLOCK_X_SIZE_N = 12;
    /// 区块中y方向元素个数的幂值
    static const int BLOCK_Y_SIZE_N = 12;

    /// 区块中x方向元素个数
    static const int BLOCK_X_SIZE = 2 << (BLOCK_X_SIZE_N - 1);
    /// 区块中y方向元素个数
    static const int BLOCK_Y_SIZE = 2 << (BLOCK_Y_SIZE_N - 1);

 private:
    NodeBlock allocateNewBlock() {
        NodeBlock block = new Node[BLOCK_X_SIZE * BLOCK_Y_SIZE];
        // 初始化地图中每个Node的值0
        memset(block, 0, sizeof(Node) * BLOCK_X_SIZE * BLOCK_Y_SIZE);
        return block;
    }

    BlockArray blk_;
    BlockStateArray blk_state_;

    int blk_x_num_;  // x方向区块总个数
    int blk_y_num_;  // y方向区块总个数

    static const int BLOCK_X_SIZE_MASK = BLOCK_X_SIZE - 1;
    static const int BLOCK_Y_SIZE_MASK = BLOCK_Y_SIZE - 1;

    bool is_dirty;

    string file_path_;
};

}  // namespace map
} /* namespace sros */

#endif /* DYNAMIC_BLOCK_GRAY_MAP_H_ */
