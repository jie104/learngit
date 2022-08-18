//
// Created by lfc on 2022/5/9.
//

#ifndef LIVOX_PALLET_DETECT_PICKPLACE_FRONTIER_EDGE_DETECT_HPP
#define LIVOX_PALLET_DETECT_PICKPLACE_FRONTIER_EDGE_DETECT_HPP
#include <Eigen/Dense>
#include <memory>
#include <sstream>
#include "ground_plane_map.hpp"
#include "norm_computer.hpp"

namespace standard {
struct EdgeInfo {
    int edge_index;         //边缘的索引，用来获取高度值
    int up_index;           //上边缘索引值，该值用来保证这洞一定能放栈板
    int depth;              //边缘的厚度，一般情况下，边缘需要具有厚度才能支撑重物
    int x;                  // x坐标
    int y;                  // y坐标
    Eigen::Vector3f point;  //边缘做均值滤波后的值
    float weight;           //边缘权重，该值在计算向量时候用来做加权平均
};

struct FrontierEdgeInfo {
 public:
    void computeEdgeInfo(std::vector<EdgeInfo>& boundary) {
        std::nth_element(
            boundary.begin(), boundary.begin() + edges.size() / 2, boundary.end(),
            [](const EdgeInfo& first, const EdgeInfo& second) { return first.edge_index > second.edge_index; });
        mean_index = boundary[boundary.size() / 2].edge_index;
        up_index = boundary[boundary.size() / 2].up_index;
        std::sort(boundary.begin(), boundary.end(),
                  [](const EdgeInfo& first, const EdgeInfo& second) { return first.y < second.y; });
        edges = boundary;
        mean.setZero();
        for (auto& edge : edges) {
            mean += edge.point;
        }
        start_point = edges.front();
        end_point = edges.back();
        mean = mean / edges.size();
        direction = end_point.point - start_point.point;
        for (auto& edge : edges) {
            Eigen::Vector3f delta_vector = edge.point - mean;
            float ratio = 1.0f;
            if (direction.dot(delta_vector) < 0) {
                ratio = -1.0f;
            }
            direction = direction + delta_vector * (edge.weight * ratio);
        }
        direction[2] = 0.0f;
        direction.normalize();

        length = (end_point.point - start_point.point).head<2>().norm();
        //        LOG(INFO) << "length:" << length << "," << edges.size();
        NormComputer::buildNormByTwoVector(Eigen::Vector3f(0, 0, 1), direction, norm);
    }
    std::vector<EdgeInfo> edges;  //所有边缘值
    Eigen::Vector3f direction;    //边缘的朝向，平行于边缘
    Eigen::Vector3f norm;         //边缘的法向
    Eigen::Vector3f mean;         //边缘计算的中心值
    EdgeInfo start_point;         //边缘的起始点
    EdgeInfo end_point;           //边缘的结束点
    int mean_index;               //边缘点中值索引
    int up_index;                 //上顶点，确定上下顶点之间能够容纳东西
    float length;  //边缘长度，用来做校验，边缘是否完整，目前存在没有包含边缘所有点的情况，需要持续做优化
};
class PickPlaceFrontierEdgeDetect {
 public:
    PickPlaceFrontierEdgeDetect() {}

    virtual ~PickPlaceFrontierEdgeDetect() {}

    bool findFrontierEdge(std::shared_ptr<GroundPlaneMap<Eigen::Vector3f>>& map,
                          std::vector<std::shared_ptr<FrontierEdgeInfo>>& frontier_edges) {
        map_ = map;
        map_resolution_ = map_->resolution();
        auto width = map_->width();
        auto length = map_->length();
        int x_step = 1;
        int y_step = 1;
        for (int x = 0; x < length; x += x_step) {
            //宽度优先搜索，我们认为，第一个找到的洞大概率是栈板的前沿，这种方法可快速找到满足要求的栈板
            for (int y = 0; y < width; y += y_step) {
                auto& bar = map_->bar(x, y);
                //判断相对高度是否满足要求
                if (bar.heightEnough(min_edge_height_)) {
                    std::vector<EdgeInfo> edges;
                    if (findEdgesInBar(x, y, bar, min_delta_hole_height_, edges)) {
                        for (auto& edge : edges) {
                            std::vector<EdgeInfo> boundary;
                            if (tranverseEdge(edge, boundary) && boundary.size() > 10) {
                                std::shared_ptr<FrontierEdgeInfo> frontier_edge(new FrontierEdgeInfo);
                                frontier_edge->computeEdgeInfo(boundary);
                                if (optimizeEdge(frontier_edge) && frontier_edge->length > 0.8f) {
                                    if (checkIsBoundary(frontier_edge->edges, frontier_edge->norm, 0.5f)) {
                                        LOG(INFO)
                                            << "ege info:" << frontier_edge->length << "," << frontier_edge->mean[0]
                                            << "," << frontier_edge->mean[1] << "," << frontier_edge->mean[2];
                                        eraseEdges(frontier_edge->edges);
                                        frontier_edges.push_back(frontier_edge);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if (frontier_edges.size()) {
            filterSurplusEdges(frontier_edges);
            return true;
        }
        return false;
    }

 private:
    void filterSurplusEdges(std::vector<std::shared_ptr<FrontierEdgeInfo>>& frontier_edges) {
        auto overlapEdge = [&](std::shared_ptr<FrontierEdgeInfo>& first, std::shared_ptr<FrontierEdgeInfo>& second,const float &max_dist_in_height,const float& height_thresh) {
            Eigen::Vector3f delta_mean = second->mean - first->mean;
            auto project_value = fabs(first->direction.dot(delta_mean));
            auto x_direction_value = fabs(first->norm.dot(delta_mean));
            auto height_value = fabs(delta_mean.dot(Eigen::Vector3f(0, 0, 1)));
//            LOG(INFO) << "height value:" << height_value << "," << project_value << "," << x_direction_value;
            if (height_value < height_thresh) {
                if (project_value < first->length / 2.0f && height_value < max_dist_in_height) {
                    return true;
                }
            }
            return false;
        };

        std::sort(frontier_edges.begin(), frontier_edges.end(),
                  [](const std::shared_ptr<FrontierEdgeInfo>& first, const std::shared_ptr<FrontierEdgeInfo>& second) {
                      return first->length > second->length;
                  });
        std::vector<std::shared_ptr<FrontierEdgeInfo>> bk_edges;
        bk_edges.swap(frontier_edges);
        int edge_size = bk_edges.size();
        std::vector<bool> states(edge_size, true);
        for (int i = 0; i < edge_size; ++i) {
            if (states[i]) {
                states[i] = false;
                auto& edge = bk_edges[i];
                frontier_edges.push_back(edge);
                for (int j = i + 1; j < edge_size; ++j) {
                    if (overlapEdge(edge, bk_edges[j], overlap_dist_thresh_, overlap_height_thresh_)) {
                        states[j] = false;
                    }
                }
            }
        }
    }

    void searchAdjacentEdgeInYByDir(Eigen::Vector3f& mean, Eigen::Vector3f& direction, float step,
                                    const EdgeInfo& cand_edge, std::vector<EdgeInfo>& boundary) {
        int unfounded = 0;
        int y = map_->coordY(mean[1]);
        float i = 0;
        int array_index[7] = {0, -1, 1, -2, 2, -3, 3};
        int last_y = 0;
        while (y < map_->width() && y >= 0) {
            i++;
            auto curr_coord = mean + direction * step * i;
            y = map_->coordY(curr_coord[1]);
            if (last_y == y) {
                continue;
            }
            last_y = y;
            unfounded++;
            int max_search_thresh = 4;
            int real_index = 0;
            std::vector<int> reals;
            for (int j = 0; j <= max_search_thresh; ++j) {
                auto x = map_->coordX(curr_coord[0]) + array_index[j];
                if (map_->inGrid(x, y)) {
                    EdgeInfo edge;
                    auto& bar = map_->bar(x, y);
                    if (checkEdge(bar, cand_edge.edge_index, cand_edge.up_index, real_index)) {
                        if (checkPointThresh(x, y, bar.pointByIndex(real_index), 2 * map_resolution_)) {
                            unfounded = 0;
                            creatEdge(x, y, real_index, cand_edge.up_index, computeDeep(bar, real_index), bar, edge);
                            boundary.push_back(edge);
                            break;
                        }
                    } else {
                        reals.push_back(real_index);
                    }
                } else {
                    break;
                }
            }
            if (unfounded >= 2) {
                std::stringstream ss;
                for (auto& real : reals) {
                    ss << real << ",";
                }
                //              LOG(INFO) << "real index:" << ss.str() << ",size:" << reals.size();
                break;
            } else {
                reals.clear();
            }
        }
    };

    bool optimizeEdge(std::shared_ptr<FrontierEdgeInfo>& frontier_edge) {
        auto curr_x = map_->coordX(frontier_edge->mean[0]);
        auto curr_y = map_->coordY(frontier_edge->mean[1]);
        int array[3] = {0, -1, 1};
        bool find_edge = false;
        EdgeInfo edge;
        for (int i = 0; i < 3; ++i) {
            auto real_x = curr_x + array[i];
            if (map_->inGrid(real_x, curr_y)) {
                auto& bar = map_->bar(real_x, curr_y);
                int real_index = 0;
                if (checkEdge(bar, frontier_edge->mean_index, frontier_edge->up_index, real_index)) {
                    if (checkPointThresh(real_x, curr_y, bar.pointByIndex(real_index), 2 * map_resolution_)) {
                        find_edge = true;
                        creatEdge(real_x, curr_y, real_index, frontier_edge->up_index, computeDeep(bar, real_index),
                                  bar, edge);
                        break;
                    }
                }
            }
        }
        if (find_edge) {
            std::vector<EdgeInfo> boundary;
            boundary.push_back(edge);
            auto virtual_edge = edge;
            virtual_edge.edge_index = frontier_edge->mean_index;
            searchAdjacentEdgeInYByDir(frontier_edge->mean, frontier_edge->direction, map_resolution_, virtual_edge,
                                       boundary);
            searchAdjacentEdgeInYByDir(frontier_edge->mean, frontier_edge->direction, -map_resolution_, virtual_edge,
                                       boundary);
            if (boundary.size() >= 10) {
                int delta_size = 100;
                int last_boundary_size = boundary.size();
                int iterater_count = 0;
                frontier_edge->computeEdgeInfo(boundary);
                while (delta_size > 5 || iterater_count++ < 3) {
                    virtual_edge.edge_index = frontier_edge->mean_index;

                    boundary.clear();
                    boundary.push_back(edge);
                    searchAdjacentEdgeInYByDir(frontier_edge->mean, frontier_edge->direction, -map_resolution_,
                                               virtual_edge, boundary);
                    searchAdjacentEdgeInYByDir(frontier_edge->mean, frontier_edge->direction, map_resolution_,
                                               virtual_edge, boundary);
                    frontier_edge->computeEdgeInfo(boundary);
                    delta_size = boundary.size() - last_boundary_size;
                    last_boundary_size = boundary.size();
                    if (iterater_count > 6) {
                        break;
                    }
                }
                return true;
            }
        }
        return false;
    }

    void eraseEdges(std::vector<EdgeInfo>& edges) {
        auto delta_x_index = 3;
        for (auto& edge : edges) {
            for (int j = -delta_x_index; j <= delta_x_index; ++j) {
                auto x = edge.x + j;
                auto y = edge.y;
                if (map_->inGrid(x, y)) {
                    auto& bar = map_->bar(x, y);
                    int index_size = bar.points_index.size();
                    for (int i = edge.edge_index; i < edge.up_index; ++i) {
                        if (i < index_size) {
                            bar.points_index[i] = 0;
                        }
                    }
                }
            }
        }
    }

    bool checkIsBoundary(std::vector<EdgeInfo>& edges, Eigen::Vector3f norm, float check_length) {
        int incre_step = 0;
        if (norm.dot(Eigen::Vector3f(1, 0, 0)) < 0.0) {
            norm = -norm;
        }
        norm.normalize();

        float x_step = map_resolution_;
        int incre_found_solid = 0;
        int incre_sum_count = 0;
        for (auto& edge : edges) {
            if (incre_step++ % 1 == 0) {  //每隔三个计算一次
                incre_sum_count++;
                float curr_x = 0.0f;
                while (curr_x < check_length) {
                    curr_x += x_step;
                    Eigen::Vector3f curr_point = edge.point + curr_x * norm;
                    int grid_x = map_->coordX(curr_point[0]);
                    int grid_y = map_->coordY(curr_point[1]);
                    if (map_->inGrid(grid_x, grid_y)) {
                        auto& bar = map_->bar(grid_x, grid_y);
                        if (getBarHoleRatio(bar, edge.edge_index, edge.up_index) < 0.8f) {
                            incre_found_solid++;
                            break;
                        }
                    }
                }
            }
        }
        if (float(incre_found_solid) / float(incre_sum_count) < 0.1) {
            return true;
        }
        return false;
    }

    float getBarHoleRatio(GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar, int down_index, int up_index) {
        int incre_solid = 0;
        int index_size = bar.points_index.size();
        if ((index_size - down_index) < neighbor_delta_height_index_) {
            return 1.0f;
        }
        for (int j = down_index + 1; j <= up_index; ++j) {
            if (j < index_size) {
                if (bar.points_index[j] > 0) {
                    incre_solid++;
                }
            }
        }
        return 1.0f - (float)incre_solid / float(up_index - down_index);
    }

    bool tranverseEdge(EdgeInfo& edge, std::vector<EdgeInfo>& boundary) {
        auto width = map_->width();
        boundary.push_back(edge);
        //沿着宽度方向向两边搜索
        int array[5] = {0, -1, 1, -2, 2};
        searchAdjacentHoleInY(edge.y, width, 1, edge.x, edge, boundary, 5, array);
        searchAdjacentHoleInY(edge.y, width, -1, edge.x, edge, boundary, 5, array);
        return boundary.size() >= 2;
    }

    void searchAdjacentHoleInY(int start_y, int width, int step, int start_center_x, const EdgeInfo& edge,
                               std::vector<EdgeInfo>& boundary, int search_size, int* array) {
        int unfounded = 0;
        int y = start_y;
        while (y < width && y >= 0) {
            y += step;
            unfounded++;
            EdgeInfo adja_edge;
            if (searchAdjacentHoleInX(start_center_x, y, edge, start_center_x, adja_edge, search_size, array)) {
                unfounded = 0;
                boundary.push_back(adja_edge);
            }
            if (unfounded >= 2) {  //连续性判定，如果连续两个点都找不到，那么就退出
                break;
            }
        }
    }

    bool searchAdjacentHoleInX(int up_center_x, int y, const EdgeInfo& edge, int& real_x, EdgeInfo& adj_edge,
                               int search_size, int* array) {
        //有时洞的搜索需要向里搜，防止栈板潜在上下两层栈板内侧，如果一直向外，容易出现搜索异常
        for (int i = 0; i < search_size; ++i) {
            auto x = up_center_x + array[i];
            if (map_->inGrid(x, y)) {
                auto& bar = map_->bar(x, y);
                int real_index;
                if (checkEdge(bar, edge.edge_index, edge.up_index, real_index)) {
                    if (checkPointThresh(x, y, bar.pointByIndex(real_index), 2 * map_resolution_)) {
                        creatEdge(x, y, real_index, edge.up_index, computeDeep(bar, real_index), bar, adj_edge);
                        real_x = x;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    bool checkEdge(GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar, int index, int up_index, int& real_index) {
        int delta_index = neighbor_delta_height_index_;
        bool is_same_height = false;
        int index_size = bar.points_index.size();
        if (index_size - index > neighbor_delta_height_index_) {
            for (int i = delta_index; i >= -delta_index; --i) {
                auto pos_index = index + i;
                if (pos_index < index_size - 1 && pos_index > 0) {
                    if (bar.points_index[pos_index] > 0 && bar.points_index[pos_index + 1] < 0) {
                        int incre_solid = 0;
                        for (int j = pos_index + 1; j <= up_index; ++j) {
                            if (j < index_size) {
                                if (bar.points_index[j] > 0) {
                                    incre_solid++;
                                }
                            }
                        }
                        if ((float)incre_solid / float(up_index - pos_index) < 0.3) {
                            is_same_height = true;
                            real_index = pos_index;
                            break;
                        }
                    }
                }
            }
            if (!is_same_height) {
                real_index = 200001;
            }
        } else if (index_size - index > -neighbor_delta_height_index_) {
            real_index = index_size - 1;
            if (bar.points_index[real_index] > 0) {
                is_same_height = true;
            }
            if (!is_same_height) {
                real_index = 100001;
            }
        } else {
            real_index = index_size - index;
        }
        if (is_same_height) {
            auto depth = computeDeep(bar, real_index);
            if (depth >= min_depth_index_) {
                return is_same_height;
            } else {
                real_index = 400001 + depth;
            }
        }
        return false;
    }

    bool findEdgesInBar(int x, int y, GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar, float min_delta_height,
                        std::vector<EdgeInfo>& edge_infos) {
        auto& bars = bar.points_index;
        auto size = bars.size();
        int up_index = -1;
        int down_index = -1;
        int min_i = map_->heightStep() * min_delta_height;
        edge_infos.clear();
        for (int i = 1; i < size - 1; ++i) {
            if (bars[i] * bars[i - 1] < 0 && bars[i + 1] * bars[i - 1] < 0) {
                if (bars[i - 1] > 0) {
                    down_index = i - 1;
                }
                if (bars[i] > 0 && bars[i + 1] > 0) {
                    up_index = i;
                    if (down_index > 0) {
                        int incre = 0;
                        for (int j = down_index + 1; j < up_index; ++j) {
                            if (bars[j] == -1) {
                                incre++;
                            }
                        }
                        auto delta_index = up_index - down_index;
                        if (float(incre) / (float)delta_index > 0.7f) {
                            if (delta_index > min_i) {
                                auto height =
                                    map_->toHeight(down_index);  //下边沿是要堆叠的平面。目前认为存在堆叠到洞里的情况
                                if (checkPointThresh(x, y, bar.pointByIndex(down_index), 2 * map_resolution_) &&
                                    height >= min_edge_height_) {
                                    auto depth = computeDeep(bar, down_index);
                                    if (depth >= min_depth_index_) {
                                        edge_infos.emplace_back();
                                        creatEdge(x, y, down_index, up_index, depth, bar, edge_infos.back());
                                    }
                                }
                                up_index = -1;
                                down_index = -1;
                            }
                        }
                    }
                }
            }
        }
        auto bar_end_index = size - 1;
        if (map_->toHeight(bar_end_index) >= min_edge_height_) {
            if (bar.points_index[bar_end_index] > 0) {
                auto depth = computeDeep(bar, down_index);
                if (depth >= min_depth_index_) {
                    if (checkPointThresh(x, y, bar.pointByIndex(bar_end_index), 2 * map_resolution_)) {
                        edge_infos.emplace_back();
                        creatEdge(x, y, bar_end_index, bar_end_index + map_->barIndex(min_delta_hole_height_), depth,
                                  bar, edge_infos.back());
                    }
                }
            }
        }
        return edge_infos.size();
    }

    int computeDeep(GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar, int down_index) {
        int deep = 0;
        int found = 0;
        for (int i = down_index; i >= 0; --i) {
            found++;
            if (bar.points_index[i] > 0) {
                deep++;
                found = 0;
            } else {
                if (found >= 2) {
                    return deep;
                }
            }
        }
        return deep;
    }

    void creatEdge(int x, int y, int down, int up, int deep, const GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar,
                   EdgeInfo& edge) {
        edge.edge_index = down;
        edge.up_index = up;
        edge.point = bar.pointByIndex(down);
        edge.x = x;
        edge.y = y;
        edge.depth = deep;
        edge.weight = 0.0f;
        for (int i = 0; i < down; ++i) {
            if (bar.points_index[i] > 0) {
                edge.weight += 1.0f;
            }
        }
    }

    bool checkPointThresh(int x, int y, const Eigen::Vector3f& up_point, float dist_thresh) {
        Eigen::Vector2f point(map_->toWorldX(x), map_->toWorldY(y));
        auto dist_1 = (up_point.head<2>() - point).norm();
        return dist_1 <= dist_thresh;
    }
    std::shared_ptr<GroundPlaneMap<Eigen::Vector3f>> map_;  //投影到地面的柱状地图，将高度信息变成直方柱图
    float map_resolution_ = 0.02f;
    float min_edge_height_ = 0.1f;  //栈板最小高度，相对高度（最大高度-最小高度）小于该值的，当做地面处理
    float min_delta_hole_height_ = 0.25f;  //小于这个洞的高度，推测是无法把栈板插进去的。
    int min_depth_index_ = 3;  // 最小深度值，该值是用来确定是否是横梁的,只有具有一定深度的横梁，才能够放货物
    int neighbor_delta_height_index_ = 3;  //相邻两个网格，如果高度值相差在3×resolution范围内，则认为高度相同
    float overlap_dist_thresh_ = 0.5f;//如果两个边缘重合很高，需要去掉边长小的
    float overlap_height_thresh_ = 0.15f;//如果重合度高两个边缘高度差大于15cm，则保留两个边缘
};
}  // namespace standard

#endif  // LIVOX_PALLET_DETECT_PICKPLACE_FRONTIER_EDGE_DETECT_HPP
