//
// Created by lfc on 2022/5/10.
//

#ifndef LIVOX_PALLET_DETECT_STANDARD_PALLET_DETECT_HPP
#define LIVOX_PALLET_DETECT_STANDARD_PALLET_DETECT_HPP
#include <Eigen/Dense>
#include "ground_plane_map.hpp"

namespace standard {
//查找到的洞信息，对于洞而言，从下到上中间是镂空的；对于平面而言，从下到上是实心的，本程序通过该方法区分平面和洞
struct HoleInfo {
    int up_index;                //洞的上边沿的高度索引
    int down_index;              //洞的下边沿的高度索引
    int mid_index;               //洞的中心位置的索引
    int x;                       //投影到地面的X坐标
    int y;                       //投影到地面的Y坐标
    Eigen::Vector3f up_point;    //上边沿高度索引对应的实际三维位置
    Eigen::Vector3f down_point;  //下边沿高度索引对应的实际三维位置
};
//所有洞点连接在一起形成了边界，这里边界只有上下边界点
struct BorderInfo {
    /// 用于把计算的到的边缘点转化成洞的边界，包含排序/高度计算/宽度计算/方向计算
    /// \param border
    void computeBorderInfo(std::vector<HoleInfo>& border) {
        std::sort(border.begin(), border.end(),
                  [](const HoleInfo& first, const HoleInfo& second) { return first.y < second.y; });
        borders = border;
        direction = Eigen::Vector3f::Zero();
        mean = Eigen::Vector3f::Zero();
        float mean_hole_height;
        up_index = 0;
        down_index = 0;
        for (auto& bo : border) {
            direction += bo.up_point - border.front().up_point;
            //            direction += bo.down_point - border.front().down_point;
            direction[2] = 0;
            //            mean += bo.down_point + bo.up_point;
            //由于down_point存在
            mean += bo.up_point;

//            LOG(INFO) << "bo.up_point: (" << bo.up_point.x() << ", " << bo.up_point.y() << ", " << bo.up_point.z()
//                      << "), bo.down_point: (" << bo.down_point.x() << ", " << bo.down_point.y() << ", "
//                      << bo.down_point.z() << ")";
            hole_height_in_grid += bo.up_index - bo.down_index;
            hole_height_in_world += bo.up_point[2] - bo.down_point[2];
            up_index += bo.up_index;
            down_index += bo.down_index;
        }
        //        mean = mean / (border.size() * 2.0);
        mean = mean / border.size();
        LOG(INFO) << "The mean point of upper edge of hole is:(" << mean[0] << ", " << mean[1] << ", " << mean[2] << "）";
//        LOG(INFO) << "border.size:" << border.size();
        hole_height_in_grid = hole_height_in_grid / (border.size());
        hole_height_in_world = hole_height_in_world / (border.size());
        up_index = roundf(up_index / (border.size()));
        down_index = floorf(down_index / (border.size()));
        direction.normalize();
        std::vector<Eigen::Vector3f> points;
        for (auto& bor : borders) {
            //            LOG(INFO)<<"border.x:"<<border.x;
            //            LOG(INFO)<<"border.y:"<<border.y;
            points.push_back(bor.up_point);
            //            points.push_back(bor.down_point);
        }
        length = (border.back().up_point - border.front().up_point).norm();
        //        length += (border.back().down_point - border.front().down_point).norm();
        //        length *= 0.5f;
        LOG(INFO) << "The length of the up hole border:" << length;
        float weight = 0.0;
        Eigen::Vector3f norm_f;
        LOG(INFO) << "The direction of upper edge:(" << direction.x() << ", " << direction.y() << ", " << direction.z() << ")";
        NormComputer::buildNormByTwoVector(Eigen::Vector3f(0, 0, 1), direction, norm_f);
        NormComputer::buildNormByCross(mean, points, norm_f, norm_f, weight);
        norm = norm_f;
    }
    std::vector<HoleInfo> borders;  //所有边界点
    Eigen::Vector3f mean;           //洞的中心位置
    Eigen::Vector3f direction;      //洞的中心朝向，是洞延伸的方向
    Eigen::Vector3f norm;           //洞的发现朝向，是洞平面的法线
    float length{0.};               //边界的长度
    float hole_height_in_grid;      //
    float hole_height_in_world;     //边界的实际高度
    float up_index;                 //边界平均的上沿索引
    float down_index;               //边界平均的下沿索引
};

struct PalletInfo {
    std::shared_ptr<BorderInfo> first;            //第一个洞的边界
    std::shared_ptr<BorderInfo> second;           //第二个洞的边界
    std::vector<Eigen::Vector3f> surface_points;  //中间墩的所有点集
    Eigen::Vector3f mean;                         //栈板中心位置
    Eigen::Vector3f norm;                         //栈板中心朝向
    float length;                                 //栈板中间墩的长度
    float height;                                 //栈板中间的高度，该高度实际为洞的高度
};

class StandardPalletDetect {
 public:
    StandardPalletDetect() {}

    bool tranverseGridForPallet(std::shared_ptr<GroundPlaneMap<Eigen::Vector3f>>& map,
                                std::vector<PalletInfo>& pallets) {
        map_ = map;
        map_resolution_ = map_->resolution();
        auto width = map_->width();
        auto length = map_->length();
        int x_step = 1;
        int y_step = 1;
        //        LOG(INFO) << "The map_ width is: " << width << ", length is: " << length << ". ";
        for (int x = 0; x < length; x += x_step) {
            //宽度优先搜索，我们认为，第一个找到的洞大概率是栈板的前沿，这种方法可快速找到满足要求的栈板
            for (int y = 0; y < width; y += y_step) {
                auto& bar = map_->bar(x, y);
                //判断相对高度是否满足要求
                if (bar.heightEnough(min_pallet_height_)) {
                    std::vector<HoleInfo> bar_hole_infos;
                    //先找到所有点集中第一个洞，这里是把每个网格沿着高度方向上所有满足的洞都找到
                    if (findHolesInBar(x, y, bar, min_hole_height_, max_hole_height_, bar_hole_infos)) {
                        //                        LOG(INFO) << "find holes, bar_hole_infos.size: " <<
                        //                        bar_hole_infos.size(); for(auto hole:bar_hole_infos){
                        //                            find_edges_.push_back(hole);
                        //                            LOG(INFO)<<"edges:( "<<hole.x<<", "<<hole.y<<",
                        //                            "<<map_->toHeight(hole.up_index)<<")";
                        //                        }
                        searchPalletFromHoles(bar_hole_infos, pallets);
                    }
                }
            }
        }
        LOG(INFO)<<"------------------ Pallet detection completed!------------------";
        return pallets.size();
    }

 private:
    bool searchPalletFromHoles(std::vector<HoleInfo>& bar_hole_infos, std::vector<PalletInfo>& pallets) {
        for (auto& hole : bar_hole_infos) {
            //            Eigen::Vector3f mid_point = (hole.up_point + hole.down_point) * 0.5f;
            //            LOG(INFO)<<"search holes, mid_point:("<<mid_point[0]<<", "<<mid_point[1]<<",
            //            "<<mid_point[2]<<")";
            // 加入调试接口

            //                find_edges_.push_back(hole);

            std::vector<HoleInfo> border;
            tranverseBorder(hole, border);  //对于满足要求的洞，要做连续化搜索，找到洞的连续边界
            if (border.size() > min_border_hole_pixel_size_ &&
                border.size() < max_border_hole_pixel_size_) {  //如果连续边界大于一定值，则认为是找到一个满足要求的洞
                std::sort(border.begin(), border.end(),
                          [](const HoleInfo& first, const HoleInfo& second) { return first.y < second.y; });

                // 加入调试接口
                for (const auto& bor : border) {
                    find_edges_.push_back(bor);
                }

                std::vector<HoleInfo> start_surfaces, end_surfaces;
                //对于满足要求的洞，需要寻找洞的左右边缘的平面，平面也可以用实心的洞表示，所以两者结构体构成相同
                if (tranverseSurface(border.front(), border.back(), start_surfaces, end_surfaces)) {
                    //                    //调试接口
                    for (auto surface : start_surfaces) {
                        find_surfaces_.push_back(surface);
                    }
                    for (auto surface : end_surfaces) {
                        find_surfaces_.push_back(surface);
                    }

                    HoleInfo start_info, end_info;
                    Eigen::Vector3f start_mean, end_mean;
                    //计算两个平面的中心位置
                    if (computeMeanSurface(start_surfaces, start_info, start_mean) &&
                        computeMeanSurface(end_surfaces, end_info, end_mean)) {
                        std::vector<Eigen::Vector3f> third_surfaces;
                        std::vector<PalletInfo> mult_pallets;
                        //利用两个平面的中心位置，推断出第三个平面的位置，注意，这里有可能存在栈板并排的情况，需要外部加以区分，这里所有满足（洞--墩--洞）的栈板都会被找到
                        if (findThirdSurface(hole, start_mean, end_mean, third_surfaces)) {
                            for (auto& third_surface : third_surfaces) {
                                PalletInfo pallet;
                                //如果找到第三个墩了，那么就可以认为大概率是栈板，可以进入check和优化环节了
                                if (checkPallet(hole, start_mean, end_mean, third_surface, pallet)) {
                                    mult_pallets.push_back(pallet);
                                }
                            }
                            LOG(INFO) << "find mult_pallets size: " << mult_pallets.size();
                            for (auto& pallet : mult_pallets) {
                                //把当前栈板所在区域进行清空，防止后面再次进入搜索
                                erasePalletBar(pallet);
                                //单帧中找到的左右栈板都会存在这里
                                pallets.push_back(pallet);
                                //以下是调试接口，不用理会
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    bool checkPallet(HoleInfo& hole, Eigen::Vector3f& start_mean, Eigen::Vector3f& end_mean,
                     Eigen::Vector3f& third_mean, PalletInfo& pallet) {
        std::vector<HoleInfo> first_holes, second_holes;
        HoleInfo first_hole, second_hole;
        LOG(INFO) << "ready to check pallet";
        recomputeHoleInfo(hole, start_mean, end_mean, first_hole, first_holes);
        if ((end_mean - start_mean).dot(third_mean - start_mean) > 0) {
            Eigen::Vector3f rep_mean = start_mean;
            start_mean = end_mean;
            end_mean = rep_mean;
        }
        recomputeHoleInfo(hole, start_mean, third_mean, second_hole, second_holes);
        LOG(INFO) << "first_holes.size:" << first_holes.size() << ", second_holes：" << second_holes.size();

        // 调试接口
        for (const auto& tmp_hole : second_holes) {
            //            LOG(INFO) << "second_holes: x:" << tmp_hole.x << ", y: " << tmp_hole.y << ", to world:("
            //                      << map_->toWorldX(tmp_hole.x) << ", " << map_->toWorldY(tmp_hole.y) << ")"
            //                      << ", up_point: (" << tmp_hole.up_point.x() << ", " << tmp_hole.up_point.y() << ", "
            //                      << tmp_hole.up_point.z() << ")";
            find_holes_.push_back(tmp_hole);
        }

        if (first_holes.size() >= min_border_hole_pixel_size_ - 2 &&
            second_holes.size() >= min_border_hole_pixel_size_ - 2) {
            std::shared_ptr<BorderInfo> first_border_ptr(new BorderInfo);
            std::shared_ptr<BorderInfo> second_border_ptr(new BorderInfo);
            first_border_ptr->computeBorderInfo(first_holes);
            second_border_ptr->computeBorderInfo(second_holes);
            //            LOG(INFO) << "first_border_ptr->hole_height_in_world:" <<
            //            first_border_ptr->hole_height_in_world
            //                      << ", second_border_ptr->hole_height_in_world:" <<
            //                      second_border_ptr->hole_height_in_world;
            if (fabs(first_border_ptr->hole_height_in_world - second_border_ptr->hole_height_in_world) <
                pallet_hole_height_err_) {
                pallet.first = first_border_ptr;
                pallet.second = second_border_ptr;
                computePalletCenterInfo(first_hole, start_mean, end_mean, third_mean, pallet);
                return true;
            }
        }
        return false;
    }

    void erasePalletBar(PalletInfo& pallet) {
        int first_max_y = pallet.first->borders.back().y;
        int second_max_y = pallet.second->borders.back().y;
        Eigen::Vector3f end_point, start_point;

        if (first_max_y > second_max_y) {
            end_point = pallet.first->borders.back().up_point;
            start_point = pallet.second->borders.front().up_point;
        } else {
            end_point = pallet.second->borders.back().up_point;
            start_point = pallet.first->borders.front().up_point;
        }
        start_point = start_point - 0.1 * pallet.norm;
        end_point = end_point - 0.1 * pallet.norm;
        float remove_length = 2.0;
        Eigen::Vector3f back_end_point = end_point + remove_length * pallet.norm;
        Eigen::Vector3f back_start_point = start_point + remove_length * pallet.norm;

        Eigen::Vector3f delta_vector = end_point - start_point;
        auto remove_width = delta_vector.head<2>().norm();
        delta_vector.normalize();
        float x_incre = 0.0f, y_incre = 0.0f;
        while (x_incre < remove_length) {
            Eigen::Vector3f real_start = start_point + x_incre * pallet.norm;
            x_incre += map_resolution_;
            y_incre = 0.0f;
            while (y_incre < remove_width) {
                Eigen::Vector3f curr_real = real_start + y_incre * delta_vector;
                int x = map_->coordX(curr_real[0]);
                int y = map_->coordY(curr_real[1]);
                if (map_->inGrid(x, y)) {
                    deleteSelectedBar(pallet.first->up_index, pallet.first->down_index, map_->bar(x, y));
                }
                y_incre += map_resolution_;
            }
        }
    }

    void deleteSelectedBar(int up, int down, GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar) {
        if (down <= 0) {
            down = 0;
        }
        auto size = bar.points_index.size();
        for (int i = down; i <= up; ++i) {
            if (i < size) {
                if (bar.points_index[i] < 0) {
                    bar.points_index[i] = 0;
                }
            }
        }
    }

    bool isSurface(GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar, int up, int down, int& real_up, int& real_down) {
        if (down < 0) {
            down = 0;
        }
        real_up = up, real_down = down;
        auto points_size = bar.points_index.size();
        if (points_size <= real_up || real_up < real_down + 2) {
            return false;
        }
        if (bar.points_index[real_up] == -1) {
            real_up--;
            if (bar.points_index[real_up] == -1) {
                return false;
            }
        }
        if (bar.points_index[real_down] == -1) {
            real_down++;
            if (points_size <= real_down || bar.points_index[real_down] == -1) {
                return false;
            }
        }
        //沿着z方向向下生长
        auto lowest_height = real_up - static_cast<int>(max_pallet_height_ / map_resolution_);
        lowest_height = lowest_height > 0 ? lowest_height : 0;
        for (size_t i = down; i > lowest_height + 1; --i) {
            //是否需要考虑==0，即未初始化的情况?
            if (bar.points_index[i] <= 0 /*&& bar.points_index[i - 1] <= 0*/) {
                break;
            }
            real_down = i;
        }
        float incre = 0;
        for (int j = real_down; j < real_up; ++j) {
            incre = bar.points_index[j] > 0 ? incre + 1 : incre;
        }
        //        LOG(INFO) << "find surfaces, real_up: " << real_up << ", real_down: " << real_down << ", down: " <<
        //        down
        //                  << ", incre: " << incre << ", percentage: " << incre / (real_up - real_down) * 100 << "%.";
        return (incre / static_cast<float>(real_up - real_down) >= 0.8);
    }

    bool tranverseBorder(HoleInfo& hole, std::vector<HoleInfo>& border) {
        auto up_x = hole.x;
        auto up_y = hole.y;
        int up_center_x = up_x;
        auto width = map_->width();
        border.push_back(hole);
        //沿着宽度方向向两边搜索
        searchAdjacentHoleInY(up_y, width, 1, up_center_x, hole, border);
        searchAdjacentHoleInY(up_y, width, -1, up_center_x, hole, border);
        return border.size() >= 2;
    }

    bool findThirdSurface(HoleInfo& cand_hole, Eigen::Vector3f& first_mean, Eigen::Vector3f& second_mean,
                          std::vector<Eigen::Vector3f>& third_means) {
        auto checkThirdSurface = [&](const Eigen::Vector3f& third_mean, Eigen::Vector3f& opt_third_mean) {
            auto third_mean_x = map_->coordX(third_mean[0]);
            auto third_mean_y = map_->coordY(third_mean[1]);
            std::vector<HoleInfo> surfaces;
            //在当前位置中心向左右各搜索一个网格，进行遍历
            for (int i = -1; i <= 1; ++i) {
                checkNeiborSurface(third_mean_x, third_mean_y + i, -2, 2, cand_hole.up_index, cand_hole.down_index,
                                   true, surfaces);
            }
            if (!surfaces.empty()) {
                opt_third_mean.setZero();
                int incre_sum = 0;
                for (auto& sur : surfaces) {
                    for (int i = sur.down_index; i <= sur.up_index; ++i) {
                        auto& bar = map_->bar(sur.x, sur.y);
                        if (bar.points_index.size() > i) {
                            if (bar.points_index[i] != -1 &&
                                checkPointThresh(sur.x, sur.y, bar.pointByIndex(i), 4 * map_resolution_)) {
                                opt_third_mean += bar.pointByIndex(i);
                                incre_sum++;
                            }
                        }
                    }
                }
                if (incre_sum != 0) {
                    //计算均值
                    opt_third_mean = opt_third_mean / (float(incre_sum));
                    return true;
                }
            }
            return false;
        };
        //如果是栈板，那么一定是对称的，利用这种特性可快速定位第三个墩的中心位置
        auto third_mean = 2 * first_mean - second_mean;
        auto forth_mean = 2 * second_mean - first_mean;
        Eigen::Vector3f opt_third_mean;
        if (checkThirdSurface(third_mean, opt_third_mean)) {
            third_means.push_back(opt_third_mean);

            //            //调试接口
            //            HoleInfo third_pier_mean;
            //            third_pier_mean.x = map_->coordX(opt_third_mean[0]);
            //            third_pier_mean.y = map_->coordY(opt_third_mean[1]);
            //            third_pier_mean.up_index = map_->barIndex(opt_third_mean[2]);
            //            third_pier_mean.up_point = opt_third_mean;
            //            find_third_pier_mean_.push_back(third_pier_mean);
        }
        if (checkThirdSurface(forth_mean, opt_third_mean)) {
            LOG(INFO) << "find the third pallet mean: (" << opt_third_mean[0] << ", " << opt_third_mean[1] << ", "
                      << opt_third_mean[2] << ").";
            third_means.push_back(opt_third_mean);

            //调试接口
            HoleInfo third_pier_mean;
            third_pier_mean.x = map_->coordX(opt_third_mean[0]);
            third_pier_mean.y = map_->coordY(opt_third_mean[1]);
            third_pier_mean.up_index = map_->barIndex(opt_third_mean[2]);
            third_pier_mean.up_point = opt_third_mean;
            find_third_pier_mean_.push_back(third_pier_mean);

            // 调试接口，显示第一个墩和第二个墩的中心点
            third_pier_mean.x = map_->coordX(first_mean[0]);
            third_pier_mean.y = map_->coordY(first_mean[1]);
            third_pier_mean.up_index = map_->barIndex(first_mean[2]);
            third_pier_mean.up_point = first_mean;
            find_third_pier_.push_back(third_pier_mean);

            third_pier_mean.x = map_->coordX(second_mean[0]);
            third_pier_mean.y = map_->coordY(second_mean[1]);
            third_pier_mean.up_index = map_->barIndex(second_mean[2]);
            third_pier_mean.up_point = second_mean;
            find_third_pier_.push_back(third_pier_mean);
        }
        return !third_means.empty();
    }

    bool computePalletCenterInfo(const HoleInfo& cand_hole, Eigen::Vector3f center_mean, Eigen::Vector3f start_mean,
                                 Eigen::Vector3f end_mean, PalletInfo& pallet) {
        Eigen::Vector3f direction = end_mean - start_mean;
        direction.normalize();
        std::vector<Eigen::Vector3f> center_points;
        computeCenterSurface(cand_hole.up_index, cand_hole.down_index, center_mean, direction, pallet.length,
                             pallet.height, center_points);
        Eigen::Vector3f curr_center_mean = Eigen::Vector3f::Zero();
        int incre = 0;
        for (auto& point : center_points) {
            curr_center_mean += point;
            incre++;
        }
        curr_center_mean = curr_center_mean / incre;
        Eigen::Vector3f norm;
        float weight = 0.0;
        NormComputer::buildNormByTwoVector(Eigen::Vector3f(0, 0, 1), direction, norm);
        NormComputer::buildNormByCross(curr_center_mean, center_points, norm, norm, weight);
        pallet.norm = norm;
        pallet.mean = curr_center_mean;
        LOG(INFO) << "norm:(" << norm[0] << "," << norm[1] << "," << norm[2] << ")";
        LOG(INFO) << "current center mean:(" << curr_center_mean[0] << "," << curr_center_mean[1] << ","
                  << curr_center_mean[2] << ")"
                  << ", point size: " << center_points.size() << ", length: " << pallet.length
                  << ", height: " << pallet.height;
        pallet.surface_points = center_points;
        return true;
    }

    bool checkNeiborSurface(int mean_x, int mean_y, int min_delta_x, int max_delta_x, int up_index, int down_index,
                            bool return_all, std::vector<HoleInfo>& surfaces) {
        bool get_surface = false;
        //一般平面因为观测问题是有厚度的，这里是沿着厚度方向做遍历，遍历前后2cm
        for (int i = min_delta_x; i <= max_delta_x; ++i) {
            auto curr_x = mean_x + i;
            if (map_->inGrid(curr_x, mean_y)) {
                HoleInfo adja_hole;
                auto& bar = map_->bar(curr_x, mean_y);
                int real_up, real_down;
                if (isSurface(bar, up_index, down_index, real_up, real_down)) {
                    if (checkPointThresh(curr_x, mean_y, bar.points[bar.points_index[real_up]],
                                         bar.points[bar.points_index[real_down]], 4 * map_resolution_)) {
                        creatHole(curr_x, mean_y, real_up, real_down, bar, adja_hole);
                        surfaces.push_back(adja_hole);
                        get_surface = true;
                        if (!return_all) {
                            return true;
                        }
                    }
                }
            }
        }
        return get_surface;
    }

    bool computeCenterSurface(int up_index, int down_index, const Eigen::Vector3f& center_mean,
                              const Eigen::Vector3f& direction, float& length, float& height,
                              std::vector<Eigen::Vector3f>& points) {
        auto computeLengthAndHeight = [&](std::vector<HoleInfo>& surfaces, float& length, float& height) {
            std::sort(surfaces.begin(), surfaces.end(),
                      [](const HoleInfo& first, const HoleInfo& second) { return first.y < second.y; });
            Eigen::Vector3f start_mean = Eigen::Vector3f::Zero(), end_mean = Eigen::Vector3f::Zero();
            int start_incre = 0, end_incre = 0;
            auto surface = surfaces.front();
            for (int i = surface.down_index; i <= surface.up_index; ++i) {
                auto& bar = map_->bar(surface.x, surface.y);
                if (bar.points_index[i] != -1 &&
                    checkPointThresh(surface.x, surface.y, bar.pointByIndex(i), 2 * map_resolution_)) {
                    start_mean += bar.pointByIndex(i);
                    start_incre++;
                }
            }
            surface = surfaces.back();
            for (int i = surface.down_index; i <= surface.up_index; ++i) {
                auto& bar = map_->bar(surface.x, surface.y);
                if (bar.points_index[i] != -1 &&
                    checkPointThresh(surface.x, surface.y, bar.pointByIndex(i), 2 * map_resolution_)) {
                    end_mean += bar.pointByIndex(i);
                    end_incre++;
                }
            }
            if (start_incre > 0 && end_incre > 0) {
                length = (start_mean / start_incre - end_mean / end_incre).head<2>().norm();
            } else {
                return false;
            }
            height = 0;
            for (auto& surface : surfaces) {
                height = height > surface.up_point.z() - surface.down_point.z()
                             ? height
                             : surface.up_point.z() - surface.down_point.z();
            }
            return true;
        };

        auto searchAdjacentSurfaceInYByDir = [&](const Eigen::Vector3f& mean, const Eigen::Vector3f& direction,
                                                 float step, int up_index, int down_index,
                                                 std::vector<HoleInfo>& border) {
            int unfounded = 0;
            int y = map_->coordY(mean[1]);
            int i = 0;
            int last_y = 0;
            while (y < map_->width() && y >= 0) {
                i++;
                auto curr_coord = mean + direction * step * i;
                y = map_->coordY(curr_coord[1]);
                if (last_y == y) {
                    continue;
                }
                last_y = y;
                auto curr_x = map_->coordX(curr_coord[0]);
                unfounded++;
                //对于平面而言，存在因为计算误差导致中心点向里收的情况，需要利用传感器特性，朝着传感器方向多搜索一个网格，所以是(-2,1)
                if (checkNeiborSurface(curr_x, y, -2, 1, up_index, down_index, false, border)) {
                    unfounded = 0;
                }
                if (unfounded >= 2) {
                    break;
                }
            }
        };
        std::vector<HoleInfo> surfaces;
        auto mean_x = map_->coordX(center_mean[0]);
        auto mean_y = map_->coordY(center_mean[1]);
        checkNeiborSurface(mean_x, mean_y, -1, 1, up_index, down_index, true, surfaces);
        if (!surfaces.empty()) {
            searchAdjacentSurfaceInYByDir(center_mean, direction, map_resolution_, up_index, down_index, surfaces);
            searchAdjacentSurfaceInYByDir(center_mean, direction, -map_resolution_, up_index, down_index, surfaces);
            points.clear();
            computeLengthAndHeight(surfaces, length, height);
            for (auto& surface : surfaces) {
                for (int i = surface.down_index; i <= surface.up_index; ++i) {
                    auto& bar = map_->bar(surface.x, surface.y);
                    if (bar.points_index[i] != -1 &&
                        checkPointThresh(surface.x, surface.y, bar.pointByIndex(i), 2 * map_resolution_)) {
                        points.push_back(bar.pointByIndex(i));
                    }
                }
            }
            return true;
        }
        return false;
    }

    bool checkHoleIsHole(HoleInfo& hole) {
        int check_x = hole.x + 1;
        int check_y = hole.y;
        if (map_->inGrid(check_x, check_y)) {
            auto& bar = map_->bar(check_x, check_y);
            int real_up, real_down;
            if (!isSurface(bar, hole.up_index, hole.down_index, real_up, real_down)) {
                return true;
            }
        }
        return false;
    }

    bool recomputeHoleInfo(const HoleInfo& cand_hole, Eigen::Vector3f first_mean, Eigen::Vector3f second_mean,
                           HoleInfo& hole, std::vector<HoleInfo>& holes) {
        //优先搜索使用向量计算出的网格，如果不是洞，需要向外先计算一个网格，如果不是，那就向里计算一个网格
        int array_index[9] = {0, -1, 1, -2, 2, -3, 3, -4, 4};

        auto searchAdjacentHoleInYByDir = [&](Eigen::Vector3f& mean, Eigen::Vector3f& direction, float step,
                                              const HoleInfo& hole, std::vector<HoleInfo>& border) {
            int unfounded = 0;
            int y = map_->coordY(mean[1]);
            int i = 0;
//                        LOG(INFO) << "searchAdjacentHoleInYByDir: the hole point. up_index: " << hole.up_index
//                                  << ", down_index: " << hole.down_index;
            while (y < map_->width() && y >= 0) {
                i++;
                auto curr_coord = mean + direction * step * i;
                y = map_->coordY(curr_coord[1]);
                unfounded++;
                for (int j = 0; j <= 6; ++j) {
                    auto x = map_->coordX(curr_coord[0]) + array_index[j];
                    if (map_->inGrid(x, y)) {
                        HoleInfo adja_hole;
                        auto& bar = map_->bar(x, y);
                        int real_up, real_down;
                        if (checkHole(bar, hole.up_index, hole.down_index, real_up, real_down)) {
//                                                        LOG(INFO)<<"check holes here.";

                            //                            if (checkPointThresh(x, y,
                            //                            bar.points[bar.points_index[real_up]],
                            //                                                 bar.points[bar.points_index[real_down]],
                            //                                                 4 * map_resolution_)) {
                            creatHole(x, y, real_up, real_down, bar, adja_hole);
                            //                                LOG(INFO)<<"find holes here.";
                            if (checkHoleIsHole(adja_hole)) {
//                                                                    LOG(INFO)<<"checkHoleIsHole here.";
                                unfounded = 0;
                                border.push_back(adja_hole);
                                break;
                            }
                            //                            }
                        }
                    } else {
                        break;
                    }
                }
                if (unfounded >= 4) {
                    break;
                }
            }
        };

        Eigen::Vector3f hole_mean = (first_mean + second_mean) * 0.5;

        int mid_index = roundf((cand_hole.up_index + cand_hole.down_index) * 0.5f);
        int up_index = mid_index, down_index = mid_index;
        bool find_up_index = false, find_down_index = false;
        for (int i = 0; i <= 4; ++i) {
            auto curr_y = map_->coordY(hole_mean[1]);
            auto curr_x = map_->coordX(hole_mean[0]) + array_index[i];
            auto& bar = map_->bar(curr_x, curr_y);
            while (up_index < bar.points_index.size()) {
                up_index++;
                if (bar.points_index[up_index - 1] * bar.points_index[up_index] <= 0) {
                    if (bar.points_index[up_index] >= 0) {
                        find_up_index = true;
                        break;
                    }
                }
            }

            if (find_up_index /*&& find_down_index*/) {
//                LOG(INFO) << "recomputeHoleInfo: find the first hole point. up_index: " << up_index << ","
//                          << "down_index: " << down_index << ", point:( " << bar.pointByIndex(down_index).x() << ", "
//                          << bar.pointByIndex(down_index).y() << ", " << bar.pointByIndex(down_index).z() << ")";
                //                LOG(INFO) << "pointIndex: "<<bar.points_index[down_index];
                creatHole(map_->coordX(hole_mean[0]), map_->coordY(hole_mean[1]), up_index, down_index, bar, hole);
                holes.clear();
                holes.push_back(hole);

                //                // 调试接口，显示孔洞点上表面点
                //                find_holes_.push_back(hole);

                Eigen::Vector3f line_vector = second_mean - first_mean;
                line_vector.normalize();
                line_vector = first_mean - hole_mean;
                line_vector.normalize();
                searchAdjacentHoleInYByDir(hole_mean, line_vector, map_resolution_, hole, holes);
                line_vector = second_mean - hole_mean;
                line_vector.normalize();
                searchAdjacentHoleInYByDir(hole_mean, line_vector, map_resolution_, hole, holes);
                return holes.size() >= 2;
            }
        }

        return false;
    }

    bool computeMeanSurface(std::vector<HoleInfo>& surface, HoleInfo& center_surface, Eigen::Vector3f& mean) {
        std::sort(surface.begin(), surface.end(),
                  [](const HoleInfo& first, const HoleInfo& second) { return first.y < second.y; });
        int center_y = roundf((surface.front().y + surface.back().y) * 0.5f);
        std::vector<HoleInfo> centers;
        for (auto& bar : surface) {
            if (abs(bar.y - center_y) <= 1) {
                centers.push_back(bar);
            }
        }

        if (centers.size()) {
            mean.setZero();
            int incre_sum = 0;
            for (auto& sur : centers) {
                for (int i = sur.down_index; i <= sur.up_index; ++i) {
                    if (i > 0) {
                        auto& bar = map_->bar(sur.x, sur.y);
                        if (bar.points_index[i] != -1 &&
                            checkPointThresh(sur.x, sur.y, bar.pointByIndex(i), 2 * map_resolution_)) {
                            mean += bar.pointByIndex(i);
                            incre_sum++;
                        }
                    }
                }
            }
            mean = mean / (float(incre_sum));
            auto mean_x = map_->coordX(mean[0]);
            auto mean_y = map_->coordY(mean[1]);
            if (map_->inGrid(mean_x, mean_y)) {
                auto& bar = map_->bar(mean_x, mean_y);
                int real_up, real_down;
                if (isSurface(bar, centers.back().up_index, centers.back().down_index, real_up, real_down)) {
                    creatHole(mean_x, mean_y, real_up, real_down, bar, center_surface);
                    return true;
                }
            }
        }
        return false;
    }

    bool tranverseSurface(HoleInfo& start_hole, HoleInfo& end_hole, std::vector<HoleInfo>& start_surfaces,
                          std::vector<HoleInfo>& end_surfaces) {
        auto width = map_->width();
        auto findFirstSurface = [&](HoleInfo& curr_hole, int direction, int search_offset, HoleInfo& hole) {
            //            int real_y, real_x;
            for (int i = 0; i < search_offset; ++i) {
                int y = curr_hole.y + i * direction;
                for (int j = -search_offset; j < search_offset; ++j) {
                    int x = curr_hole.x + j;
                    if (map_->inGrid(x, y)) {
                        auto& bar = map_->bar(x, y);
                        int real_up, real_down;
                        if (isSurface(bar, curr_hole.up_index, curr_hole.down_index, real_up, real_down)) {
                            creatHole(x, y, real_up, real_down, bar, hole);
                            return true;
                        }
                    }
                }
            }
            return false;
        };
        auto cur_hole_width = std::abs(end_hole.y - start_hole.y);
        HoleInfo first_surface;
        //这里主要是担心我们的洞找的有问题，所以才使用了非常大范围搜索平面，目的是提升鲁棒性
        if (findFirstSurface(end_hole, 1, surface_initial_search_offset_, first_surface)) {
            end_surfaces.push_back(first_surface);
            searchAdjacentSurfaceInY(first_surface, width, 1, cur_hole_width, end_surfaces);
        }
        if (findFirstSurface(start_hole, -1, surface_initial_search_offset_, first_surface)) {
            start_surfaces.push_back(first_surface);
            searchAdjacentSurfaceInY(first_surface, width, -1, cur_hole_width, start_surfaces);
        }
        return start_surfaces.size() >= 2 && end_surfaces.size() >= 2;
    }

    void searchAdjacentHoleInY(int start_y, int width, int step, int start_center_x, const HoleInfo& hole,
                               std::vector<HoleInfo>& border) {
        int unfounded = 0;
        int y = start_y;

        while (y < width && y >= 0) {
            y += step;
            unfounded++;
            HoleInfo adja_hole;
            if (searchAdjacentHoleInX(start_center_x, y, hole, start_center_x, adja_hole)) {
                unfounded = 0;
                border.push_back(adja_hole);
            }
            if (unfounded >= 4) {  //连续性判定，如果连续4个点都找不到，那么就退出
                break;
            }
        }
    }

    void searchAdjacentSurfaceInY(const HoleInfo& hole, int width, int step, int cur_hole_width,
                                  std::vector<HoleInfo>& surfaces) {
        int unfounded = 0;
        int y = hole.y;
        int start_center_x = hole.x;
        std::vector<HoleInfo> tmp_surfaces;
        while (y < width && y >= 0) {
            y += step;
            unfounded++;
            std::vector<HoleInfo> adja_surface;
            if (searchAdjacentSurfaceInX(start_center_x, y, hole, start_center_x, adja_surface)) {
                unfounded = 0;
                //                //只有宽度小于一定阈值的才认为是墩
                //                if (adja_surface.size()<max_pier_width_pixel_size){
                tmp_surfaces.insert(tmp_surfaces.end(), adja_surface.begin(), adja_surface.end());
                //                }
            }
            if (unfounded >= 4) {  //连续性判定，如果连续两个点都找不到，那么就退出
                break;
            }
        }
        auto max_pier_width_pixel = cur_hole_width;
        auto min_pier_width_pixel = cur_hole_width / 4.0;  //墩的最小宽度应为孔的宽度的1/4.
        if (!tmp_surfaces.empty()) {
            auto cur_pier_width_pixel = std::abs(tmp_surfaces.back().y - tmp_surfaces.front().y);

            // 限制墩的宽度最大不能超过孔的宽度的两倍
            if (cur_pier_width_pixel < 2 * max_pier_width_pixel && cur_pier_width_pixel > min_pier_width_pixel) {
                surfaces.insert(surfaces.end(), tmp_surfaces.begin(), tmp_surfaces.end());
            }
        }
    }

    bool searchAdjacentHoleInX(int up_center_x, int y, const HoleInfo& hole, int& real_x, HoleInfo& adj_hole) {
        //有时洞的搜索需要向里搜，防止栈板潜在上下两层栈板内侧，如果一直向外，容易出现搜索异常

        for (int i = -4; i < 4; ++i) {
            auto x = up_center_x + i;
            if (map_->inGrid(x, y)) {
                auto& bar = map_->bar(x, y);
                int real_up, real_down;
                if (checkHole(bar, hole.up_index, hole.down_index, real_up, real_down)) {
                    if (checkPointThresh(x, y, bar.pointByIndex(real_up), bar.pointByIndex(real_down),
                                         4 * map_resolution_)) {
                        creatHole(x, y, real_up, real_down, bar, adj_hole);
                        real_x = x;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    bool searchAdjacentSurfaceInX(int up_center_x, int y, const HoleInfo& hole, int& real_x,
                                  std::vector<HoleInfo>& surfaces) {
        real_x = up_center_x - 3;
        for (int i = -1; i < 3; ++i) {
            auto x = up_center_x + i;
            if (map_->inGrid(x, y)) {
                auto& bar = map_->bar(x, y);
                int real_up, real_down;
                if (isSurface(bar, hole.up_index, hole.down_index, real_up, real_down)) {
                    if (checkPointThresh(x, y, bar.pointByIndex(real_up), bar.pointByIndex(real_down),
                                         4 * map_resolution_)) {
                        real_x = real_x > x ? real_x : x;

                        HoleInfo adj_surface;
                        creatHole(x, y, real_up, real_down, bar, adj_surface);
                        surfaces.push_back(adj_surface);
                    }
                }
            }
        }
        return !surfaces.empty();
    }

    void creatHole(int x, int y, int up, int down, const GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar,
                   HoleInfo& hole) {
        hole.up_index = up;
        hole.down_index = down;
        hole.up_point = bar.pointByIndex(up);
        hole.down_point = bar.pointByIndex(down);
        hole.x = x;
        hole.y = y;
        hole.mid_index = (up + down) / 2;
    }

    bool checkHole(GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar, int up_index, int down_index, int& real_up,
                   int& real_down) {
        real_up = 0, real_down = down_index;
        for (int i = up_index + 1; i > down_index; --i) {
            if (i < bar.points_index.size()) {
                if ((bar.points_index[i] * bar.points_index[i - 1] < 0)/* &&
                    (bar.points_index[i + 1] * bar.points_index[i - 1] < 0)*/) {
                    if (bar.points_index[i] > 0) {
                        real_up = i;
                        break;
                    }
                }
            }
        }
        //沿着X方向搜索时，空洞上边缘在Z轴上的上下偏移量不能超过2cm
        auto delta_index = real_up - up_index;
        if (real_up > 0 && std::abs(delta_index) <= (0.02 / map_resolution_)) {
            float incre = 0.;
            for (int j = down_index; j < real_up; ++j) {
                if (bar.points_index[j] == -1) {
                    incre++;
                }
            }
            if (incre / static_cast<float>(real_up - down_index) > hole_check_percentage_) {
                return true;
            }
        }

        return false;
    }

    bool checkPointThresh(int x, int y, const Eigen::Vector3f& up_point, const Eigen::Vector3f& down_point,
                          float dist_thresh) {
        Eigen::Vector2f point(map_->toWorldX(x), map_->toWorldY(y));
        auto dist_1 = (up_point.head<2>() - point).norm();
        auto dist_2 = (down_point.head<2>() - point).norm();
        return dist_1 <= dist_thresh && dist_2 <= dist_thresh;
    }

    bool checkPointThresh(int x, int y, const Eigen::Vector3f& up_point, float dist_thresh) {
        Eigen::Vector2f point(map_->toWorldX(x), map_->toWorldY(y));
        auto dist_1 = (up_point.head<2>() - point).norm();
        return dist_1 <= dist_thresh;
    }

    bool findHolesInBar(int x, int y, GroundPlaneMap<Eigen::Vector3f>::HeightBar& bar, float min_height,
                        float max_height, std::vector<HoleInfo>& hole_infos) {
        auto& bars = bar.points_index;
        auto size = bars.size();
        int up_index = -1;
        hole_infos.clear();

        auto min_hole_height_pixel = static_cast<int>(min_pallet_height_ / map_resolution_);
        for (int i = 1; i < size - 2; ++i) {
            if (bars[i] > 0 && bars[i + 1] > 0 /*&&bars[i+2]>0*/) {
                if (bars[i - 1] < 0) {
                    up_index = i;
                    auto min_down_index = up_index - min_hole_height_pixel;
                    if (min_down_index > 0) {
                        int incre = 0;
                        for (int j = min_down_index; j <= up_index; ++j) {
                            if (bars[j] == -1) {
                                incre++;
                            }
                        }
                        if ((float(incre) / min_hole_height_pixel) > hole_check_percentage_) {
                            hole_infos.emplace_back();
                            creatHole(x, y, up_index, min_down_index, bar, hole_infos.back());
                            up_index = -1;
                        }
                    }
                }
            }
        }

        /*
        for (int i = 1; i < size - 1; ++i) {
            if (bars[i] * bars[i - 1] < 0 && bars[i + 1] * bars[i - 1] < 0) {
                auto theory_down_index = (i-1) - min_hole_height_pixel;
//                LOG(INFO)<<"theory_down_index:"<<theory_down_index<<", min_hole_height_pixel:"<<min_hole_height_pixel;
                if (bars[i - 1] < 0 && theory_down_index > 0 && bars[theory_down_index] < 0 &&
                    bars[theory_down_index + 1] < 0) {
                    up_index = i - 1;
                }

                if (up_index != -1) {
                    hole_infos.emplace_back();
                    creatHole(x, y, up_index, theory_down_index, bar, hole_infos.back());
                    up_index = -1;
                    down_index = -1;
                }
            }
        }
        */
        return !hole_infos.empty();
    }

    std::shared_ptr<GroundPlaneMap<Eigen::Vector3f>> map_;  //投影到地面的柱状地图，将高度信息变成直方柱图
    float min_pallet_height_ = 0.08f;  //栈板最小高度，相对高度（最大高度-最小高度）小于该值的，当做地面处理
    float max_pallet_height_ = 0.2f;  //栈板可能的最大高度，单位m
    float map_resolution_ = 0.01f;    //投影到地面上网格的分辨率，每个网格独立构造一个柱状条
    float min_hole_height_ = 0.05f;   //洞的最小高度，小于该值的洞不认为是栈板
    float max_hole_height_ = 0.2f;    //洞的最大高度，大于该值的洞不认为是栈板
    float pallet_hole_height_err_ = 0.05f;  //栈板左右洞的高度差，该值若设置大一些，鲁棒性就会增强，成功率提升
    float hole_check_percentage_ = 0.7f;  //判断为洞的概率，从上边沿到下边沿中间状态为空洞网格的概率
    int min_border_hole_pixel_size_ = 10;  // 10cm/1cm = 10，20说明最窄的洞宽度是20cm
    int max_border_hole_pixel_size_ = 50;  // 50cm/1cm=50个体素，表示孔洞上表边缘的最大长度为50cm
    int surface_initial_search_offset_ = 10;  // 10cm范围，该值一般不用配置。该值为范围搜索阈值，不用设置

    float max_pier_width_pixel_size = 30;  //最大墩宽度为30cm

 public:
    std::vector<HoleInfo> find_edges_;            // 查找到的上边缘
    std::vector<HoleInfo> find_surfaces_;         // 查找到墩的表面
    std::vector<HoleInfo> find_third_pier_mean_;  // 查找到的第三个墩的中心点
    std::vector<HoleInfo> find_third_pier_;       // 查找到的第三个墩平面
    std::vector<HoleInfo> find_holes_;            // 查找到的孔洞
 public:
    std::vector<HoleInfo> get_edges_found() const { return find_edges_; }
    std::vector<HoleInfo> get_surfaces_found() const { return find_surfaces_; }
    std::vector<HoleInfo> get_third_pier_mean_found() const { return find_third_pier_mean_; }
    std::vector<HoleInfo> get_third_pier_found() const { return find_third_pier_; }
    std::vector<HoleInfo> get_holes_found() const { return find_holes_; }
};
}  // namespace standard

#endif  // LIVOX_PALLET_DETECT_STANDARD_PALLET_DETECT_HPP
