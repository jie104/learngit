//
// Created by lfc on 2023/1/3.
//

#ifndef RACK_DETECTOR_CLUSTER_DETECTOR_HPP
#define RACK_DETECTOR_CLUSTER_DETECTOR_HPP
namespace rack_detection{
struct ClusterPoint {
    int min_index;
    float min_index_range;
    int max_index;
    float max_index_range;
    Eigen::Vector2f mean;
    std::vector<Eigen::Vector2f> infos;

    //计算聚类点平均值
    Eigen::Vector2f computeCenterPoint() {
        Eigen::Vector2f center_point;
        center_point.setZero();
        for (auto &info:infos) {
            center_point += info;
        }
        if (infos.size()) {
            mean = center_point * (1/(double) infos.size());
        }else{
            mean.setZero();
        }
        return mean;
    }
};

typedef std::shared_ptr<ClusterPoint> ClusterPoint_Ptr;


class ClusterDetector {
 public:
    ClusterDetector(float max_filter_range = 5.0):max_filter_range_(max_filter_range){

    }
    template <class ScanType>
    void extractClusters(ScanType &scan,std::vector<ClusterPoint_Ptr> &clusters){
        std::vector<bool> points_state(scan->ranges.size(), true);
        filterScanTrailingPoint(scan, points_state, 2, 0.01);       //提取托尾点
        extractClusters(scan, points_state, clusters);  //提取货架腿聚点
        for(auto& cluster:clusters){
            cluster->computeCenterPoint();  //计算货架腿的聚点平均值
        }
        combineClusters(clusters);  //将靠得较近的点云进一步组合，求聚点平均值
    }
 private:
    void combineClusters(std::vector<ClusterPoint_Ptr> &clusters){
        if (clusters.size()) {
            std::vector<ClusterPoint_Ptr> bk_clusters;
            ClusterPoint_Ptr curr_cluster = clusters[0];
            bk_clusters.push_back(curr_cluster);
            int cluster_size = clusters.size();
            for (int i = 1; i < cluster_size; ++i) {
                if((curr_cluster->mean - clusters[i]->mean).norm()<0.05f){
                    for (auto &info : clusters[i]->infos) {
                        curr_cluster->infos.push_back(info);
                    }
                    curr_cluster->computeCenterPoint();
                }else{
                    curr_cluster = clusters[i];
                    bk_clusters.push_back(curr_cluster);
                }
            }
//            LOG(INFO) << "size change:" << clusters.size() << "," << bk_clusters.size();
            clusters.swap(bk_clusters);
        }
    }

    //提取过滤后的货架腿的聚点
    template <class ScanType>
    void extractClusters(ScanType &scan, std::vector<bool> &points_state, std::vector<ClusterPoint_Ptr> &clusters) {
        auto &ranges = scan->ranges;
        auto range_min = scan->range_min;
        auto range_max = scan->range_max;
        double curr_angle = scan->angle_min;
        double angle_incre = scan->angle_increment;
        int index = 0;
        clusters.clear();
        clusters.emplace_back(new (ClusterPoint));
        for (auto &range:ranges) {
            if (range < range_min || range > max_filter_range_ || !points_state[index]) {
                if (clusters.back()->infos.empty()) {

                } else if (clusters.back()->infos.size() == 1) {//孤立点
                    ranges[clusters.back()->min_index] = 0;
                    clusters.back()->infos.clear();
                } else {
                    clusters.emplace_back(new (ClusterPoint));
                }
            } else {
                if (clusters.back()->infos.empty()) {
                    clusters.back()->min_index = index;
                    clusters.back()->min_index_range = range;
                }
                clusters.back()->infos.emplace_back();
                clusters.back()->max_index = index;
                clusters.back()->max_index_range = range;
                auto &info = clusters.back()->infos.back();
                info = Eigen::Vector2f(range * cos(curr_angle), range * sin(curr_angle));
            }
            index++;
            curr_angle += angle_incre;
        }
        if (clusters.back()->infos.empty()) {
            clusters.resize(clusters.size() - 1);
        }
    }

    //拖尾点滤除
    template <class ScanType>
    void filterScanTrailingPoint(ScanType &scan, std::vector<bool> &points_state, int step = 2,
                                 double min_thresh = 0.02) {//注意，这里如果拖尾部分有一个点被滤掉了，那其他点则无法识别出为拖尾点了，因为，这里有距离判断
        auto &ranges = scan->ranges;
        double cos_increment = cos(scan->angle_increment * (double) step * 2.0);
        double theta_thresh = sin((double) scan->angle_increment * (double) step * 2.0) / sin(0.34);//临界值,用于识别断点
        int scan_size = ranges.size() - step;
        for (int i = step; i < scan_size; i++) {
            if (ranges[i] == 0 || ranges[i] > max_filter_range_) {
                continue;
            }

            double dist_1 = std::sqrt(ranges[i + step] * ranges[i + step] + ranges[i] * ranges[i] -
                                      2 * ranges[i + step] * ranges[i] * cos_increment);

            double range_thresh_1 = ranges[i + step] * theta_thresh + min_thresh;
            if (dist_1 > range_thresh_1) {
                int remove_gap = step;
                for (int j = -remove_gap; j <= remove_gap; ++j) {
                    points_state[i + j] = false;
                }
            }
        }
    }

    float max_filter_range_ = 5.0;
};
}


#endif  // RACK_DETECTOR_CLUSTER_DETECTOR_HPP
