//
// Created by lfc on 17-9-25.
//

#include <glog/logging.h>
#include "rack_recognition.h"
#include "fitting/fitting_proc_factory.h"
namespace recog{


RackRecognition::RackRecognition() :BaseRecognition(TYPE_RECOG_RACK) {
    info.reset(new RackRecogInfo);
    fitting_proc = fitting::FittingProcFactory::getFittingProcessor(fitting::TYPE_FITPROC_CIRCLE);
}

RackRecognition::~RackRecognition() {

}

bool RackRecognition::extractFeature(Scan_Ptr scan, FeatureClusters_Ptr clusters) {
    if (scan&&clusters) {
        std::vector<int> inten_indexes;
        auto &ranges = scan->ranges;
        int range_size = ranges.size();
        float angle_min = scan->angle_min;
        float angle_incre = scan->angle_increment;
        for (int i = 0; i < range_size; ++i) {
            if(scan->intensities[i]>info->intensity_thresh){
                inten_indexes.push_back(i);
            }
        }
        if (!inten_indexes.size()) {
            return false;
        }

        auto &point_clusters = clusters->clusters;
        point_clusters.clear();
        FeatureCluster cluster;
        Point point_0;
        float index_0 = inten_indexes[0];
        point_0.x = ranges[index_0] * cos(angle_min + index_0 * angle_incre);
        point_0.y = ranges[index_0] * sin(angle_min + index_0 * angle_incre);

        cluster.points.push_back(point_0);

        int inten_size = inten_indexes.size();

        for (int j = 1; j < inten_size; ++j) {
            float index = inten_indexes[j];
            Point tmp_point;
            tmp_point.x = ranges[index] * cos(angle_min + index * angle_incre);
            tmp_point.y = ranges[index] * sin(angle_min + index * angle_incre);
            if (inten_indexes[j] - inten_indexes[j - 1] == 1) {//如果小于等于2，就可以过滤掉中间噪点了。
                cluster.points.push_back(tmp_point);
            }else {
                cluster.id++;
                if (cluster.points.size() > info->num_count) {
                    point_clusters.push_back(cluster);
                }
                cluster.points.clear();
                cluster.points.push_back(tmp_point);
            }
        }
        if (cluster.points.size() > info->num_count) {
            cluster.id++;
            point_clusters.push_back(cluster);
        }
        return point_clusters.size() > 1;
    }
    LOG(INFO) << "err to get the cluster!";
    return false;
}

bool RackRecognition::getPose(FeatureClusters_Ptr clusters,std::vector<fitting::FeatPara_Ptr>& feat_paras,Eigen::Vector3f& out_pose) {
    feat_paras.clear();
    for (auto &cluster:clusters->clusters) {
        fitting::FeatPara_Ptr feat_ptr;
        feat_ptr.reset(new fitting::FeatPara);
        fitting_proc->computeFeatPara(cluster, feat_ptr);
        feat_paras.push_back(feat_ptr);
    }

    auto min_feat = getMinDistPara(feat_paras);//确定圆柱是否是最近的,如果不是,有可能出问题
    if (!min_feat) {
        return false;
    }
    Eigen::Vector2f first_point(min_feat->a, min_feat->b);//

    fitting::FeatPara_Ptr other_feat;
    for (auto &feat_para:feat_paras) {
        Eigen::Vector2f center_point(feat_para->a, feat_para->b);
        if(fabs((first_point-center_point).norm()-info->rack_length)<0.05) {
            other_feat = feat_para;
            break;
        }
    }//找到另外一个满足要求的feature para,如果不满足,则退出,返回零
    if (!other_feat) {
        LOG(INFO) << "cannot get the para!";
        return false;
    }

    out_pose = computePose(min_feat, other_feat);

    return true;
}


fitting::FeatPara_Ptr RackRecognition::getMinDistPara(std::vector<fitting::FeatPara_Ptr> &feat_paras) {
    if (!feat_paras.size()) {
        return 0;
    }
    float mindist = sqrtf(feat_paras[0]->a * feat_paras[0]->a + feat_paras[0]->b * feat_paras[0]->b);
    fitting::FeatPara_Ptr out_feat = feat_paras[0];
    for (auto feat:feat_paras) {
        float dist = sqrtf(feat->a * feat->a + feat->b * feat->b);
        if (mindist > dist) {
            mindist = dist;
            out_feat = feat;
        }
    }
    return out_feat;
}

Eigen::Vector3f RackRecognition::computePose(fitting::FeatPara_Ptr &this_para, fitting::FeatPara_Ptr &that_para) {
    Eigen::Vector2f point_1(-info->rack_width / 2.0, info->rack_length / 2.0);
    Eigen::Vector2f point_2(-info->rack_width / 2.0, -info->rack_length / 2.0);
    Eigen::Vector2f this_point(this_para->a, this_para->b);
    Eigen::Vector2f that_point(that_para->a, that_para->b);
    Eigen::Matrix2f this_cov = this_para->cov;
    Eigen::Matrix2f that_cov = that_para->cov;

    Eigen::Vector2f delta_match_point = point_2 - point_1;
    Eigen::Vector2f delta_origin_point = that_point - this_point;
    float muti_value = delta_match_point[0] * delta_origin_point[0] + delta_match_point[1] * delta_origin_point[1];

    float theta_1 = atan2(this_point[0], this_point[1]);
    float theta_2 = atan2(that_point[0], that_point[1]);

    float delta_theta = theta_1 - theta_2;

    normalizeAngle(delta_theta);

    if (delta_theta > 0) {
        Eigen::Vector2f tmp_point = this_point;
        this_point = that_point;
        that_point = tmp_point;
        this_cov = that_para->cov;
        that_cov = this_para->cov;
    }
    icp::MatchPair match_pair;
    std::vector<icp::MatchPair> pairs;
    match_pair.origin_point = this_point;
    match_pair.match_point = point_1;
    match_pair.info = this_cov;
    pairs.push_back(match_pair);
    match_pair.origin_point = that_point;
    match_pair.match_point = point_2;
    match_pair.info = that_cov;
    pairs.push_back(match_pair);
    Eigen::Vector3f out_pose = Eigen::Vector3f::Zero();
    icp_processor.processGIcp(pairs, out_pose);
    return out_pose;
}
}