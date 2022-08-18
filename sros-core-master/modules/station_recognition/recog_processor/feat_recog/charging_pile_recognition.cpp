//
// Created by liuyan on 18-3-9.
//
#include <glog/logging.h>
#include "fitting/line_processor.h"
#include "charging_pile_recognition.h"
#include "fitting/fitting_proc_factory.h"

namespace recog {
    const float DISTANCE_THRESHOLD = 0.05;
    ChargingPileRecognition::ChargingPileRecognition() : BaseRecognition(TYPE_RECOG_CHARGINGPILE) {
        charging_pile_info.reset(new ChargingRecogInfo);
        fitting_proc = fitting::FittingProcFactory::getFittingProcessor(fitting::TYPE_FITPROC_LINE);

    }

    ChargingPileRecognition::~ChargingPileRecognition() {

    }

//    bool ChargingPileRecognition::extractFeature(Scan_Ptr scan, FeatureClusters_Ptr clusters) {
//        if (scan&&clusters) {
//            LOG(INFO) << "ChargingPileRecognition extractFeature";
//            std::vector<int> inten_indexes;
//            auto &ranges = scan->ranges;
//            int range_size = ranges.size();
//            float angle_min = scan->angle_min;
//            float angle_incre = scan->angle_increment;
//            for (int i = 0; i < range_size; ++i) {
//                if (scan->intensities[i] > charging_pile_info->intensity_thresh) {
//                    //LOG(INFO) <<"i" << i;
//                    inten_indexes.push_back(i);
//                }
//            }
//            if (!inten_indexes.size()) {
//                return false;
//            }
//
//            auto &point_clusters = clusters->clusters;
//            point_clusters.clear();
//            FeatureCluster cluster;
////            Point point_0;
////            float index_0 = inten_indexes[0];
////            point_0.x = ranges[index_0] * cos(angle_min + index_0 * angle_incre);
////            point_0.y = ranges[index_0] * sin(angle_min + index_0 * angle_incre);
////            point_0.intensity = scan->intensities[index_0];
////            cluster.points.push_back(point_0);
//
//            int inten_size = inten_indexes.size();
//            LOG(INFO) <<" inten_size " <<inten_size;
//            for (int j = 1; j < inten_size; ++j) {
//                float index = inten_indexes[j];
//                if(index >= scan->intensities.size())
//                    return false;
//                Point tmp_point;
//                tmp_point.x = ranges[index] * cos(angle_min + index * angle_incre);
//                tmp_point.y = ranges[index] * sin(angle_min + index * angle_incre);
//                tmp_point.intensity = scan->intensities[index];
//                if (fabs(inten_indexes[j] - inten_indexes[j - 1]) == 1) {//如果小于等于2，就可以过滤掉中间噪点了。
//                    if(j==1){
//                        Point point_0;
//                        float index_0 = inten_indexes[0];
//                        point_0.x = ranges[index_0] * cos(angle_min + index_0 * angle_incre);
//                        point_0.y = ranges[index_0] * sin(angle_min + index_0 * angle_incre);
//                        point_0.intensity = scan->intensities[index_0];
//                        cluster.points.push_back(point_0);
//                    }
//                    //LOG(INFO) << "j " <<j;
//                    cluster.points.push_back(tmp_point);
//
//                } else {
//                    cluster.id++;
//                    if (cluster.points.size() > charging_pile_info->num_count) {
//                        point_clusters.push_back(cluster);
//                        LOG(INFO)<<" id " <<point_clusters.back().id;
//                    }
//                    cluster.points.clear();
//                    cluster.points.push_back(tmp_point);
//                }
//            }
//            if (cluster.points.size() > charging_pile_info->num_count) {
//                cluster.id++;
//                point_clusters.push_back(cluster);
//                LOG(INFO)<<" id " <<point_clusters.back().id;
//
//            }
//            LOG(INFO) << "point_clusters.size() " << point_clusters.size();
//            if(point_clusters.size() == 1 || point_clusters.empty())
//            {
//                LOG(INFO) << "check only one or no lmk";
//                return false;
//            }
//            else{
//                LOG(INFO) <<"####=====####";
//                return true;
//            }
//        }
//        LOG(INFO) << "err to get the cluster!";
//        return false;
//
//    }
    bool ChargingPileRecognition::extractFeature(Scan_Ptr scan, FeatureClusters_Ptr clusters) {
        computeNormalizedIntensities(scan);
        getLandmarks(scan);
        getLandMarkFeature(scan,clusters);
        if(clusters->getSize()>1)
            return true;
        else
            return false;
    }

    bool ChargingPileRecognition::getPose(FeatureClusters_Ptr clusters, std::vector<fitting::FeatPara_Ptr> &feat_paras,
                                          Eigen::Vector3f &out_pose) {
        //CHECK_EQ(clusters->clusters.size(), 2);//因为站点位置只有两个反光贴，应该只会有两个feat_pares 但可能有其他反光贴
        feat_paras.clear();
        if (clusters->clusters.empty() && clusters->clusters.size() >= 2) //至少两个
            return false;
        fitting::FeatPara_Ptr feat_para_ptr;
        feat_para_ptr.reset(new fitting::FeatPara);
        double min_distance = 4.0;//因为检测充电站点在4m之内

        for (int i = 0; i < clusters->clusters.size() - 1; i++) {
            if (checkTwoFeatureOnSameLine(clusters->clusters[i], clusters->clusters[i + 1], feat_para_ptr)) {
                double first_line_distance, second_line_distance;
                feat_paras.push_back(feat_para_ptr);
                Point center_pose_first, center_pose_second;
                getLineDistance(clusters->clusters[i], first_line_distance);
                LOG(INFO) << "first_line_distance " << first_line_distance;
                getLineDistance(clusters->clusters[i + 1], second_line_distance);
                LOG(INFO) << "second_line_distance " << second_line_distance;

                if ((fabs(first_line_distance - charging_pile_info->reflective_stickers_length) < 0.02) &&
                    (fabs(second_line_distance - charging_pile_info->reflective_stickers_length) < 0.02)) {
                    getLineCenterPose(clusters->clusters[i], center_pose_first);
                    getLineCenterPose(clusters->clusters[i + 1], center_pose_second);
                }
                double distance_between_two_point = sqrt(
                        (center_pose_first.x - center_pose_second.x) * (center_pose_first.x - center_pose_second.x) +
                        (center_pose_first.y - center_pose_second.y) * (center_pose_first.y - center_pose_second.y)
                );
                if (fabs(distance_between_two_point - charging_pile_info->charging_pile_width +
                         charging_pile_info->reflective_stickers_length) < DISTANCE_THRESHOLD) {
                    Eigen::Vector3f out_pose_candidate = computeChargingPileCenterPose(center_pose_first, center_pose_second);

                    auto out_pose_candidate_distance = std::hypot(out_pose_candidate[0], out_pose_candidate[1]);
                    if(min_distance > out_pose_candidate_distance){
                        out_pose = out_pose_candidate;
                        min_distance = out_pose_candidate_distance;
                    }
                    LOG(INFO) << "x " << out_pose[0] << " y " << " " << out_pose[1] << " theta " << out_pose[2];
                    LOG(WARNING) << "the distance to laser " << min_distance;
                    LOG(INFO) << " x0 " << (center_pose_first.x + center_pose_second.x) / 2.0 << " y0 "
                              << (center_pose_first.y + center_pose_second.y) / 2.0;
                } else {
                    LOG(ERROR) << "two point find wrong " << " distance_between_two_point "
                               << distance_between_two_point;
                    LOG(ERROR) << "distance " << distance_between_two_point - charging_pile_info->charging_pile_width +
                                                 charging_pile_info->reflective_stickers_length;
                    //how to do
                    continue;
                }
            } else
                continue;
        }
        if(min_distance<4.0){
            return true;
        }
        else
            return false;
    }
//        if (checkTwoFeatureOnSameLine(clusters->clusters[0], clusters->clusters[1], feat_para_ptr)) {
//
//
//
//            LOG(INFO) << "center_pose_first.x "<<center_pose_first.x << " center_pose_first.y "<<center_pose_first.y <<
//                      " center_pose_first.intensity"<<center_pose_first.intensity;
//            LOG(INFO) << "center_pose_second.x "<<center_pose_second.x << " center_pose_second.y "<<center_pose_second.y <<
//                      " center_pose_second.intensity"<<center_pose_second.intensity;
//
//
//
//        } else {
//            for (auto &cluster:clusters->clusters) {
//                fitting::FeatPara_Ptr feat_ptr;
//                feat_ptr.reset(new fitting::FeatPara);
//                fitting_proc->computeFeatPara(cluster, feat_ptr);
//                feat_paras.push_back(feat_ptr);
//            }
//            return false;
//        }
//    }
    Eigen::Vector3f ChargingPileRecognition::computeChargingPileCenterPose(const Point &first_line_center_pose,const Point &second_line_center_pose ){
        Eigen::Vector2f real_point1(0,
                                     (charging_pile_info->charging_pile_width-charging_pile_info->reflective_stickers_length)/2.0);
        Eigen::Vector2f real_point2(0,
                                    -(charging_pile_info->charging_pile_width-charging_pile_info->reflective_stickers_length)/2.0);
        Eigen::Vector2f first_point(first_line_center_pose.x, first_line_center_pose.y);
        Eigen::Vector2f second_point(second_line_center_pose.x,second_line_center_pose.y);

        double first_theta = atan2(first_point[0],first_point[1]);
        double second_theta = atan2(second_point[0],second_point[1]);

        double delta_theta = first_theta - second_theta;
        normalizeAngle(delta_theta);
        if(delta_theta>0){
            Eigen::Vector2f tmp_point = first_point;
            first_point = second_point;
            second_point = tmp_point;
        }
        icp::MatchPair match_pair;
        std::vector <icp::MatchPair> pairs;
        match_pair.origin_point = first_point;
        match_pair.match_point = real_point1;
        match_pair.info=Eigen::Matrix2f::Identity();
        pairs.push_back(match_pair);

        match_pair.origin_point = second_point;
        match_pair.match_point = real_point2;
        match_pair.info=Eigen::Matrix2f::Identity();
        pairs.push_back(match_pair);
        Eigen::Vector3f out_pose = Eigen::Vector3f::Zero();
        icp_processor.processGIcp(pairs, out_pose);
        return out_pose;

    }

    bool ChargingPileRecognition::checkTwoFeatureOnSameLine(const FeatureCluster &first_feature,
                                                            const FeatureCluster &second_feature,
                                                            fitting::FeatPara_Ptr &feat_para) {
        FeatureCluster features = first_feature;
        features.points.insert(features.points.begin(),second_feature.points.begin(),second_feature.points.end());
        fitting_proc->computeFeatPara(features, feat_para);
        LOG(INFO) << "feat_para->rmse " <<feat_para->rmse;
        if(feat_para->rmse>0.3) {
            return false;
        }
        return true;
    }
    void ChargingPileRecognition::getLineDistance(const FeatureCluster &feature,double &length){
        std::shared_ptr<fitting::LineProcessor> fitting_line_proc = std::dynamic_pointer_cast<fitting::LineProcessor>(fitting_proc);
        fitting_line_proc->computeLineDistance(feature,length);
    }
    void ChargingPileRecognition::getLineCenterPose(const FeatureCluster &feature,Point &center_pose){
        std::shared_ptr<fitting::LineProcessor> fitting_line_proc = std::dynamic_pointer_cast<fitting::LineProcessor>(fitting_proc);
        fitting_line_proc->computeLineCenterPose(feature,center_pose);
    }

    //计算归一化强度,噪点的归一化强度为0
    void ChargingPileRecognition::computeNormalizedIntensities(Scan_Ptr scan) {
        normalized_intensities_.clear();
        normalized_intensities_.reserve(scan->intensities.size());
        float standard_intensity;
        for (unsigned int i = 0; i < scan->intensities.size(); i++) {
            if (scan->ranges[i] < 2.f) {
                standard_intensity = 1700.f;
            }
            else {
                standard_intensity = 3600.f / (scan->ranges[i] + 1.6f) + 700.f;
            }
            if (scan->intensities[i] < charging_pile_info->intensity_weak_threshold) {
                normalized_intensities_.push_back(0.f);
            }
            else {
                normalized_intensities_.push_back(scan->intensities[i] / standard_intensity);
            }
        }
    }
    void ChargingPileRecognition::getLandmarks(Scan_Ptr scan) {
        landmarks_.clear();
        const unsigned int size = scan->intensities.size();
        unsigned int begin_index = 0;
        unsigned int end_index = 0;
        unsigned int loop_end_index = size;
        bool is_start_flag = false;
        unsigned int weak_count = 0;
        for (unsigned int index = 0; index < size; index++) {
            if (isNormalizedIntensityStrong(index)) {
                if (!is_start_flag) {
                    begin_index = index;
                    is_start_flag = true;
                }
                else {
                    end_index = index - weak_count;
                    if ( end_index - 1 >=0 &&!isRangeClose(scan->ranges[index], scan->ranges[end_index - 1])) {
                        if (begin_index == 0 && isNormalizedIntensityStrong(size - 1) &&
                            isRangeClose(scan->ranges.back(), scan->ranges.front())) {
                            loop_end_index = end_index;
                        }
                        else if (isStrongCountEnough(end_index - begin_index)) {
                            landmarks_.emplace_back(begin_index, end_index);
                        }
                        begin_index = index;
                    }
                    weak_count = 0;
                }
            }
            else if (is_start_flag) {
                end_index = index - weak_count;
                if (++weak_count <= charging_pile_info->weak_count_threshold) {
                    continue;
                }
                if (begin_index == 0 && isNormalizedIntensityStrong(size - 1) &&
                    isRangeClose(scan->ranges.back(), scan->ranges.front())) {
                    loop_end_index = end_index;
                }
                else if (isStrongCountEnough(end_index - begin_index)) {
                    landmarks_.emplace_back(begin_index, end_index);
                }
                is_start_flag = false;
                weak_count = 0;
            }
        }
        //首尾相接处的特殊处理
        if (loop_end_index != size && isStrongCountEnough(loop_end_index + size - begin_index)) {
            landmarks_.emplace_back(begin_index, loop_end_index + size);
        }
    }
    bool ChargingPileRecognition::isNormalizedIntensityStrong(const unsigned int index) {
        return normalized_intensities_[index] >= 0.5f;
    }
    bool ChargingPileRecognition::isStrongCountEnough(const unsigned int count) {
        return count >= charging_pile_info->strong_count_threshold;
    }
    bool ChargingPileRecognition::isRangeClose(const float lrange, const float rrange) {
        return std::abs(lrange - rrange) <= charging_pile_info->range_close_threshold;
    }
    void ChargingPileRecognition::getLandMarkFeature(Scan_Ptr scan ,FeatureClusters_Ptr clusters){
        auto iter = landmarks_.begin();
        unsigned  int size = scan->intensities.size();
        auto &point_clusters = clusters->clusters;
        point_clusters.clear();
        unsigned  int land_mark_number=0;
        LOG(INFO) << "scan->intensities.size() " << scan->intensities.size() << " landmarks_.size() " <<landmarks_.size();
        while(iter!=landmarks_.end()){
            FeatureCluster cluster;
            land_mark_number++;
            for(unsigned int i =iter->first;i<iter->second;i++){
                if(isNormalizedIntensityStrong(i)) {
                    const unsigned int index = i % size;
                    Point tmp_point;
                    tmp_point.x = scan->ranges[index] * cos(scan->angle_min + index * scan->angle_increment);
                    tmp_point.y = scan->ranges[index] * sin(scan->angle_min + index * scan->angle_increment);
                    tmp_point.intensity = scan->intensities[index];
                    cluster.points.push_back(tmp_point);
                    cluster.id=land_mark_number;
                }
            }
            point_clusters.push_back(cluster);
            iter++;
        }
    }

}