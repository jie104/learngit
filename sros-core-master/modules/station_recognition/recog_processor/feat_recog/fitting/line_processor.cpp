//
// Created by liuyan on 18-3-9.
//

#include "line_processor.h"
#include <glog/logging.h>
namespace fitting{
    LineProcessor::LineProcessor() :FittingProcessor(TYPE_FITPROC_LINE){}
    LineProcessor::~LineProcessor() {}
    /*
     * 拟合直线方程为a * x + b * y + c = 0
     */
    void LineProcessor::computeFeatPara(recog::FeatureCluster &cluster, FeatPara_Ptr feat_para) {
        auto &points = cluster.points;
        const int num_point = points.size();
        if(num_point<3){
            LOG(WARNING) << "err to get enough points";
        }
        double sum_x = 0.0,sum_y = 0.0;
        for(const auto &point :points){
            sum_x += point.x;
            sum_y += point.y;
        }
        const double mean_x = sum_x/num_point;
        const double mean_y = sum_y/num_point;

        double sum_uu =0.0,sum_uv=0.0,sum_vv=0.0;
        for(const auto &point:points){
            sum_uu += pow2(point.x-mean_x);
            sum_uv += (point.x - mean_x) *(point.y - mean_y);
            sum_vv += pow2(point.x - mean_y);
        }
        const double lambda = (sum_uu + sum_vv - std::hypot(sum_uu - sum_vv, 2.f * sum_uv)) / 2.f;
        const double div = std::hypot(sum_uv, lambda - sum_uu);
        double a = sum_uv / div;
        double b = (lambda - sum_uu) / div;
        double c = -mean_x * a - mean_y * b;
        double sum_bias = 0.f;
        for (const auto &point : points) {
            sum_bias += pow2(a * point.x + b * point.y + c) / (pow2(a) + pow2(b));
        }
        double bias = std::sqrt(sum_bias / num_point);
        feat_para->a = a;
        feat_para->b = b;
        feat_para->r = c;
        feat_para->rmse = bias;

    }
    void LineProcessor::computeCov(recog::FeatureCluster &cluster, FeatPara_Ptr feat_para) {

    }
    void LineProcessor::computeLineDistance(const recog::FeatureCluster &cluster , double &length){
        auto &points = cluster.points;
        auto first_point = points[0];
        auto last_point = points.back();
        length = sqrt(pow2(first_point.x -last_point.x) + pow2(first_point.y - last_point.y));
    }
    void LineProcessor::computeLineCenterPose(const recog::FeatureCluster &cluster , recog::Point  &center_pose){
        double inten_sum =0,x_inten_sum=0,y_inten_sum=0;
        for(const auto &point : cluster.points){
            inten_sum += point.intensity;
            x_inten_sum += point.x * point.intensity;
            y_inten_sum += point.y * point.intensity;
        }
        center_pose.x=x_inten_sum/inten_sum;
        center_pose.y=y_inten_sum/inten_sum;
        center_pose.intensity=inten_sum/cluster.points.size();
    }
}