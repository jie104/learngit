//
// Created by lfc on 17-9-25.
//

#include "circle_processor.h"
#include <glog/logging.h>
namespace fitting{

CircleProcessor::CircleProcessor():FittingProcessor(TYPE_FITPROC_CIRCLE) {

}

CircleProcessor::~CircleProcessor() {

}

void CircleProcessor::computeFeatPara(recog::FeatureCluster& cluster, FeatPara_Ptr feat_para) {
    auto &points = cluster.points;
    int m_nNum = points.size();
    if (m_nNum<3) {
        LOG(INFO) << "err to get enough point!";
    }

    int i=0;

    double X1=0;
    double Y1=0;
    double X2=0;
    double Y2=0;
    double X3=0;
    double Y3=0;
    double X1Y1=0;
    double X1Y2=0;
    double X2Y1=0;

    for (i=0;i<m_nNum;i++) {
        double x,y;
        x = points[i].x;
        y = points[i].y;

        X1 = X1 + x;
        Y1 = Y1 + y;
        X2 = X2 + x * x;
        Y2 = Y2 + y * y;
        X3 = X3 + x * x * x;
        Y3 = Y3 + y * y * y;
        X1Y1 = X1Y1 + x * y;
        X1Y2 = X1Y2 + x * y * y;
        X2Y1 = X2Y1 + x * x * y;
    }

    double C,D,E,G,H,N;
    double a,b,c;
    N = m_nNum;
    C = N*X2 - X1*X1;
    D = N*X1Y1 - X1*Y1;
    E = N*X3 + N*X1Y2 - (X2+Y2)*X1;
    G = N*Y2 - Y1*Y1;
    H = N*X2Y1 + N*Y3 - (X2+Y2)*Y1;
    a = (H*D-E*G)/(C*G-D*D);
    b = (H*C-E*D)/(D*D-G*C);
    c = -(a*X1 + b*Y1 + X2 + Y2)/N;

    feat_para->a = a/(-2);
    feat_para->b = b/(-2);
    feat_para->r = sqrt(a * a + b * b - 4 * c) / 2;
    computeCov(cluster, feat_para);
}

void CircleProcessor::computeCov(recog::FeatureCluster &cluster, FeatPara_Ptr feat_para) {
    //xi-x = (d-r)/d*(xi-a)
    //yi-y = (d-r)/d*(yi-a)
    auto &points = cluster.points;
    float x_0 = feat_para->a;
    float y_0 = feat_para->b;
    float r = feat_para->r;
    if (r == 0||!points.size()) {
        LOG(INFO) << "the r is err! will return false!";
    }
    double sum_xx = 0;
    double sum_yy = 0;
    double sum_xy = 0;
    double sum_dist = 0;
    double point_size = points.size();
    for (auto &point:points) {
        double delta_x = point.x - x_0;
        double delta_y = point.y - y_0;

        double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
        if (dist <= 0.0001) {
            LOG(INFO) << "the dist is too small!";
            continue;
        }
        double ratio = (dist - r) / dist;
        double err_x = ratio * delta_x;
        double err_y = ratio * delta_y;
        sum_xx += err_x * err_x;
        sum_xy += err_x * err_y;
        sum_yy += err_y * err_y;
        sum_dist += (dist - r) * (dist - r);
    }
    feat_para->cov(0, 0) = sum_xx / point_size;
    feat_para->cov(1, 1) = sum_yy / point_size;
    feat_para->cov(1, 0) = sum_xy / point_size;
    feat_para->cov(0, 1) = feat_para->cov(1,0);
    feat_para->rmse = sqrt(sum_dist / point_size);
}
}