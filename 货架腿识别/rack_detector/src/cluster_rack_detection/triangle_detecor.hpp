//
// Created by lfc on 2023/1/3.
//

#ifndef RACK_DETECTOR_TRIANGLE_DETECOR_HPP
#define RACK_DETECTOR_TRIANGLE_DETECOR_HPP
#include "cluster_detector.hpp"

namespace rack_detection{

struct EdgeInfo{
    ClusterPoint_Ptr this_cluster;
    ClusterPoint_Ptr other_cluster;
    Eigen::Vector2f norm;
    float length;
    void computerEdge(const ClusterPoint_Ptr &this_cluster,const ClusterPoint_Ptr& other_cluster){
        norm = other_cluster->mean - this_cluster->mean;    //向量以this_cluster->mean为起点
        this->this_cluster = this_cluster;
        this->other_cluster = other_cluster;
        length = norm.norm();
        norm.normalized();
    }
};
typedef std::shared_ptr<EdgeInfo> EdgeInfo_Ptr;

struct TriangleInfo{
    std::vector<ClusterPoint_Ptr> points;
    ClusterPoint_Ptr length_point;
    ClusterPoint_Ptr width_point;
    ClusterPoint_Ptr corner_point;
    Eigen::Vector2f mean;
    float length;
    float width;
    void computeTriangle(const EdgeInfo_Ptr& edge_1,const EdgeInfo_Ptr& edge_2){
        corner_point = edge_1->this_cluster;
        length_point = edge_1->length > edge_2->length ? edge_1->other_cluster : edge_2->other_cluster;
        width_point = edge_1->length > edge_2->length ? edge_2->other_cluster : edge_1->other_cluster;
        length = edge_1->length > edge_2->length ? edge_1->length : edge_2->length;
        width = edge_1->length > edge_2->length ? edge_2->length : edge_1->length;
        points.clear();
        points.push_back(corner_point);
        points.push_back(length_point);
        points.push_back(width_point);
        mean = (length_point->mean + width_point->mean) * 0.5;
    }

    //判断是否是同一个货架
    bool equal(const std::shared_ptr<TriangleInfo>& triangle,float dist_thresh = 0.1){
        if(fabs(triangle->length - length)<dist_thresh && fabs(triangle->width - width)<dist_thresh){
            if((triangle->mean - mean).norm()<dist_thresh){
                int count = 0;
                for (auto& point : points) {
                    for (auto& cand_point : triangle->points) {
                        if (point == cand_point) {
                            count++;
                        }
                    }
                }
                return count >= 2;
            }
        }
        return false;
    }
};
typedef std::shared_ptr<TriangleInfo> TriangleInfo_Ptr;


class TriangleDetecor {
 public:
    void detectAngle(const std::vector<ClusterPoint_Ptr> &points,std::vector<TriangleInfo_Ptr>& triangles){
        for(auto& point:points) {
            std::vector<EdgeInfo_Ptr> edges;
            for(auto& other_point:points) {
                if (point != other_point) {
                    auto length = (other_point->mean - point->mean).norm();
                    for (auto& another_point:points) {
                        if (point!=another_point && other_point!=another_point ) {
                            if (length >= min_dist_ && length <= max_dist_&& IsNearestPoint(point,other_point,another_point)) {
                                edges.emplace_back(new EdgeInfo);
                                edges.back()->computerEdge(point, other_point);
                            }
                        }
                    }
                }
            }
            findTriangle(edges, triangles);
        }
    }

 private:

    bool IsNearestPoint(const ClusterPoint_Ptr& point,const ClusterPoint_Ptr& other_point,
                        const ClusterPoint_Ptr& another_point){
        if (PointToLineDistance(point,other_point,another_point)< 0.1){ //数值待调试
            Eigen::Vector2f origin_another_v=(another_point->mean-point->mean).normalized();
            Eigen::Vector2f other_another_v=(another_point->mean-other_point->mean).normalized();
            if (origin_another_v.dot(other_another_v)<-0.7){    //数值待调试
                return false;
            }
        }
        return true;
    }

    double PointToLineDistance(const ClusterPoint_Ptr& point,const ClusterPoint_Ptr& other_point,
                               const ClusterPoint_Ptr& another_point){
            Eigen::Vector2f a=other_point->mean-point->mean;
            Eigen::Vector2f b=another_point->mean-point->mean;
            double  distance=Eigen::Vector3f(a[0],a[1],0).cross(Eigen::Vector3f(b[0],b[1],0)).norm()/a.norm();
            return distance;
    }

    //解析直线方程
    Eigen::Vector3f solveLineParam(const ClusterPoint_Ptr& point,const ClusterPoint_Ptr& other_point){
        Eigen::Vector3f params;
        params[0]=other_point->mean[1]-point->mean[1];
        params[1]=point->mean[0]-other_point->mean[0];
        params[2]=point->mean[1]*(other_point->mean[0]-point->mean[0])-point->mean[0]*(other_point->mean[1]-point->mean[1]);
        return params;
    }

    //点到直线的距离
    double PointToLineDistance(Eigen::Vector3f params,const ClusterPoint_Ptr& point){
        double distance;
        double num1= fabs(params[0]*point->mean[0]+params[1]*point->mean[1]+params[2]);
        double num2=std::sqrt(params[0]*params[0]+params[1]*params[1]);
        distance=num1/num2;
        return distance;
    }

    //找到雷达5米内的所有货架信息，将货架腿位姿以直角三角形的信息存储在triangles中
    void findTriangle(const std::vector<EdgeInfo_Ptr> &edges,std::vector<TriangleInfo_Ptr>& triangles){
        int size = edges.size();
        for (int i = 0; i < size; ++i) {
            auto& edge = edges[i];
            for (int j = i + 1; j < size; ++j) {
                auto& other_edge = edges[j];
                if(fabs(edge->norm.dot(other_edge->norm))<0.1) {    //dot 向量点乘
                    triangles.emplace_back(new TriangleInfo);
                    triangles.back()->computeTriangle(edge, other_edge);
                }
            }
        }
    }

    float min_dist_ = 0.4f;
    float max_dist_ = 2.5f;
};
}


#endif  // RACK_DETECTOR_TRIANGLE_DETECOR_HPP
