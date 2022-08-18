#pragma once

#include <eigen3/Eigen/Core>

namespace feature
{
    using Point2f = Eigen::Vector2f;

    //信标特征点
    struct LandmarkFeaturePoint
    {
        Point2f coordinate_;
        float normalized_intensity_;
    };

    //信标类型
    enum class LandmarkFeatureType
    {
        FLAT = 1, CYLINDER, UNKNOWN,
    };

    class LandmarkFeature
    {
    public:
        LandmarkFeature(const std::vector<LandmarkFeaturePoint>& landmark_feature_points)
        {
            Point2f center = computeCenter(landmark_feature_points);//用强度加权的质心
            computeWidthAndIncidentAngle(landmark_feature_points, center);//计算信标宽度和入射角
            computeNormalizedIntensityByIncidentAngle(landmark_feature_points);
            computeType(landmark_feature_points, center);//初始化信标类型
            computePosition(center);
            bias_ = computeMedianAbsoluteDeviation(landmark_feature_points);
        }

    public:
        LandmarkFeatureType type_;  //信标类型
        Point2f coordinate_;    //信标坐标
        float distance_;    //距离
        float angle_;   //角度
        float bias_;    //偏差、、
        float width_;//根据协方差拟合出来的信标的宽度
        float incident_angle_;//到信标中心的入射角
        float normalized_mean_intensity_;
    private:
        static constexpr float flat_width_ = 0.05f;   //平面型反光贴宽度.原来为10cm
        static constexpr float cylinder_radius_ = 0.025f;   //圆柱半径
        static constexpr float cylinder_width_ = 1.732 * cylinder_radius_;   //圆柱型反光贴宽度 sin(pi/3) * 2r = 1.732r。默认圆心夹角120°
        static constexpr float extend_length_ = 0.8546f * cylinder_radius_;   //延长线段长度 [pi/3 + sqrt(3)/4]/sqrt(3) = 0.8546。怎么计算的？几何
        static constexpr float coefficient_threshold_ = 0.1f;//22.5    //系数阈值 4200 * 0.025 * sin(pi/3) / pi = 28.94。怎么计算的:假设两端相切时圆心角为120°
        static constexpr float bias_ratio_ = 0.5;//0.25    //偏差比值//0.6
        static constexpr float MAD_threshold_ = 0.06f;    //绝对中位差阈值(圆柱型反光贴的MAD高于此阈值)
        static constexpr float normalized_intensity_threshold_ = 0.7;//根据入射角归一化后的强度阈值

        //计算信标特征点集的质心
        Point2f computeCenter(const std::vector<LandmarkFeaturePoint>& landmark_feature_points)
        {
            float sum_x_by_intensity = 0.f, sum_y_by_intensity = 0.f, sum_intensity = 0.f;
            for (const auto& point : landmark_feature_points)
            {
                const float normalized_intensity = point.normalized_intensity_;
                sum_intensity += normalized_intensity;
                sum_x_by_intensity += point.coordinate_.x() * normalized_intensity;//用强度加权的信标中心位置
                sum_y_by_intensity += point.coordinate_.y() * normalized_intensity;
            }
            return Point2f(sum_x_by_intensity / sum_intensity, sum_y_by_intensity / sum_intensity);
        }

        void computeWidthAndIncidentAngle(const std::vector<LandmarkFeaturePoint>& landmark_feature_points, const Point2f& center){
            //用协方差矩阵求解信标的入射角和宽度
            float sum_intensity = 0;
            Eigen::Matrix2f covar_matrix = Eigen::Matrix2f::Zero();
            for(const auto& point : landmark_feature_points){
                Eigen::Vector2f del = point.coordinate_ - center;
                float intensity = point.normalized_intensity_;
                covar_matrix += intensity * del * del.transpose();
                sum_intensity += intensity;
            }
            covar_matrix /= sum_intensity;

            Eigen::EigenSolver<Eigen::Matrix2f> sol(covar_matrix);
            Eigen::Matrix2f eigen_values = sol.pseudoEigenvalueMatrix();
            Eigen::Matrix2f eigen_vectors = sol.pseudoEigenvectors();

            float angle_of_landmark = 0.0;
            Eigen::Vector2f long_eigen_vector;//长轴方向
            if(fabs(eigen_values(0, 0)) > fabs(eigen_values(1, 1))){
                long_eigen_vector = eigen_vectors.col(0);
            }else {
                long_eigen_vector = eigen_vectors.col(1);
            }
            angle_of_landmark = std::atan(long_eigen_vector(1) / (long_eigen_vector(0)+0.0000000001));
//            std::cout << "Angle_by_covar: " << angle_of_landmark / M_PI*180.0 << std::endl;
            float landmark_angle_from_lidar = std::atan(center(1)/(center(0)+0.0000000001));
            incident_angle_ = std::abs(std::abs(landmark_angle_from_lidar-angle_of_landmark) - M_PI/2.0);
//            std::cout << "Incident angle by covariance:   " << _incident_angle / M_PI * 180.0 << std::endl;
            Eigen::Vector2f point_to_center;
            point_to_center = landmark_feature_points.front().coordinate_ - center;
            float len1 = std::abs(long_eigen_vector.dot(point_to_center)) / long_eigen_vector.norm();
            point_to_center = landmark_feature_points.back().coordinate_ - center;
            float len2 = std::abs(long_eigen_vector.dot(point_to_center)) / long_eigen_vector.norm();
            width_ = len1+len2;
//            std::cout << "Width_by_covar: " << _width << std::endl;
        }

        //用入射角加权，计算信标强度，归一化后的强度与距离、入射角都无关，理论上单位化后的强度只与材质相关
        void computeNormalizedIntensityByIncidentAngle(const std::vector<LandmarkFeaturePoint>& landmark_feature_points){
            double sum_intensity = 0.0;
            for(const auto& landmark_point : landmark_feature_points){
                sum_intensity += landmark_point.normalized_intensity_;
            }
            double mean_intensity = sum_intensity / landmark_feature_points.size();
            normalized_mean_intensity_ = mean_intensity / (std::cos(incident_angle_)+0.0000000001) - 0.5 * incident_angle_ + 0.2;
        }

        //计算信标的类型
        void computeType(const std::vector<LandmarkFeaturePoint>& landmark_feature_points, const Point2f& center)
        {
            if (center.norm() * static_cast<float>(landmark_feature_points.size()) > coefficient_threshold_)//距离和点数相乘
            {
                if(landmark_feature_points.size() < 5){//如果点少于5个，计算宽度的误差很大，入射角和宽度的计算误差都比较大
                    type_ = LandmarkFeatureType::UNKNOWN;
                }else if(width_ > (1.f + bias_ratio_) * flat_width_)//过宽不计算
                {
                    type_ = LandmarkFeatureType::UNKNOWN;
                }else if(width_ > (1.f - bias_ratio_) * flat_width_){//宽窄合适
                    if(normalized_mean_intensity_ > normalized_intensity_threshold_){//0.7
                        type_ = LandmarkFeatureType::FLAT;
                    }else{
                        type_ = LandmarkFeatureType::UNKNOWN;
                    }
                }else{
                    type_ = LandmarkFeatureType::UNKNOWN;
                }
            }
            else
            {
                type_ = LandmarkFeatureType::UNKNOWN;
            }
        }

        //计算信标特征点集的绝对中位差
        float computeMedianAbsoluteDeviation(const std::vector<LandmarkFeaturePoint>& landmark_feature_points)
        {
            std::vector<float> intensities;
            for (const auto& point : landmark_feature_points)
            {
                intensities.push_back(point.normalized_intensity_);
            }
            std::nth_element(intensities.begin(), intensities.begin() + intensities.size() / 2, intensities.end());//只能保证第n个位置上是第n大的数，前面的都比他小，不一定有序
            const float median = intensities[intensities.size() / 2];
            float sum = 0.f;
            for (const auto& intensity : intensities)
            {
                sum += std::abs(intensity - median);
            }
            return sum / static_cast<float>(intensities.size());//越小说明强度越集中
        }

        //计算信标的位置信息
        void computePosition(const Point2f& center)
        {
            angle_ = std::atan2(center.y(), center.x());
            switch (type_)
            {
                case LandmarkFeatureType::FLAT:
                    coordinate_ = center;
                    break;
                case LandmarkFeatureType::CYLINDER:
                    coordinate_.x() = center.x() + extend_length_ * std::cos(angle_);
                    coordinate_.y() = center.y() + extend_length_ * std::sin(angle_);
                    break;
                default:
                    break;
            }
            distance_ = coordinate_.norm();
        }
    };
}