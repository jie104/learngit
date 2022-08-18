#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace feature
{
    using Point2f = Eigen::Vector2f;//using

    //线段特征
    class LineSegmentFeature
    {
    public:
        explicit LineSegmentFeature(const std::vector<Point2f>& points, const std::vector<Eigen::Vector2f>& cos_sin_pairs)//explicit关键字只能用于修饰只有一个参数的类构造函数,防止自动转换参数类型
        {
            orthogonalLeastSquareFit(points, cos_sin_pairs);//根据点集使用最小二乘拟合出直线的方程和协方差
            computeEdges(points.front(), points.back());//根据方程和首尾点的坐标，计算直线的端点
            computeCoordinateAndLength();//计算线段的中心和长度
        }

        LineSegmentFeature(const LineSegmentFeature& rhs) = default;//使用默认的复制构造函数和赋值构造函数

        LineSegmentFeature& operator=(const LineSegmentFeature& rhs) = default;//有返回值

        bool operator==(const LineSegmentFeature& rhs) const
        {
            return front_edge_ == rhs.front_edge_ && back_edge_ == rhs.back_edge_ && covariance_ == rhs.covariance_;
        }

    public:
        //极坐标系方程参数(角度theta, 距离rho)
        Eigen::Vector2f polar_parameters_;

        //笛卡尔坐标系方程参数(A, B, C)
        Eigen::Vector3f cartesian_parameters_;

        //协方差矩阵
        Eigen::Matrix2f covariance_;

        //线段端点
        Point2f front_edge_;
        Point2f back_edge_;

        //线段中心坐标
        Point2f coordinate_;

        //线段长度
        float length_;

        //点到线段所在直线的距离
        float distToPoint(const Point2f& point) const
        {
            const float A = cartesian_parameters_.x();
            const float B = cartesian_parameters_.y();
            const float C = cartesian_parameters_.z();
            return std::abs(A * point.x() + B * point.y() + C);//按照目前的策略，A与B的平方和恰好为1
        }

        //线段尾端到下条线段头端的距离
        float distToNext(const LineSegmentFeature& next) const
        {
            return (back_edge_ - next.front_edge_).norm();
        }

        //另一线段端点到本线段的距离平方和偏差
        float squaredBias(const LineSegmentFeature& rhs) const
        {
            float squared_bias = 0.f;
            const Eigen::Vector2f A_B = back_edge_ - front_edge_;
            const Eigen::Vector2f A_Ap = rhs.front_edge_ - front_edge_;
            const Eigen::Vector2f B_Bp = rhs.back_edge_ - back_edge_;
            if(A_Ap.transpose() * A_B >= 0)//等同于点乘？把vector完完全全当做矩阵
                squared_bias += distToPoint(rhs.front_edge_) * distToPoint(rhs.front_edge_);
            else
                squared_bias += A_Ap.transpose() * A_Ap;//什么意思？
            if(B_Bp.transpose() * A_B <= 0)
                squared_bias += distToPoint(rhs.back_edge_) * distToPoint(rhs.back_edge_);
            else
                squared_bias += B_Bp.transpose() * B_Bp;
            return squared_bias;
        }

        //极坐标参数偏差
        Eigen::Vector2f diffOfPolarParameters(const LineSegmentFeature& rhs) const
        {
            Eigen::Vector2f diff = rhs.polar_parameters_ - polar_parameters_;
            diff.x() = std::remainder(diff.x(), M_PI * 2.f);//取余数
            return diff;
        }

        //线段所在直线间的卡方值
        float chiSquare(const LineSegmentFeature& rhs) const
        {
            const Eigen::Vector2f diff = diffOfPolarParameters(rhs);
            return diff.transpose() * (covariance_ + rhs.covariance_).inverse() * diff;//直线间差异的方差
        }

        //修剪线段的端点坐标,并更新线段位置坐标及长度
        void trimEdge(const Point2f& edge, bool isFrontFlag)
        {
            if (isFrontFlag)
                front_edge_ = edge;
            else
                back_edge_ = edge;
            computeCoordinateAndLength();
        }

        //计算和另一条直线的的夹角，正的锐角
        float computeAngleWithLine(const LineSegmentFeature& line){
            float diff = std::abs(polar_parameters_(0)-line.polar_parameters_(0));//0~2pi
            if(diff > M_PI)
                diff -= M_PI;//0~pi
            if(diff > M_PI / 2.0)
                diff = M_PI - diff;//取两个夹角中的锐角
            return diff;
        }

    private:
        //正交最小二乘法拟合线段所在直线
        void orthogonalLeastSquareFit(const std::vector<Point2f>& points, const std::vector<Point2f>& cos_sin_pairs)
        {
            std::tuple<float, float, float, float, float> para = computePointsParameters(points);//tuple用于返回多个参数，使用std::get<>获取元素，类似于pair
            computeLineParameters(para);//根据上一步计算出来的中间量，计算直线的极坐标式和欧式坐标
            computeCovariance(points, cos_sin_pairs, para);
        }

        //计算点集参数(mean_x, mean_y, sum_uu, sum_uv, sum_vv)
        std::tuple<float, float, float, float, float> computePointsParameters(const std::vector<Point2f>& points)//tuple用于返回多个参数
        {
            const float n_inv = 1.f / static_cast<float>(points.size());
            float sum_x = 0.f, sum_y = 0.f, sum_uu = 0.f, sum_uv = 0.f, sum_vv = 0.f;
            for (const auto& point : points)
            {
                sum_x += point.x();
                sum_y += point.y();
            }
            const float mean_x = sum_x * n_inv;
            const float mean_y = sum_y * n_inv;
            for (const auto& point : points)
            {
                sum_uu += (point.x() - mean_x) * (point.x() - mean_x);
                sum_uv += (point.x() - mean_x) * (point.y() - mean_y);
                sum_vv += (point.y() - mean_y) * (point.y() - mean_y);
            }
            return std::make_tuple(mean_x, mean_y, sum_uu, sum_uv, sum_vv);//均值和方差
        }

        /*
         * 计算线段的直线方程参数
         * 极坐标系直线方程参数theta, rho
         * 笛卡尔坐标系直线方程参数A, B, C
         * 直线方程为 x * cos(theta) + y * sin(theta) = rho
         * 所以A = cos(theta), B = sin(theta), C = -rho
         */
        void computeLineParameters(std::tuple<float, float, float, float, float> para)
        {
            const float theta = std::atan2(-2.f * std::get<3>(para), std::get<4>(para) - std::get<2>(para)) * 0.5f;//tuple不支持迭代，好像只能通过get函数模板索引
            const float rho = std::get<0>(para) * std::cos(theta) + std::get<1>(para) * std::sin(theta);
            if (rho < 0.f)//距离为负数，说明由于周期性角度超过±PI
            {
                polar_parameters_.y() = -rho;
                if (theta > 0.f)
                    polar_parameters_.x() = theta - M_PI;
                else
                    polar_parameters_.x() = theta + M_PI;
            }
            else
            {
                polar_parameters_.y() = rho;
                polar_parameters_.x() = theta;
            }
            cartesian_parameters_.x() = std::cos(polar_parameters_.x());
            cartesian_parameters_.y() = std::sin(polar_parameters_.x());
            cartesian_parameters_.z() = -polar_parameters_.y();
        }

        //计算协方差矩阵
        void computeCovariance(const std::vector<Point2f>& points, const std::vector<Point2f>& cos_sin_pairs, const std::tuple<float, float, float, float, float>& para)
        {
            covariance_ << 0.f, 0.f, 0.f, 0.f;
            const float a = std::get<4>(para) - std::get<2>(para);
            const float b = std::get<3>(para) * 2.f;
            const float k = 1.f / (a * a + b * b);
            const float n_inv = 1.f / static_cast<float>(points.size());
            const float cos_n = cartesian_parameters_.x() * n_inv;
            const float sin_n = cartesian_parameters_.y() * n_inv;
            const float d = std::get<1>(para) * cartesian_parameters_.x() - std::get<0>(para) * cartesian_parameters_.y();
            Eigen::Matrix2f Ai;
            Eigen::Vector2f Ji;
            for (unsigned int i = 0; i < points.size(); i++)
            {
                Ai(1, 0) = (a * (std::get<1>(para) - points[i].y()) + b * (std::get<0>(para) - points[i].x())) * k;
                Ai(1, 1) = (a * (std::get<0>(para) - points[i].x()) - b * (std::get<1>(para) - points[i].y())) * k;
                Ai(0, 0) = cos_n + d * Ai(1, 0);
                Ai(0, 1) = sin_n + d * Ai(1, 1);
                Ji = Ai * cos_sin_pairs[i];
                covariance_(1, 1) += Ji.x() * Ji.x();
                covariance_(0, 1) += Ji.x() * Ji.y();
                covariance_(1, 0) += Ji.x() * Ji.y();
                covariance_(0, 0) += Ji.y() * Ji.y();
            }
        }

        //计算点到线段所在直线的垂足坐标
        Point2f computeFootPoint(const Point2f& point) const
        {
            const float A = cartesian_parameters_.x();
            const float B = cartesian_parameters_.y();
            const float C = cartesian_parameters_.z();
            const float x = point.x();
            const float y = point.y();
            return Point2f(B * B * x - A * B * y - A * C, A * A * y - A * B * x - B * C);//可以推导的，有公式
        }

        //计算线段端点
        void computeEdges(const Point2f& front_point, const Point2f& back_point)//端点往直线上投影
        {
            front_edge_ = computeFootPoint(front_point);
            back_edge_ = computeFootPoint(back_point);
        }

        //计算线段中心坐标和长度
        void computeCoordinateAndLength()
        {
            coordinate_ = (front_edge_ + back_edge_) * 0.5f;
            length_ = (front_edge_ - back_edge_).norm();
        }

    };

    //角点特征
    class CornerFeature
    {
    public:
        explicit CornerFeature(const LineSegmentFeature& prev, const LineSegmentFeature& next) : prev_(prev), next_(next)
        {
            computeCoordinate();//计算交点坐标
            computeIncludedAngle();//计算夹角
            computeDirection();//计算角平分线
        }

        CornerFeature(const CornerFeature& rhs) = default;

        CornerFeature& operator=(const CornerFeature& rhs) = default;

        bool operator==(const CornerFeature& rhs) const
        {
            return prev_ == rhs.prev_ && next_ == rhs.next_;
        }

    public:
        //角点两边的线段特征
        LineSegmentFeature prev_;
        LineSegmentFeature next_;

        //角点坐标
        Point2f coordinate_;

        //角点两边的夹角
        float included_angle_;

        Eigen::Vector2f direction_;

        //角点间的偏差
        float squaredBias(const CornerFeature& rhs) const
        {
            const auto bias = rhs.coordinate_ - coordinate_;
            return bias.transpose() * bias;//欧式距离的平方
        }

    private:
        //计算角点坐标
        void computeCoordinate()
        {
            const float A1 = prev_.cartesian_parameters_.x();
            const float B1 = prev_.cartesian_parameters_.y();
            const float C1 = prev_.cartesian_parameters_.z();
            const float A2 = next_.cartesian_parameters_.x();
            const float B2 = next_.cartesian_parameters_.y();
            const float C2 = next_.cartesian_parameters_.z();
            const float k = 1.f / (A1 * B2 - A2 * B1);
            coordinate_ = Point2f((B1 * C2 - B2 * C1) * k, (A2 * C1 - A1 * C2) * k);//求解线性方程组即可
        }

//        //计算角点两边的夹角
//        void computeIncludedAngle()
//        {
//            _included_angle = std::remainder(_next.polar_parameters_.x() - _prev.polar_parameters_.x(), PI * 2.f);//可正可负，不一定是哪个夹角，因为两个边没有方向
//        }

        //计算在雷达的视角下，两条相邻的线段所成的夹角，范围0-360°
        void computeIncludedAngle(){
            Eigen::Vector3f v1 = Eigen::Vector3f::Zero();
            v1.head(2) = next_.back_edge_-next_.front_edge_;
            Eigen::Vector3f v2 = Eigen::Vector3f::Zero();
            v2.head(2) = prev_.front_edge_-prev_.back_edge_;
            included_angle_ = std::acos(v1.dot(v2)/(v1.norm()*v2.norm()));//两个线段形成的夹角中小于180的那个角
            if((v1.cross(v2))(2) < 0){//说明在雷达看来夹角大于180
                included_angle_ = 2.0 * M_PI - included_angle_;
            }
        }

        //计算角平分线，由顶点向外的单位向量
        void computeDirection(){
            Eigen::Vector2f pre = prev_.front_edge_ - prev_.back_edge_;
            pre = pre / pre.norm();
            Eigen::Vector2f next = next_.back_edge_ - next_.front_edge_;
            next = next / next.norm();
            Eigen::Vector2f direction = pre + next;
            direction_ = direction / direction.norm();
        }

    };
}
