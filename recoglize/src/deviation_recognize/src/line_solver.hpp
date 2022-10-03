//
// Created by lfc on 18-10-10.
//

#ifndef PROJECT_LINE_SOLVER_HPP
#define PROJECT_LINE_SOLVER_HPP

#include "detector_point.hpp"
#include <glog/logging.h>

namespace detector{

class LineSolver {
public:
    struct LinePara {
        double A;
        double B;
        double C;
        //A*x + B*y =C;
        //y = a * x + b;
        double angle = 0;
        double var;
    };

    static double distToLine(LinePara& para,const Point& point){//求解点到直线的距离
        double line_sqrt = std::hypot(para.A, para.B);
        if (line_sqrt == 0) {
            LOG(INFO) << "line is wrong!";
            return 0;
        }
        return fabs(para.A * point.x + para.B * point.y + para.C) / line_sqrt;
    }

    bool solve(const std::vector<Point> &points, LinePara &para) {
        if (points.size() <= 4) {
            LOG(INFO) << "the size is small!" << points.size();
            return false;
        }
        double mean_x = 0, mean_y = 0;
        double sum_uu = 0, sum_uv = 0, sum_vv = 0, point_num = 0;

        //修整补偿，floor取整函数，[x]
        int trim_offset = floor((double) points.size() * abandon_ration + 0.5);

        int point_size = points.size();

        int end_index = point_size - trim_offset;
        int start_index = trim_offset;

        for (int i = start_index; i < end_index; ++i) {
            mean_x += points[i].x;
            mean_y += points[i].y;
            point_num += 1;
        }

        mean_x = mean_x / point_num;
        mean_y = mean_y / point_num;
        for (int i = start_index; i < end_index; ++i) {
            sum_uu += square(points[i].x - mean_x);
            sum_uv += (points[i].x - mean_x) * (points[i].y - mean_y);
            sum_vv += square(points[i].y - mean_y);
        }
        double theta = atan2(-2 * sum_uv, sum_vv - sum_uu) * 0.5;   
        // double theta = atan2(sum_uv,sum_uu) ;       //范围[-M_PI,M_PI] 
        double rho = mean_x * cos(theta) + mean_y * sin(theta);

        if (rho < 0) {
            para.C = -rho;
            theta -= M_PI;
        }

        para.C = -fabs(rho);
        para.A = cos(theta);
        para.B = sin(theta);
        para.angle = atan2(-para.A, para.B);
        normalizeAngle(para.angle);
        computeCov(points, para);
       LOG(INFO) << "para:" << para.angle * 180 / M_PI;
        return true;
    }

    static bool solveByToPoint(Point &start,Point &end, LinePara &para) {//两点式
        double delta_x = end.x - start.x;
        double delta_y = end.y - start.y;

        para.A = delta_y;
        para.B = -delta_x;
        para.C = delta_x * start.y - delta_y * start.x;
        para.var = 1.0;
        para.angle = atan2(-para.A, para.B);
        normalizeAngle(para.angle);
//        LOG(INFO) << "para:" << para.angle * 180 / M_PI;
        return true;
    }

    //已知直线l外一点A，求直线上一点B，使得AB垂直于直线l
    void computeCrossPoint(const LinePara &para, const Point &input, Point &cross) {    
        double x_0 = input.x;
        double y_0 = input.y;
        double A = para.A;
        double B = para.B;
        double C = para.C;
        double dinominator = A * A + B * B;
        cross.x = (B * B * x_0 - A * B * y_0 - A * C) / (dinominator);
        cross.y = (A * A * y_0 - A * B * x_0 - B * C) / (dinominator);
    }

    //计算两条直线交点
    void computeCorner(const LinePara &para_1, const LinePara &para_2, Point &corner) {
        double a_0 = para_1.A;
        double a_1 = para_2.A;
        double b_0 = para_1.B;
        double b_1 = para_2.B;
        double c_0 = para_1.C;
        double c_1 = para_2.C;

        double dinominator = a_0 * b_1 - a_1 * b_0;
        if (fabs(dinominator) <= 0.0001) {
//            LOG(INFO) << "the dinominator is wrong! will return!";
            corner.x = 0;
            corner.y = 0;
            return;
        }
        corner.x = (b_0 * c_1 - b_1 * c_0) / (dinominator);
        corner.y = -(a_0 * c_1 - a_1 * c_0) / (dinominator);
    }

    //计算自己定义直线的倾斜角，满足与给定两点的倾斜角差值绝对值不大于M_PI_2
    void correctDirection(LinePara &para, Point &start, Point &end) {
        double real_angle = atan2(end.y - start.y, end.x - start.x);
        double delta_angle = real_angle - para.angle;
        normalizeAngle(delta_angle);
        if (fabs(delta_angle) > M_PI_2) {
            para.angle += M_PI;
            normalizeAngle(para.angle);
        }
//        LOG(INFO) << "para angle:" << para.angle * 180 / 3.1415926;
    }

    double computeAngle(Point &start, Point &end) {
        return atan2(end.y - start.y, end.x - start.x);
    }

    //将角度转化为范围为-M_PI_2到M_PI_2
    template<class T> static void normalizeAngle(T &angle) {
        const T &max_angle = (T) (2 * M_PI);

        angle = fmod(angle, (max_angle));   //fmod(x,y),表示x除以y的余数
        if (angle >= (T) (M_PI)) {
            angle -= max_angle;
        } else if (angle < -(T) (M_PI)) {
            angle += max_angle;
        }
    }

private:
    void computeCov(const std::vector<Point> &points, LinePara &para) {
        para.var = 1.0;
    }

    template<typename T>
    inline T square(const T &value) {
        return value * value;
    }


    const double abandon_ration = 0.0625;//根据L型的长宽确定,丢弃的合理量
};

}
#endif //PROJECT_LINE_SOLVER_HPP
