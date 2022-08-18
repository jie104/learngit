//
// Created by ljh on 2021/11/25.
//

#ifndef PERCEPTION_SOLUTION_MACHINE_VISION_HPP

// INCLUDE
#include <string>
#include <iostream>
#include <map>
#include <sys/time.h>
#include <chrono>
#include <opencv2/opencv.hpp>

// CODE
namespace mvision {

/**
 * @brief 筛选的特征项
 */
    enum SelectShapeType {
        SELECT_AREA,                // 选中区域面积
        SELECT_RECTANGULARITY,      // 选中区域矩形度
        SELECT_WIDTH,               // 选中区域宽度（平行于坐标轴）
        SELECT_HEIGHT,              // 选中区域高度（平行于坐标轴）
        SELECT_ROW,                 // 选中区域中心行索引
        SELECT_COLUMN,              // 选中区域中心列索引
        SELECT_RECT2_LEN1,          // 选中区域最小外接矩形的一半长度
        SELECT_RECT2_LEN2,          // 选中区域最小外接矩形的一半宽度
        SELECT_RECT2_PHI,           // 选中区域最小外接矩形的方向
        SELECT_ELLIPSE_RA,          // 选中区域外接椭圆的长半轴
        SELECT_ELLIPSE_RB,          // 选中区域外接椭圆的短半轴
        SELECT_ELLIPSE_PHI          // 选中区域外接椭圆的方向
    };

/**
 * @brief 多特征的筛选关系
 */
    enum SelectOperation {
        SELECT_AND,                 // 与
        SELECT_OR                   // 或
    };


    enum SortCriterion {
        FIRST_POINT,                // 区域第一行的最左侧的点
        LAST_POINT,                 // 区域最后一行的最右侧的点
        UPPER_LEFT,                 // 区域周围矩形的左上角
        UPPER_RIGHT,                // 区域周围矩形的右上角
        LOWER_LEFT,                 // 区域周围矩形的左下角
        LOWER_RIGHT                 // 区域周围矩形的右下角
    };

    enum SortDirection {
        ROW,                        // 区域按行排列，即从左到右，从上到下
        COLUMN                      // 区域按列排列，即从上到下，从左到右
    };

    enum ShapeTransType {
        CONVEX,                     // Convex hull.
        ELLIPSE,                    // Ellipse with the same moments and area as the input region.
        OUTTER_CIRCLE,              // Smallest enclosing circle.
        INNER_CIRCLE,               // Largest circle fitting into the region.
        RECTANGLE1,                 // Smallest enclosing rectangle parallel to the coordinate axes.
        RECTANGLE2,                 // Smallest enclosing rectangle.
        INNER_RECTANGLE1,           // Largest axis-parallel rectangle fitting into the region.
        INNER_CENTER                // The point on the skeleton of the input region having the smallest distance to the center of gravity of the input region.
    };


/**
 * @brief 绘制斜矩形
 * @param img
 * @param rect2
 * @param color
 * @param thickness
 */
    static void drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect2, cv::Scalar color, int thickness) {
        cv::Point2f pts[4];
        rect2.points(pts);
        for (int i = 0; i < 4; ++i) {
            cv::line(img, pts[i], pts[(i + 1) % 4], color, thickness);
        }
    }

/**
 * @brief 变换区域的形状
 * @param src 输入图像
 * @param dst 输出图像
 * @param type 变换形状
 */
    static void shapeTrans(cv::Mat src, cv::Mat &dst, ShapeTransType type) {
        dst = cv::Mat(src.size(), CV_8UC1, cv::Scalar(0));
        std::vector <std::vector<cv::Point>> contours;
        cv::findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        int n = contours.size();
        for (int i = 0; i < n; i++) {
            if (type == ShapeTransType::RECTANGLE1) {
                cv::Rect rect = cv::boundingRect(contours[i]);
                cv::rectangle(dst, rect, cv::Scalar(255), CV_FILLED);
            } else if (type == ShapeTransType::OUTTER_CIRCLE) {
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contours[i], center, radius);
                circle(dst, center, radius, cv::Scalar(255), CV_FILLED);
            } else if (type == ShapeTransType::CONVEX) {
                std::vector <cv::Point> conver;
                cv::convexHull(contours[i], conver);
                std::vector <std::vector<cv::Point>> pconver;
                pconver.push_back(conver);
                cv::fillPoly(dst, pconver, cv::Scalar(255));
            } else if (type == ShapeTransType::RECTANGLE2) {
                cv::RotatedRect rect2 = cv::minAreaRect(contours[i]);
                std::vector <cv::Point> box_points;
                cv::boxPoints(rect2, box_points);
                cv::fillPoly(dst, box_points, cv::Scalar(255));
            } else if (type == ShapeTransType::ELLIPSE) {
                cv::RotatedRect rect2 = cv::fitEllipse(contours[i]);
                cv::ellipse(dst, rect2, cv::Scalar(255), CV_FILLED);
            }
        }
    }

/**
 * @brief 填充空洞算法
 * @param src 输入图像
 * @param dst 输出图像
 */
    static void fillUp(cv::Mat src, cv::Mat &dst) {
        dst = cv::Mat(src.size(), CV_8UC1, cv::Scalar(0));
        std::vector <std::vector<cv::Point>> contours;
        cv::findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        cv::drawContours(dst, contours, -1, cv::Scalar(255), CV_FILLED);
    }

/**
 * @brief 上下区间二值化算法
 * @param gray 输入的单通道图像
 * @param thresh 输出的二值化图像
 * @param minvalue 下限阈值
 * @param maxvalue 上限阈值
 */
    static void threshold2(cv::Mat gray, cv::Mat &thresh, int minValue, int maxValue) {
        cv::Mat thresh1;
        cv::Mat thresh2;
        cv::threshold(gray, thresh1, minValue, 255, cv::THRESH_BINARY);
        cv::threshold(gray, thresh2, maxValue, 255, cv::THRESH_BINARY_INV);
        thresh = thresh1 & thresh2;
    }

/**
 * 计算二值化图像的面积与中心
 * @param src 输入的二值化图像
 * @param area 输出的面积
 * @param center 输出的中心点
 */
    static void areaCenter(cv::Mat src, int &area, cv::Point2f &center) {
        int pixelsCount = src.rows * src.cols;
        area = 0;
        center = cv::Point2f(0, 0);
        float centerX = 0;
        float centerY = 0;
        int mcol = src.cols;
        for (int i = 0; i < pixelsCount; i++) {
            if (src.data[i] == 255) {
                area++;

                int x = i % mcol;
                int y = i / mcol;

                centerX += x;
                centerY += y;
            }
        }
        if (area > 0) {
            centerX /= area;
            centerY /= area;
            center = cv::Point2f(centerX, centerY);
        }
    }

/**
 * @brief 通过特征类型进行region筛选
 * @param src 输入图像
 * @param dst 输出图像
 * @param types 特征类型
 * @param operation 操作符
 * @param mins 下限值
 * @param maxs 上限值
 * @return 筛选出的数量
 */
    static int selectShape(cv::Mat src, cv::Mat &dst, std::vector <SelectShapeType> types, SelectOperation operation,
                           std::vector<double> mins, std::vector<double> maxs) {
        if (!(types.size() == mins.size() && mins.size() == maxs.size()))
            return 0;

        int num = types.size();
        dst = cv::Mat(src.size(), CV_8UC1, cv::Scalar(0));

        std::vector <std::vector<cv::Point>> contours;
        cv::findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        int cnum = contours.size();

        std::vector <std::vector<cv::Point>> selectContours;
        for (int i = 0; i < cnum; i++) {
            bool isAnd = true;
            bool isOr = false;
            for (int j = 0; j < num; j++) {
                double mind = mins[j];
                double maxd = maxs[j];
                if (mind > maxd) {
                    mind = maxs[j];
                    maxd = mins[j];
                }
                if (types[j] == SELECT_AREA) {
                    cv::Mat temp = cv::Mat(src.size(), CV_8UC1, cv::Scalar(0));
                    std::vector <std::vector<cv::Point>> pconver;
                    pconver.push_back(contours[i]);
                    cv::drawContours(temp, pconver, -1, cv::Scalar(255), CV_FILLED);
                    cv::bitwise_and(src, temp, temp);
                    int area;
                    cv::Point2f center;
                    areaCenter(temp, area, center);
                    if (area >= mind && area <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_RECTANGULARITY) {
                    cv::Mat temp = cv::Mat(src.size(), CV_8UC1, cv::Scalar(0));
                    std::vector <std::vector<cv::Point>> pconver;
                    pconver.push_back(contours[i]);
                    cv::drawContours(temp, pconver, -1, cv::Scalar(255), CV_FILLED);
                    cv::bitwise_and(src, temp, temp);
                    int area;
                    cv::Point2f center;
                    areaCenter(temp, area, center);
                    cv::RotatedRect rect = minAreaRect(contours[i]);
                    double rectangularity = area / rect.size.area();
                    if (rectangularity >= mind && rectangularity <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_WIDTH) {
                    cv::Rect rect = cv::boundingRect(contours[i]);
                    if (rect.width >= mind && rect.width <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_HEIGHT) {
                    cv::Rect rect = cv::boundingRect(contours[i]);
                    if (rect.height >= mind && rect.height <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_ROW) {
                    cv::Mat temp = cv::Mat(src.size(), CV_8UC1, cv::Scalar(0));
                    std::vector <std::vector<cv::Point>> pconver;
                    pconver.push_back(contours[i]);
                    cv::drawContours(temp, pconver, -1, cv::Scalar(255), CV_FILLED);
                    cv::bitwise_and(src, temp, temp);
                    int area;
                    cv::Point2f center;
                    areaCenter(temp, area, center);
                    if (center.y >= mind && center.y <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_COLUMN) {
                    cv::Mat temp = cv::Mat(src.size(), CV_8UC1, cv::Scalar(0));
                    std::vector <std::vector<cv::Point>> pconver;
                    pconver.push_back(contours[i]);
                    cv::drawContours(temp, pconver, -1, cv::Scalar(255), CV_FILLED);
                    cv::bitwise_and(src, temp, temp);
                    int area;
                    cv::Point2f center;
                    areaCenter(temp, area, center);
                    if (center.x >= mind && center.x <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_RECT2_LEN1) {
                    cv::RotatedRect rect = cv::minAreaRect(contours[i]);
                    double len = rect.size.width;
                    if (rect.size.width < rect.size.height)
                        len = rect.size.height;
                    if (len / 2 >= mind && len / 2 <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_RECT2_LEN2) {
                    cv::RotatedRect rect = cv::minAreaRect(contours[i]);
                    double len = rect.size.height;
                    if (rect.size.width < rect.size.height)
                        len = rect.size.width;
                    if (len / 2 >= mind && len / 2 <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_RECT2_PHI) {
                    cv::RotatedRect rect = cv::minAreaRect(contours[i]);
                    float angle = 0;
                    if (angle < 0) angle += 180;
                    if (rect.size.width < rect.size.height) {
                        angle = rect.angle;
                        angle -= 90;
                        if (angle < 0) angle += 180;
                    } else {
                        angle = rect.angle;
                    }
                    if (angle >= mind && angle <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_ELLIPSE_RA) {
                    if (contours[i].size() < 5) {
                        isAnd &= false;
                        isOr |= false;
                        continue;
                    }
                    cv::RotatedRect rect = cv::fitEllipse(contours[i]);
                    double len = rect.size.width;
                    if (rect.size.width < rect.size.height)
                        len = rect.size.height;
                    if (len / 2 >= mind && len / 2 <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_ELLIPSE_RB) {
                    if (contours[i].size() < 5) {
                        isAnd &= false;
                        isOr |= false;
                        continue;
                    }
                    cv::RotatedRect rect = cv::fitEllipse(contours[i]);
                    double len = rect.size.height;
                    if (rect.size.width < rect.size.height)
                        len = rect.size.width;
                    if (len / 2 >= mind && len / 2 <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                } else if (types[j] == SELECT_ELLIPSE_PHI) {
                    if (contours[i].size() < 5) {
                        isAnd &= false;
                        isOr |= false;
                        continue;
                    }
                    cv::RotatedRect rect = cv::fitEllipse(contours[i]);
                    float angle = 0;
                    if (angle < 0) angle += 180;
                    if (rect.size.width < rect.size.height) {
                        angle = rect.angle;
                        angle -= 90;
                        if (angle < 0) angle += 180;
                    } else {
                        angle = rect.angle;
                    }
                    if (angle >= mind && angle <= maxd) {
                        isAnd &= true;
                        isOr |= true;
                    } else {
                        isAnd &= false;
                        isOr |= false;
                    }
                }
            }
            if (isAnd && operation == SELECT_AND)
                selectContours.push_back(contours[i]);
            if (isOr && operation == SELECT_OR)
                selectContours.push_back(contours[i]);
        }
        cv::drawContours(dst, selectContours, -1, cv::Scalar(255), CV_FILLED);
        cv::bitwise_and(src, dst, dst);
        return selectContours.size();
    }

/**
 * @brief 根据区域的位置进行区域排序
 * @param src 输入图像
 * @param contours 输入轮廓组
 * @param pos 排序后的索引
 * @param sc 排序基准点
 * @param isDue true:从小到大排序 false：从大到小排序
 * @param sd 排序方向
 * @return true:排序成功 false：排序失败
 */
    static bool sortRegion(cv::Mat src, std::vector <std::vector<cv::Point>> contours, std::vector<int> &pos,
                           SortCriterion sc, bool isDue, SortDirection sd) {
        int count = contours.size();
        pos.resize(count);
        std::vector <cv::Point> points;
        for (int i = 0; i < count; i++) {
            pos[i] = i;
            cv::Rect rect = cv::boundingRect(contours[i]);
            if (sc == FIRST_POINT) {
                int row = rect.y;
                for (int col = rect.x; col <= rect.x + rect.width; col++) {
                    if (src.at<uchar>(row, col) > 0) {
                        points.push_back(cv::Point(col, row));
                        break;
                    }
                }
            } else if (sc == LAST_POINT) {
                int row = rect.y + rect.height;
                for (int col = rect.x + rect.width; col >= rect.x; col--) {
                    if (src.at<uchar>(row, col) > 0) {
                        points.push_back(cv::Point(col, row));
                        break;
                    }
                }
            } else if (sc == UPPER_LEFT)
                points.push_back(rect.tl());
            else if (sc == UPPER_RIGHT)
                points.push_back(cv::Point(rect.x + rect.width, rect.y));
            else if (sc == LOWER_LEFT)
                points.push_back(cv::Point(rect.x, rect.y + rect.height));
            else if (sc == LOWER_RIGHT)
                points.push_back(rect.br());
        }
        int np = points.size();
        if (np != count)
            return false;
        if (sd == ROW) {
            for (int i = 0; i < count - 1; i++) {
                for (int j = 0; j < count - 1 - i; j++) {
                    if (isDue) {
                        if (points[j].y > points[j + 1].y) {
                            cv::Point temp = points[j];
                            points[j] = points[j + 1];
                            points[j + 1] = temp;

                            int index = pos[j];
                            pos[j] = pos[j + 1];
                            pos[j + 1] = index;
                        }
                    } else {
                        if (points[j].y < points[j + 1].y) {
                            cv::Point temp = points[j];
                            points[j] = points[j + 1];
                            points[j + 1] = temp;

                            int index = pos[j];
                            pos[j] = pos[j + 1];
                            pos[j + 1] = index;
                        }
                    }
                }
            }
            for (int i = 0; i < count - 1; i++) {
                for (int j = 0; j < count - 1 - i; j++) {
                    if (points[j].y == points[j + 1].y) {
                        if (isDue) {
                            if (points[j].x > points[j + 1].x) {
                                cv::Point temp = points[j];
                                points[j] = points[j + 1];
                                points[j + 1] = temp;

                                int index = pos[j];
                                pos[j] = pos[j + 1];
                                pos[j + 1] = index;
                            }
                        } else {
                            if (points[j].x < points[j + 1].x) {
                                cv::Point temp = points[j];
                                points[j] = points[j + 1];
                                points[j + 1] = temp;

                                int index = pos[j];
                                pos[j] = pos[j + 1];
                                pos[j + 1] = index;
                            }
                        }
                    }
                }
            }
        } else if (sd == COLUMN) {
            for (int i = 0; i < count - 1; i++) {
                for (int j = 0; j < count - 1 - i; j++) {
                    if (isDue) {
                        if (points[j].x > points[j + 1].x) {
                            cv::Point temp = points[j];
                            points[j] = points[j + 1];
                            points[j + 1] = temp;

                            int index = pos[j];
                            pos[j] = pos[j + 1];
                            pos[j + 1] = index;
                        }
                    } else {
                        if (points[j].x < points[j + 1].x) {
                            cv::Point temp = points[j];
                            points[j] = points[j + 1];
                            points[j + 1] = temp;

                            int index = pos[j];
                            pos[j] = pos[j + 1];
                            pos[j + 1] = index;
                        }
                    }
                }
            }
            for (int i = 0; i < count - 1; i++) {
                for (int j = 0; j < count - 1 - i; j++) {
                    if (points[j].x == points[j + 1].x) {
                        if (isDue) {
                            if (points[j].y > points[j + 1].y) {
                                cv::Point temp = points[j];
                                points[j] = points[j + 1];
                                points[j + 1] = temp;

                                int index = pos[j];
                                pos[j] = pos[j + 1];
                                pos[j + 1] = index;
                            }
                        } else {
                            if (points[j].y < points[j + 1].y) {
                                cv::Point temp = points[j];
                                points[j] = points[j + 1];
                                points[j + 1] = temp;

                                int index = pos[j];
                                pos[j] = pos[j + 1];
                                pos[j + 1] = index;
                            }
                        }
                    }
                }
            }
        }
        return true;
    }

/**
 * @brief 绘制十字
 * @param[in] img 目标图像
 * @param[in] point 十字中心点
 * @param[in] color 颜色
 * @param[in] size 十字尺寸
 * @param[in] thickness 粗细
 */
    static void drawCross(cv::Mat img, cv::Point point, cv::Scalar color, int size, int thickness) {
        //绘制横线
        cv::line(img, cv::Point(point.x - size / 2, point.y), cv::Point(point.x + size / 2, point.y), color, thickness,
                 8, 0);
        //绘制竖线
        cv::line(img, cv::Point(point.x, point.y - size / 2), cv::Point(point.x, point.y + size / 2), color, thickness,
                 8, 0);
        return;
    }

/**
 *
 * @param srcImage
 * @param dstImage
 */
    static void LargestConnecttedComponent(cv::Mat srcImage, cv::Mat &dstImage) {
        cv::Mat temp;
        cv::Mat labels;
        srcImage.copyTo(temp);

        //1. 标记连通域
        int n_comps = connectedComponents(temp, labels, 4, CV_16U);
        std::vector<int> histogram_of_labels;
        //初始化labels的个数为0
        for (int i = 0; i < n_comps; i++) {
            histogram_of_labels.push_back(0);
        }

        int rows = labels.rows;
        int cols = labels.cols;
        //计算每个labels的个数
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                histogram_of_labels.at(labels.at<unsigned short>(row, col)) += 1;
            }
        }
        histogram_of_labels.at(0) = 0; //将背景的labels个数设置为0

        //2. 计算最大的连通域labels索引
        int maximum = 0;
        int max_idx = 0;
        for (int i = 0; i < n_comps; i++) {
            if (histogram_of_labels.at(i) > maximum) {
                maximum = histogram_of_labels.at(i);
                max_idx = i;
            }
        }

        //3. 将最大连通域标记为1
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                if (labels.at<unsigned short>(row, col) == max_idx) {
                    labels.at<unsigned short>(row, col) = 255;
                } else {
                    labels.at<unsigned short>(row, col) = 0;
                }
            }
        }

        //4. 将图像更改为CV_8U格式
        labels.convertTo(dstImage, CV_8U);
    }

    static cv::Rect rectCenterScale(cv::Rect rect , cv::Size size){
        rect =  rect + size;
        cv::Point pt;
        pt.x = cvRound(size.width/2.0);
        pt.y = cvRound(size.height/2.0);
        return (rect-pt);
    }


}

#define PERCEPTION_SOLUTION_MACHINE_VISION_HPP

#endif //PERCEPTION_SOLUTION_MACHINE_VISION_HPP
